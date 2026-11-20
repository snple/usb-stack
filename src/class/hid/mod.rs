//! USB HID (Human Interface Device) class.
//!
//! The module is split in two halves:
//!
//! * [`HidClass`] — the USB side.  It owns the interface and the interrupt
//!   endpoints, writes the interface/HID/endpoint descriptors, serves the report
//!   descriptor, answers the HID class requests (`Get/Set Report`,
//!   `Get/Set Idle`, `Get/Set Protocol`) and forwards reports to the device.
//! * [`HidDevice`] — the HID side.  A report descriptor, endpoint sizes and the
//!   reports the function exchanges with the host.
//!
//! Ready-made functions live in the submodules:
//!
//! | Function | Reports |
//! |----------|---------|
//! | [`keyboard::Keyboard`] | boot keyboard: 8-byte input, 1-byte LED output |
//! | [`mouse::Mouse`] | boot mouse: 4-byte input |
//! | [`custom::CustomHid`] | caller-supplied report descriptor |
//! | [`power::PowerDevice`] | HID Power Device: status, measurements, battery, configuration, controls |
//!
//! # Example
//!
//! ```ignore
//! use my_hal_usb::{UsbBusAllocator, UsbDeviceBuilder};
//! use my_hal_usb::class::hid::{HidClass, keyboard::{Keyboard, KeyboardReport}};
//! use my_hal_usb::MyBus;
//!
//! let alloc = UsbBusAllocator::new(MyBus::new(/* peripheral */));
//! let mut kbd = HidClass::new(&alloc, Keyboard::new());
//! let mut dev = UsbDeviceBuilder::new(&alloc, 0x2E8A, 0x000B)
//!     .product("Pico HID Keyboard")
//!     .build();
//!
//! loop {
//!     dev.poll(&mut [&mut kbd]);
//!
//!     let report = KeyboardReport::default();
//!     let _ = kbd.send_input(&report.to_bytes());
//! }
//! ```
//!
//! # Reaching the device
//!
//! The class owns the device, so applications drive it through
//! [`HidClass::device_mut`].

pub mod custom;
pub mod device;
pub mod keyboard;
pub mod mouse;
pub mod power;

pub use device::{HidDevice, HidSpec, ReportType};

use crate::bus::{InterfaceNumber, UsbBus, UsbBusAllocator};
use crate::class::UsbClass;
use crate::control::{ControlIn, ControlOut, Recipient, RequestType, StandardRequest};
use crate::descriptor::{DescriptorWriter, class_code};
use crate::endpoint::{EndpointAddress, EndpointIn, EndpointOut, In, Out};
use crate::error::{UsbError, UsbResult};

// ─── HID constants ───────────────────────────────────────────────────────────

/// HID subclass (`bInterfaceSubClass`).
pub mod hid_subclass {
    /// No subclass — report protocol only.
    pub const NONE: u8 = 0x00;
    /// Boot interface subclass — the device also implements a boot protocol.
    pub const BOOT: u8 = 0x01;
}

/// HID protocol (`bInterfaceProtocol`), only meaningful for boot devices.
pub mod hid_protocol {
    /// No protocol.
    pub const NONE: u8 = 0x00;
    /// Keyboard boot protocol.
    pub const KEYBOARD: u8 = 0x01;
    /// Mouse boot protocol.
    pub const MOUSE: u8 = 0x02;
}

/// Values exchanged by `Get Protocol` / `Set Protocol` (HID 1.11 §7.2.5).
///
/// Not to be confused with the interface descriptor's protocol field
/// ([`hid_protocol`]), which says *which* boot protocol a device implements.
pub mod hid_protocol_mode {
    /// Boot protocol — reports use the fixed boot format.
    pub const BOOT: u8 = 0x00;
    /// Report protocol — reports follow the report descriptor.
    pub const REPORT: u8 = 0x01;
}

/// HID class descriptor types.
pub mod hid_descriptor_type {
    /// HID descriptor (`0x21`), written by [`HidClass`](super::HidClass).
    pub const HID: u8 = 0x21;
    /// Report descriptor (`0x22`), served for `Get Descriptor`.
    pub const REPORT: u8 = 0x22;
    /// Physical descriptor (`0x23`), rarely used.
    pub const PHYSICAL: u8 = 0x23;
}

/// HID class-specific request codes (`bRequest`).
pub mod hid_request {
    /// `Get Report` — read a report from the device.
    pub const GET_REPORT: u8 = 0x01;
    /// `Get Idle` — current idle rate, in 4 ms units.
    pub const GET_IDLE: u8 = 0x02;
    /// `Get Protocol` — `0` = boot protocol, `1` = report protocol.
    pub const GET_PROTOCOL: u8 = 0x03;
    /// `Set Report` — write a report to the device.
    pub const SET_REPORT: u8 = 0x09;
    /// `Set Idle` — set the idle rate.
    pub const SET_IDLE: u8 = 0x0A;
    /// `Set Protocol`.
    pub const SET_PROTOCOL: u8 = 0x0B;
}

/// `bcdHID` value written to the HID descriptor (HID 1.11).
const BCD_HID: u16 = 0x0111;

/// Buffer for one interrupt OUT packet — the full-speed maximum packet size.
const OUT_PACKET_MAX: usize = 64;

// ─── HidClass ────────────────────────────────────────────────────────────────

/// USB HID class — the transport for a [`HidDevice`].
///
/// Allocates one interface, an interrupt IN endpoint for input reports and, if
/// the device's [`HidSpec`] asks for one, an interrupt OUT endpoint.  The
/// endpoint sizes and the interface class/protocol come from the device's spec.
pub struct HidClass<'a, B: UsbBus, D> {
    interface: InterfaceNumber,
    int_in: EndpointIn<'a, B>,
    int_out: Option<EndpointOut<'a, B>>,
    idle_rate: u8,
    protocol: u8,
    configured: bool,
    device: D,
}

impl<'a, B: UsbBus, D: HidDevice<B>> HidClass<'a, B, D> {
    /// Create a HID class for `device`, allocating its interface and endpoints.
    ///
    /// Must be called before the [`UsbBusAllocator`] is frozen by building the
    /// device.
    pub fn new(alloc: &'a UsbBusAllocator<B>, device: D) -> Self {
        let spec = device.spec();
        let interface = alloc.interface();
        let int_in = alloc.interrupt::<In>(spec.in_report_size, spec.poll_interval_ms);
        let int_out = if spec.out_report_size == 0 {
            None
        } else {
            Some(alloc.interrupt::<Out>(spec.out_report_size, spec.poll_interval_ms))
        };
        Self {
            interface,
            int_in,
            int_out,
            idle_rate: 0,
            // Report protocol: devices here always send descriptor-format
            // reports, and boot devices' descriptors match the boot format.
            protocol: hid_protocol_mode::REPORT,
            configured: false,
            device,
        }
    }

    // ── Device access ─────────────────────────────────────────────────────────

    /// The wrapped HID function.
    pub fn device(&self) -> &D {
        &self.device
    }

    /// The wrapped HID function, to update its state from the main loop.
    pub fn device_mut(&mut self) -> &mut D {
        &mut self.device
    }

    // ── State ─────────────────────────────────────────────────────────────────

    /// `true` once the host has configured the device.
    pub fn is_configured(&self) -> bool {
        self.configured
    }

    /// Interface number allocated for this function.
    pub fn interface(&self) -> InterfaceNumber {
        self.interface
    }

    /// Report descriptor served to the host.
    pub fn report_descriptor(&self) -> &'static [u8] {
        self.device.spec().report_descriptor
    }

    // ── Input reports ─────────────────────────────────────────────────────────

    /// Send an input report to the host.
    ///
    /// `report` is the raw report, including the report ID byte when the
    /// descriptor uses report IDs.  Returns
    /// [`WouldBlock`](UsbError::WouldBlock) when the previous report has not
    /// been collected yet — the caller retries later.
    pub fn send_input(&self, report: &[u8]) -> UsbResult<()> {
        if !self.configured {
            return Err(UsbError::InvalidState);
        }
        self.int_in.write(report).map(|_| ())
    }

    /// Let the device send the input reports it queued.
    ///
    /// Called automatically from every [`UsbClass::poll`], so calling it is only
    /// useful to shorten the latency by one loop iteration.
    pub fn flush(&mut self) {
        if self.configured {
            self.device.poll(&self.int_in);
        }
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    /// `true` if the request is a class/standard request addressed at this
    /// function's interface.
    fn is_our_interface(&self, request: &crate::control::Request) -> bool {
        request.recipient == Recipient::Interface && request.index as u8 == u8::from(self.interface)
    }

    /// `true` if the device implements the boot protocol.
    fn is_boot_device(&self) -> bool {
        self.device.spec().subclass == hid_subclass::BOOT
    }
}

// ─── UsbClass<B> implementation ───────────────────────────────────────────────

impl<B: UsbBus, D: HidDevice<B>> UsbClass<B> for HidClass<'_, B, D> {
    fn configuration_descriptors(&self, w: &mut DescriptorWriter) -> UsbResult<()> {
        let spec = self.device.spec();
        let num_endpoints = 1 + self.int_out.is_some() as u8;
        w.interface(
            self.interface,
            num_endpoints,
            class_code::HID,
            spec.subclass,
            spec.protocol,
        )?;

        // HID descriptor: one class descriptor follows, the report descriptor.
        let report_len = spec.report_descriptor.len() as u16;
        w.write(
            hid_descriptor_type::HID,
            &[
                (BCD_HID >> 8) as u8,
                BCD_HID as u8,
                0x00, // bCountryCode — not localised
                0x01, // bNumDescriptors
                hid_descriptor_type::REPORT,
                report_len as u8,
                (report_len >> 8) as u8,
            ],
        )?;

        w.endpoint(
            self.int_in.address(),
            self.int_in.ep_type(),
            self.int_in.max_packet_size(),
            spec.poll_interval_ms,
        )?;

        if let Some(ref ep) = self.int_out {
            w.endpoint(
                ep.address(),
                ep.ep_type(),
                ep.max_packet_size(),
                spec.poll_interval_ms,
            )?;
        }

        Ok(())
    }

    fn control_in(&mut self, xfer: ControlIn<'_, B>) {
        let req = *xfer.request();

        if !self.is_our_interface(&req) {
            return;
        }

        match req.request_type {
            // Get Descriptor (HID Report) — hand out the report descriptor.
            RequestType::Standard if req.request == StandardRequest::GET_DESCRIPTOR => {
                let (desc_type, _) = req.descriptor_type_index();
                if desc_type == hid_descriptor_type::REPORT {
                    let _ = xfer.accept_with_static(self.device.spec().report_descriptor);
                }
            }
            RequestType::Class => match req.request {
                hid_request::GET_REPORT => {
                    let Some(report_type) = ReportType::from_w_value(req.value) else {
                        let _ = xfer.reject();
                        return;
                    };
                    let report_id = req.value as u8;
                    // The closure fills the control buffer from device state; an
                    // error leaves the request unanswered and the device layer
                    // stalls EP0.
                    let _ = xfer.accept(|buf| {
                        self.device
                            .get_report(report_type, report_id, buf)
                            .ok_or(UsbError::InvalidState)
                    });
                }
                // Idle rate is transport state: report it back, remember writes,
                // and ignore its value (input reports are sent on change).
                hid_request::GET_IDLE => {
                    let _ = xfer.accept_with(&[self.idle_rate]);
                }
                // Report protocol is always supported; a non-boot device has
                // nothing else to offer, but answering is friendlier than
                // stalling a host that asks anyway.
                hid_request::GET_PROTOCOL => {
                    let _ = xfer.accept_with(&[self.protocol]);
                }
                _ => {
                    let _ = xfer.reject();
                }
            },
            _ => {}
        }
    }

    fn control_out(&mut self, xfer: ControlOut<'_, B>) {
        let req = *xfer.request();

        if !self.is_our_interface(&req) {
            return;
        }

        if req.request_type != RequestType::Class {
            return;
        }

        match req.request {
            hid_request::SET_REPORT => {
                let Some(report_type) = ReportType::from_w_value(req.value) else {
                    let _ = xfer.reject();
                    return;
                };
                let report_id = req.value as u8;
                if self.device.set_report(report_type, report_id, xfer.data()) {
                    let _ = xfer.accept();
                } else {
                    let _ = xfer.reject();
                }
            }
            hid_request::SET_IDLE => {
                self.idle_rate = (req.value >> 8) as u8;
                let _ = xfer.accept();
            }
            // Only a boot device can be asked to switch protocols; the reports
            // of the ready-made boot devices are already boot-format.
            hid_request::SET_PROTOCOL if self.is_boot_device() => {
                self.protocol = req.value as u8;
                let _ = xfer.accept();
            }
            _ => {
                let _ = xfer.reject();
            }
        }
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        let mut packet = [0u8; OUT_PACKET_MAX];

        let Self {
            int_out, device, ..
        } = self;
        let Some(ep) = int_out else { return };
        if ep.address() != addr {
            return;
        }
        let Ok(len) = ep.read(&mut packet) else {
            return;
        };
        if len == 0 {
            return;
        }

        // Interrupt OUT reports carry their report ID as the first byte when the
        // descriptor uses report IDs (HID 1.11 §7.2.2); without them the whole
        // packet is report data.
        if device.spec().report_ids {
            device.output_report(packet[0], &packet[1..len]);
        } else {
            device.output_report(0, &packet[..len]);
        }
    }

    fn configure(&mut self) {
        self.configured = true;
        self.device.configure();
    }

    fn deconfigure(&mut self) {
        self.configured = false;
        self.device.deconfigure();
    }

    fn reset(&mut self) {
        self.configured = false;
        self.idle_rate = 0;
        self.protocol = hid_protocol_mode::REPORT;
        self.device.reset();
    }

    fn poll(&mut self) {
        self.flush();
    }
}
