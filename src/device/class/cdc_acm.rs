//! CDC ACM (USB Communication Device Class – Abstract Control Model).
//!
//! Creates a virtual serial port visible to the host operating system.
//!
//! # Example
//!
//! ```ignore
//! use my_hal_usb::{UsbBusAllocator, UsbDeviceBuilder};
//! use my_hal_usb::class::cdc_acm::CdcAcm;
//! use my_hal_usb::MyBus;
//!
//! let alloc = UsbBusAllocator::new(MyBus::new(/* peripheral */));
//! let mut serial = CdcAcm::new(&alloc);
//! let mut dev = UsbDeviceBuilder::new(&alloc, 0x1234, 0x5678)
//!     .product("CDC Serial")
//!     .composite_with_iads()
//!     .build();
//!
//! loop {
//!     dev.poll(&mut [&mut serial]);
//!     let mut buf = [0u8; 64];
//!     if let Ok(n) = serial.read(&mut buf) {
//!         let _ = serial.write(&buf[..n]);
//!     }
//! }
//! ```

use crate::descriptor::{cdc_descriptor, cdc_protocol, cdc_subclass, class_code};
use crate::device::bus::{InterfaceNumber, UsbBus, UsbBusAllocator};
use crate::device::class::UsbClass;
use crate::device::control::{ControlIn, ControlOut};
use crate::device::descriptor::DescriptorWriter;
use crate::device::endpoint::{EndpointIn, EndpointOut};
use crate::device::error::{UsbError, UsbResult};
use crate::request::{Recipient, RequestType};

// ─── CDC-ACM request codes ────────────────────────────────────────────────────

mod req {
    pub const SET_LINE_CODING: u8 = 0x20;
    pub const GET_LINE_CODING: u8 = 0x21;
    pub const SET_CONTROL_LINE_STATE: u8 = 0x22;
    pub const SEND_BREAK: u8 = 0x23;
}

// ─── LineCoding ───────────────────────────────────────────────────────────────

/// Serial port line-coding parameters (7-byte CDC structure).
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LineCoding {
    /// Data terminal rate in bits per second.
    pub baud_rate: u32,
    /// Stop bits: 0 = 1 bit, 1 = 1.5 bits, 2 = 2 bits.
    pub stop_bits: u8,
    /// Parity: 0 = None, 1 = Odd, 2 = Even, 3 = Mark, 4 = Space.
    pub parity: u8,
    /// Data bits: 5, 6, 7, 8, or 16.
    pub data_bits: u8,
}

impl Default for LineCoding {
    fn default() -> Self {
        Self {
            baud_rate: 115_200,
            stop_bits: 0,
            parity: 0,
            data_bits: 8,
        }
    }
}

impl LineCoding {
    fn to_bytes(self) -> [u8; 7] {
        let b = self.baud_rate.to_le_bytes();
        [
            b[0],
            b[1],
            b[2],
            b[3],
            self.stop_bits,
            self.parity,
            self.data_bits,
        ]
    }

    fn from_bytes(b: &[u8]) -> Option<Self> {
        if b.len() < 7 {
            return None;
        }
        Some(Self {
            baud_rate: u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
            stop_bits: b[4],
            parity: b[5],
            data_bits: b[6],
        })
    }
}

// ─── CdcAcm ───────────────────────────────────────────────────────────────────

/// CDC ACM virtual serial port class.
///
/// Construct with [`CdcAcm::new`], passing the [`UsbBusAllocator`].
pub struct CdcAcm<'a, B: UsbBus> {
    comm_if: InterfaceNumber,
    data_if: InterfaceNumber,
    int_in: EndpointIn<'a, B>,
    bulk_in: EndpointIn<'a, B>,
    bulk_out: EndpointOut<'a, B>,
    line_coding: LineCoding,
    dtr: bool,
    rts: bool,
    configured: bool,
}

impl<'a, B: UsbBus> CdcAcm<'a, B> {
    /// Create a CDC ACM instance, allocating the required endpoints and interfaces.
    ///
    /// Bulk endpoints use the bus's [`UsbBus::MAX_PACKET_SIZE`] (64 bytes on
    /// full-speed buses, 512 on high-speed ones).
    pub fn new(alloc: &'a UsbBusAllocator<B>) -> Self {
        let comm_if = alloc.interface();
        let data_if = alloc.interface();
        let int_in = alloc.interrupt(8, 16);
        let bulk_size = B::MAX_PACKET_SIZE;
        let bulk_in = alloc.bulk(bulk_size);
        let bulk_out = alloc.bulk(bulk_size);
        Self {
            comm_if,
            data_if,
            int_in,
            bulk_in,
            bulk_out,
            line_coding: LineCoding::default(),
            dtr: false,
            rts: false,
            configured: false,
        }
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /// Read received bytes from the host into `buf`.
    ///
    /// Returns the number of bytes read, or [`UsbError::WouldBlock`] when no
    /// data is available.
    pub fn read(&self, buf: &mut [u8]) -> UsbResult<usize> {
        if !self.configured {
            return Err(UsbError::InvalidState);
        }
        self.bulk_out.read(buf)
    }

    /// Write bytes to the host.
    ///
    /// Returns the number of bytes actually written, or [`UsbError::WouldBlock`]
    /// when the previous packet has not yet been sent.
    pub fn write(&self, data: &[u8]) -> UsbResult<usize> {
        if !self.configured {
            return Err(UsbError::InvalidState);
        }
        self.bulk_in.write(data)
    }

    /// Queue `data`, splitting it into endpoint-sized packets.
    ///
    /// While the endpoint is busy the call spins, spending one unit of `budget`
    /// per attempt; the budget bounds the time spent waiting for a host that is
    /// not collecting. Returns the number of bytes queued.
    pub fn write_all(&self, data: &[u8], mut budget: u32) -> usize {
        if !self.configured {
            return 0;
        }
        let max_packet = self.bulk_in.max_packet_size() as usize;
        let mut sent = 0;
        while sent < data.len() {
            let end = (sent + max_packet).min(data.len());
            loop {
                match self.bulk_in.write(&data[sent..end]) {
                    Ok(_) => break,
                    Err(UsbError::WouldBlock) if budget > 0 => {
                        budget -= 1;
                        core::hint::spin_loop();
                    }
                    Err(_) => return sent,
                }
            }
            sent = end;
        }
        sent
    }

    /// The communication interface number.
    #[inline]
    pub fn communication_interface(&self) -> InterfaceNumber {
        self.comm_if
    }

    /// The data interface number.
    #[inline]
    pub fn data_interface(&self) -> InterfaceNumber {
        self.data_if
    }

    /// `true` if the host has opened the serial port (DTR set).
    pub fn dtr(&self) -> bool {
        self.dtr
    }

    /// `true` if the host has asserted RTS.
    pub fn rts(&self) -> bool {
        self.rts
    }

    /// `true` if the device is configured and the host has opened the serial port.
    pub fn port_open(&self) -> bool {
        self.configured && self.dtr
    }

    /// `true` if the device has been configured by the host.
    pub fn configured(&self) -> bool {
        self.configured
    }

    /// Returns the current line coding (baud rate, data bits, etc.).
    pub fn line_coding(&self) -> LineCoding {
        self.line_coding
    }
}

// ─── UsbClass<B> implementation ───────────────────────────────────────────────

impl<B: UsbBus> UsbClass<B> for CdcAcm<'_, B> {
    fn configuration_descriptors(&self, w: &mut DescriptorWriter) -> UsbResult<()> {
        // ── IAD (Interface Association Descriptor) ────────────────────────────
        w.iad(
            self.comm_if,
            2, // 2 interfaces
            class_code::CDC,
            cdc_subclass::ACM,
            cdc_protocol::NONE,
        )?;

        // ── Communication Interface ───────────────────────────────────────────
        w.interface(
            self.comm_if,
            1, // 1 endpoint (interrupt IN)
            class_code::CDC,
            cdc_subclass::ACM,
            cdc_protocol::NONE,
        )?;

        // Header functional descriptor.
        w.write(
            cdc_descriptor::CS_INTERFACE,
            &[cdc_descriptor::HEADER, 0x10, 0x01], // bcdCDC 1.10
        )?;
        // Call Management functional descriptor.
        w.write(
            cdc_descriptor::CS_INTERFACE,
            &[
                cdc_descriptor::CALL_MANAGEMENT,
                0x00, // bmCapabilities: no call management
                self.data_if.into(),
            ],
        )?;
        // ACM functional descriptor.
        w.write(
            cdc_descriptor::CS_INTERFACE,
            &[
                cdc_descriptor::ABSTRACT_CONTROL_MANAGEMENT,
                0x02, // bmCapabilities: supports SetLineCoding/GetLineCoding/SetControlLineState
            ],
        )?;
        // Union functional descriptor.
        w.write(
            cdc_descriptor::CS_INTERFACE,
            &[
                cdc_descriptor::UNION,
                self.comm_if.into(),
                self.data_if.into(),
            ],
        )?;
        // Interrupt IN endpoint.
        w.endpoint(
            self.int_in.address(),
            self.int_in.ep_type(),
            self.int_in.max_packet_size(),
            self.int_in.interval(),
        )?;

        // ── Data Interface ────────────────────────────────────────────────────
        w.interface(
            self.data_if,
            2, // 2 endpoints (bulk IN + OUT)
            class_code::CDC_DATA,
            0x00,
            0x00,
        )?;
        w.endpoint(
            self.bulk_out.address(),
            self.bulk_out.ep_type(),
            self.bulk_out.max_packet_size(),
            0,
        )?;
        w.endpoint(
            self.bulk_in.address(),
            self.bulk_in.ep_type(),
            self.bulk_in.max_packet_size(),
            0,
        )?;

        Ok(())
    }

    fn control_in(&mut self, xfer: ControlIn<'_, B>) {
        let req = xfer.request();

        if req.request_type != RequestType::Class || req.recipient != Recipient::Interface {
            return;
        }
        if req.index as u8 != u8::from(self.comm_if) {
            return;
        }

        match req.request {
            req::GET_LINE_CODING => {
                let bytes = self.line_coding.to_bytes();
                let _ = xfer.accept_with(&bytes);
            }
            _ => {
                let _ = xfer.reject();
            }
        }
    }

    fn control_out(&mut self, xfer: ControlOut<'_, B>) {
        let req = xfer.request();

        if req.request_type != RequestType::Class || req.recipient != Recipient::Interface {
            return;
        }
        if req.index as u8 != u8::from(self.comm_if) {
            return;
        }

        match req.request {
            req::SET_LINE_CODING => {
                if let Some(lc) = LineCoding::from_bytes(xfer.data()) {
                    self.line_coding = lc;
                    #[cfg(feature = "defmt")]
                    crate::log::debug!("CDC: SET_LINE_CODING baud={}", lc.baud_rate);
                }
                let _ = xfer.accept();
            }
            req::SET_CONTROL_LINE_STATE => {
                let value = req.value;
                self.dtr = value & 0x01 != 0;
                self.rts = value & 0x02 != 0;
                #[cfg(feature = "defmt")]
                crate::log::debug!("CDC: control line state dtr={} rts={}", self.dtr, self.rts);
                let _ = xfer.accept();
            }
            req::SEND_BREAK => {
                // Ignore break signals (no hardware support).
                let _ = xfer.accept();
            }
            _ => {
                let _ = xfer.reject();
            }
        }
    }

    fn configure(&mut self) {
        self.configured = true;
    }

    fn deconfigure(&mut self) {
        self.configured = false;
    }

    fn reset(&mut self) {
        self.configured = false;
        self.dtr = false;
        self.rts = false;
    }
}
