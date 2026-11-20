//! USB device state machine and builder.
//!
//! Create a [`UsbDevice`] via [`UsbDeviceBuilder`], then call
//! [`poll`](UsbDevice::poll) frequently (every ≤ 10 ms) from the main loop or
//! an interrupt handler.

use core::cmp::min;

use crate::bus::{InterfaceNumber, PollResult, StringIndex, UsbBus, UsbBusAllocator};
use crate::class::UsbClass;
use crate::control::{
    ControlIn, ControlOut, ControlPipe, Feature, Recipient, Request, RequestType, StandardRequest,
};
use crate::descriptor::{DescriptorWriter, descriptor_type, lang_id};
use crate::endpoint::{Direction, EndpointAddress, EndpointType};
use crate::error::UsbResult;

// ─── DeviceState ─────────────────────────────────────────────────────────────

/// High-level USB device state.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeviceState {
    /// Just created or reset — default address (0).
    Default,
    /// Address assigned, not yet configured.
    Addressed,
    /// Fully configured — data transfers are permitted.
    Configured,
    /// Suspended by the host.
    Suspend,
}

/// The bConfiguration value when not configured.
pub const CONFIGURATION_NONE: u8 = 0;
/// The bConfiguration value for the single configuration we expose.
pub const CONFIGURATION_VALUE: u8 = 1;
/// Default alternate setting.
pub const DEFAULT_ALTERNATE_SETTING: u8 = 0;

// ─── DeviceInfo ──────────────────────────────────────────────────────────────

/// Static device metadata.  Passed to [`UsbDeviceBuilder`] via the fluent API.
pub struct DeviceInfo<'a> {
    /// Vendor ID (USB-IF assigned).
    pub vendor_id: u16,
    /// Product ID.
    pub product_id: u16,
    /// Device class code (0 = specified at interface level).
    pub device_class: u8,
    /// Device subclass code.
    pub device_sub_class: u8,
    /// Device protocol code.
    pub device_protocol: u8,
    /// Maximum EP0 packet size — must be 8, 16, 32 or 64.
    pub max_packet_size_0: u8,
    /// Device release in BCD (e.g. `0x0100` = 1.00).
    pub device_release: u16,
    /// USB spec compliance: `0x0200` = USB 2.0, `0x0210` = USB 2.1.
    pub usb_rev: u16,
    /// Manufacturer string (English).
    pub manufacturer: Option<&'a str>,
    /// Product string (English).
    pub product: Option<&'a str>,
    /// Serial number string (English).
    pub serial_number: Option<&'a str>,
    /// `true` if the device has its own power supply.
    pub self_powered: bool,
    /// `true` if the device can signal remote wake-up.
    pub supports_remote_wakeup: bool,
    /// Maximum power draw in 2 mA units (e.g. 50 = 100 mA).
    pub max_power: u8,
}

// ─── UsbDevice ───────────────────────────────────────────────────────────────

/// USB composite device.
///
/// Ties together the bus driver, a control-transfer state machine, and one or
/// more [`UsbClass`] implementations.
///
/// The lifetime `'a` is the lifetime of the [`UsbBusAllocator`] from which this
/// device and all classes were built.
pub struct UsbDevice<'a, B: UsbBus> {
    bus: &'a B,
    info: DeviceInfo<'a>,
    control: ControlPipe,
    state: DeviceState,
    pending_address: u8,
    remote_wakeup_enabled: bool,
    self_powered: bool,
    suspended_state: Option<DeviceState>,
}

// ─── Internals ────────────────────────────────────────────────────────────────

type Classes<'c, B> = [&'c mut dyn UsbClass<B>];

impl<'a, B: UsbBus> UsbDevice<'a, B> {
    // ── Construction ─────────────────────────────────────────────────────────

    /// Build a `UsbDevice` from an allocator.
    ///
    /// Allocates EP0, calls `freeze()` on the allocator (enabling the peripheral),
    /// and returns the ready device.
    pub(crate) fn build(alloc: &'a UsbBusAllocator<B>, info: DeviceInfo<'a>) -> Self {
        // Allocate EP0 control endpoints (must be first allocations).
        alloc
            .alloc::<crate::endpoint::Out>(
                Some(EndpointAddress::EP0_OUT),
                EndpointType::Control,
                info.max_packet_size_0 as u16,
                0,
            )
            .expect("EP0 OUT allocation failed");
        alloc
            .alloc::<crate::endpoint::In>(
                Some(EndpointAddress::EP0_IN),
                EndpointType::Control,
                info.max_packet_size_0 as u16,
                0,
            )
            .expect("EP0 IN allocation failed");

        let bus = alloc.freeze();
        let control = ControlPipe::new(info.max_packet_size_0 as u16);
        Self {
            bus,
            self_powered: info.self_powered,
            info,
            control,
            state: DeviceState::Default,
            pending_address: 0,
            remote_wakeup_enabled: false,
            suspended_state: None,
        }
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /// Current device state.
    pub fn state(&self) -> DeviceState {
        self.state
    }

    /// `true` if the device is fully configured.
    pub fn is_configured(&self) -> bool {
        self.state == DeviceState::Configured
    }

    /// Borrow the underlying bus (for platform-specific calls).
    pub fn bus(&self) -> &B {
        self.bus
    }

    /// Whether host remote wake-up is currently enabled.
    pub fn remote_wakeup_enabled(&self) -> bool {
        self.remote_wakeup_enabled
    }

    /// Override the self-powered flag at runtime.
    pub fn set_self_powered(&mut self, is_self_powered: bool) {
        self.self_powered = is_self_powered;
    }

    /// Force a USB disconnect + re-enumeration (not supported on all buses).
    pub fn force_reset(&mut self) -> UsbResult<()> {
        self.bus.force_reset()
    }

    /// Poll the bus for events and dispatch them to `classes`.
    ///
    /// Returns `true` if any class may have pending data to read or write.
    ///
    /// **Call this at least once every 10 ms while connected.**
    pub fn poll(&mut self, classes: &mut Classes<'_, B>) -> bool {
        let pr = self.bus.poll();

        // Handle suspend/resume first, even before other events.
        if self.state == DeviceState::Suspend {
            match pr {
                PollResult::Suspend | PollResult::None => return false,
                _ => {
                    self.bus.resume();
                    self.state = self.suspended_state.take().unwrap_or(DeviceState::Default);
                }
            }
        }

        match pr {
            PollResult::None => {}
            PollResult::Reset => self.handle_reset(classes),
            PollResult::Suspend => {
                self.bus.suspend();
                self.suspended_state = Some(self.state);
                self.state = DeviceState::Suspend;
            }
            PollResult::Resume => {}
            PollResult::Data {
                ep_out,
                ep_in_complete,
                ep_setup,
            } => {
                return self.handle_data(classes, ep_out, ep_in_complete, ep_setup);
            }
        }

        false
    }

    // ── Event handlers ────────────────────────────────────────────────────────

    fn handle_reset(&mut self, classes: &mut Classes<'_, B>) {
        self.state = DeviceState::Default;
        self.pending_address = 0;
        self.remote_wakeup_enabled = false;
        self.control.reset();
        self.bus.reset();
        for cls in classes.iter_mut() {
            cls.reset();
        }
        #[cfg(feature = "defmt")]
        crate::log::info!("USB: reset");
    }

    fn handle_data(
        &mut self,
        classes: &mut Classes<'_, B>,
        ep_out: u16,
        ep_in_complete: u16,
        ep_setup: u16,
    ) -> bool {
        let mut active = false;

        // ── EP0 ─────────────────────────────────────────────────────────────
        // Priority: SETUP > OUT data > IN-complete (matches USB spec timing).
        if (ep_setup | ep_out | ep_in_complete) & 1 != 0 {
            if ep_setup & 1 != 0 {
                if let Some(req) = self.control.handle_setup(self.bus) {
                    self.dispatch_control(classes, req);
                }
            } else if ep_out & 1 != 0 {
                match self.control.handle_out(self.bus) {
                    Ok(Some(req)) => self.dispatch_control(classes, req),
                    Ok(None) => {}
                    Err(_e) => {
                        #[cfg(feature = "defmt")]
                        crate::log::warn!("EP0 OUT error: {:?}", _e);
                    }
                }
            } else if ep_in_complete & 1 != 0 {
                match self.control.handle_in_complete(self.bus) {
                    Ok(true) => {
                        // Status stage complete.
                        if !B::SET_ADDRESS_BEFORE_STATUS && self.pending_address != 0 {
                            self.bus.set_device_address(self.pending_address);
                            self.pending_address = 0;
                            self.state = DeviceState::Addressed;
                        }
                    }
                    Ok(false) => {}
                    Err(_e) => {
                        #[cfg(feature = "defmt")]
                        crate::log::warn!("EP0 IN-complete error: {:?}", _e);
                    }
                }
            }
            active = true;
        }

        // ── EP1–7 ────────────────────────────────────────────────────────────
        for ep_num in 1usize..crate::MAX_ENDPOINTS {
            let bit = 1u16 << ep_num;
            if ep_setup & bit != 0 {
                let addr = EndpointAddress::out(ep_num as u8);
                for cls in classes.iter_mut() {
                    cls.endpoint_setup(addr);
                }
                active = true;
            }
            if ep_out & bit != 0 {
                let addr = EndpointAddress::out(ep_num as u8);
                for cls in classes.iter_mut() {
                    cls.endpoint_out(addr);
                }
                active = true;
            }
            if ep_in_complete & bit != 0 {
                let addr = EndpointAddress::in_(ep_num as u8);
                for cls in classes.iter_mut() {
                    cls.endpoint_in_complete(addr);
                }
                active = true;
            }
        }

        // Give classes an unconditional poll on every data event.
        for cls in classes.iter_mut() {
            cls.poll();
        }

        active
    }

    // ── Control transfer dispatch ─────────────────────────────────────────────

    fn dispatch_control(&mut self, classes: &mut Classes<'_, B>, req: Request) {
        if req.direction == Direction::In {
            self.control_in(classes, req);
        } else {
            self.control_out(classes, req);
        }
    }

    fn control_in(&mut self, classes: &mut Classes<'_, B>, req: Request) {
        // Let classes have first go.
        for cls in classes.iter_mut() {
            cls.control_in(ControlIn::new(&mut self.control, self.bus, &req));
            if !self.control.waiting_for_response() {
                return;
            }
        }

        if req.request_type != RequestType::Standard {
            if self.control.waiting_for_response() {
                let _ = self.control.reject(self.bus);
            }
            return;
        }

        // Standard IN requests.
        let xfer = ControlIn::new(&mut self.control, self.bus, &req);
        match (req.recipient, req.request) {
            (Recipient::Device, StandardRequest::GET_STATUS) => {
                let status: u16 = if self.self_powered { 0x0001 } else { 0x0000 }
                    | if self.remote_wakeup_enabled {
                        0x0002
                    } else {
                        0x0000
                    };
                let _ = xfer.accept_with(&status.to_le_bytes());
            }
            (Recipient::Interface, StandardRequest::GET_STATUS) => {
                let _ = xfer.accept_with(&0u16.to_le_bytes());
            }
            (Recipient::Endpoint, StandardRequest::GET_STATUS) => {
                let ep = req.endpoint_address();
                let stalled = self.bus.is_stalled(ep);
                let status: u16 = if stalled { 0x0001 } else { 0x0000 };
                let _ = xfer.accept_with(&status.to_le_bytes());
            }
            (Recipient::Device, StandardRequest::GET_DESCRIPTOR) => {
                self.handle_get_descriptor(classes, req);
            }
            (Recipient::Device, StandardRequest::GET_CONFIGURATION) => {
                let val: u8 = match self.state {
                    DeviceState::Configured => CONFIGURATION_VALUE,
                    _ => CONFIGURATION_NONE,
                };
                let xfer = ControlIn::new(&mut self.control, self.bus, &req);
                let _ = xfer.accept_with(&[val]);
            }
            (Recipient::Interface, StandardRequest::GET_INTERFACE) => {
                if req.index > u8::MAX as u16 {
                    let _ = ControlIn::new(&mut self.control, self.bus, &req).reject();
                    return;
                }
                let iface = InterfaceNumber(req.index as u8);
                for cls in classes.iter_mut() {
                    if let Some(alt) = cls.alt_setting(iface) {
                        let xfer = ControlIn::new(&mut self.control, self.bus, &req);
                        let _ = xfer.accept_with(&[alt]);
                        return;
                    }
                }
                let xfer = ControlIn::new(&mut self.control, self.bus, &req);
                let _ = xfer.accept_with(&[DEFAULT_ALTERNATE_SETTING]);
            }
            _ => {}
        }

        if self.control.waiting_for_response() {
            let _ = self.control.reject(self.bus);
        }
    }

    fn control_out(&mut self, classes: &mut Classes<'_, B>, req: Request) {
        // Let classes have first go.
        for cls in classes.iter_mut() {
            cls.control_out(ControlOut::new(&mut self.control, self.bus, &req));
            if !self.control.waiting_for_response() {
                return;
            }
        }

        if req.request_type != RequestType::Standard {
            if self.control.waiting_for_response() {
                let _ = self.control.reject(self.bus);
            }
            return;
        }

        // Standard OUT requests.
        let xfer = ControlOut::new(&mut self.control, self.bus, &req);
        match (req.recipient, req.request, req.value) {
            (Recipient::Device, StandardRequest::CLEAR_FEATURE, Feature::DEVICE_REMOTE_WAKEUP) => {
                self.remote_wakeup_enabled = false;
                let _ = xfer.accept();
            }
            (Recipient::Endpoint, StandardRequest::CLEAR_FEATURE, Feature::ENDPOINT_HALT) => {
                let ep = req.endpoint_address();
                self.bus.set_stalled(ep, false);
                let _ = xfer.accept();
            }
            (Recipient::Device, StandardRequest::SET_FEATURE, Feature::DEVICE_REMOTE_WAKEUP) => {
                self.remote_wakeup_enabled = true;
                let _ = xfer.accept();
            }
            (Recipient::Endpoint, StandardRequest::SET_FEATURE, Feature::ENDPOINT_HALT) => {
                let ep = req.endpoint_address();
                self.bus.set_stalled(ep, true);
                let _ = xfer.accept();
            }
            (Recipient::Device, StandardRequest::SET_ADDRESS, 1..=127) => {
                if B::SET_ADDRESS_BEFORE_STATUS {
                    self.bus.set_device_address(req.value as u8);
                    self.state = DeviceState::Addressed;
                } else {
                    self.pending_address = req.value as u8;
                }
                let xfer = ControlOut::new(&mut self.control, self.bus, &req);
                let _ = xfer.accept();
                #[cfg(feature = "defmt")]
                crate::log::debug!("USB: SET_ADDRESS {}", req.value as u8);
            }
            (Recipient::Device, StandardRequest::SET_CONFIGURATION, _) => {
                let config = req.value as u8;
                match config {
                    CONFIGURATION_NONE => {
                        self.state = DeviceState::Addressed;
                        self.bus.reset();
                        for cls in classes.iter_mut() {
                            cls.deconfigure();
                        }
                    }
                    CONFIGURATION_VALUE => {
                        self.state = DeviceState::Configured;
                        for cls in classes.iter_mut() {
                            cls.configure();
                        }
                        #[cfg(feature = "defmt")]
                        crate::log::info!("USB: configured");
                    }
                    _ => {
                        let xfer = ControlOut::new(&mut self.control, self.bus, &req);
                        let _ = xfer.reject();
                        return;
                    }
                }
                let xfer = ControlOut::new(&mut self.control, self.bus, &req);
                let _ = xfer.accept();
            }
            (Recipient::Interface, StandardRequest::SET_INTERFACE, _) => {
                if req.index > u8::MAX as u16 {
                    let _ = xfer.reject();
                    return;
                }
                let iface = InterfaceNumber(req.index as u8);
                let alt = req.value as u8;
                for cls in classes.iter_mut() {
                    if cls.set_alt_setting(iface, alt) {
                        let xfer = ControlOut::new(&mut self.control, self.bus, &req);
                        let _ = xfer.accept();
                        return;
                    }
                }
                let xfer = ControlOut::new(&mut self.control, self.bus, &req);
                let _ = xfer.reject();
            }
            _ => {}
        }

        if self.control.waiting_for_response() {
            let _ = self.control.reject(self.bus);
        }
    }

    // ── Descriptor building ───────────────────────────────────────────────────

    fn handle_get_descriptor(&mut self, classes: &mut Classes<'_, B>, req: Request) {
        let (desc_type, desc_index) = req.descriptor_type_index();
        let requested_len = req.length as usize;

        // Use the control pipe buffer for all descriptors.
        let accepted = match desc_type {
            descriptor_type::DEVICE => self.build_device_descriptor_and_accept(req),
            descriptor_type::CONFIGURATION => self.build_config_descriptor_and_accept(classes, req),
            descriptor_type::STRING => {
                self.build_string_descriptor_and_accept(classes, desc_index, req, requested_len)
            }
            _ => false,
        };

        if !accepted && self.control.waiting_for_response() {
            let _ = self.control.reject(self.bus);
        }
    }

    fn build_device_descriptor_and_accept(&mut self, req: Request) -> bool {
        let info = &self.info;
        // Determine string indices.
        let i_mfr = if info.manufacturer.is_some() { 1 } else { 0 };
        let i_prod = if info.product.is_some() { 2 } else { 0 };
        let i_sn = if info.serial_number.is_some() { 3 } else { 0 };
        let desc: [u8; 18] = [
            18,                      // bLength
            descriptor_type::DEVICE, // bDescriptorType
            info.usb_rev as u8,
            (info.usb_rev >> 8) as u8, // bcdUSB
            info.device_class,
            info.device_sub_class,
            info.device_protocol,
            info.max_packet_size_0, // bMaxPacketSize0
            info.vendor_id as u8,
            (info.vendor_id >> 8) as u8, // idVendor
            info.product_id as u8,
            (info.product_id >> 8) as u8, // idProduct
            info.device_release as u8,
            (info.device_release >> 8) as u8, // bcdDevice
            i_mfr,
            i_prod,
            i_sn,
            1, // bNumConfigurations
        ];
        let xfer = ControlIn::new(&mut self.control, self.bus, &req);
        xfer.accept_with(&desc).is_ok()
    }

    fn build_config_descriptor_and_accept(
        &mut self,
        classes: &mut Classes<'_, B>,
        req: Request,
    ) -> bool {
        // Build into a temporary stack buffer large enough for typical devices.
        let mut buf = [0u8; 256];
        let mut writer = DescriptorWriter::new(&mut buf);

        let info = &self.info;
        let bm_attr = 0x80
            | if info.self_powered { 0x40 } else { 0x00 }
            | if info.supports_remote_wakeup {
                0x20
            } else {
                0x00
            };

        if writer.configuration(bm_attr, info.max_power).is_err() {
            return false;
        }
        for cls in classes.iter() {
            if cls.configuration_descriptors(&mut writer).is_err() {
                return false;
            }
        }
        writer.end_configuration();
        let len = min(writer.position(), req.length as usize);

        let xfer = ControlIn::new(&mut self.control, self.bus, &req);
        xfer.accept_with(&buf[..len]).is_ok()
    }

    fn build_string_descriptor_and_accept(
        &mut self,
        classes: &mut Classes<'_, B>,
        index: u8,
        req: Request,
        requested_len: usize,
    ) -> bool {
        let lang_id_value = req.index;
        let mut buf = [0u8; 128];

        let written = match index {
            0 => DescriptorWriter::write_lang_ids(&mut buf, &[lang_id::EN_US]).ok(),
            1 => self
                .info
                .manufacturer
                .and_then(|s| DescriptorWriter::write_string(&mut buf, s).ok()),
            2 => self
                .info
                .product
                .and_then(|s| DescriptorWriter::write_string(&mut buf, s).ok()),
            3 => self
                .info
                .serial_number
                .and_then(|s| DescriptorWriter::write_string(&mut buf, s).ok()),
            _ => {
                // Ask each class.
                let string_idx = StringIndex::new(index);
                classes
                    .iter()
                    .find_map(|cls| cls.string(string_idx, lang_id_value))
                    .and_then(|s| DescriptorWriter::write_string(&mut buf, s).ok())
            }
        };

        if let Some(total) = written {
            let len = min(total, requested_len);
            let xfer = ControlIn::new(&mut self.control, self.bus, &req);
            return xfer.accept_with(&buf[..len]).is_ok();
        }
        false
    }
}

// ─── UsbDeviceBuilder ─────────────────────────────────────────────────────────

/// Builder for [`UsbDevice`].
///
/// # Example
///
/// ```ignore
/// use my_hal_usb::{UsbBusAllocator, UsbDeviceBuilder};
/// use my_hal_usb::class::cdc_acm::CdcAcm;
/// use my_hal_usb::MyBus;
///
/// let alloc = UsbBusAllocator::new(MyBus::new(/* peripheral */));
/// let mut serial = CdcAcm::new(&alloc);
/// let mut usb_dev = UsbDeviceBuilder::new(&alloc, 0x1234, 0x5678)
///     .product("My Device")
///     .manufacturer("My Company")
///     .serial_number("001")
///     .build();
///
/// loop {
///     if usb_dev.poll(&mut [&mut serial]) { /* … */ }
/// }
/// ```
pub struct UsbDeviceBuilder<'a, B: UsbBus> {
    alloc: &'a UsbBusAllocator<B>,
    info: DeviceInfo<'a>,
}

impl<'a, B: UsbBus> UsbDeviceBuilder<'a, B> {
    /// Create a builder with the required VID/PID pair.
    pub fn new(alloc: &'a UsbBusAllocator<B>, vendor_id: u16, product_id: u16) -> Self {
        Self {
            alloc,
            info: DeviceInfo {
                vendor_id,
                product_id,
                device_class: 0x00,
                device_sub_class: 0x00,
                device_protocol: 0x00,
                max_packet_size_0: 64,
                device_release: 0x0100,
                usb_rev: 0x0200,
                manufacturer: None,
                product: None,
                serial_number: None,
                self_powered: false,
                supports_remote_wakeup: false,
                max_power: 50, // 100 mA
            },
        }
    }

    /// Build the [`UsbDevice`].  Allocates EP0 and enables the peripheral.
    pub fn build(self) -> UsbDevice<'a, B> {
        UsbDevice::build(self.alloc, self.info)
    }

    // ── Fluent setters ────────────────────────────────────────────────────────

    /// Set the device class code.
    pub fn device_class(mut self, c: u8) -> Self {
        self.info.device_class = c;
        self
    }
    /// Set the device subclass code.
    pub fn device_sub_class(mut self, c: u8) -> Self {
        self.info.device_sub_class = c;
        self
    }
    /// Set the device protocol code.
    pub fn device_protocol(mut self, c: u8) -> Self {
        self.info.device_protocol = c;
        self
    }
    /// Set the device release number (BCD).
    pub fn device_release(mut self, r: u16) -> Self {
        self.info.device_release = r;
        self
    }
    /// Set the USB specification release (BCD, e.g. `0x0200`).
    pub fn usb_rev(mut self, r: u16) -> Self {
        self.info.usb_rev = r;
        self
    }
    /// Set the EP0 maximum packet size (8, 16, 32 or 64).
    pub fn max_packet_size_0(mut self, s: u8) -> Self {
        self.info.max_packet_size_0 = s;
        self
    }
    /// Set the manufacturer string.
    pub fn manufacturer(mut self, s: &'a str) -> Self {
        self.info.manufacturer = Some(s);
        self
    }
    /// Set the product string.
    pub fn product(mut self, s: &'a str) -> Self {
        self.info.product = Some(s);
        self
    }
    /// Set the serial-number string.
    pub fn serial_number(mut self, s: &'a str) -> Self {
        self.info.serial_number = Some(s);
        self
    }
    /// Declare that the device is self-powered.
    pub fn self_powered(mut self, v: bool) -> Self {
        self.info.self_powered = v;
        self
    }
    /// Declare remote-wakeup support.
    pub fn supports_remote_wakeup(mut self, v: bool) -> Self {
        self.info.supports_remote_wakeup = v;
        self
    }
    /// Set maximum power in mA (stored as mA/2 per USB spec).
    pub fn max_power_ma(mut self, ma: u16) -> Self {
        self.info.max_power = (ma / 2).min(250) as u8;
        self
    }

    /// Configure as a composite device with Interface Association Descriptors.
    pub fn composite_with_iads(mut self) -> Self {
        self.info.device_class = 0xEF;
        self.info.device_sub_class = 0x02;
        self.info.device_protocol = 0x01;
        self
    }
}
