//! [`HidDevice`] — what a HID function has to provide.

use crate::device::bus::UsbBus;
use crate::device::class::hid::{hid_protocol, hid_subclass};
use crate::device::endpoint::EndpointIn;

// ─── Report type ─────────────────────────────────────────────────────────────

/// Kind of report, as selected by the `wValue` high byte of `Get Report` and
/// `Set Report`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReportType {
    /// Input report: device → host.
    Input,
    /// Output report: host → device.
    Output,
    /// Feature report: readable and writable configuration.
    Feature,
}

impl ReportType {
    /// Decode the `wValue` high byte of a `Get Report` / `Set Report` request.
    pub(crate) fn from_w_value(value: u16) -> Option<Self> {
        match (value >> 8) as u8 {
            1 => Some(Self::Input),
            2 => Some(Self::Output),
            3 => Some(Self::Feature),
            _ => None,
        }
    }
}

// ─── HidSpec ─────────────────────────────────────────────────────────────────

/// Static description of a HID function.
///
/// Everything a [`HidClass`](super::HidClass) needs to allocate its endpoints
/// and write its descriptors, plus the two facts about the reports that cannot
/// be derived from the report descriptor itself (its size and whether it
/// declares report IDs).
///
/// ```ignore
/// use my_hal_usb::class::hid::{HidSpec, hid_protocol, hid_subclass};
///
/// static MY_DESCRIPTOR: &[u8] = &[0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0xC0];
///
/// const SPEC: HidSpec = HidSpec::new(MY_DESCRIPTOR, 8)
///     .subclass(hid_subclass::BOOT)
///     .protocol(hid_protocol::KEYBOARD)
///     .out_report_size(1)
///     .poll_interval_ms(8);
/// ```
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct HidSpec {
    /// Report descriptor, served for `Get Descriptor (HID Report)`.
    pub report_descriptor: &'static [u8],
    /// Size of the interrupt IN endpoint — at least the largest input report,
    /// including its report ID byte when report IDs are used.
    pub in_report_size: u16,
    /// Size of the interrupt OUT endpoint, or `0` for a function without one.
    pub out_report_size: u16,
    /// HID subclass: `0` (none) or [`hid_subclass::BOOT`].
    pub subclass: u8,
    /// HID protocol: `0` (none) or a boot protocol.
    pub protocol: u8,
    /// Interrupt endpoint polling interval, in milliseconds.
    pub poll_interval_ms: u8,
    /// `true` when the report descriptor declares report IDs.
    ///
    /// Output reports on the interrupt OUT endpoint then carry the report ID as
    /// their first byte (HID 1.11 §7.2.2), whereas `Set Report` passes it in
    /// `wValue` and leaves it out of the data.
    pub report_ids: bool,
}

impl HidSpec {
    /// A spec with no subclass, no protocol, no OUT endpoint, a 10 ms poll
    /// interval and no report IDs.
    pub const fn new(report_descriptor: &'static [u8], in_report_size: u16) -> Self {
        Self {
            report_descriptor,
            in_report_size,
            out_report_size: 0,
            subclass: hid_subclass::NONE,
            protocol: hid_protocol::NONE,
            poll_interval_ms: 10,
            report_ids: false,
        }
    }

    /// Set the HID subclass (and, for boot devices, the protocol).
    pub const fn subclass(mut self, subclass: u8) -> Self {
        self.subclass = subclass;
        self
    }

    /// Set the HID protocol reported in the interface descriptor.
    pub const fn protocol(mut self, protocol: u8) -> Self {
        self.protocol = protocol;
        self
    }

    /// Allocate an interrupt OUT endpoint of `size` bytes.
    pub const fn out_report_size(mut self, size: u16) -> Self {
        self.out_report_size = size;
        self
    }

    /// Set the interrupt endpoint polling interval, in milliseconds.
    pub const fn poll_interval_ms(mut self, interval: u8) -> Self {
        self.poll_interval_ms = interval;
        self
    }

    /// Declare that the report descriptor uses report IDs.
    pub const fn report_ids(mut self, report_ids: bool) -> Self {
        self.report_ids = report_ids;
        self
    }
}

// ─── HidDevice ───────────────────────────────────────────────────────────────

/// A HID function: a report descriptor plus the reports it exchanges with the
/// host.
///
/// Implement it to build a custom HID device, or use one of the ready-made ones:
/// [`Keyboard`](super::keyboard::Keyboard), [`Mouse`](super::mouse::Mouse),
/// [`CustomHid`](super::custom::CustomHid).
///
/// Wrap the implementation in a [`HidClass`](super::HidClass), which owns the
/// endpoints and handles the USB side:
///
/// ```ignore
/// let mut hid = HidClass::new(&alloc, MyHidDevice::new());
/// loop {
///     usb_dev.poll(&mut [&mut hid]);
///     hid.device_mut().update(...);
/// }
/// ```
///
/// All methods except [`spec`](Self::spec) have default implementations, so a
/// device only writes what it actually supports.
pub trait HidDevice<B: UsbBus> {
    // ── Description ───────────────────────────────────────────────────────────

    /// Report descriptor, endpoint sizes and interface class/protocol.
    fn spec(&self) -> HidSpec;

    // ── Control requests ──────────────────────────────────────────────────────

    /// `Get Report` — copy the report selected by `report_type` and `report_id`
    /// into `buf` and return its length, including the report ID byte when the
    /// spec declares report IDs.
    ///
    /// Return `None` to stall the request.
    fn get_report(
        &mut self,
        _report_type: ReportType,
        _report_id: u8,
        _buf: &mut [u8],
    ) -> Option<usize> {
        None
    }

    /// `Set Report` — handle a report written by the host.
    ///
    /// `data` starts at the first report byte; the report ID is passed in
    /// `report_id`, not included in `data` (HID 1.11 §7.2.1).  Return `false` to
    /// stall the request.
    fn set_report(&mut self, _report_type: ReportType, _report_id: u8, _data: &[u8]) -> bool {
        false
    }

    /// Output report received on the interrupt OUT endpoint.
    ///
    /// The class splits the packet for you: with `report_ids` set, the first
    /// byte is passed as `report_id` and `data` holds the rest; otherwise
    /// `report_id` is `0` and `data` is the whole packet.
    fn output_report(&mut self, _report_id: u8, _data: &[u8]) {}

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    /// Called after a USB bus reset.
    fn reset(&mut self) {}

    /// Called when the host configures the device.
    ///
    /// Devices that push input reports should mark everything as pending here so
    /// the host gets a snapshot of the current state.
    fn configure(&mut self) {}

    /// Called when the host de-configures the device.
    fn deconfigure(&mut self) {}

    /// Called on every device poll while configured.
    ///
    /// Push input reports with `input`; a report that cannot be sent yet
    /// ([`WouldBlock`](crate::device::error::UsbError::WouldBlock)) stays the device's
    /// responsibility to retry on the next call.
    fn poll(&mut self, _input: &EndpointIn<'_, B>) {}
}
