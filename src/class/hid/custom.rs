//! HID function with a caller-supplied report descriptor.

use crate::bus::UsbBus;
use crate::class::hid::{HidDevice, HidSpec, ReportType};

/// Size of the buffer holding the last report written by the host.
const REPORT_BUFFER_SIZE: usize = 64;

/// A HID function defined entirely by its report descriptor.
///
/// Input reports are sent as-is with
/// [`HidClass::send_input`](super::HidClass::send_input).  Output and feature
/// reports written by the host are buffered — in the same layout the device
/// sends (`report_id` first when the spec uses report IDs) — and read with
/// [`read_report`](Self::read_report).  `Get Report` echoes the last buffered
/// report back.
///
/// ```ignore
/// static DESCRIPTOR: &[u8] = &[/* … */];
///
/// let spec = HidSpec::new(DESCRIPTOR, 8).out_report_size(8).report_ids(true);
/// let mut hid = HidClass::new(&alloc, CustomHid::new(spec));
///
/// loop {
///     usb_dev.poll(&mut [&mut hid]);
///     let mut buf = [0u8; 8];
///     if let Some(len) = hid.device_mut().read_report(&mut buf) {
///         // host wrote a report
///     }
/// }
/// ```
pub struct CustomHid {
    spec: HidSpec,
    report: [u8; REPORT_BUFFER_SIZE],
    report_len: usize,
    report_pending: bool,
}

impl CustomHid {
    /// Create a function from a report descriptor and its endpoint geometry.
    ///
    /// Set [`HidSpec::out_report_size`] for a device that receives output or
    /// feature reports.
    pub const fn new(spec: HidSpec) -> Self {
        Self {
            spec,
            report: [0; REPORT_BUFFER_SIZE],
            report_len: 0,
            report_pending: false,
        }
    }

    /// Read the last report written by the host and return its length.
    ///
    /// `None` when nothing has arrived since the previous call.
    pub fn read_report(&mut self, buf: &mut [u8]) -> Option<usize> {
        if !self.report_pending {
            return None;
        }
        self.report_pending = false;
        let len = self.report_len.min(buf.len());
        buf[..len].copy_from_slice(&self.report[..len]);
        Some(len)
    }

    /// Buffer a report, prefixing the report ID when the spec uses them.
    fn store(&mut self, report_id: u8, data: &[u8]) {
        let mut len = 0;
        if self.spec.report_ids {
            self.report[0] = report_id;
            len = 1;
        }
        let data_len = data.len().min(REPORT_BUFFER_SIZE - len);
        self.report[len..len + data_len].copy_from_slice(&data[..data_len]);
        self.report_len = len + data_len;
        self.report_pending = true;
    }
}

impl<B: UsbBus> HidDevice<B> for CustomHid {
    fn spec(&self) -> HidSpec {
        self.spec
    }

    fn get_report(
        &mut self,
        _report_type: ReportType,
        _report_id: u8,
        buf: &mut [u8],
    ) -> Option<usize> {
        // Nothing meaningful to report before the host has written something.
        if self.report_len == 0 {
            return None;
        }
        let len = self.report_len.min(buf.len());
        buf[..len].copy_from_slice(&self.report[..len]);
        Some(len)
    }

    fn set_report(&mut self, report_type: ReportType, report_id: u8, data: &[u8]) -> bool {
        // Input reports flow the other way.
        if report_type == ReportType::Input {
            return false;
        }
        self.store(report_id, data);
        true
    }

    fn output_report(&mut self, report_id: u8, data: &[u8]) {
        self.store(report_id, data);
    }

    fn reset(&mut self) {
        self.report_len = 0;
        self.report_pending = false;
    }
}
