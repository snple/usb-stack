//! Boot-compatible mouse.

use crate::bus::UsbBus;
use crate::class::hid::{HidDevice, HidSpec, hid_protocol, hid_subclass};

/// Boot mouse report descriptor: 3 buttons, X/Y movement and a wheel, all
/// relative, in a 4-byte input report.
pub const MOUSE_REPORT_DESCRIPTOR: &[u8] = &[
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x02, // Usage (Mouse)
    0xA1, 0x01, // Collection (Application)
    0x09, 0x01, //   Usage (Pointer)
    0xA1, 0x00, //   Collection (Physical)
    // Buttons (3 bits + 5 padding)
    0x05, 0x09, //     Usage Page (Buttons)
    0x19, 0x01, //     Usage Minimum (Button 1)
    0x29, 0x03, //     Usage Maximum (Button 3)
    0x15, 0x00, //     Logical Minimum (0)
    0x25, 0x01, //     Logical Maximum (1)
    0x75, 0x01, //     Report Size (1)
    0x95, 0x03, //     Report Count (3)
    0x81, 0x02, //     Input (Data, Variable, Absolute)
    0x75, 0x05, //     Report Size (5) – padding
    0x95, 0x01, //     Report Count (1)
    0x81, 0x01, //     Input (Constant)
    // X, Y relative movement
    0x05, 0x01, //     Usage Page (Generic Desktop)
    0x09, 0x30, //     Usage (X)
    0x09, 0x31, //     Usage (Y)
    0x15, 0x81, //     Logical Minimum (-127)
    0x25, 0x7F, //     Logical Maximum (127)
    0x75, 0x08, //     Report Size (8)
    0x95, 0x02, //     Report Count (2)
    0x81, 0x06, //     Input (Data, Variable, Relative)
    // Wheel
    0x09, 0x38, //     Usage (Wheel)
    0x15, 0x81, //     Logical Minimum (-127)
    0x25, 0x7F, //     Logical Maximum (127)
    0x75, 0x08, //     Report Size (8)
    0x95, 0x01, //     Report Count (1)
    0x81, 0x06, //     Input (Data, Variable, Relative)
    0xC0, //   End Collection
    0xC0, // End Collection
];

/// Boot mouse input report: buttons, relative movement and wheel.
#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MouseReport {
    /// Button bits, see [`MouseReport::BUTTON_*`](MouseReport::BUTTON_LEFT).
    pub buttons: u8,
    /// Horizontal movement, -127..=127.
    pub x: i8,
    /// Vertical movement, -127..=127.
    pub y: i8,
    /// Wheel movement, -127..=127.
    pub wheel: i8,
}

impl MouseReport {
    /// Left button bit in [`buttons`](Self::buttons).
    pub const BUTTON_LEFT: u8 = 0x01;
    /// Right button bit in [`buttons`](Self::buttons).
    pub const BUTTON_RIGHT: u8 = 0x02;
    /// Middle button bit in [`buttons`](Self::buttons).
    pub const BUTTON_MIDDLE: u8 = 0x04;

    /// Report the state as a 4-byte boot report.
    pub fn to_bytes(self) -> [u8; 4] {
        [self.buttons, self.x as u8, self.y as u8, self.wheel as u8]
    }
}

/// Boot-compatible mouse: send a report for every movement or button change, and
/// send [`MouseReport::default`] once the movement has been reported.
pub struct Mouse;

impl Mouse {
    /// Create a boot mouse.
    pub const fn new() -> Self {
        Self
    }
}

impl Default for Mouse {
    fn default() -> Self {
        Self::new()
    }
}

impl<B: UsbBus> HidDevice<B> for Mouse {
    fn spec(&self) -> HidSpec {
        HidSpec::new(MOUSE_REPORT_DESCRIPTOR, 4)
            .subclass(hid_subclass::BOOT)
            .protocol(hid_protocol::MOUSE)
            .poll_interval_ms(8)
    }
}
