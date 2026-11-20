//! Boot-compatible keyboard.

use crate::device::bus::UsbBus;
use crate::device::class::hid::{HidDevice, HidSpec, hid_protocol, hid_subclass};

/// Boot keyboard report descriptor: 8-byte input report, 1-byte LED output report.
pub const KEYBOARD_REPORT_DESCRIPTOR: &[u8] = &[
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x06, // Usage (Keyboard)
    0xA1, 0x01, // Collection (Application)
    // Modifier keys (8 bits)
    0x05, 0x07, //   Usage Page (Keyboard/Keypad)
    0x19, 0xE0, //   Usage Minimum (Left Control)
    0x29, 0xE7, //   Usage Maximum (Right GUI)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //   Logical Maximum (1)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x08, //   Report Count (8)
    0x81, 0x02, //   Input (Data, Variable, Absolute)
    // Reserved byte
    0x75, 0x08, //   Report Size (8)
    0x95, 0x01, //   Report Count (1)
    0x81, 0x01, //   Input (Constant)
    // LED output (5 bits + 3 padding)
    0x05, 0x08, //   Usage Page (LEDs)
    0x19, 0x01, //   Usage Minimum (Num Lock)
    0x29, 0x05, //   Usage Maximum (Kana)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x05, //   Report Count (5)
    0x91, 0x02, //   Output (Data, Variable, Absolute)
    0x75, 0x03, //   Report Size (3)
    0x95, 0x01, //   Report Count (1)
    0x91, 0x01, //   Output (Constant)
    // Key array (6 keys)
    0x05, 0x07, //   Usage Page (Keyboard/Keypad)
    0x19, 0x00, //   Usage Minimum (0)
    0x29, 0xFF, //   Usage Maximum (255)
    0x15, 0x00, //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08, //   Report Size (8)
    0x95, 0x06, //   Report Count (6)
    0x81, 0x00, //   Input (Data, Array)
    0xC0, // End Collection
];

/// Modifier bits for [`KeyboardReport::modifiers`].
pub mod modifier {
    /// Left Control.
    pub const LEFT_CTRL: u8 = 1 << 0;
    /// Left Shift.
    pub const LEFT_SHIFT: u8 = 1 << 1;
    /// Left Alt.
    pub const LEFT_ALT: u8 = 1 << 2;
    /// Left GUI (Windows/Super).
    pub const LEFT_GUI: u8 = 1 << 3;
    /// Right Control.
    pub const RIGHT_CTRL: u8 = 1 << 4;
    /// Right Shift.
    pub const RIGHT_SHIFT: u8 = 1 << 5;
    /// Right Alt.
    pub const RIGHT_ALT: u8 = 1 << 6;
    /// Right GUI (Windows/Super).
    pub const RIGHT_GUI: u8 = 1 << 7;
}

/// LED bits reported back by the host (the output report).
pub mod led {
    /// Num Lock is on.
    pub const NUM_LOCK: u8 = 1 << 0;
    /// Caps Lock is on.
    pub const CAPS_LOCK: u8 = 1 << 1;
    /// Scroll Lock is on.
    pub const SCROLL_LOCK: u8 = 1 << 2;
    /// Compose is active.
    pub const COMPOSE: u8 = 1 << 3;
    /// Kana is active.
    pub const KANA: u8 = 1 << 4;
}

/// Boot keyboard input report.
///
/// A report with `keys` all zero and no modifiers means "all keys released".
#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KeyboardReport {
    /// Modifier bits, see [`modifier`].
    pub modifiers: u8,
    /// Up to six simultaneous keycodes; a zero entry means "no key".
    pub keys: [u8; 6],
}

impl KeyboardReport {
    /// Report the keys as an 8-byte boot report.
    pub fn to_bytes(self) -> [u8; 8] {
        [
            self.modifiers,
            0,
            self.keys[0],
            self.keys[1],
            self.keys[2],
            self.keys[3],
            self.keys[4],
            self.keys[5],
        ]
    }
}

/// Boot-compatible keyboard.
///
/// Pressing a key means sending a report with that key set, then sending
/// [`KeyboardReport::default`] (or the next set of keys) to release it.  Wrap it
/// in a [`HidClass`](super::HidClass) to talk to the host:
///
/// ```ignore
/// let mut kbd = HidClass::new(&alloc, Keyboard::new());
/// kbd.send_input(&KeyboardReport { keys: [0x04, 0, 0, 0, 0, 0], ..Default::default() }.to_bytes());
/// ```
pub struct Keyboard {
    leds: u8,
}

impl Keyboard {
    /// Create a keyboard with all LEDs off.
    pub const fn new() -> Self {
        Self { leds: 0 }
    }

    /// LED state the host last asked for, see [`led`].
    pub fn leds(&self) -> u8 {
        self.leds
    }
}

impl Default for Keyboard {
    fn default() -> Self {
        Self::new()
    }
}

impl<B: UsbBus> HidDevice<B> for Keyboard {
    fn spec(&self) -> HidSpec {
        HidSpec::new(KEYBOARD_REPORT_DESCRIPTOR, 8)
            .subclass(hid_subclass::BOOT)
            .protocol(hid_protocol::KEYBOARD)
            .out_report_size(1) // LED report
            .poll_interval_ms(10)
    }

    fn output_report(&mut self, _report_id: u8, data: &[u8]) {
        self.leds = data.first().copied().unwrap_or(0);
    }

    fn reset(&mut self) {
        self.leds = 0;
    }
}
