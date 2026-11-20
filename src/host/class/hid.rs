//! Boot-protocol keyboards.
//!
//! The HID boot protocol is what a BIOS speaks: an eight-byte report holding
//! the modifier bitmap and up to six concurrently pressed keys. Devices that
//! implement it are the ones that also work before an operating system has
//! loaded a driver, which makes it the cheapest way to prove a host stack works.

use crate::descriptor::class_code;
use crate::endpoint::{Direction, EndpointAddress, EndpointType};
use crate::host::controller::HostController;
use crate::host::device::Device;
use crate::host::error::{HostError, HostResult};
use crate::host::pipe::Pipe;
use crate::request::{Recipient, Request, RequestType};

/// Bytes in a boot-protocol keyboard report.
pub const REPORT_LEN: usize = 8;

/// `SET_PROTOCOL` with the boot protocol.
const HID_SET_PROTOCOL: u8 = 0x0b;
/// `SET_IDLE` with an indefinite duration.
const HID_SET_IDLE: u8 = 0x0a;

/// Modifier bits of a keyboard report.
#[allow(missing_docs)]
pub mod modifier {
    pub const LEFT_CTRL: u8 = 1 << 0;
    pub const LEFT_SHIFT: u8 = 1 << 1;
    pub const LEFT_ALT: u8 = 1 << 2;
    pub const LEFT_GUI: u8 = 1 << 3;
    pub const RIGHT_CTRL: u8 = 1 << 4;
    pub const RIGHT_SHIFT: u8 = 1 << 5;
    pub const RIGHT_ALT: u8 = 1 << 6;
    pub const RIGHT_GUI: u8 = 1 << 7;
}

/// One boot-protocol keyboard report.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Report {
    /// Modifier bitmap, see [`modifier`].
    pub modifiers: u8,
    /// Usage codes of the pressed keys, zero for unused slots.
    pub keys: [u8; 6],
}

impl Report {
    /// Parse the bytes of a report; short reports are padded with zeroes.
    pub fn parse(bytes: &[u8]) -> Self {
        let mut report = Self::default();
        if let Some((&modifiers, keys)) = bytes.split_first() {
            report.modifiers = modifiers;
            let count = keys.len().min(report.keys.len());
            report.keys[..count].copy_from_slice(&keys[..count]);
        }
        report
    }

    /// No key and no modifier is held.
    pub fn is_idle(&self) -> bool {
        self.modifiers == 0 && self.keys.iter().all(|&key| key == 0)
    }

    /// Whether `key` is held down.
    pub fn is_pressed(&self, key: u8) -> bool {
        self.keys.contains(&key)
    }

    /// Whether any of the bits in `mask` of [`modifier`] is set.
    pub fn has_modifier(&self, mask: u8) -> bool {
        self.modifiers & mask != 0
    }

    /// The keys this report holds that `previous` did not.
    pub fn new_keys<'a>(&'a self, previous: &'a Self) -> impl Iterator<Item = u8> + 'a {
        self.keys
            .iter()
            .copied()
            .filter(move |key| *key != 0 && !previous.keys.contains(key))
    }
}

/// A keyboard driven through its boot-protocol interrupt endpoint.
pub struct Keyboard<'d, C: HostController> {
    pipe: Pipe<'d, C>,
}

impl<'d, C: HostController> Keyboard<'d, C> {
    /// Find the boot keyboard interface of `device`, put it in boot protocol
    /// mode and open its interrupt IN endpoint.
    ///
    /// The device has to be configured already
    /// ([`Device::set_configuration`]), because the interface is only live
    /// afterwards.
    pub fn open(device: &Device<'d, C>) -> HostResult<Self> {
        let endpoint = {
            let configuration = device.configuration();
            // Class 3, subclass 1, protocol 1 is a boot keyboard.
            let interface = configuration
                .find_interface(class_code::HID, 1, 1)
                .ok_or(HostError::Unsupported)?;

            let interface_number = interface.number() as u16;
            let class_request = |request, value| {
                Request::custom(
                    Direction::Out,
                    RequestType::Class,
                    Recipient::Interface,
                    request,
                    value,
                    interface_number,
                    0,
                )
            };

            // Boot protocol, and reports only when they change: the simplest
            // configuration for a host that polls on demand.
            device.control(&class_request(HID_SET_PROTOCOL, 0), &mut [])?;
            device.control(&class_request(HID_SET_IDLE, 0), &mut [])?;

            interface
                .endpoints()
                .find(|endpoint| {
                    endpoint.address.is_in()
                        && matches!(endpoint.transfer_type(), EndpointType::Interrupt)
                })
                .ok_or(HostError::Malformed)?
        };

        Ok(Self {
            pipe: device.open(&endpoint)?,
        })
    }

    /// Wait for the next report.
    ///
    /// The endpoint NAKs until something changed, so this blocks for up to the
    /// NAK retry budget of the controller.
    pub fn read(&self) -> HostResult<Report> {
        let mut buffer = [0u8; REPORT_LEN];
        let length = self.pipe.read(&mut buffer)?;
        Ok(Report::parse(&buffer[..length]))
    }

    /// The interrupt IN endpoint the keyboard is read from.
    #[inline]
    pub fn endpoint(&self) -> EndpointAddress {
        self.pipe.address()
    }
}
