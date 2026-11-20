//! Parsing of the descriptors a device returns.
//!
//! Descriptors arrive as a flat byte string: the configuration descriptor is
//! followed by its interface and endpoint descriptors, and class-specific
//! descriptors may appear in between. Rather than copying them into a
//! heap-allocated tree, [`Configuration`] keeps the bytes and lends out views
//! over the parts that matter.

use crate::descriptor::descriptor_type;
use crate::endpoint::{EndpointAddress, EndpointType, IsoSynchronization, IsoUsage};
use crate::host::error::{HostError, HostResult};

#[inline]
const fn le16(bytes: &[u8], offset: usize) -> u16 {
    (bytes[offset] as u16) | ((bytes[offset + 1] as u16) << 8)
}

// ─── Device descriptor ───────────────────────────────────────────────────────

/// The standard device descriptor (USB 2.0 §9.6.1).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceDescriptor {
    /// `bcdUSB`.
    pub usb_version: u16,
    /// `bDeviceClass`.
    pub class: u8,
    /// `bDeviceSubClass`.
    pub sub_class: u8,
    /// `bDeviceProtocol`.
    pub protocol: u8,
    /// `bMaxPacketSize0`, the control endpoint packet size.
    pub max_packet0: u16,
    /// `idVendor`.
    pub vendor_id: u16,
    /// `idProduct`.
    pub product_id: u16,
    /// `bcdDevice`.
    pub device_version: u16,
    /// Manufacturer string index, zero when absent.
    pub manufacturer: u8,
    /// Product string index, zero when absent.
    pub product: u8,
    /// Serial number string index, zero when absent.
    pub serial_number: u8,
    /// `bNumConfigurations`.
    pub configurations: u8,
}

impl DeviceDescriptor {
    /// Length of the descriptor on the wire.
    pub const LEN: usize = 18;

    /// Parse the 18 bytes the device returned.
    pub fn parse(bytes: &[u8]) -> HostResult<Self> {
        if bytes.len() < Self::LEN || bytes[0] < Self::LEN as u8 {
            return Err(HostError::Malformed);
        }
        if bytes[1] != descriptor_type::DEVICE {
            return Err(HostError::Malformed);
        }
        Ok(Self {
            usb_version: le16(bytes, 2),
            class: bytes[4],
            sub_class: bytes[5],
            protocol: bytes[6],
            max_packet0: bytes[7] as u16,
            vendor_id: le16(bytes, 8),
            product_id: le16(bytes, 10),
            device_version: le16(bytes, 12),
            manufacturer: bytes[14],
            product: bytes[15],
            serial_number: bytes[16],
            configurations: bytes[17],
        })
    }
}

// ─── Raw descriptors ─────────────────────────────────────────────────────────

/// One descriptor of a configuration, header included.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Descriptor<'a> {
    /// `bDescriptorType`.
    pub kind: u8,
    /// The whole descriptor, starting with its length and type bytes.
    pub bytes: &'a [u8],
}

/// Iterates over the descriptors stored back to back in a configuration.
#[derive(Clone, Copy, Debug)]
pub struct Descriptors<'a> {
    rest: &'a [u8],
}

impl<'a> Iterator for Descriptors<'a> {
    type Item = Descriptor<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let length = *self.rest.first()? as usize;
        if length < 2 || length > self.rest.len() {
            // A truncated or zero-length descriptor: there is no sensible way
            // to continue, so report the end of the list.
            self.rest = &[];
            return None;
        }
        let (head, tail) = self.rest.split_at(length);
        self.rest = tail;
        Some(Descriptor {
            kind: head[1],
            bytes: head,
        })
    }
}

// ─── Configuration ───────────────────────────────────────────────────────────

/// A configuration descriptor together with everything that follows it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Configuration<'a> {
    raw: &'a [u8],
}

impl<'a> Configuration<'a> {
    /// Length of the configuration descriptor itself.
    pub const HEADER_LEN: usize = 9;

    /// Validate the bytes returned for `GET_DESCRIPTOR(CONFIGURATION)`.
    pub fn parse(raw: &'a [u8]) -> HostResult<Self> {
        if raw.len() < Self::HEADER_LEN || raw[1] != descriptor_type::CONFIGURATION {
            return Err(HostError::Malformed);
        }
        let total = le16(raw, 2) as usize;
        if total < Self::HEADER_LEN || total > raw.len() {
            return Err(HostError::Malformed);
        }
        Ok(Self { raw: &raw[..total] })
    }

    /// Wrap bytes that were validated earlier: a previous [`parse`](Self::parse)
    /// accepted them and nothing has changed since.
    pub(crate) fn validated(raw: &'a [u8]) -> Self {
        Self { raw }
    }

    /// The `wTotalLength` bytes of the configuration.
    #[inline]
    pub const fn bytes(&self) -> &'a [u8] {
        self.raw
    }

    /// `wTotalLength`.
    #[inline]
    pub fn total_length(&self) -> u16 {
        le16(self.raw, 2)
    }

    /// `bConfigurationValue`, the argument `SET_CONFIGURATION` expects.
    #[inline]
    pub fn value(&self) -> u8 {
        self.raw[5]
    }

    /// `bmAttributes`.
    #[inline]
    pub fn attributes(&self) -> u8 {
        self.raw[6]
    }

    /// `bMaxPower` in milliamperes (the descriptor stores 2 mA units).
    #[inline]
    pub fn max_power_ma(&self) -> u16 {
        self.raw[8] as u16 * 2
    }

    /// Every descriptor in the configuration, in wire order.
    #[inline]
    pub fn descriptors(&self) -> Descriptors<'a> {
        Descriptors { rest: self.raw }
    }

    /// The interfaces declared by the configuration.
    #[inline]
    pub fn interfaces(&self) -> Interfaces<'a> {
        Interfaces {
            raw: self.raw,
            offset: Self::HEADER_LEN,
        }
    }

    /// The first interface with the given class triple.
    pub fn find_interface(&self, class: u8, sub_class: u8, protocol: u8) -> Option<Interface<'a>> {
        self.interfaces()
            .find(|i| i.class() == class && i.sub_class() == sub_class && i.protocol() == protocol)
    }
}

// ─── Interface ───────────────────────────────────────────────────────────────

/// An interface descriptor, with its endpoints following it in the raw bytes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Interface<'a> {
    /// From the interface descriptor to the end of the configuration.
    raw: &'a [u8],
    /// Length of the interface descriptor itself.
    length: usize,
}

impl<'a> Interface<'a> {
    fn parse(bytes: &'a [u8]) -> Option<Self> {
        if bytes.len() < 9 || bytes[1] != descriptor_type::INTERFACE {
            return None;
        }
        Some(Self {
            raw: bytes,
            length: bytes[0] as usize,
        })
    }

    /// `bInterfaceNumber`.
    #[inline]
    pub fn number(&self) -> u8 {
        self.raw[2]
    }

    /// `bAlternateSetting`.
    #[inline]
    pub fn alternate_setting(&self) -> u8 {
        self.raw[3]
    }

    /// `bNumEndpoints`, which is also the number of endpoints
    /// [`endpoints`](Self::endpoints) yields unless the descriptor is truncated.
    #[inline]
    pub fn endpoint_count(&self) -> u8 {
        self.raw[4]
    }

    /// `bInterfaceClass`.
    #[inline]
    pub fn class(&self) -> u8 {
        self.raw[5]
    }

    /// `bInterfaceSubClass`.
    #[inline]
    pub fn sub_class(&self) -> u8 {
        self.raw[6]
    }

    /// `bInterfaceProtocol`.
    #[inline]
    pub fn protocol(&self) -> u8 {
        self.raw[7]
    }

    /// The endpoints of this interface.
    ///
    /// Class-specific descriptors end the list, exactly as the wire format
    /// orders them.
    #[inline]
    pub fn endpoints(&self) -> Endpoints<'a> {
        Endpoints {
            rest: &self.raw[self.length.min(self.raw.len())..],
        }
    }
}

/// Iterates over the interfaces of a configuration.
#[derive(Clone, Copy, Debug)]
pub struct Interfaces<'a> {
    raw: &'a [u8],
    offset: usize,
}

impl<'a> Iterator for Interfaces<'a> {
    type Item = Interface<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        while self.offset + 2 <= self.raw.len() {
            let start = self.offset;
            let length = self.raw[start] as usize;
            if length < 2 || start + length > self.raw.len() {
                self.offset = self.raw.len();
                return None;
            }
            self.offset += length;
            if self.raw[start + 1] == descriptor_type::INTERFACE {
                // An interface owns the descriptors that follow it, so hand out
                // everything up to the end of the configuration.
                return Interface::parse(&self.raw[start..]);
            }
        }
        None
    }
}

// ─── Endpoint ────────────────────────────────────────────────────────────────

/// A standard endpoint descriptor (USB 2.0 §9.6.6).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EndpointDescriptor {
    /// `bEndpointAddress`: number and direction.
    pub address: EndpointAddress,
    /// `bmAttributes`.
    pub attributes: u8,
    /// `wMaxPacketSize` without the high-bandwidth bits.
    pub max_packet: u16,
    /// `bInterval`.
    pub interval: u8,
}

impl EndpointDescriptor {
    /// Length of the descriptor on the wire.
    pub const LEN: usize = 7;

    /// Parse seven bytes of an endpoint descriptor.
    pub fn parse(bytes: &[u8]) -> HostResult<Self> {
        if bytes.len() < Self::LEN || bytes[0] < Self::LEN as u8 {
            return Err(HostError::Malformed);
        }
        if bytes[1] != descriptor_type::ENDPOINT {
            return Err(HostError::Malformed);
        }
        Ok(Self {
            address: EndpointAddress::from(bytes[2]),
            attributes: bytes[3],
            max_packet: le16(bytes, 4) & 0x07ff,
            interval: bytes[6],
        })
    }

    /// Transfer type encoded in `bmAttributes`.
    pub fn transfer_type(&self) -> EndpointType {
        match self.attributes & 0x03 {
            0 => EndpointType::Control,
            1 => EndpointType::Isochronous {
                synchronization: IsoSynchronization::None,
                usage: IsoUsage::Data,
            },
            2 => EndpointType::Bulk,
            _ => EndpointType::Interrupt,
        }
    }
}

/// Iterates over the endpoints that follow an interface descriptor.
#[derive(Clone, Copy, Debug)]
pub struct Endpoints<'a> {
    rest: &'a [u8],
}

impl Iterator for Endpoints<'_> {
    type Item = EndpointDescriptor;

    fn next(&mut self) -> Option<Self::Item> {
        let length = *self.rest.first()? as usize;
        if length < 2 || length > self.rest.len() || self.rest[1] != descriptor_type::ENDPOINT {
            // Class-specific descriptors end the endpoint list.
            self.rest = &[];
            return None;
        }
        let (head, tail) = self.rest.split_at(length);
        self.rest = tail;
        EndpointDescriptor::parse(head).ok()
    }
}
