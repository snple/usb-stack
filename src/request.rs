//! USB control requests.
//!
//! Both directions share one type: the device side parses the eight bytes it
//! receives on endpoint 0, the host side builds them for the requests it sends.

use crate::endpoint::{Direction, EndpointAddress};

// ─── Request type & recipient ─────────────────────────────────────────────────

/// `bmRequestType` bits 6:5 — who defined the request.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RequestType {
    /// USB 2.0 standard request.
    Standard = 0,
    /// Class-specific request.
    Class = 1,
    /// Vendor-specific request.
    Vendor = 2,
    /// Reserved encoding.
    Reserved = 3,
}

impl From<u8> for RequestType {
    fn from(v: u8) -> Self {
        match (v >> 5) & 0x03 {
            0 => RequestType::Standard,
            1 => RequestType::Class,
            2 => RequestType::Vendor,
            _ => RequestType::Reserved,
        }
    }
}

/// `bmRequestType` bits 4:0 — who the request is addressed to.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Recipient {
    /// The device as a whole.
    Device = 0,
    /// A specific interface.
    Interface = 1,
    /// A specific endpoint.
    Endpoint = 2,
    /// Another device on the bus (hub class).
    Other = 3,
    /// Reserved encoding.
    Reserved = 4,
}

impl From<u8> for Recipient {
    fn from(v: u8) -> Self {
        match v & 0x1f {
            0 => Recipient::Device,
            1 => Recipient::Interface,
            2 => Recipient::Endpoint,
            3 => Recipient::Other,
            _ => Recipient::Reserved,
        }
    }
}

// ─── Standard request codes ───────────────────────────────────────────────────

/// Standard USB request codes (`bRequest`).
#[allow(non_snake_case)]
#[allow(missing_docs)]
pub mod StandardRequest {
    pub const GET_STATUS: u8 = 0;
    pub const CLEAR_FEATURE: u8 = 1;
    pub const SET_FEATURE: u8 = 3;
    pub const SET_ADDRESS: u8 = 5;
    pub const GET_DESCRIPTOR: u8 = 6;
    pub const SET_DESCRIPTOR: u8 = 7;
    pub const GET_CONFIGURATION: u8 = 8;
    pub const SET_CONFIGURATION: u8 = 9;
    pub const GET_INTERFACE: u8 = 10;
    pub const SET_INTERFACE: u8 = 11;
    pub const SYNCH_FRAME: u8 = 12;
}

/// Standard feature selectors (`wValue`).
#[allow(non_snake_case)]
#[allow(missing_docs)]
pub mod Feature {
    pub const ENDPOINT_HALT: u16 = 0;
    pub const DEVICE_REMOTE_WAKEUP: u16 = 1;
    pub const TEST_MODE: u16 = 2;
}

// ─── Request ─────────────────────────────────────────────────────────────────

/// A parsed USB SETUP packet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Request {
    /// Transfer direction (from host's perspective).
    pub direction: Direction,
    /// Who defined this request.
    pub request_type: RequestType,
    /// Who the request is directed at.
    pub recipient: Recipient,
    /// Request code (`bRequest`).
    pub request: u8,
    /// Request-specific value (`wValue`).
    pub value: u16,
    /// Request-specific index (`wIndex`).
    pub index: u16,
    /// Number of bytes in the data stage (`wLength`).
    pub length: u16,
}

impl Request {
    /// Parse a SETUP packet from its 8-byte wire encoding.
    ///
    /// Returns `None` when fewer than eight bytes are available.
    pub fn parse(buf: &[u8]) -> Option<Self> {
        let buf: &[u8; 8] = buf.get(..8)?.try_into().ok()?;
        let bm = buf[0];
        let recipient_raw = bm & 0x1f;
        Some(Self {
            direction: Direction::from(bm),
            request_type: RequestType::from(bm),
            recipient: if recipient_raw <= 3 {
                Recipient::from(bm)
            } else {
                Recipient::Reserved
            },
            request: buf[1],
            value: u16::from_le_bytes([buf[2], buf[3]]),
            index: u16::from_le_bytes([buf[4], buf[5]]),
            length: u16::from_le_bytes([buf[6], buf[7]]),
        })
    }

    /// Build a request from its fields.
    pub const fn custom(
        direction: Direction,
        request_type: RequestType,
        recipient: Recipient,
        request: u8,
        value: u16,
        index: u16,
        length: u16,
    ) -> Self {
        Self {
            direction,
            request_type,
            recipient,
            request,
            value,
            index,
            length,
        }
    }

    /// The wire encoding of the request.
    pub const fn to_bytes(self) -> [u8; 8] {
        let value = self.value.to_le_bytes();
        let index = self.index.to_le_bytes();
        let length = self.length.to_le_bytes();
        [
            self.direction as u8 | (self.request_type as u8) << 5 | self.recipient as u8,
            self.request,
            value[0],
            value[1],
            index[0],
            index[1],
            length[0],
            length[1],
        ]
    }

    /// A standard request to the device.
    const fn standard(
        direction: Direction,
        request: u8,
        value: u16,
        index: u16,
        length: u16,
    ) -> Self {
        Self::custom(
            direction,
            RequestType::Standard,
            Recipient::Device,
            request,
            value,
            index,
            length,
        )
    }

    /// `GET_DESCRIPTOR`, which always reads from the device.
    pub const fn get_descriptor(kind: u8, index: u8, language: u16, length: u16) -> Self {
        Self::standard(
            Direction::In,
            StandardRequest::GET_DESCRIPTOR,
            ((kind as u16) << 8) | index as u16,
            language,
            length,
        )
    }

    /// `SET_ADDRESS`.
    pub const fn set_address(address: u8) -> Self {
        Self::standard(
            Direction::Out,
            StandardRequest::SET_ADDRESS,
            address as u16,
            0,
            0,
        )
    }

    /// `SET_CONFIGURATION`.
    pub const fn set_configuration(value: u8) -> Self {
        Self::standard(
            Direction::Out,
            StandardRequest::SET_CONFIGURATION,
            value as u16,
            0,
            0,
        )
    }

    /// `SET_INTERFACE`.
    pub const fn set_interface(interface: u8, alternate: u8) -> Self {
        Self::custom(
            Direction::Out,
            RequestType::Standard,
            Recipient::Interface,
            StandardRequest::SET_INTERFACE,
            alternate as u16,
            interface as u16,
            0,
        )
    }

    /// `GET_STATUS`, which always returns two bytes.
    pub const fn get_status(recipient: Recipient, index: u16) -> Self {
        Self::custom(
            Direction::In,
            RequestType::Standard,
            recipient,
            StandardRequest::GET_STATUS,
            0,
            index,
            2,
        )
    }

    /// `CLEAR_FEATURE`.
    pub const fn clear_feature(recipient: Recipient, feature: u16, index: u16) -> Self {
        Self::custom(
            Direction::Out,
            RequestType::Standard,
            recipient,
            StandardRequest::CLEAR_FEATURE,
            feature,
            index,
            0,
        )
    }

    /// `SET_FEATURE`.
    pub const fn set_feature(recipient: Recipient, feature: u16, index: u16) -> Self {
        Self::custom(
            Direction::Out,
            RequestType::Standard,
            recipient,
            StandardRequest::SET_FEATURE,
            feature,
            index,
            0,
        )
    }

    /// `CLEAR_FEATURE(ENDPOINT_HALT)` for an endpoint.
    pub const fn clear_halt(endpoint: u8) -> Self {
        Self::clear_feature(Recipient::Endpoint, Feature::ENDPOINT_HALT, endpoint as u16)
    }

    /// Extract (descriptor_type, descriptor_index) from `wValue` as used in
    /// GET_DESCRIPTOR requests.
    #[inline]
    pub fn descriptor_type_index(&self) -> (u8, u8) {
        ((self.value >> 8) as u8, self.value as u8)
    }

    /// Extract the endpoint address from `wIndex` for endpoint-directed requests.
    #[inline]
    pub fn endpoint_address(&self) -> EndpointAddress {
        EndpointAddress::from((self.index & 0x8f) as u8)
    }
}
