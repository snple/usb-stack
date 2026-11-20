//! USB SETUP packet parser.

use crate::endpoint::{Direction, EndpointAddress};
use crate::error::{UsbError, UsbResult};

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
    pub fn parse(buf: &[u8]) -> UsbResult<Self> {
        if buf.len() < 8 {
            return Err(UsbError::ParseError);
        }
        let bm = buf[0];
        let recipient_raw = bm & 0x1f;
        Ok(Self {
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
