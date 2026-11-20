//! The hardware abstraction the host protocol layer runs on.
//!
//! A controller exposes what the USB 2.0 host controller model needs: the state
//! of the root port, channels, and one transaction at a time on a channel. It
//! knows nothing about device requests or descriptors.

use crate::endpoint::{EndpointAddress, EndpointType};
use crate::host::error::HostResult;

// ─── Speed ───────────────────────────────────────────────────────────────────

/// Bus speed negotiated with a device.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Speed {
    /// 1.5 Mbit/s.
    Low,
    /// 12 Mbit/s.
    Full,
    /// 480 Mbit/s.
    High,
}

impl Speed {
    /// Maximum packet size of a bulk endpoint at this speed.
    #[inline]
    pub const fn max_bulk_packet(self) -> u16 {
        match self {
            Self::High => 512,
            _ => 64,
        }
    }
}

// ─── Device address ──────────────────────────────────────────────────────────

/// The 7-bit address the host assigned to a device.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceAddress(u8);

impl DeviceAddress {
    /// The address every device answers to before `SET_ADDRESS`.
    pub const ZERO: Self = Self(0);
    /// The address [`Host::enumerate`](crate::host::Host::enumerate) assigns.
    pub const FIRST: Self = Self(1);

    /// Wrap a raw address.
    ///
    /// # Panics
    ///
    /// Panics when `value` does not fit in seven bits.
    pub const fn new(value: u8) -> Self {
        assert!(value < 0x80, "a USB device address is seven bits wide");
        Self(value)
    }

    /// The raw address byte.
    #[inline]
    pub const fn raw(self) -> u8 {
        self.0
    }
}

// ─── Port status ─────────────────────────────────────────────────────────────

/// What the root port reports.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PortStatus {
    /// A device is attached.
    pub connected: bool,
    /// The port finished a reset and is talking to the device.
    pub enabled: bool,
    /// Speed of the attached device; only meaningful while `enabled`.
    pub speed: Speed,
    /// The port draws too much current.
    pub overcurrent: bool,
}

// ─── Channel and transactions ────────────────────────────────────────────────

/// A host channel: the hardware slot a single endpoint is driven through.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Channel(u8);

impl Channel {
    /// Wrap a raw channel index.
    ///
    /// # Panics
    ///
    /// Panics when `index` is 16 or larger: the core implements at most sixteen
    /// channels.
    pub const fn new(index: u8) -> Self {
        assert!(index < 16, "the core implements at most 16 host channels");
        Self(index)
    }

    /// The channel index, as used by the `HCCHAR`/`HCINT`/`HCTSIZ` registers.
    #[inline]
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// The data toggle a packet starts with.
///
/// The protocol layer tracks toggles itself instead of relying on the
/// controller to remember them, so that a channel can be reprogrammed (after
/// `SET_ADDRESS`, for instance) without losing the sequence.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Toggle {
    /// DATA0.
    Data0,
    /// DATA1.
    Data1,
}

/// Endpoint characteristics a channel is programmed with.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EndpointConfig {
    /// Endpoint number and direction.
    pub address: EndpointAddress,
    /// Control, bulk or interrupt.
    pub transfer: EndpointType,
    /// Maximum packet size.
    pub max_packet: u16,
    /// Polling interval in frames, for interrupt endpoints. Controllers that
    /// poll on demand ignore it.
    pub interval: u8,
}

// ─── HostController ──────────────────────────────────────────────────────────

/// The peripheral side of the host stack.
///
/// Every method takes `&self`, because pipes and the device handle are handed
/// out by reference; implementations use interior mutability for the registers
/// and the channel table.
///
/// The interface is transaction oriented: one call moves at most one packet.
/// The protocol layer splits transfers into packets, which keeps controllers
/// simple and makes the FIFO requirements predictable.
pub trait HostController {
    /// Read the root port.
    fn port_status(&self) -> PortStatus;

    /// Drive a bus reset on the root port, wait for the device to be enabled
    /// and report the negotiated speed.
    fn reset_port(&self) -> HostResult<PortStatus>;

    /// Take a free channel.
    fn alloc_channel(&self) -> HostResult<Channel>;

    /// Give a channel back.
    fn free_channel(&self, channel: Channel);

    /// Program a channel for one endpoint of `device`.
    ///
    /// Called once before the first transfer and again whenever the device
    /// address or the packet size changes.
    fn configure(
        &self,
        channel: Channel,
        device: DeviceAddress,
        endpoint: &EndpointConfig,
    ) -> HostResult<()>;

    /// Send an eight-byte SETUP packet.
    fn setup(&self, channel: Channel, packet: &[u8]) -> HostResult<()>;

    /// Receive one packet into `buffer`, which must hold
    /// `endpoint.max_packet` bytes at most.
    ///
    /// Returns the number of bytes the device sent; that is zero for a
    /// zero-length packet.
    fn transfer_in(&self, channel: Channel, toggle: Toggle, buffer: &mut [u8])
    -> HostResult<usize>;

    /// Send one packet from `data`, which must hold `endpoint.max_packet` bytes
    /// at most.
    fn transfer_out(&self, channel: Channel, toggle: Toggle, data: &[u8]) -> HostResult<()>;

    /// Wait for at least `millis` milliseconds.
    ///
    /// The protocol layer needs the delays the USB specification mandates (2 ms
    /// after `SET_ADDRESS`, for example) but has no time base of its own.
    fn wait_millis(&self, millis: u32);
}
