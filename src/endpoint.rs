//! USB endpoint types and handles.
//!
//! [`Endpoint`] (and its type aliases [`EndpointIn`] / [`EndpointOut`]) provide
//! type-safe access to an individual USB endpoint.  A class constructor receives
//! a reference to the [`crate::bus::UsbBusAllocator`], which allocates endpoints
//! and returns handles that borrow from that allocator.  The owned bus inside the
//! allocator is therefore kept alive for as long as any endpoint handle exists.

use core::marker::PhantomData;

use crate::bus::UsbBus;
use crate::error::UsbResult;

// ─── Direction ───────────────────────────────────────────────────────────────

/// Marker trait for endpoint direction.
pub trait EndpointDir: Sized + 'static {
    /// The concrete direction value for this marker.
    const DIRECTION: Direction;
}

/// Direction marker for device-to-host (IN) endpoints.
pub struct In;
impl EndpointDir for In {
    const DIRECTION: Direction = Direction::In;
}

/// Direction marker for host-to-device (OUT) endpoints.
pub struct Out;
impl EndpointDir for Out {
    const DIRECTION: Direction = Direction::Out;
}

/// USB transfer direction (host-centric: IN = device→host, OUT = host→device).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Direction {
    /// Host-to-device.
    Out = 0x00,
    /// Device-to-host.
    In = 0x80,
}

impl From<u8> for Direction {
    #[inline]
    fn from(v: u8) -> Self {
        if v & 0x80 != 0 {
            Direction::In
        } else {
            Direction::Out
        }
    }
}

// ─── EndpointType ─────────────────────────────────────────────────────────────

/// USB endpoint transfer type.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EndpointType {
    /// Control endpoint — only EP0 uses this in practice.
    Control,
    /// Isochronous endpoint — time-critical, unreliable.
    Isochronous {
        /// Synchronization type.
        synchronization: IsoSynchronization,
        /// Usage type.
        usage: IsoUsage,
    },
    /// Bulk endpoint — large, reliable, best-effort.
    Bulk,
    /// Interrupt endpoint — small, reliable, bounded latency.
    Interrupt,
}

impl EndpointType {
    /// Encode this type into the `bmAttributes` transfer-type field (USB 2.0 §9.6.6).
    pub fn bm_attributes(self) -> u8 {
        match self {
            EndpointType::Control => 0b00,
            EndpointType::Bulk => 0b10,
            EndpointType::Interrupt => 0b11,
            EndpointType::Isochronous {
                synchronization,
                usage,
            } => {
                let sync = match synchronization {
                    IsoSynchronization::None => 0b00,
                    IsoSynchronization::Asynchronous => 0b01,
                    IsoSynchronization::Adaptive => 0b10,
                    IsoSynchronization::Synchronous => 0b11,
                };
                let usg = match usage {
                    IsoUsage::Data => 0b00,
                    IsoUsage::Feedback => 0b01,
                    IsoUsage::ImplicitFeedbackData => 0b10,
                };
                0b01 | (sync << 2) | (usg << 4)
            }
        }
    }
}

/// Isochronous synchronization type (USB 2.0 §5.12.4.1).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IsoSynchronization {
    /// No synchronization.
    None,
    /// Asynchronous sink/source.
    Asynchronous,
    /// Adaptive sink/source.
    Adaptive,
    /// Synchronous sink/source.
    Synchronous,
}

/// Isochronous usage type (USB 2.0 §9.6.6).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IsoUsage {
    /// Isochronous data endpoint.
    Data,
    /// Isochronous feedback endpoint.
    Feedback,
    /// Implicit feedback data endpoint.
    ImplicitFeedbackData,
}

// ─── EndpointAddress ──────────────────────────────────────────────────────────

/// Type-safe USB endpoint address (number + direction encoded in one byte).
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct EndpointAddress(u8);

impl EndpointAddress {
    /// EP0 OUT address (control, host→device).
    pub const EP0_OUT: EndpointAddress = EndpointAddress(0x00);
    /// EP0 IN address (control, device→host).
    pub const EP0_IN: EndpointAddress = EndpointAddress(0x80);

    /// Construct from an endpoint number and direction.
    #[inline]
    pub const fn new(number: u8, direction: Direction) -> Self {
        Self((number & 0x0f) | direction as u8)
    }

    /// Construct an OUT endpoint address.
    #[inline]
    pub const fn out(number: u8) -> Self {
        Self(number & 0x0f)
    }

    /// Construct an IN endpoint address.
    #[inline]
    pub const fn in_(number: u8) -> Self {
        Self((number & 0x0f) | 0x80)
    }

    /// Raw address byte.
    #[inline]
    pub const fn raw(self) -> u8 {
        self.0
    }

    /// Endpoint number (0–15).
    #[inline]
    pub const fn number(self) -> u8 {
        self.0 & 0x0f
    }

    /// Transfer direction.
    #[inline]
    pub fn direction(self) -> Direction {
        Direction::from(self.0)
    }

    /// `true` if this is an IN (device→host) endpoint.
    #[inline]
    pub const fn is_in(self) -> bool {
        self.0 & 0x80 != 0
    }

    /// `true` if this is an OUT (host→device) endpoint.
    #[inline]
    pub const fn is_out(self) -> bool {
        !self.is_in()
    }
}

impl From<u8> for EndpointAddress {
    #[inline]
    fn from(v: u8) -> Self {
        Self(v)
    }
}

impl From<EndpointAddress> for u8 {
    #[inline]
    fn from(a: EndpointAddress) -> Self {
        a.0
    }
}

impl core::fmt::Debug for EndpointAddress {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "EP{} {}",
            self.number(),
            if self.is_in() { "IN" } else { "OUT" }
        )
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for EndpointAddress {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "EP{} {}",
            self.number(),
            if self.is_in() { "IN" } else { "OUT" }
        );
    }
}

// ─── Endpoint handle ──────────────────────────────────────────────────────────

/// A handle to a specific USB endpoint.
///
/// The lifetime `'a` is tied to the [`crate::bus::UsbBusAllocator`] that produced
/// this handle, ensuring the underlying bus cannot be dropped or moved while the
/// handle is alive.
///
/// Use the type aliases [`EndpointIn`] and [`EndpointOut`] for convenience.
pub struct Endpoint<'a, B: UsbBus, D: EndpointDir> {
    /// Shared reference to the bus — valid for `'a`.
    bus: &'a B,
    address: EndpointAddress,
    ep_type: EndpointType,
    max_packet_size: u16,
    interval: u8,
    _dir: PhantomData<fn() -> D>,
}

impl<'a, B: UsbBus, D: EndpointDir> Endpoint<'a, B, D> {
    /// Construct a new endpoint handle (called only by `UsbBusAllocator`).
    pub(crate) fn new(
        bus: &'a B,
        address: EndpointAddress,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> Self {
        Self {
            bus,
            address,
            ep_type,
            max_packet_size,
            interval,
            _dir: PhantomData,
        }
    }

    /// Endpoint address (number + direction).
    #[inline]
    pub fn address(&self) -> EndpointAddress {
        self.address
    }

    /// Transfer type of this endpoint.
    #[inline]
    pub fn ep_type(&self) -> EndpointType {
        self.ep_type
    }

    /// Maximum packet size in bytes.
    #[inline]
    pub fn max_packet_size(&self) -> u16 {
        self.max_packet_size
    }

    /// Polling interval for interrupt / isochronous endpoints.
    #[inline]
    pub fn interval(&self) -> u8 {
        self.interval
    }

    /// Stall this endpoint.
    pub fn stall(&self) {
        self.bus.set_stalled(self.address, true);
    }

    /// Clear the stall condition on this endpoint.
    pub fn unstall(&self) {
        self.bus.set_stalled(self.address, false);
    }
}

/// A device-to-host (IN) endpoint handle.
pub type EndpointIn<'a, B> = Endpoint<'a, B, In>;
/// A host-to-device (OUT) endpoint handle.
pub type EndpointOut<'a, B> = Endpoint<'a, B, Out>;

impl<B: UsbBus> EndpointIn<'_, B> {
    /// Write a single packet to this endpoint.
    ///
    /// Returns the number of bytes actually written.  If the USB peripheral's
    /// transmit buffer is still occupied, returns [`UsbError::WouldBlock`](crate::error::UsbError::WouldBlock).
    pub fn write(&self, data: &[u8]) -> UsbResult<usize> {
        self.bus.write(self.address, data)
    }
}

impl<B: UsbBus> EndpointOut<'_, B> {
    /// Read a single packet from this endpoint into `buf`.
    ///
    /// Returns the number of bytes read.  Returns [`UsbError::WouldBlock`](crate::error::UsbError::WouldBlock) when
    /// no packet is available.
    pub fn read(&self, buf: &mut [u8]) -> UsbResult<usize> {
        self.bus.read(self.address, buf)
    }
}

// Safety: Endpoint<'a, B, D> is Send/Sync if B is Send/Sync; the shared &'a B
// is sound because the bus uses interior mutability for hardware state.
unsafe impl<B: UsbBus + Sync, D: EndpointDir> Sync for Endpoint<'_, B, D> {}
unsafe impl<B: UsbBus + Send, D: EndpointDir> Send for Endpoint<'_, B, D> {}
