//! Endpoint handles owned by the device stack.
//!
//! [`Endpoint`] (and its type aliases [`EndpointIn`] / [`EndpointOut`]) provide
//! type-safe access to an individual USB endpoint.  A class constructor receives
//! a reference to the [`UsbBusAllocator`](crate::device::bus::UsbBusAllocator), which allocates endpoints and returns
//! handles that borrow from that allocator.  The owned bus inside the allocator
//! is therefore kept alive for as long as any endpoint handle exists.

use core::marker::PhantomData;

use crate::device::bus::UsbBus;
use crate::device::error::UsbResult;
use crate::endpoint::{EndpointAddress, EndpointDir, EndpointType, In, Out};

// ─── Endpoint handle ──────────────────────────────────────────────────────────

/// A handle to a specific USB endpoint.
///
/// The lifetime `'a` is tied to the [`crate::device::bus::UsbBusAllocator`] that produced
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
    /// transmit buffer is still occupied, returns [`UsbError::WouldBlock`](crate::device::error::UsbError::WouldBlock).
    pub fn write(&self, data: &[u8]) -> UsbResult<usize> {
        self.bus.write(self.address, data)
    }
}

impl<B: UsbBus> EndpointOut<'_, B> {
    /// Read a single packet from this endpoint into `buf`.
    ///
    /// Returns the number of bytes read.  Returns [`UsbError::WouldBlock`](crate::device::error::UsbError::WouldBlock) when
    /// no packet is available.
    pub fn read(&self, buf: &mut [u8]) -> UsbResult<usize> {
        self.bus.read(self.address, buf)
    }
}

// Safety: Endpoint<'a, B, D> is Send/Sync if B is Send/Sync; the shared &'a B
// is sound because the bus uses interior mutability for hardware state.
unsafe impl<B: UsbBus + Sync, D: EndpointDir> Sync for Endpoint<'_, B, D> {}
unsafe impl<B: UsbBus + Send, D: EndpointDir> Send for Endpoint<'_, B, D> {}
