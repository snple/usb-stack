//! USB bus trait and allocator.
//!
//! # Architecture
//!
//! - [`UsbBus`] — hardware-abstraction trait implemented by the peripheral driver.
//! - [`UsbBusAllocator`] — wraps a bus during the pre-enable allocation phase,
//!   then *freezes* it (enabling the peripheral) and returns a shared bus reference
//!   that both [`UsbDevice`](crate::device::UsbDevice) and class endpoint handles
//!   can borrow from.
//!
//! ## Key improvements over `usb-device`
//!
//! - No `portable_atomic` / `AtomicPtr` — endpoint handles borrow `&'a B` directly.
//! - No `mem::forget` hack — `UnsafeCell<B>` + `Cell<bool>` give safe interior-
//!   mutability without leaking the `RefMut` borrow-count.
//! - `UsbClass<B>` is not required to be generic over `B` by the allocator itself;
//!   Endpoint handles carry `&'a B` which is erased to `&'a dyn UsbBus` where needed.

use core::cell::{Cell, UnsafeCell};

use crate::endpoint::{Direction, Endpoint, EndpointAddress, EndpointDir, EndpointType};
use crate::error::{UsbError, UsbResult};

// ─── PollResult ───────────────────────────────────────────────────────────────

/// Events returned by [`UsbBus::poll`].
pub enum PollResult {
    /// No events to report.
    None,
    /// The USB bus was reset by the host.
    Reset,
    /// One or more endpoints have received or completed data.
    ///
    /// Each field is a 16-bit mask where bit *n* corresponds to endpoint *n*.
    Data {
        /// Endpoints that received an OUT packet.
        ep_out: u16,
        /// Endpoints whose IN transfer completed.
        ep_in_complete: u16,
        /// Endpoints that received a SETUP packet (normally only EP0, bit 0).
        ep_setup: u16,
    },
    /// The host asserted USB suspend.
    Suspend,
    /// The host deasserted USB suspend (resume).
    Resume,
}

impl PollResult {
    /// `true` for [`PollResult::Data`], i.e. when endpoint events are pending.
    #[inline]
    pub fn is_data(self) -> bool {
        matches!(self, Self::Data { .. })
    }
}

// ─── UsbBus trait ─────────────────────────────────────────────────────────────

/// Low-level hardware abstraction trait for a USB peripheral.
///
/// Implement this trait to add support for a new microcontroller.  All methods
/// that mutate hardware state take `&self` (not `&mut self`) because the bus is
/// shared by reference between [`UsbDevice`](crate::device::UsbDevice) and
/// endpoint handles.  Implementations must use interior mutability (atomics,
/// `UnsafeCell`, etc.) where needed.
///
/// # `alloc_ep` vs later methods
///
/// [`alloc_ep`](UsbBus::alloc_ep) takes `&mut self` and is only called *before*
/// [`enable`](UsbBus::enable).  All other methods take `&self`.
pub trait UsbBus: Sized {
    // ── Allocation phase ────────────────────────────────────────────────────

    /// Allocate an endpoint.
    ///
    /// Called by [`UsbBusAllocator`] before [`enable`](UsbBus::enable).
    ///
    /// * `dir`        — endpoint direction.
    /// * `addr`       — if `Some`, request exactly that address; if `None`, use
    ///   the next available one.
    /// * `ep_type`    — transfer type.
    /// * `max_packet` — maximum packet size in bytes.
    /// * `interval`   — polling interval (interrupt/isochronous endpoints).
    fn alloc_ep(
        &mut self,
        dir: Direction,
        addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet: u16,
        interval: u8,
    ) -> UsbResult<EndpointAddress>;

    /// Enable the USB peripheral.  Called once, after all endpoints have been
    /// allocated.  The device will be reset shortly after by the host.
    fn enable(&mut self);

    // ── Bus events ──────────────────────────────────────────────────────────

    /// Reset endpoint state after a USB bus reset.
    ///
    /// All previously allocated endpoints must be re-initialised to the state
    /// they were in right after [`alloc_ep`](UsbBus::alloc_ep).
    fn reset(&self);

    /// Set the device USB address.
    fn set_device_address(&self, addr: u8);

    /// Poll for bus events.  Usually called from an interrupt handler or the
    /// main loop.  Must be called at least once every 10 ms while connected.
    fn poll(&self) -> PollResult;

    // ── Endpoint I/O ────────────────────────────────────────────────────────

    /// Write a single packet to an IN endpoint.
    ///
    /// Returns the number of bytes actually written.
    ///
    /// # Errors
    /// - [`WouldBlock`](UsbError::WouldBlock) — previous packet still pending.
    /// - [`BufferOverflow`](UsbError::BufferOverflow) — `buf` exceeds `max_packet`.
    fn write(&self, addr: EndpointAddress, buf: &[u8]) -> UsbResult<usize>;

    /// Read a single packet from an OUT endpoint into `buf`.
    ///
    /// Returns the number of bytes read.
    ///
    /// # Errors
    /// - [`WouldBlock`](UsbError::WouldBlock) — no packet available.
    /// - [`BufferOverflow`](UsbError::BufferOverflow) — packet exceeds `buf`.
    fn read(&self, addr: EndpointAddress, buf: &mut [u8]) -> UsbResult<usize>;

    /// Set or clear the STALL condition on an endpoint.
    fn set_stalled(&self, addr: EndpointAddress, stalled: bool);

    /// Return `true` if the endpoint is currently stalled.
    fn is_stalled(&self, addr: EndpointAddress) -> bool;

    // ── Power management ────────────────────────────────────────────────────

    /// Enter USB suspend mode.
    fn suspend(&self);

    /// Resume from suspend mode.
    fn resume(&self);

    /// Simulate a USB disconnect (force re-enumeration).
    ///
    /// The default implementation returns [`Unsupported`](UsbError::Unsupported).
    fn force_reset(&self) -> UsbResult<()> {
        Err(UsbError::Unsupported)
    }

    // ── Quirks ──────────────────────────────────────────────────────────────

    /// When `true`, [`set_device_address`](UsbBus::set_device_address) must be
    /// called *before* the status stage of a SET_ADDRESS request is completed
    /// (non-standard, required by some peripherals).
    ///
    /// Default: `false` (USB 2.0 spec §9.4.6 behaviour).
    const SET_ADDRESS_BEFORE_STATUS: bool = false;

    /// Largest packet size bulk and interrupt endpoints support on this bus.
    ///
    /// Classes use it to size their endpoints: 64 bytes for full-speed buses,
    /// 512 for high-speed ones. Default: 64.
    const MAX_PACKET_SIZE: u16 = 64;
}

// ─── Opaque handle types ──────────────────────────────────────────────────────

/// An opaque handle to an allocated USB interface number.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterfaceNumber(pub(crate) u8);

impl InterfaceNumber {
    /// Construct an interface number from its raw value.
    ///
    /// Classes normally obtain one from
    /// [`UsbBusAllocator::interface`](crate::bus::UsbBusAllocator::interface);
    /// this is for protocol tables and tests that carry a fixed number.
    #[inline]
    pub const fn new(number: u8) -> Self {
        Self(number)
    }
}

impl From<InterfaceNumber> for u8 {
    fn from(n: InterfaceNumber) -> u8 {
        n.0
    }
}

/// An opaque handle to an allocated USB string descriptor index.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StringIndex(u8);

impl StringIndex {
    pub(crate) fn new(index: u8) -> Self {
        Self(index)
    }
}

impl From<StringIndex> for u8 {
    fn from(s: StringIndex) -> u8 {
        s.0
    }
}

// ─── UsbBusAllocator ──────────────────────────────────────────────────────────

/// Wraps a [`UsbBus`] during the endpoint-allocation phase.
///
/// Classes call the `bulk`, `interrupt`, `interface`, and `string` helper methods
/// in their constructors to obtain typed endpoint handles and resource indices.
/// Once all classes are constructed, [`UsbDevice::build`](crate::device::UsbDevice)
/// calls `freeze()` internally, which enables the
/// peripheral and returns a `&B` reference that persists for the allocator's
/// lifetime.
///
/// ## Why no `mem::forget`
///
/// The original `usb-device` crate used `mem::forget` on a `RefMut` guard to
/// permanently lock a `RefCell<B>`.  This crate instead uses `UnsafeCell<B>` +
/// `Cell<bool>` for the frozen flag.  The safety invariant is:
/// - Before `freeze`: only `alloc_ep` (via `&mut *self.bus.get()`) mutates the bus,
///   and it is called sequentially (single-threaded embedded context).
/// - After `freeze`: no `&mut B` is ever created again; all `&B` users are sound.
pub struct UsbBusAllocator<B: UsbBus> {
    bus: UnsafeCell<B>,
    frozen: Cell<bool>,
    next_interface: Cell<u8>,
    next_string_index: Cell<u8>,
}

// Safety: access management is described in the module doc; correct only for
// single-threaded embedded contexts where `B: Send`.
unsafe impl<B: UsbBus + Send> Send for UsbBusAllocator<B> {}
unsafe impl<B: UsbBus + Sync> Sync for UsbBusAllocator<B> {}

impl<B: UsbBus> UsbBusAllocator<B> {
    /// Wrap a bus implementation.  The bus must not yet be enabled.
    pub fn new(bus: B) -> Self {
        Self {
            bus: UnsafeCell::new(bus),
            frozen: Cell::new(false),
            next_interface: Cell::new(0),
            // Indices 0–3 are reserved: 0=lang-IDs, 1=manufacturer, 2=product, 3=serial.
            next_string_index: Cell::new(4),
        }
    }

    // ── Low-level allocation ─────────────────────────────────────────────────

    /// Allocate an endpoint with the specified direction and parameters.
    ///
    /// This delegates to [`UsbBus::alloc_ep`].  Prefer the typed helpers
    /// (`bulk`, `interrupt`, …) in class constructors.
    pub fn alloc<D: EndpointDir>(
        &self,
        addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet: u16,
        interval: u8,
    ) -> UsbResult<Endpoint<'_, B, D>> {
        assert!(
            !self.frozen.get(),
            "cannot allocate endpoints after UsbBusAllocator::freeze()"
        );
        let ep_addr = unsafe {
            (*self.bus.get()).alloc_ep(D::DIRECTION, addr, ep_type, max_packet, interval)?
        };
        // Safety: bus is inside UnsafeCell; no &mut B exists after this call returns.
        let bus: &B = unsafe { &*self.bus.get() };
        Ok(Endpoint::new(bus, ep_addr, ep_type, max_packet, interval))
    }

    // ── Typed helpers ────────────────────────────────────────────────────────

    /// Allocate a control endpoint (panics on allocation failure).
    pub fn control<D: EndpointDir>(&self, max_packet: u16) -> Endpoint<'_, B, D> {
        self.alloc(None, EndpointType::Control, max_packet, 0)
            .expect("control endpoint allocation failed")
    }

    /// Allocate a bulk endpoint (panics on allocation failure).
    pub fn bulk<D: EndpointDir>(&self, max_packet: u16) -> Endpoint<'_, B, D> {
        self.alloc(None, EndpointType::Bulk, max_packet, 0)
            .expect("bulk endpoint allocation failed")
    }

    /// Allocate an interrupt endpoint (panics on allocation failure).
    pub fn interrupt<D: EndpointDir>(&self, max_packet: u16, interval: u8) -> Endpoint<'_, B, D> {
        self.alloc(None, EndpointType::Interrupt, max_packet, interval)
            .expect("interrupt endpoint allocation failed")
    }

    /// Allocate an isochronous endpoint (panics on allocation failure).
    pub fn isochronous<D: EndpointDir>(
        &self,
        synchronization: crate::endpoint::IsoSynchronization,
        usage: crate::endpoint::IsoUsage,
        max_packet: u16,
        interval: u8,
    ) -> Endpoint<'_, B, D> {
        self.alloc(
            None,
            EndpointType::Isochronous {
                synchronization,
                usage,
            },
            max_packet,
            interval,
        )
        .expect("isochronous endpoint allocation failed")
    }

    // ── Resource handles ─────────────────────────────────────────────────────

    /// Allocate the next sequential interface number.
    pub fn interface(&self) -> InterfaceNumber {
        assert!(
            !self.frozen.get(),
            "cannot allocate interfaces after freeze()"
        );
        let n = self.next_interface.get();
        self.next_interface.set(n + 1);
        InterfaceNumber(n)
    }

    /// Allocate the next sequential string descriptor index.
    pub fn string(&self) -> StringIndex {
        assert!(!self.frozen.get(), "cannot allocate strings after freeze()");
        let i = self.next_string_index.get();
        self.next_string_index.set(i + 1);
        StringIndex(i)
    }

    // ── Freeze ───────────────────────────────────────────────────────────────

    /// Enable the USB peripheral and return a long-lived shared reference to it.
    ///
    /// After this call no further endpoint / interface / string allocation is
    /// possible.  The returned `&B` has the same lifetime as the allocator itself.
    ///
    /// # Safety
    ///
    /// Callers must ensure they do not call any `&mut B` method on the underlying
    /// bus after this point.  `UsbDevice::build` is the only intended caller.
    pub(crate) fn freeze(&self) -> &B {
        if !self.frozen.get() {
            self.frozen.set(true);
            // Safety: no concurrent mutable access; frozen flag prevents further
            // &mut B creation through the allocator API.
            unsafe { (*self.bus.get()).enable() };
        }
        // Safety: bus lives as long as &self; no &mut B will be created again.
        unsafe { &*self.bus.get() }
    }

    // ── Internal accessors ───────────────────────────────────────────────────

    #[allow(dead_code)]
    pub(crate) fn num_interfaces(&self) -> u8 {
        self.next_interface.get()
    }
}
