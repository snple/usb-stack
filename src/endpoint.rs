//! USB endpoint vocabulary.
//!
//! Endpoint numbers, directions, transfer types and the direction markers both
//! directions of the stack use. The device-side handles that own an allocated
//! endpoint live in [`crate::device::endpoint`].

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
