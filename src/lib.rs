#![doc = include_str!("../README.md")]
#![no_std]
#![deny(missing_docs)]

/// Logging facade.
///
/// With the `defmt` feature these are `defmt`'s own macros; without it they
/// expand to nothing so the stack builds without the logging dependency.
#[cfg(feature = "defmt")]
pub mod log {
    pub use defmt::*;
}

/// No-op logging macros used when the `defmt` feature is disabled.
#[cfg(not(feature = "defmt"))]
#[allow(missing_docs)]
pub mod log {
    #[macro_export]
    macro_rules! trace {
        ($($arg:tt)*) => {};
    }
    #[macro_export]
    macro_rules! debug {
        ($($arg:tt)*) => {};
    }
    #[macro_export]
    macro_rules! info {
        ($($arg:tt)*) => {};
    }
    #[macro_export]
    macro_rules! warn {
        ($($arg:tt)*) => {};
    }
    #[macro_export]
    macro_rules! error {
        ($($arg:tt)*) => {};
    }

    pub use crate::{debug, error, info, trace, warn};
}

// ── Modules ──────────────────────────────────────────────────────────────────

pub mod descriptor;
pub mod device;
pub mod endpoint;
pub mod host;
pub mod request;

// ── Crate-level constants ─────────────────────────────────────────────────────

/// Maximum number of endpoints the device stack can track (endpoint numbers
/// 0–15, IN and OUT). Peripheral implementations may support fewer.
pub const MAX_ENDPOINTS: usize = 16;

/// Maximum packet size for EP0 (full-speed USB 2.0).
pub const CONTROL_MAX_PACKET_SIZE: u16 = 64;

/// Maximum packet size for bulk/interrupt endpoints (full-speed USB 2.0).
pub const BULK_MAX_PACKET_SIZE: u16 = 64;
