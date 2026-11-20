//! USB error types.

/// USB operation result type.
pub type UsbResult<T> = Result<T, UsbError>;

/// Errors returned by USB stack operations.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum UsbError {
    /// An operation would block — no data available or buffer full.
    WouldBlock,
    /// A buffer was too small for the data to read, or provided data exceeds length constraints.
    BufferOverflow,
    /// The endpoint address is invalid or was not previously allocated.
    InvalidEndpoint,
    /// All endpoints of the requested type have been allocated.
    EndpointOverflow,
    /// The packet buffer memory available to the USB controller has been exhausted.
    EndpointMemoryOverflow,
    /// Operation is not supported by this bus implementation.
    Unsupported,
    /// Operation is not valid in the current state.
    InvalidState,
    /// A SETUP packet or descriptor could not be parsed.
    ParseError,
}
