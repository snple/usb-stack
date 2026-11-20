//! Errors reported by the host stack.

/// What went wrong on the host side.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HostError {
    /// Every host channel is in use.
    NoChannel,
    /// The device did not answer within the retry budget.
    Timeout,
    /// The endpoint answered with a STALL handshake.
    Stall,
    /// A transaction kept failing (CRC, bit stuff, false EOP) and the retries
    /// were used up.
    Transaction,
    /// Nothing is attached to the port, or the device went away.
    Disconnected,
    /// The buffer is too small for the reply or larger than a packet of the
    /// endpoint.
    Overflow,
    /// A descriptor or a reply does not follow the USB grammar.
    Malformed,
    /// The device answered something the protocol does not allow here.
    Protocol,
    /// The controller cannot do what was asked (isochronous endpoints, for
    /// example).
    Unsupported,
}

/// Result alias for the host stack.
pub type HostResult<T> = Result<T, HostError>;
