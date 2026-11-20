//! Control transfer state machine.
//!
//! [`ControlPipe`] manages EP0 state without being generic over the bus type.
//! It takes `&B` only for the actual hardware I/O operations (read / write),
//! keeping the state machine logic bus-agnostic.
//!
//! [`ControlIn`] and [`ControlOut`] are short-lived handles passed to
//! [`UsbClass`](crate::device::class::UsbClass) callbacks so classes can accept or reject
//! a control transfer ergonomically.

use core::cmp::min;

use crate::device::bus::UsbBus;
use crate::device::error::{UsbError, UsbResult};
use crate::endpoint::EndpointAddress;
use crate::request::Request;

// ─── Internal state ───────────────────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum State {
    Idle,
    /// IN transfer: data stage in progress, sending chunks.
    DataIn {
        offset: usize,
        len: usize,
    },
    /// IN transfer: last chunk sent, need a ZLP.
    DataInZlp,
    /// IN transfer: ZLP sent, waiting for status OUT.
    DataInLast,
    /// STATUS IN: waiting for the host's ZLP OUT to complete.
    StatusOut,
    /// CLASS / VENDOR: response built, waiting for our accept/reject call.
    CompleteIn(Request),
    /// STATUS OUT: sending ZLP IN to acknowledge OUT data received.
    StatusIn,
    /// HOST→DEVICE data stage.
    DataOut {
        req: Request,
        offset: usize,
        len: usize,
    },
    /// Data OUT complete, waiting for accept/reject.  `len` is the number of
    /// bytes the host actually sent, which is zero for the zero-length OUT
    /// transfers some requests use.
    CompleteOut {
        len: usize,
    },
    Error,
}

// ─── ControlPipe ─────────────────────────────────────────────────────────────

/// Maximum size of the EP0 control transfer staging buffer.
pub const CONTROL_BUF_SIZE: usize = 256;

/// EP0 control transfer state machine + staging buffer.
///
/// Not generic over the bus type; bus I/O is performed via trait-object-style
/// `&B` references passed on each call.
pub struct ControlPipe {
    state: State,
    buf: [u8; CONTROL_BUF_SIZE],
    /// EP0 maximum packet size (8, 16, 32 or 64).
    max_packet_size: u16,
    /// Optional static buffer used for large IN responses (avoids copy).
    static_buf: Option<&'static [u8]>,
}

impl Default for ControlPipe {
    fn default() -> Self {
        Self::new(crate::CONTROL_MAX_PACKET_SIZE)
    }
}

impl ControlPipe {
    /// Create an idle control pipe with an empty staging buffer.
    pub const fn new(max_packet_size: u16) -> Self {
        Self {
            state: State::Idle,
            buf: [0u8; CONTROL_BUF_SIZE],
            max_packet_size,
            static_buf: None,
        }
    }

    /// Reset to idle (called on bus reset).
    pub fn reset(&mut self) {
        self.state = State::Idle;
        self.static_buf = None;
    }

    /// Returns `true` when waiting for a class or device to call `accept_*` / `reject`.
    pub fn waiting_for_response(&self) -> bool {
        matches!(self.state, State::CompleteIn(_) | State::CompleteOut { .. })
    }

    /// Data received during the OUT data stage.
    ///
    /// Empty for a zero-length OUT transfer, which several requests use (an
    /// SMBus quick command, for example).
    pub fn data(&self) -> &[u8] {
        match self.state {
            State::CompleteOut { len } => &self.buf[..len],
            State::DataOut { offset, .. } => &self.buf[..offset],
            _ => &[],
        }
    }

    // ── Event handlers ────────────────────────────────────────────────────

    /// Called when the bus reports a SETUP packet on EP0.
    ///
    /// Reads the 8-byte packet from the bus and returns the parsed [`Request`]
    /// if we should dispatch it immediately, or `None` if we're still in a data
    /// stage (OUT transfers with a wLength > 0 need more packets first).
    pub fn handle_setup<B: UsbBus>(&mut self, bus: &B) -> Option<Request> {
        let mut raw = [0u8; 8];
        match bus.read(EndpointAddress::EP0_OUT, &mut raw) {
            Ok(n) if n >= 8 => {}
            _ => return None,
        }

        let req = Request::parse(&raw)?;

        // A new SETUP always aborts any previous transfer.
        self.static_buf = None;
        bus.set_stalled(EndpointAddress::EP0_OUT, false);
        bus.set_stalled(EndpointAddress::EP0_IN, false);

        use crate::endpoint::Direction;
        if req.direction == Direction::Out {
            if req.length > 0 {
                if (req.length as usize) > self.buf.len() {
                    // Data won't fit — silently ignore.
                    return None;
                }
                self.state = State::DataOut {
                    req,
                    offset: 0,
                    len: req.length as usize,
                };
                None
            } else {
                self.state = State::CompleteOut { len: 0 };
                Some(req)
            }
        } else {
            self.state = State::CompleteIn(req);
            Some(req)
        }
    }

    /// Called when the bus reports an OUT packet on EP0 (non-SETUP).
    pub fn handle_out<B: UsbBus>(&mut self, bus: &B) -> UsbResult<Option<Request>> {
        match self.state {
            State::DataOut {
                req,
                ref mut offset,
                len,
            } => {
                let n = match bus.read(EndpointAddress::EP0_OUT, &mut self.buf[*offset..]) {
                    Ok(n) => n,
                    Err(UsbError::WouldBlock) => return Ok(None),
                    Err(e) => {
                        self.set_error(bus);
                        return Err(e);
                    }
                };
                *offset += n;
                if *offset >= len {
                    // `offset` is how much the host actually sent, which is
                    // what the class has to see rather than the staging buffer.
                    self.state = State::CompleteOut { len: *offset };
                    return Ok(Some(req));
                }
                Ok(None)
            }
            // Host terminated DATA IN early with ZLP OUT.
            State::DataIn { .. } | State::DataInZlp | State::DataInLast | State::StatusOut => {
                let _ = bus.read(EndpointAddress::EP0_OUT, &mut []);
                self.state = State::Idle;
                Ok(None)
            }
            _ => {
                let _ = bus.read(EndpointAddress::EP0_OUT, &mut []);
                self.set_error(bus);
                Ok(None)
            }
        }
    }

    /// Called when the bus reports an IN-complete on EP0.
    ///
    /// Returns `true` when the entire transfer (including status stage) completed.
    pub fn handle_in_complete<B: UsbBus>(&mut self, bus: &B) -> UsbResult<bool> {
        match self.state {
            State::DataIn { offset, len } => {
                self.send_in_chunk(bus, offset, len)?;
                Ok(false)
            }
            State::DataInZlp => {
                bus.write(EndpointAddress::EP0_IN, &[])?;
                self.state = State::DataInLast;
                Ok(false)
            }
            State::DataInLast => {
                bus.set_stalled(EndpointAddress::EP0_OUT, false);
                self.state = State::StatusOut;
                Ok(false)
            }
            State::StatusIn => {
                self.state = State::Idle;
                Ok(true)
            }
            // Spurious IN-complete in other states is fine to ignore.
            _ => Ok(false),
        }
    }

    // ── Response helpers ──────────────────────────────────────────────────

    /// Accept a control IN request by writing `data` into the staging buffer.
    pub fn accept_in<B: UsbBus>(
        &mut self,
        bus: &B,
        f: impl FnOnce(&mut [u8]) -> UsbResult<usize>,
    ) -> UsbResult<()> {
        let req = match self.state {
            State::CompleteIn(r) => r,
            _ => return Err(UsbError::InvalidState),
        };
        self.static_buf = None;
        let len = f(&mut self.buf)?;
        if len > self.buf.len() {
            self.set_error(bus);
            return Err(UsbError::BufferOverflow);
        }
        self.start_in_transfer(bus, req, len)
    }

    /// Accept a control IN request with a pre-built slice.
    pub fn accept_with<B: UsbBus>(&mut self, bus: &B, data: &[u8]) -> UsbResult<()> {
        self.accept_in(bus, |buf| {
            if data.len() > buf.len() {
                return Err(UsbError::BufferOverflow);
            }
            buf[..data.len()].copy_from_slice(data);
            Ok(data.len())
        })
    }

    /// Accept a control IN request with a `'static` slice (zero-copy).
    pub fn accept_with_static<B: UsbBus>(&mut self, bus: &B, data: &'static [u8]) -> UsbResult<()> {
        let req = match self.state {
            State::CompleteIn(r) => r,
            _ => return Err(UsbError::InvalidState),
        };
        self.static_buf = Some(data);
        self.start_in_transfer(bus, req, data.len())
    }

    /// Accept a control OUT request (send ZLP status IN).
    pub fn accept_out<B: UsbBus>(&mut self, bus: &B) -> UsbResult<()> {
        match self.state {
            State::CompleteOut { .. } => {}
            _ => return Err(UsbError::InvalidState),
        }
        bus.write(EndpointAddress::EP0_IN, &[])?;
        self.state = State::StatusIn;
        Ok(())
    }

    /// Reject the pending control request by stalling EP0.
    pub fn reject<B: UsbBus>(&mut self, bus: &B) -> UsbResult<()> {
        if !self.waiting_for_response() {
            return Err(UsbError::InvalidState);
        }
        self.set_error(bus);
        Ok(())
    }

    // ── Private helpers ───────────────────────────────────────────────────

    fn start_in_transfer<B: UsbBus>(
        &mut self,
        bus: &B,
        req: Request,
        data_len: usize,
    ) -> UsbResult<()> {
        let len = min(data_len, req.length as usize);
        self.state = State::DataIn { offset: 0, len };
        self.send_in_chunk(bus, 0, len)
    }

    fn send_in_chunk<B: UsbBus>(&mut self, bus: &B, offset: usize, len: usize) -> UsbResult<()> {
        let mps = self.max_packet_size.max(8) as usize;
        let chunk_len = min(len - offset, mps);
        let src = self.static_buf.unwrap_or(&self.buf);
        let written = bus.write(EndpointAddress::EP0_IN, &src[offset..offset + chunk_len])?;
        let new_offset = offset + written;
        if new_offset >= len {
            self.static_buf = None;
            self.state = if written == mps {
                State::DataInZlp
            } else {
                State::DataInLast
            };
        } else {
            self.state = State::DataIn {
                offset: new_offset,
                len,
            };
        }
        Ok(())
    }

    fn set_error<B: UsbBus>(&mut self, bus: &B) {
        bus.set_stalled(EndpointAddress::EP0_IN, true);
        bus.set_stalled(EndpointAddress::EP0_OUT, true);
        self.state = State::Error;
    }
}

// ─── ControlIn / ControlOut handles ──────────────────────────────────────────

/// Handle for a pending control IN transfer.
///
/// Passed to [`UsbClass::control_in`](crate::device::class::UsbClass::control_in).
/// Call one of the `accept_*` methods or `reject` to respond; dropping without
/// calling either passes the request to the next handler.
pub struct ControlIn<'a, B: UsbBus> {
    pipe: &'a mut ControlPipe,
    bus: &'a B,
    req: &'a Request,
}

impl<'a, B: UsbBus> ControlIn<'a, B> {
    pub(crate) fn new(pipe: &'a mut ControlPipe, bus: &'a B, req: &'a Request) -> Self {
        Self { pipe, bus, req }
    }

    /// The SETUP packet that initiated this transfer.
    pub fn request(&self) -> &Request {
        self.req
    }

    /// Accept with data written by a closure into the pipe buffer.
    pub fn accept(self, f: impl FnOnce(&mut [u8]) -> UsbResult<usize>) -> UsbResult<()> {
        self.pipe.accept_in(self.bus, f)
    }

    /// Accept with a pre-built data slice.
    pub fn accept_with(self, data: &[u8]) -> UsbResult<()> {
        self.pipe.accept_with(self.bus, data)
    }

    /// Accept with a `'static` slice (avoids copy for large descriptors).
    pub fn accept_with_static(self, data: &'static [u8]) -> UsbResult<()> {
        self.pipe.accept_with_static(self.bus, data)
    }

    /// Reject by stalling EP0.
    pub fn reject(self) -> UsbResult<()> {
        self.pipe.reject(self.bus)
    }
}

/// Handle for a pending control OUT transfer.
///
/// Passed to [`UsbClass::control_out`](crate::device::class::UsbClass::control_out).
pub struct ControlOut<'a, B: UsbBus> {
    pipe: &'a mut ControlPipe,
    bus: &'a B,
    req: &'a Request,
}

impl<'a, B: UsbBus> ControlOut<'a, B> {
    pub(crate) fn new(pipe: &'a mut ControlPipe, bus: &'a B, req: &'a Request) -> Self {
        Self { pipe, bus, req }
    }

    /// The SETUP packet that initiated this transfer.
    pub fn request(&self) -> &Request {
        self.req
    }

    /// The data received in the OUT data stage.
    pub fn data(&self) -> &[u8] {
        self.pipe.data()
    }

    /// Accept by sending a ZLP status IN.
    pub fn accept(self) -> UsbResult<()> {
        self.pipe.accept_out(self.bus)
    }

    /// Reject by stalling EP0.
    pub fn reject(self) -> UsbResult<()> {
        self.pipe.reject(self.bus)
    }
}
