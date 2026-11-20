//! i2c-tiny-usb — a vendor-specific USB to I2C bridge.
//!
//! Implements the protocol of Till Harbaum's *i2c-tiny-usb* adapter, spoken by
//! the Linux `i2c-tiny-usb` driver (`drivers/i2c/busses/i2c-tiny-usb.c`) and by
//! the `i2c_usb` test tool shipped with the original firmware.  Every transfer
//! travels over the control pipe, so the function declares one vendor-specific
//! interface and no endpoints at all.
//!
//! # Protocol
//!
//! | `bRequest` | `wValue` | `wIndex` | Data | Answer |
//! |------------|----------|----------|------|--------|
//! | [`cmd::ECHO`] | value | — | — | the value bytes, for link testing |
//! | [`cmd::GET_FUNC`] | — | — | — | functionality as a little-endian `u32` |
//! | [`cmd::SET_DELAY`] | bit delay in µs | — | — | — |
//! | [`cmd::GET_STATUS`] | — | — | — | status of the last transfer |
//! | [`cmd::I2C_IO`] with flags | `i2c_msg` flags | 7-bit address | message bytes | read data on reads |
//!
//! The host marks the first message of a transaction with
//! [`cmd::I2C_IO_BEGIN`] and the last one with [`cmd::I2C_IO_END`]; a message
//! without `BEGIN` continues the previous one with a repeated START, one
//! without `END` leaves the bus busy.  After every message the host reads
//! `GET_STATUS` and gives up with `-EREMOTEIO` when it is
//! [`status::ADDRESS_NAK`].
//!
//! The I2C address travels in `wIndex`, which normally holds the interface
//! number, so requests are answered for any interface index.  That matches the
//! reference firmware, which is a single-function device.  Vendor requests the
//! adapter does not recognise are left alone, so it can coexist with another
//! vendor-specific function.
//!
//! # Supported transactions
//!
//! * one message — address, optional data, START and STOP
//! * write followed by read — the repeated-start pattern that SMBus emulation
//!   uses, and therefore what `i2cdetect`, `i2cget` and `i2cdump` produce
//! * several writes — concatenated into a single message
//!
//! A hardware I2C peripheral cannot pause in the middle of a transaction, so
//! the bytes of a message that does not end the transaction are buffered (up to
//! [`PENDING_WRITE_MAX`] of them) and replayed together with the message that
//! ends it.  Reads must be the last message of a transaction, 10-bit addresses
//! are not supported, SMBus block reads need the read phase to continue after
//! the first byte ([`flags::RECV_LEN`], so
//! [`func::SMBUS_READ_BLOCK_DATA`] is not advertised either), and a transfer has
//! to fit the 256-byte control buffer.  Such requests are stalled instead of
//! being silently mistranslated.
//!
//! # Example
//!
//! ```ignore
//! use my_hal_usb::{UsbBusAllocator, UsbDeviceBuilder};
//! use my_hal_usb::class::i2c_tiny_usb::I2cTinyUsb;
//! use my_hal_usb::MyBus;
//!
//! let alloc = UsbBusAllocator::new(MyBus::new(/* peripheral */));
//! let mut adapter = I2cTinyUsb::new(&alloc, MyI2cBus::new());
//! let mut dev = UsbDeviceBuilder::new(&alloc, 0x0403, 0xC631)
//!     .device_release(0x0100)
//!     .manufacturer("harbaum.org")
//!     .product("i2c-tiny-usb")
//!     .build();
//!
//! dev.bus().connect();
//! loop {
//!     dev.poll(&mut [&mut adapter]);
//! }
//! ```

use crate::bus::{InterfaceNumber, UsbBus, UsbBusAllocator};
use crate::class::UsbClass;
use crate::control::{ControlIn, ControlOut, Recipient, Request, RequestType};
use crate::descriptor::{DescriptorWriter, class_code};
use crate::error::UsbResult;

// ─── Protocol constants ──────────────────────────────────────────────────────

/// Vendor control requests (`bRequest`).
pub mod cmd {
    /// Echo `wValue` back; used to check that the link is alive.
    pub const ECHO: u8 = 0;
    /// Report the adapter's functionality as a little-endian `u32`.
    pub const GET_FUNC: u8 = 1;
    /// Set the bit delay in microseconds (`wValue`).
    pub const SET_DELAY: u8 = 2;
    /// Report the status of the last transfer.
    pub const GET_STATUS: u8 = 3;
    /// Perform one I2C message; ORed with the flags below.
    pub const I2C_IO: u8 = 4;
    /// This message starts a transaction (START).
    pub const I2C_IO_BEGIN: u8 = 1 << 0;
    /// This message ends a transaction (STOP).
    pub const I2C_IO_END: u8 = 1 << 1;
}

/// Values returned by [`cmd::GET_STATUS`].
pub mod status {
    /// Nothing transferred yet, or a buffered write whose result is not known
    /// yet.  The host treats anything but [`ADDRESS_NAK`] as success.
    pub const IDLE: u8 = 0;
    /// The address was acknowledged.
    pub const ADDRESS_ACK: u8 = 1;
    /// The address or a data byte was not acknowledged.
    pub const ADDRESS_NAK: u8 = 2;
}

/// Functionality bits reported by [`cmd::GET_FUNC`] (`I2C_FUNC_*` from
/// `include/uapi/linux/i2c.h`).
pub mod func {
    /// Full `I2C_RDWR` support: arbitrary messages with START/STOP control.
    pub const I2C: u32 = 0x0000_0001;
    /// 10-bit addresses.
    pub const TEN_BIT_ADDR: u32 = 0x0000_0002;
    /// `I2C_M_*` flags that bend the protocol (`I2C_M_NOSTART`, `I2C_M_REV_DIR_ADDR`, …).
    pub const PROTOCOL_MANGLING: u32 = 0x0000_0004;
    /// Packet error checking.
    pub const SMBUS_PEC: u32 = 0x0000_0008;
    /// SMBus quick command, the shortest bus probe.
    pub const SMBUS_QUICK: u32 = 0x0001_0000;
    /// SMBus read byte.
    pub const SMBUS_READ_BYTE: u32 = 0x0002_0000;
    /// SMBus write byte.
    pub const SMBUS_WRITE_BYTE: u32 = 0x0004_0000;
    /// SMBus read byte with a command byte.
    pub const SMBUS_READ_BYTE_DATA: u32 = 0x0008_0000;
    /// SMBus write byte with a command byte.
    pub const SMBUS_WRITE_BYTE_DATA: u32 = 0x0010_0000;
    /// SMBus read word with a command byte.
    pub const SMBUS_READ_WORD_DATA: u32 = 0x0020_0000;
    /// SMBus write word with a command byte.
    pub const SMBUS_WRITE_WORD_DATA: u32 = 0x0040_0000;
    /// SMBus process call.
    pub const SMBUS_PROC_CALL: u32 = 0x0080_0000;
    /// SMBus read block with a command byte.
    pub const SMBUS_READ_BLOCK_DATA: u32 = 0x0100_0000;
    /// SMBus write block with a command byte.
    pub const SMBUS_WRITE_BLOCK_DATA: u32 = 0x0200_0000;
    /// SMBus read I2C block.
    pub const SMBUS_READ_I2C_BLOCK: u32 = 0x0400_0000;
    /// SMBus write I2C block.
    pub const SMBUS_WRITE_I2C_BLOCK: u32 = 0x0800_0000;

    /// SMBus operations the kernel can emulate on top of plain I2C transfers.
    pub const SMBUS_EMUL: u32 = SMBUS_QUICK
        | SMBUS_READ_BYTE
        | SMBUS_WRITE_BYTE
        | SMBUS_READ_BYTE_DATA
        | SMBUS_WRITE_BYTE_DATA
        | SMBUS_READ_WORD_DATA
        | SMBUS_WRITE_WORD_DATA
        | SMBUS_PROC_CALL
        | SMBUS_WRITE_BLOCK_DATA
        | SMBUS_READ_I2C_BLOCK
        | SMBUS_WRITE_I2C_BLOCK
        | SMBUS_PEC;

    /// What a plain I2C master offers: I2C transfers plus emulated SMBus.
    pub const DEFAULT: u32 = I2C | SMBUS_EMUL;
}

/// Message flags the host passes in `wValue` (`I2C_M_*` from
/// `include/uapi/linux/i2c.h`).
pub mod flags {
    /// Read message; mirrors the transfer direction.
    pub const RD: u16 = 0x0001;
    /// 10-bit address (not supported).
    pub const TEN: u16 = 0x0010;
    /// Do not emit a START — continue with a repeated START.
    pub const NOSTART: u16 = 0x4000;
    /// Reverse the direction bit (protocol mangling, not supported).
    pub const REV_DIR_ADDR: u16 = 0x2000;
    /// Ignore a NAK (protocol mangling, not supported).
    pub const IGNORE_NAK: u16 = 0x1000;
    /// Do not acknowledge the last read byte (protocol mangling, not supported).
    pub const NO_RD_ACK: u16 = 0x0800;
    /// SMBus block read: read a length byte, then that many more bytes.
    ///
    /// Not supported — it needs the read phase to continue after the first
    /// byte, which the adapter cannot express.  Functionality therefore omits
    /// [`super::func::SMBUS_READ_BLOCK_DATA`].
    pub const RECV_LEN: u16 = 0x0400;
}

/// Default bit delay, matching the driver's default of 100 kHz.
const DEFAULT_DELAY_US: u16 = 10;

/// Bytes of a not-yet-finished write that may precede a repeated-start read.
pub const PENDING_WRITE_MAX: usize = 16;

// ─── Request decoding ────────────────────────────────────────────────────────

/// What a vendor request asks for.
///
/// [`cmd::ECHO`] … [`cmd::GET_STATUS`] are 0…3 and carry no flags, while
/// [`cmd::I2C_IO`] is 4 with [`cmd::I2C_IO_BEGIN`] (1) and [`cmd::I2C_IO_END`]
/// (2) ORed in, so its flags live in the low two bits of 4…7.  Decoding has to
/// look at the whole byte: masking those two bits off would fold `GET_FUNC`,
/// `SET_DELAY` and `GET_STATUS` onto `ECHO`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Opcode {
    /// Echo `wValue` back.
    Echo,
    /// Report the functionality.
    GetFunc,
    /// Set the bit delay.
    SetDelay,
    /// Report the last transfer's status.
    GetStatus,
    /// An I2C message, with the transaction markers it carried.
    I2cIo { begin: bool, end: bool },
    /// A request this adapter does not implement.
    Unknown,
}

impl Opcode {
    /// Decode `bRequest`.
    const fn decode(request: u8) -> Self {
        // The flags only ever appear on top of `I2C_IO` (4), which is what this
        // comparison checks before stripping them.
        const FLAGS: u8 = cmd::I2C_IO_BEGIN | cmd::I2C_IO_END;

        if request & !FLAGS == cmd::I2C_IO {
            Opcode::I2cIo {
                begin: request & cmd::I2C_IO_BEGIN != 0,
                end: request & cmd::I2C_IO_END != 0,
            }
        } else {
            match request {
                cmd::ECHO => Opcode::Echo,
                cmd::GET_FUNC => Opcode::GetFunc,
                cmd::SET_DELAY => Opcode::SetDelay,
                cmd::GET_STATUS => Opcode::GetStatus,
                _ => Opcode::Unknown,
            }
        }
    }
}

// The decoder is the protocol contract, so pin the whole command space here:
// a mistake would only show up against a real host otherwise.
const _: () = {
    assert!(matches!(Opcode::decode(cmd::ECHO), Opcode::Echo));
    assert!(matches!(Opcode::decode(cmd::GET_FUNC), Opcode::GetFunc));
    assert!(matches!(Opcode::decode(cmd::SET_DELAY), Opcode::SetDelay));
    assert!(matches!(Opcode::decode(cmd::GET_STATUS), Opcode::GetStatus));
    // `I2C_IO` with each combination of the two transaction flags.
    assert!(matches!(
        Opcode::decode(cmd::I2C_IO),
        Opcode::I2cIo {
            begin: false,
            end: false
        }
    ));
    assert!(matches!(
        Opcode::decode(cmd::I2C_IO | cmd::I2C_IO_BEGIN),
        Opcode::I2cIo {
            begin: true,
            end: false
        }
    ));
    assert!(matches!(
        Opcode::decode(cmd::I2C_IO | cmd::I2C_IO_END),
        Opcode::I2cIo {
            begin: false,
            end: true
        }
    ));
    assert!(matches!(
        Opcode::decode(cmd::I2C_IO | cmd::I2C_IO_BEGIN | cmd::I2C_IO_END),
        Opcode::I2cIo {
            begin: true,
            end: true
        }
    ));
    assert!(matches!(Opcode::decode(5), Opcode::I2cIo { .. }));
    assert!(matches!(Opcode::decode(0x80), Opcode::Unknown));
};

// ─── I2C bus ─────────────────────────────────────────────────────────────────

/// Failure of an [`I2cBus`] operation.
///
/// The protocol can only tell the host "not acknowledged", which the driver
/// turns into `-EREMOTEIO`; the distinction is kept for the application.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum I2cError {
    /// A device did not acknowledge its address or a data byte.
    NoAcknowledge,
    /// Arbitration lost, bus timeout, or a peripheral fault.
    BusFault,
}

/// I2C master the adapter drives.
///
/// Implement it around the bus driver at hand — see the `usb_i2c_tiny` example
/// for one wrapping a hardware I2C peripheral.  The class translates the host's
/// `i2c_msg` flags: a lone message becomes a [`write`](Self::write) or
/// [`read`](Self::read), a write followed by a read becomes a
/// [`write_read`](Self::write_read) with a repeated START between the phases.
pub trait I2cBus {
    /// Write `data` to the 7-bit `address`, START to STOP.
    ///
    /// An empty slice only probes the address, which is how bus scanning works.
    fn write(&mut self, address: u8, data: &[u8]) -> Result<(), I2cError>;

    /// Read from the 7-bit `address`, START to STOP.
    fn read(&mut self, address: u8, data: &mut [u8]) -> Result<(), I2cError>;

    /// Write `data`, then read `buffer` from the same address with a repeated
    /// START between the phases.
    fn write_read(&mut self, address: u8, data: &[u8], buffer: &mut [u8]) -> Result<(), I2cError>;

    /// Bit delay requested by `Set Delay`.
    ///
    /// 100 kHz corresponds to 10 µs, so a peripheral with a configurable baud
    /// rate can use `1_000_000 / microseconds`.  Bit-banged buses use it as the
    /// half-period; hardware that cannot change speed may ignore it.
    fn set_delay(&mut self, microseconds: u16) {
        let _ = microseconds;
    }
}

// ─── Message decoding ────────────────────────────────────────────────────────

/// One `i2c_msg` from the host, as far as the adapter cares.
#[derive(Clone, Copy)]
struct Message {
    /// 7-bit address (`wIndex`).
    address: u8,
    /// `i2c_msg` flags (`wValue`).
    flags: u16,
    /// First message of the transaction (`I2C_IO_BEGIN`).
    start: bool,
    /// Last message of the transaction (`I2C_IO_END`).
    end: bool,
}

impl Message {
    fn decode(request: &Request) -> Self {
        Self {
            address: request.index as u8,
            flags: request.value,
            start: request.request & cmd::I2C_IO_BEGIN != 0,
            end: request.request & cmd::I2C_IO_END != 0,
        }
    }

    /// The host asked for data (`I2C_M_RD`).
    fn is_read(&self) -> bool {
        self.flags & flags::RD != 0
    }

    /// The host used a 10-bit address.
    fn is_ten_bit(&self) -> bool {
        self.flags & flags::TEN != 0
    }
}

// ─── I2cTinyUsb ──────────────────────────────────────────────────────────────

/// i2c-tiny-usb adapter: wraps an [`I2cBus`] and serves the vendor protocol.
///
/// The adapter needs no endpoints of its own — everything travels over the
/// control pipe — so the type is only parameterised by the bus it drives.
pub struct I2cTinyUsb<I: I2cBus> {
    interface: InterfaceNumber,
    bus: I,
    functionality: u32,
    delay_us: u16,
    status: u8,
    pending_len: usize,
    pending: [u8; PENDING_WRITE_MAX],
    transfers: u32,
    failures: u32,
}

impl<I: I2cBus> I2cTinyUsb<I> {
    /// Create an adapter that reports [`func::DEFAULT`] functionality.
    ///
    /// Call it before building the device, like any other class.
    pub fn new<B: UsbBus>(alloc: &UsbBusAllocator<B>, bus: I) -> Self {
        Self::with_functionality(alloc, bus, func::DEFAULT)
    }

    /// Create an adapter that reports `functionality` for `Get Functionality`.
    ///
    /// Report what the bus really supports: bits for features that are not
    /// implemented make the kernel issue transfers the adapter has to stall.
    pub fn with_functionality<B: UsbBus>(
        alloc: &UsbBusAllocator<B>,
        bus: I,
        functionality: u32,
    ) -> Self {
        Self {
            interface: alloc.interface(),
            bus,
            functionality,
            delay_us: DEFAULT_DELAY_US,
            status: status::IDLE,
            pending_len: 0,
            pending: [0; PENDING_WRITE_MAX],
            transfers: 0,
            failures: 0,
        }
    }

    // ── Bus access ────────────────────────────────────────────────────────────

    /// The wrapped I2C bus.
    pub fn bus(&self) -> &I {
        &self.bus
    }

    /// The wrapped I2C bus, e.g. to use it from the application as well.
    pub fn bus_mut(&mut self) -> &mut I {
        &mut self.bus
    }

    // ── State ─────────────────────────────────────────────────────────────────

    /// Functionality reported to the host.
    pub fn functionality(&self) -> u32 {
        self.functionality
    }

    /// Change the reported functionality.
    pub fn set_functionality(&mut self, functionality: u32) {
        self.functionality = functionality;
    }

    /// Bit delay requested by the host, in microseconds.
    pub fn delay_us(&self) -> u16 {
        self.delay_us
    }

    /// Status reported by `Get Status`.
    pub fn status(&self) -> u8 {
        self.status
    }

    /// Transfers performed for the host.
    pub fn transfers(&self) -> u32 {
        self.transfers
    }

    /// Transfers that failed and were reported to the host as a NAK.
    pub fn failures(&self) -> u32 {
        self.failures
    }

    // ── Internals ─────────────────────────────────────────────────────────────

    /// Record the outcome of one bus operation.
    fn finish(&mut self, result: Result<(), I2cError>) {
        self.transfers = self.transfers.wrapping_add(1);
        match result {
            Ok(()) => self.status = status::ADDRESS_ACK,
            Err(_) => {
                self.failures = self.failures.wrapping_add(1);
                self.status = status::ADDRESS_NAK;
            }
        }
    }

    /// Give up on a request that the protocol cannot express: clear the
    /// transaction and stall so the host sees an error instead of wrong data.
    fn abandon(&mut self) {
        self.pending_len = 0;
        self.failures = self.failures.wrapping_add(1);
        self.status = status::ADDRESS_NAK;
    }

    /// Append `data` to the bytes of the transaction in progress, if they fit.
    fn append_pending(&mut self, data: &[u8]) -> bool {
        let end = self.pending_len + data.len();
        if end > PENDING_WRITE_MAX {
            return false;
        }
        self.pending[self.pending_len..end].copy_from_slice(data);
        self.pending_len = end;
        true
    }

    /// A write message: either a complete write, or the start of one that a
    /// later message finishes.
    fn write_message<B: UsbBus>(&mut self, msg: Message, xfer: ControlOut<'_, B>) {
        let data = xfer.data();

        if msg.is_ten_bit() {
            self.abandon();
            let _ = xfer.reject();
            return;
        }

        if msg.start {
            // A new transaction starts here, so nothing carries over.
            self.pending_len = 0;
        }

        if msg.start && msg.end {
            // Complete write in one message, including the address-only probe.
            let result = self.bus.write(msg.address, data);
            self.finish(result);
        } else if msg.end {
            // The buffered bytes and this message form one write: a peripheral
            // cannot pause between the two control transfers.
            if !self.append_pending(data) {
                self.abandon();
                let _ = xfer.reject();
                return;
            }
            let length = self.pending_len;
            let result = self.bus.write(msg.address, &self.pending[..length]);
            self.pending_len = 0;
            self.finish(result);
        } else if self.append_pending(data) {
            // Nothing on the bus yet: the write happens once the message that
            // ends the transaction arrives.
            self.status = status::IDLE;
        } else {
            self.abandon();
            let _ = xfer.reject();
            return;
        }

        let _ = xfer.accept();
    }

    /// A read message, which has to be the last one of a transaction.
    fn read_message<B: UsbBus>(&mut self, msg: Message, length: u16, xfer: ControlIn<'_, B>) {
        if msg.is_ten_bit() || !msg.end || msg.flags & flags::RECV_LEN != 0 {
            // A read in the middle of a transaction would need the bus to stay
            // open across control transfers, which a peripheral cannot do, and
            // an SMBus block read would need it to stay open within one.
            self.abandon();
            let _ = xfer.reject();
            return;
        }

        if msg.start {
            self.pending_len = 0;
        }

        let address = msg.address;
        let requested = length as usize;

        // With a buffered write in front this is the repeated-start pattern,
        // otherwise a plain read.  A failed read answers with zeros and reports
        // a NAK, so the host discards the data — the same trade the reference
        // firmware makes.
        let _ = xfer.accept(|buf| {
            let count = requested.min(buf.len());
            let result = if self.pending_len > 0 {
                let written = self.pending_len;
                let result =
                    self.bus
                        .write_read(address, &self.pending[..written], &mut buf[..count]);
                self.pending_len = 0;
                result
            } else if count == 0 {
                // A zero-length read (SMBus quick read) still probes the
                // address, which the bus driver only does for a write.
                self.bus.write(address, &[])
            } else {
                self.bus.read(address, &mut buf[..count])
            };

            match result {
                Ok(()) => {
                    self.finish(Ok(()));
                    Ok(count)
                }
                Err(error) => {
                    self.finish(Err(error));
                    buf[..count].fill(0);
                    Ok(count)
                }
            }
        });
    }
}

// ─── UsbClass<B> implementation ───────────────────────────────────────────────

impl<B: UsbBus, I: I2cBus> UsbClass<B> for I2cTinyUsb<I> {
    fn configuration_descriptors(&self, w: &mut DescriptorWriter) -> UsbResult<()> {
        // Control transfers only: the interface carries no endpoints.
        w.interface(self.interface, 0, class_code::VENDOR, 0x00, 0x00)
    }

    fn control_in(&mut self, xfer: ControlIn<'_, B>) {
        let req = *xfer.request();

        if req.request_type != RequestType::Vendor || req.recipient != Recipient::Interface {
            return;
        }

        // The I2C address travels in `wIndex`, so the interface number cannot be
        // compared here.
        match Opcode::decode(req.request) {
            Opcode::Echo => {
                let _ = xfer.accept_with(&req.value.to_le_bytes());
            }
            Opcode::GetFunc => {
                let _ = xfer.accept_with(&self.functionality.to_le_bytes());
            }
            Opcode::GetStatus => {
                let _ = xfer.accept_with(&[self.status]);
            }
            Opcode::I2cIo { .. } => self.read_message(Message::decode(&req), req.length, xfer),
            // Set Delay is an OUT request and the rest is not ours: leave the
            // request for another class, or let the device layer stall it.
            _ => {}
        }
    }

    fn control_out(&mut self, xfer: ControlOut<'_, B>) {
        let req = *xfer.request();

        if req.request_type != RequestType::Vendor || req.recipient != Recipient::Interface {
            return;
        }

        match Opcode::decode(req.request) {
            Opcode::SetDelay => {
                // The reference firmware never lets the delay become zero.
                let delay = req.value.max(1);
                self.delay_us = delay;
                self.bus.set_delay(delay);
                let _ = xfer.accept();
            }
            Opcode::I2cIo { .. } => {
                let msg = Message::decode(&req);
                if msg.is_read() {
                    // Reads arrive as control IN transfers.
                    let _ = xfer.reject();
                } else {
                    self.write_message(msg, xfer);
                }
            }
            // The remaining commands are IN requests; anything else is not ours.
            _ => {}
        }
    }

    fn configure(&mut self) {
        self.pending_len = 0;
        self.status = status::IDLE;
    }

    fn reset(&mut self) {
        // Same as `configure`, plus the counters.
        self.pending_len = 0;
        self.status = status::IDLE;
        self.transfers = 0;
        self.failures = 0;
    }
}
