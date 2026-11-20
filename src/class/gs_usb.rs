//! USB GS_USB class — CAN bus adapter protocol.
//!
//! Implements the Geschwister Schneider / candleLight USB-CAN protocol used by
//! the Linux `gs_usb` driver (`drivers/net/can/usb/gs_usb.c`).
//!
//! # Protocol overview
//!
//! - Interface class: vendor-specific (0xFF).
//! - Bulk IN / OUT endpoints carry [`GsHostFrame`] packets.
//! - Vendor control transfers handle configuration (bit timing, mode, etc.).
//!
//! # Example
//!
//! ```ignore
//! use my_hal_usb::{UsbBusAllocator, UsbDeviceBuilder};
//! use my_hal_usb::class::gs_usb::GsUsbClass;
//! use my_hal_usb::MyBus;
//!
//! let alloc = UsbBusAllocator::new(MyBus::new(/* peripheral */));
//! let mut gs_usb = GsUsbClass::new(&alloc);
//! let mut dev = UsbDeviceBuilder::new(&alloc, 0x1D50, 0x606F)
//!     .product("candleLight RP2350")
//!     .build();
//!
//! loop {
//!     if dev.poll(&mut [&mut gs_usb]) {
//!         // Handle CAN frames via gs_usb.try_recv() / gs_usb.try_send()
//!     }
//! }
//! ```

use crate::bus::{InterfaceNumber, UsbBus, UsbBusAllocator};
use crate::class::UsbClass;
use crate::control::{ControlIn, ControlOut, Recipient, RequestType};
use crate::descriptor::DescriptorWriter;
use crate::endpoint::{EndpointIn, EndpointOut};
use crate::error::{UsbError, UsbResult};

// ─── Vendor-specific control requests ─────────────────────────────────────────

/// GS_USB vendor control request codes (`bRequest`).
///
/// Mirrors the constants used by the Linux `gs_usb` driver and `gs_usb_rs` host library.
pub mod gs_usb_breq {
    /// Host frame format version (device→host: 1 byte; host→device: 4 bytes LE).
    pub const HOST_FORMAT: u8 = 0;
    /// Set CAN bit timing (host→device, writes `GsDeviceBitTiming`, 20 bytes).
    pub const BITTIMING: u8 = 1;
    /// Set CAN controller mode + flags (host→device, writes 8 bytes: u32 mode + u32 flags).
    pub const MODE: u8 = 2;
    /// Bus error reporting state (host→device, writes `u32`).
    pub const BERR: u8 = 3;
    /// Get bit-timing constraints (device→host, reads `GsDeviceBtConst`, 40 bytes).
    pub const BT_CONST: u8 = 4;
    /// Get device configuration (device→host, reads `GsDeviceConfig`, 12 bytes).
    pub const DEVICE_CONFIG: u8 = 5;
    /// Get/set timestamp value (device→host reads, host→device writes).
    pub const TIMESTAMP: u8 = 6;
    /// Identify device (e.g. blink LED).
    pub const IDENTIFY: u8 = 7;
    /// Get user ID (device→host).
    pub const GET_USER_ID: u8 = 8;
    /// Set user ID (host→device).
    pub const SET_USER_ID: u8 = 9;
    /// Set CAN FD data-phase bit timing (host→device, writes `GsDeviceBitTiming`, 20 bytes).
    pub const DATA_BITTIMING: u8 = 10;
    /// Get extended bit-timing constraints incl. CAN FD (device→host, 72 bytes).
    pub const BT_CONST_EXT: u8 = 11;
    /// Set termination resistor value (host→device).
    pub const SET_TERMINATION: u8 = 12;
    /// Get termination resistor value (device→host).
    pub const GET_TERMINATION: u8 = 13;
    /// Get CAN controller state + error counters (device→host, 12 bytes).
    pub const GET_STATE: u8 = 14;
}

// ─── CAN controller mode constants ────────────────────────────────────────────

/// CAN controller operational mode (`GS_USB_BREQ_MODE` first word).
pub mod gs_can_mode {
    /// Reset the CAN controller (no-op if already reset).
    pub const RESET: u32 = 0;
    /// Put the CAN controller into operational mode.
    pub const START: u32 = 1;
}

// ─── CAN mode flags (used in the second u32 of MODE control OUT) ──────────────

/// Mode flags for the second word of a `GS_USB_BREQ_MODE` request.
pub mod gs_can_mode_flags {
    /// Listen-only mode (no ACKs).
    pub const LISTEN_ONLY: u32 = 1 << 0;
    /// Loopback mode (for testing).
    pub const LOOP_BACK: u32 = 1 << 1;
    /// Triple sampling mode.
    pub const TRIPLE_SAMPLE: u32 = 1 << 2;
    /// One-shot mode (no retransmission).
    pub const ONE_SHOT: u32 = 1 << 3;
    /// Hardware timestamp mode.
    pub const HW_TIMESTAMP: u32 = 1 << 4;
    /// Identify / blink LED.
    pub const IDENTIFY: u32 = 1 << 5;
    /// User-ID mode.
    pub const USER_ID: u32 = 1 << 6;
    /// Pad packets to max packet size.
    pub const PAD_PKTS: u32 = 1 << 7;
    /// CAN-FD mode.
    pub const FD: u32 = 1 << 8;
}

// ─── CAN frame flags ──────────────────────────────────────────────────────────

/// Per-frame flags in [`GsCanFrame`](struct.GsCanFrame.html) (same bits as `canfd_frame.flags`).
pub mod gs_can_flag {
    /// RX overflow (data lost).
    pub const OVERFLOW: u8 = 1 << 0;
    /// CAN-FD frame.
    pub const FD: u8 = 1 << 1;
    /// Bit Rate Switch (FD frame transmitted at data bitrate).
    pub const BRS: u8 = 1 << 2;
    /// Error State Indicator.
    pub const ESI: u8 = 1 << 3;
}

// ─── Wire-format structures (packed C-compatible layout) ──────────────────────

/// A CAN frame as transmitted over USB bulk endpoints.
///
/// Wire format used by both the Linux `gs_usb` driver and candleLight firmware.
/// Must be `#[repr(C, packed)]` for correct binary compatibility.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GsHostFrame {
    /// Echo ID: host→device frames carry a sequence number echoed back in
    /// device→host TX-complete frames.
    pub echo_id: u32,
    /// CAN identifier (11-bit or 29-bit depending on flags).
    pub can_id: u32,
    /// Data Length Code (0–8 for classic CAN, 0–64 for CAN-FD).
    pub can_dlc: u8,
    /// Channel number (0 for single-channel devices).
    pub channel: u8,
    /// Flags (see [`gs_can_flag`]).
    pub flags: u8,
    /// Reserved, must be 0.
    pub reserved: u8,
    /// CAN data payload (classic CAN uses only first 8 bytes).
    pub data: [u8; 8],
}

impl GsHostFrame {
    /// Size of this struct on the wire.
    pub const SIZE: usize = core::mem::size_of::<Self>(); // 20 bytes

    /// Create a new frame from its raw bytes.
    ///
    /// # Safety
    ///
    /// `bytes` must be at least 20 bytes and contain a valid `GsHostFrame`
    /// wire encoding.
    pub fn from_bytes(bytes: &[u8]) -> Self {
        assert!(bytes.len() >= Self::SIZE);
        // Safety: GsHostFrame is repr(C, packed), plain-old-data.
        unsafe { core::ptr::read_unaligned(bytes.as_ptr() as *const Self) }
    }

    /// Serialise this frame into a byte slice.
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        // Safety: GsHostFrame is repr(C, packed), plain-old-data.
        unsafe { core::ptr::read_unaligned(self as *const Self as *const [u8; Self::SIZE]) }
    }
}

/// View a `repr(C, packed)` struct as a byte slice.
///
/// # Safety
///
/// `T` must be `repr(C, packed)` with no interior padding.
/// The returned slice is valid only as long as `t` is alive.
unsafe fn as_bytes<T: Sized>(t: &T) -> &[u8] {
    // Safety: caller guarantees T is repr(C, packed) POD.
    // The slice is constructed from a valid reference.
    unsafe { core::slice::from_raw_parts((t as *const T).cast::<u8>(), core::mem::size_of::<T>()) }
}

/// Read a `repr(C, packed)` struct from a byte slice (safe copy version).
///
/// Unlike the unsafe reference-based approach, this copies bytes into a
/// stack-local and returns by value, avoiding alignment UB.
fn read_packed<T: Copy>(bytes: &[u8]) -> T {
    assert!(bytes.len() >= core::mem::size_of::<T>());
    // Safety: we copy size_of::<T>() bytes from a valid slice into a
    // stack-local that is properly aligned for T (repr(C, packed) has
    // alignment 1).
    unsafe { core::ptr::read_unaligned(bytes.as_ptr() as *const T) }
}

// ─── Device configuration structs ─────────────────────────────────────────────

/// Response to `GS_USB_BREQ_DEVICE_CONFIG`.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GsDeviceConfig {
    /// Reserved (was `icount` in older protocol versions).
    pub reserved1: u8,
    /// Reserved.
    pub reserved2: u8,
    /// Reserved.
    pub reserved3: u8,
    /// Interface count (typically 1).
    pub icount: u8,
    /// Firmware version (user-defined).
    pub sw_version: u32,
    /// Hardware version (user-defined).
    pub hw_version: u32,
}

impl GsDeviceConfig {
    /// Size of the wire structure in bytes.
    pub const SIZE: usize = core::mem::size_of::<Self>(); // 12 bytes
}

/// Response to `GS_USB_BREQ_BT_CONST` — CAN bit-timing constraints.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GsDeviceBtConst {
    /// Feature flags (reserved).
    pub feature: u32,
    /// CAN peripheral clock frequency in Hz.
    pub fclk_can: u32,
    /// Minimum time segment 1 (in time quanta).
    pub tseg1_min: u32,
    /// Maximum time segment 1.
    pub tseg1_max: u32,
    /// Minimum time segment 2.
    pub tseg2_min: u32,
    /// Maximum time segment 2.
    pub tseg2_max: u32,
    /// Maximum Synchronisation Jump Width.
    pub sjw_max: u32,
    /// Minimum Baud Rate Prescaler.
    pub brp_min: u32,
    /// Maximum Baud Rate Prescaler.
    pub brp_max: u32,
    /// BRP increment (usually 1).
    pub brp_inc: u32,
}

impl GsDeviceBtConst {
    /// Size of the wire structure in bytes.
    pub const SIZE: usize = core::mem::size_of::<Self>(); // 40 bytes
}

/// Host→device request to set CAN bit timing.
#[repr(C, packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GsDeviceBitTiming {
    /// Propagation segment (time quanta).
    pub prop_seg: u32,
    /// Phase segment 1 (time quanta).
    pub phase_seg1: u32,
    /// Phase segment 2 (time quanta).
    pub phase_seg2: u32,
    /// Synchronisation Jump Width.
    pub sjw: u32,
    /// Baud Rate Prescaler.
    pub brp: u32,
}

impl GsDeviceBitTiming {
    /// Size of the wire structure in bytes.
    pub const SIZE: usize = core::mem::size_of::<Self>(); // 20 bytes
}

// ─── GsUsbClass ──────────────────────────────────────────────────────────────

/// USB GS_USB class implementing the candleLight / Linux `gs_usb` protocol.
///
/// Provides a bidirectional CAN channel via USB bulk endpoints plus
/// vendor-specific control requests for configuration.
pub struct GsUsbClass<'a, B: UsbBus> {
    interface: InterfaceNumber,
    bulk_in: EndpointIn<'a, B>,
    bulk_out: EndpointOut<'a, B>,
    device_config: GsDeviceConfig,
    bt_const: GsDeviceBtConst,
    /// Currently requested bit timing.
    bit_timing: GsDeviceBitTiming,
    /// CAN controller mode (0 = reset, 1 = start).
    mode: u32,
    /// Mode flags (GS_CAN_MODE_FLAGS_*).
    mode_flags: u32,
    configured: bool,
}

impl<'a, B: UsbBus> GsUsbClass<'a, B> {
    /// Create a new GS_USB class with default device configuration.
    ///
    /// Call [`set_device_config`](Self::set_device_config) and
    /// [`set_bt_const`](Self::set_bt_const) before connecting to set
    /// device-specific parameters.
    pub fn new(alloc: &'a UsbBusAllocator<B>) -> Self {
        let interface = alloc.interface();
        let bulk_in = alloc.bulk(64);
        let bulk_out = alloc.bulk::<crate::endpoint::Out>(64);

        Self {
            interface,
            bulk_in,
            bulk_out,
            device_config: GsDeviceConfig {
                icount: 1,
                sw_version: 0x0001_0000,
                hw_version: 0x0001_0000,
                ..Default::default()
            },
            bt_const: GsDeviceBtConst {
                // Feature flags: report capabilities the firmware supports.
                feature: gs_can_mode_flags::LISTEN_ONLY
                    | gs_can_mode_flags::LOOP_BACK
                    | gs_can_mode_flags::ONE_SHOT
                    | gs_can_mode_flags::IDENTIFY,
                fclk_can: 40_000_000, // 40 MHz – compatible with candleLight / gs_usb_rs
                tseg1_min: 1,
                tseg1_max: 256,
                tseg2_min: 1,
                tseg2_max: 128,
                sjw_max: 128,
                brp_min: 1,
                brp_max: 1024,
                brp_inc: 1,
            },
            bit_timing: GsDeviceBitTiming::default(),
            mode: gs_can_mode::RESET,
            mode_flags: 0,
            configured: false,
        }
    }

    // ── Configuration setters (call before USB enumeration) ──────────────────

    /// Set the device configuration reported to the host.
    pub fn set_device_config(&mut self, cfg: GsDeviceConfig) {
        self.device_config = cfg;
    }

    /// Set the bit-timing constraints reported to the host.
    pub fn set_bt_const(&mut self, c: GsDeviceBtConst) {
        self.bt_const = c;
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /// `true` when the device is configured by the host.
    pub fn is_configured(&self) -> bool {
        self.configured
    }

    /// Current CAN controller mode.
    pub fn mode(&self) -> u32 {
        self.mode
    }

    /// Current mode flags.
    pub fn mode_flags(&self) -> u32 {
        self.mode_flags
    }

    /// Current bit timing parameters.
    pub fn bit_timing(&self) -> &GsDeviceBitTiming {
        &self.bit_timing
    }

    /// Attempt to read a CAN frame from the host (bulk OUT).
    ///
    /// Returns `Ok(Some(frame))` when a frame is available,
    /// `Ok(None)` when no data is waiting, or an error.
    pub fn try_recv(&self) -> UsbResult<Option<GsHostFrame>> {
        if !self.configured {
            return Err(UsbError::InvalidState);
        }
        let mut buf = [0u8; GsHostFrame::SIZE];
        match self.bulk_out.read(&mut buf) {
            Ok(n) if n == GsHostFrame::SIZE => {
                let frame = GsHostFrame::from_bytes(&buf);
                Ok(Some(frame))
            }
            Ok(_) => Ok(None), // Short packet — ignore
            Err(UsbError::WouldBlock) => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// Send a CAN frame to the host (bulk IN).
    ///
    /// Returns `Ok(())` on success, or an error (e.g. `WouldBlock` if the
    /// previous packet hasn't been sent yet).
    pub fn try_send(&self, frame: &GsHostFrame) -> UsbResult<()> {
        if !self.configured {
            return Err(UsbError::InvalidState);
        }
        let bytes = frame.to_bytes();
        self.bulk_in.write(&bytes)?;
        Ok(())
    }
}

// ─── UsbClass<B> implementation ───────────────────────────────────────────────

impl<B: UsbBus> UsbClass<B> for GsUsbClass<'_, B> {
    fn configuration_descriptors(&self, w: &mut DescriptorWriter) -> UsbResult<()> {
        // Vendor-specific interface with two bulk endpoints.
        w.interface(
            self.interface,
            2,    // IN + OUT bulk
            0xFF, // bInterfaceClass: vendor-specific
            0x00, // bInterfaceSubClass
            0x00, // bInterfaceProtocol
        )?;

        // Bulk IN endpoint (device→host CAN frames).
        w.endpoint(
            self.bulk_in.address(),
            self.bulk_in.ep_type(),
            self.bulk_in.max_packet_size(),
            0, // interval (ignored for bulk)
        )?;

        // Bulk OUT endpoint (host→device CAN frames).
        w.endpoint(
            self.bulk_out.address(),
            self.bulk_out.ep_type(),
            self.bulk_out.max_packet_size(),
            0,
        )?;

        Ok(())
    }

    fn control_in(&mut self, xfer: ControlIn<'_, B>) {
        let req = xfer.request();

        // Only handle vendor requests to this interface.
        if req.request_type != RequestType::Vendor {
            return;
        }
        // Accept both Interface and Device recipients (candleLight uses both).
        if req.recipient != Recipient::Interface && req.recipient != Recipient::Device {
            return;
        }

        match req.request {
            gs_usb_breq::HOST_FORMAT => {
                // Linux driver expects at least version 1.
                let _ = xfer.accept_with(&[1u8]);
            }
            gs_usb_breq::DEVICE_CONFIG => {
                let raw = unsafe { as_bytes(&self.device_config) };
                let len = core::cmp::min(raw.len(), req.length as usize);
                let _ = xfer.accept_with(&raw[..len]);
            }
            gs_usb_breq::BT_CONST => {
                let raw = unsafe { as_bytes(&self.bt_const) };
                let len = core::cmp::min(raw.len(), req.length as usize);
                let _ = xfer.accept_with(&raw[..len]);
            }
            gs_usb_breq::TIMESTAMP => {
                // Return zero timestamp (device doesn't provide HW timestamps).
                let ts: u32 = 0;
                let raw = unsafe { as_bytes(&ts) };
                let _ = xfer.accept_with(raw);
            }
            gs_usb_breq::GET_USER_ID => {
                // Return zero (no user ID set).
                let uid: u32 = 0;
                let raw = unsafe { as_bytes(&uid) };
                let _ = xfer.accept_with(raw);
            }
            gs_usb_breq::BT_CONST_EXT => {
                // Device doesn't support CAN-FD; reject gracefully.
                let _ = xfer.reject();
            }
            gs_usb_breq::GET_TERMINATION => {
                // Return zero (no termination control).
                let term: u32 = 0;
                let raw = unsafe { as_bytes(&term) };
                let _ = xfer.accept_with(raw);
            }
            gs_usb_breq::GET_STATE => {
                // Return CAN state + error counters as 3 × u32.
                // [state(u32), rx_err_cnt(u32), tx_err_cnt(u32)]
                let state: [u32; 3] = [
                    match self.mode {
                        gs_can_mode::START => 0, // ERROR_ACTIVE (simplified)
                        _ => 4,                  // STOPPED
                    },
                    0, // rx error count
                    0, // tx error count
                ];
                let raw = unsafe { as_bytes(&state) };
                let len = core::cmp::min(raw.len(), req.length as usize);
                let _ = xfer.accept_with(&raw[..len]);
            }
            _ => {
                let _ = xfer.reject();
            }
        }
    }

    fn control_out(&mut self, xfer: ControlOut<'_, B>) {
        let req = xfer.request();

        if req.request_type != RequestType::Vendor {
            return;
        }
        if req.recipient != Recipient::Interface && req.recipient != Recipient::Device {
            return;
        }

        let data = xfer.data();

        match req.request {
            gs_usb_breq::HOST_FORMAT => {
                // Host sends byte-order hint (0x0000BEEF LE). Accept silently.
                let _ = xfer.accept();
            }
            gs_usb_breq::BITTIMING => {
                if data.len() >= GsDeviceBitTiming::SIZE {
                    self.bit_timing = read_packed(&data[..GsDeviceBitTiming::SIZE]);
                }
                let _ = xfer.accept();
            }
            gs_usb_breq::MODE => {
                // gs_usb_rs sends DeviceMode: [mode(u32 LE), flags(u32 LE)] = 8 bytes.
                // Linux kernel sends just mode (u32 LE) = 4 bytes.
                if data.len() >= 4 {
                    self.mode = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                }
                if data.len() >= 8 {
                    self.mode_flags = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
                } else {
                    self.mode_flags = 0;
                }
                let _ = xfer.accept();
            }
            gs_usb_breq::BERR => {
                // Bus-error reporting: accept the value but implementation
                // is hardware-specific.
                let _ = xfer.accept();
            }
            gs_usb_breq::TIMESTAMP => {
                // Host sets timestamp — accept silently.
                let _ = xfer.accept();
            }
            gs_usb_breq::IDENTIFY => {
                // Blink LED / identify: hardware-specific, accept for now.
                let _ = xfer.accept();
            }
            gs_usb_breq::SET_USER_ID => {
                // Accept silently (no persistent storage).
                let _ = xfer.accept();
            }
            gs_usb_breq::DATA_BITTIMING => {
                // CAN-FD data-phase timing: accept but ignore (no FD support).
                let _ = xfer.accept();
            }
            gs_usb_breq::SET_TERMINATION => {
                // Accept silently (no termination control).
                let _ = xfer.accept();
            }
            _ => {
                let _ = xfer.reject();
            }
        }
    }

    fn configure(&mut self) {
        self.configured = true;
        self.mode = gs_can_mode::RESET;
    }

    fn deconfigure(&mut self) {
        self.configured = false;
        self.mode = gs_can_mode::RESET;
    }

    fn reset(&mut self) {
        self.configured = false;
        self.mode = gs_can_mode::RESET;
    }
}
