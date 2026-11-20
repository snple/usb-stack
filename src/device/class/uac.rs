//! USB Audio Class 1.0 (UAC 1.0).
//!
//! Implements "Universal Serial Bus Device Class Definition for Audio Devices",
//! Release 1.0.  Supports one input stream (e.g. microphone) and/or one output
//! stream (e.g. speaker / headphones) with discrete or continuous sampling rates.
//!
//! # Example — stereo speaker with microphone
//!
//! ```ignore
//! use my_hal_usb::{UsbBusAllocator, UsbDeviceBuilder};
//! use my_hal_usb::class::uac::{
//!     AudioClassBuilder, Format, StreamConfig, TerminalType,
//! };
//! use my_hal_usb::MyBus;
//!
//! static RATES_48K: [u32; 1] = [48000];
//!
//! let alloc = UsbBusAllocator::new(MyBus::new(/* peripheral */));
//! let mut audio = AudioClassBuilder::new()
//!     .input(
//!         StreamConfig::new_discrete(
//!             Format::S16le, 1, &RATES_48K, TerminalType::InMicrophone,
//!         ).unwrap())
//!     .output(
//!         StreamConfig::new_discrete(
//!             Format::S24le, 2, &RATES_48K, TerminalType::OutSpeaker,
//!         ).unwrap())
//!     .build(&alloc);
//!
//! let mut dev = UsbDeviceBuilder::new(&alloc, 0x1234, 0x5678)
//!     .product("USB Audio")
//!     .composite_with_iads()
//!     .build();
//!
//! loop {
//!     dev.poll(&mut [&mut audio]);
//!     let mut buf = [0u8; 288];
//!     if let Ok(n) = audio.read(&mut buf) {
//!         // send to DAC …
//!     }
//!     let pcm = [0u8; 96];
//!     let _ = audio.write(&pcm);
//! }
//! ```

use crate::device::bus::{InterfaceNumber, UsbBus, UsbBusAllocator};
use crate::device::class::UsbClass;
use crate::device::control::{ControlIn, ControlOut};
use crate::device::descriptor::DescriptorWriter;
use crate::device::endpoint::{EndpointIn, EndpointOut};
use crate::device::error::{UsbError, UsbResult};
use crate::endpoint::{EndpointAddress, EndpointType, IsoSynchronization, IsoUsage};
use crate::request::{Recipient, RequestType};

// ─── Terminal types (USB Audio 1.0) ──────────────────────────────────────────

/// USB Audio Terminal Types from "Universal Serial Bus Device Class Definition
/// for Terminal Types, Release 1.0".
#[rustfmt::skip]
#[repr(u16)]
#[non_exhaustive]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum TerminalType {
    // USB Terminal Types
    /// Undefined USB terminal.
    UsbUndefined  = 0x0100,
    /// USB streaming terminal.
    UsbStreaming  = 0x0101,
    /// Vendor-specific USB terminal.
    UsbVendor     = 0x01FF,

    // Input Terminal Types
    /// Undefined input terminal.
    InUndefined                   = 0x0200,
    /// Microphone input terminal.
    InMicrophone                  = 0x0201,
    /// Desktop microphone input terminal.
    InDesktopMicrophone           = 0x0202,
    /// Personal microphone input terminal.
    InPersonalMicrophone          = 0x0203,
    /// Omni-directional microphone input terminal.
    InOmniDirectionalMicrophone   = 0x0204,
    /// Microphone array input terminal.
    InMicrophoneArray             = 0x0205,
    /// Processing microphone array input terminal.
    InProcessingMicrophoneArray   = 0x0206,

    // Output Terminal Types
    /// Undefined output terminal.
    OutUndefined                  = 0x0300,
    /// Speaker output terminal.
    OutSpeaker                    = 0x0301,
    /// Headphones output terminal.
    OutHeadphones                 = 0x0302,
    /// Head-mounted display audio output terminal.
    OutHeadMountedDisplayAudio    = 0x0303,
    /// Desktop speaker output terminal.
    OutDesktopSpeaker             = 0x0304,
    /// Room speaker output terminal.
    OutRoomSpeaker                = 0x0305,
    /// Communication speaker output terminal.
    OutCommunicationSpeaker       = 0x0306,
    /// Low-frequency-effects speaker output terminal.
    OutLowFrequencyEffectsSpeaker = 0x0307,

    // External Terminal Types
    /// Undefined external terminal.
    ExtUndefined               = 0x0600,
    /// Analog connector external terminal.
    ExtAnalogConnector         = 0x0601,
    /// Digital audio interface external terminal.
    ExtDigitalAudioInterface   = 0x0602,
    /// Line connector external terminal.
    ExtLineConnector           = 0x0603,
    /// Legacy audio connector external terminal.
    ExtLegacyAudioConnector    = 0x0604,
    /// S/PDIF connector external terminal.
    ExtSpdifConnector          = 0x0605,
    /// IEEE 1394 DA stream external terminal.
    Ext1394DaStream            = 0x0606,
    /// IEEE 1394 DV stream soundtrack external terminal.
    Ext1394DvStreamSoundtrack  = 0x0607,
}

impl From<TerminalType> for u16 {
    fn from(t: TerminalType) -> u16 {
        t as u16
    }
}

// ─── UAC class-specific constants ─────────────────────────────────────────────

/// Audio Interface Class Code (USB-IF).
pub const UAC_CLASS: u8 = 0x01;
/// Audio Control subclass.
pub const UAC_SUBCLASS_CONTROL: u8 = 0x01;
/// Audio Streaming subclass.
pub const UAC_SUBCLASS_STREAMING: u8 = 0x02;

/// Class-Specific Interface descriptor type (CS_INTERFACE).
const CS_INTERFACE: u8 = 0x24;
/// Class-Specific Endpoint descriptor type (CS_ENDPOINT).
const CS_ENDPOINT: u8 = 0x25;

// AC descriptor subtypes
const AC_HEADER: u8 = 0x01;
const AC_INPUT_TERMINAL: u8 = 0x02;
const AC_OUTPUT_TERMINAL: u8 = 0x03;
const AC_FEATURE_UNIT: u8 = 0x06;

// AS descriptor subtypes
const AS_GENERAL: u8 = 0x01;
const AS_FORMAT_TYPE: u8 = 0x02;

// EP descriptor subtypes
const EP_GENERAL: u8 = 0x01;

// Feature Unit control selectors
const FU_MUTE: u8 = 0x01;
const FU_VOLUME: u8 = 0x02;

// Audio Class-Specific Request Codes (UAC 1.0 §5.2.2)
const CUR_SET: u8 = 0x01;
const CUR_GET: u8 = 0x81;
const MIN_GET: u8 = 0x82;
const MAX_GET: u8 = 0x83;
const RES_GET: u8 = 0x84;

/// PCM format tag (Audio Data Format Type I).
const PCM_TAG: u16 = 0x0001;
/// Format Type I code.
const FORMAT_TYPE_I: u8 = 0x01;

/// Terminal/Unit IDs used in the topology.
///
/// Output (speaker) topology: IT(USB,2) → FU(3) → OT(Speaker,4)
/// Input (mic) topology:     IT(Mic,5)  → FU(6) → OT(USB,7)
const TERM_ID_SPEAKER_USB: u8 = 0x02;
const TERM_ID_SPEAKER_FU: u8 = 0x03;
const TERM_ID_SPEAKER_OUT: u8 = 0x04;
const TERM_ID_MIC_IN: u8 = 0x05;
const TERM_ID_MIC_FU: u8 = 0x06;
const TERM_ID_MIC_USB: u8 = 0x07;

/// Maximum isochronous endpoint packet size for full-speed USB.
const MAX_ISO_SIZE: u32 = 1023;

// ─── Audio format ────────────────────────────────────────────────────────────

/// Audio sample format.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    /// Signed 16-bit little-endian (2 bytes/subframe).
    S16le,
    /// Signed 24-bit little-endian (3 bytes/subframe).
    S24le,
}

impl Format {
    const fn bytes(self) -> u8 {
        match self {
            Format::S16le => 2,
            Format::S24le => 3,
        }
    }
    const fn bits(self) -> u8 {
        match self {
            Format::S16le => 16,
            Format::S24le => 24,
        }
    }
}

// ─── Sampling rates ──────────────────────────────────────────────────────────

/// Supported sampling rates for an audio stream.
#[derive(Debug, Clone)]
pub enum Rates {
    /// Continuous range `[min, max]` in Hz (0 = continuous in descriptor).
    Continuous(u32, u32),
    /// Discrete set of frequencies in Hz.
    Discrete(&'static [u32]),
}

// ─── StreamConfig ────────────────────────────────────────────────────────────

/// Configuration for one audio stream (input or output).
#[derive(Debug, Clone)]
pub struct StreamConfig {
    /// Sample format.
    pub format: Format,
    /// Number of channels (1 = mono, 2 = stereo, …).
    pub channels: u8,
    /// Supported sampling rates.
    pub rates: Rates,
    /// Terminal type for this stream.
    pub terminal_type: TerminalType,
    /// Computed maximum isochronous endpoint packet size in bytes.
    pub ep_size: u16,
}

impl StreamConfig {
    /// Create with one or more discrete sampling rates.
    ///
    /// `rates` must be a `&'static [u32]` for lifetime simplicity in `no_std`
    /// embedded contexts.
    pub fn new_discrete(
        format: Format,
        channels: u8,
        rates: &'static [u32],
        terminal_type: TerminalType,
    ) -> Result<Self, UsbError> {
        let max_rate = *rates.iter().max().unwrap_or(&48000);
        let ep_size = Self::calc_ep_size(format, channels, max_rate)?;
        Ok(Self {
            format,
            channels,
            rates: Rates::Discrete(rates),
            terminal_type,
            ep_size,
        })
    }

    /// Create with a continuous frequency range `[min_rate, max_rate]`.
    pub fn new_continuous(
        format: Format,
        channels: u8,
        min_rate: u32,
        max_rate: u32,
        terminal_type: TerminalType,
    ) -> Result<Self, UsbError> {
        if min_rate >= max_rate {
            return Err(UsbError::InvalidState);
        }
        let ep_size = Self::calc_ep_size(format, channels, max_rate)?;
        Ok(Self {
            format,
            channels,
            rates: Rates::Continuous(min_rate, max_rate),
            terminal_type,
            ep_size,
        })
    }

    fn calc_ep_size(format: Format, channels: u8, max_rate: u32) -> Result<u16, UsbError> {
        let octets_per_frame = channels as u32 * format.bytes() as u32;
        let size = octets_per_frame * max_rate / 1000;
        if size > MAX_ISO_SIZE {
            return Err(UsbError::BufferOverflow);
        }
        Ok(size as u16)
    }

    fn channel_config(&self) -> u16 {
        match self.channels {
            1 => 0x0001, // L
            2 => 0x0003, // L + R
            4 => 0x0033, // L + R + LS + RS
            6 => 0x003F, // L + R + C + LFE + LS + RS
            8 => 0x00FF, // L + R + C + LFE + LS + RS + LC + RC
            _ => 0x0003, // default: stereo
        }
    }
}

// ─── AudioClassBuilder ───────────────────────────────────────────────────────

/// Builder for [`AudioClass`].
///
/// Use the fluent API to configure input/output streams, then call
/// [`build`](Self::build).
#[derive(Default)]
pub struct AudioClassBuilder {
    input: Option<StreamConfig>,
    output: Option<StreamConfig>,
}

impl AudioClassBuilder {
    /// Create a new builder with no streams configured.
    pub fn new() -> Self {
        Self {
            input: None,
            output: None,
        }
    }

    /// Configure an input stream (device → host, e.g. microphone).
    ///
    /// Calling multiple times replaces the previous configuration.
    pub fn input(mut self, cfg: StreamConfig) -> Self {
        self.input = Some(cfg);
        self
    }

    /// Configure an output stream (host → device, e.g. speaker / headphones).
    ///
    /// Calling multiple times replaces the previous configuration.
    pub fn output(mut self, cfg: StreamConfig) -> Self {
        self.output = Some(cfg);
        self
    }

    /// Build the [`AudioClass`], allocating endpoints and interfaces from `alloc`.
    pub fn build<B: UsbBus>(self, alloc: &UsbBusAllocator<B>) -> AudioClass<'_, B> {
        let control_iface = alloc.interface();

        let (in_ep, in_iface, in_cfg, in_alt) = if let Some(cfg) = self.input {
            let iface = alloc.interface();
            let ep = alloc.isochronous::<crate::endpoint::In>(
                IsoSynchronization::Asynchronous,
                IsoUsage::Data,
                cfg.ep_size,
                1,
            );
            (Some(ep), iface, Some(cfg), 0u8)
        } else {
            (None, InterfaceNumber::new(0), None, 0u8)
        };

        let (out_ep, out_iface, out_cfg, out_alt) = if let Some(cfg) = self.output {
            let iface = alloc.interface();
            let ep = alloc.isochronous::<crate::endpoint::Out>(
                IsoSynchronization::Adaptive,
                IsoUsage::Data,
                cfg.ep_size,
                1,
            );
            (Some(ep), iface, Some(cfg), 0u8)
        } else {
            (None, InterfaceNumber::new(0), None, 0u8)
        };

        AudioClass {
            control_iface,
            in_ep,
            in_iface,
            in_cfg,
            in_alt,
            out_ep,
            out_iface,
            out_cfg,
            out_alt,
            configured: false,
        }
    }
}

// ─── AudioClass ──────────────────────────────────────────────────────────────

/// USB Audio Class 1.0 device class.
///
/// Supports one streaming input (microphone / line-in) and/or one streaming
/// output (speaker / headphones).  Built via [`AudioClassBuilder`].
///
/// The host activates a stream by selecting alternate setting 1
/// (via `SET_INTERFACE`), and deactivates it by selecting alternate setting 0.
pub struct AudioClass<'a, B: UsbBus> {
    // Control interface
    control_iface: InterfaceNumber,

    // Input stream (device → host)
    in_ep: Option<EndpointIn<'a, B>>,
    in_iface: InterfaceNumber,
    in_cfg: Option<StreamConfig>,
    in_alt: u8,

    // Output stream (host → device)
    out_ep: Option<EndpointOut<'a, B>>,
    out_iface: InterfaceNumber,
    out_cfg: Option<StreamConfig>,
    out_alt: u8,

    configured: bool,
}

impl<'a, B: UsbBus> AudioClass<'a, B> {
    /// Read audio data from the host (output / speaker stream).
    ///
    /// Returns the number of bytes read into `buf`, or
    /// [`UsbError::WouldBlock`] when no isochronous packet is available.
    /// Also returns [`UsbError::InvalidState`] if the stream is not
    /// configured or not active (alt setting 0).
    pub fn read(&self, buf: &mut [u8]) -> UsbResult<usize> {
        let ep = self.out_ep.as_ref().ok_or(UsbError::InvalidState)?;
        if !self.configured || self.out_alt == 0 {
            return Err(UsbError::InvalidState);
        }
        ep.read(buf)
    }

    /// Write audio data to the host (input / microphone stream).
    ///
    /// Returns the number of bytes written, or [`UsbError::WouldBlock`]
    /// when the previous packet has not yet been sent.  Also returns
    /// [`UsbError::InvalidState`] if the stream is not configured or
    /// not active (alt setting 0).
    pub fn write(&self, buf: &[u8]) -> UsbResult<usize> {
        let ep = self.in_ep.as_ref().ok_or(UsbError::InvalidState)?;
        if !self.configured || self.in_alt == 0 {
            return Err(UsbError::InvalidState);
        }
        ep.write(buf)
    }

    /// Current alternate setting of the input (microphone) stream.
    /// Returns `None` if no input stream was configured.
    pub fn input_alt_setting(&self) -> Option<u8> {
        self.in_cfg.as_ref().map(|_| self.in_alt)
    }

    /// Current alternate setting of the output (speaker) stream.
    /// Returns `None` if no output stream was configured.
    pub fn output_alt_setting(&self) -> Option<u8> {
        self.out_cfg.as_ref().map(|_| self.out_alt)
    }

    /// `true` when the device has been fully configured by the host.
    pub fn is_configured(&self) -> bool {
        self.configured
    }

    /// `true` when the output (speaker) stream is active (alt setting ≠ 0).
    pub fn output_active(&self) -> bool {
        self.configured && self.out_alt != 0
    }

    /// `true` when the input (microphone) stream is active (alt setting ≠ 0).
    pub fn input_active(&self) -> bool {
        self.configured && self.in_alt != 0
    }

    // ── Descriptor helpers ──────────────────────────────────────────────────

    fn streaming_count(&self) -> u8 {
        self.in_cfg.is_some() as u8 + self.out_cfg.is_some() as u8
    }

    fn write_ac_header(&self, w: &mut DescriptorWriter) -> UsbResult<()> {
        let n = self.streaming_count();
        let total_len = self.ac_total_length();
        let mut hdr = [
            AC_HEADER,
            0x00,
            0x01, // bcdADC = 1.00
            total_len as u8,
            (total_len >> 8) as u8, // wTotalLength
            n,                      // bInCollection
            0x00,
            0x00, // baInterfaceNr[0..1]
        ];
        let mut idx = 6usize;
        if self.in_cfg.is_some() {
            hdr[idx] = self.in_iface.into();
            idx += 1;
        }
        if self.out_cfg.is_some() {
            hdr[idx] = self.out_iface.into();
            idx += 1;
        }
        w.write(CS_INTERFACE, &hdr[..idx])
    }

    /// Compute wTotalLength for the AC interface header — sum of all
    /// class-specific descriptor sizes within the AudioControl interface.
    fn ac_total_length(&self) -> u16 {
        let n = self.streaming_count() as u16;
        // AC Header: 2 (bLength+bType) + 6 (fixed body) + n (baInterfaceNr)
        let mut total: u16 = 8 + n;
        for cfg in [&self.in_cfg, &self.out_cfg]
            .iter()
            .filter_map(|c| c.as_ref())
        {
            // Input Terminal: 2 + 10 = 12
            total += 12;
            // Feature Unit: 2 + (6 + channels) = 8 + channels
            total += 8 + cfg.channels as u16;
            // Output Terminal: 2 + 7 = 9
            total += 9;
        }
        total
    }

    /// Return the number of channels for a given Feature Unit ID,
    /// or 0 if the ID doesn't match any of our units.
    fn fu_channels(&self, unit_id: u8) -> u8 {
        if unit_id == TERM_ID_SPEAKER_FU {
            self.out_cfg.as_ref().map_or(0, |c| c.channels)
        } else if unit_id == TERM_ID_MIC_FU {
            self.in_cfg.as_ref().map_or(0, |c| c.channels)
        } else {
            0
        }
    }

    fn write_terminal_descriptors(
        &self,
        w: &mut DescriptorWriter,
        cfg: &StreamConfig,
        input_term_id: u8,
        fu_id: u8,
        output_term_id: u8,
        is_input: bool,
    ) -> UsbResult<()> {
        let tt: u16 = cfg.terminal_type.into();
        let cc = cfg.channel_config();

        // ── Input Terminal (12 bytes) ────────────────────────────────────
        let it_type = if is_input {
            tt
        } else {
            TerminalType::UsbStreaming.into()
        };
        w.write(
            CS_INTERFACE,
            &[
                AC_INPUT_TERMINAL,
                input_term_id,
                it_type as u8,
                (it_type >> 8) as u8,
                0x00,            // bAssocTerminal
                cfg.channels,    // bNrChannels
                cc as u8,        // wChannelConfig (lo)
                (cc >> 8) as u8, // wChannelConfig (hi)
                0x00,            // iChannelNames
                0x00,            // iTerminal
            ],
        )?;

        // ── Feature Unit (7 + ch + 1 bytes body) ─────────────────────────
        self.write_feature_unit(w, cfg, fu_id, input_term_id)?;

        // ── Output Terminal (9 bytes) ────────────────────────────────────
        let ot_type = if is_input {
            TerminalType::UsbStreaming.into()
        } else {
            tt
        };
        w.write(
            CS_INTERFACE,
            &[
                AC_OUTPUT_TERMINAL,
                output_term_id,
                ot_type as u8,
                (ot_type >> 8) as u8,
                0x00,  // bAssocTerminal
                fu_id, // bSourceID → comes from Feature Unit
                0x00,  // iTerminal
            ],
        )
    }

    /// Write a Feature Unit descriptor with Mute + Volume controls.
    ///
    /// The body is: subtype(1) + unitID(1) + sourceID(1) + controlSize(1)
    ///            + ch+1 bitmap bytes + iFeature(1)
    /// Total body = 4 + (ch+1) + 1 = 6 + ch
    /// Full descriptor = 2 + body
    fn write_feature_unit(
        &self,
        w: &mut DescriptorWriter,
        cfg: &StreamConfig,
        fu_id: u8,
        source_id: u8,
    ) -> UsbResult<()> {
        let ch = cfg.channels as usize;
        // Max channels we support in the stack-allocated buffer: 8
        let mut body = [0u8; 16]; // enough for up to 8 ch (6+8=14 bytes)
        body[0] = AC_FEATURE_UNIT;
        body[1] = fu_id;
        body[2] = source_id;
        body[3] = 1; // bControlSize: 1 byte per control
        // Master channel: Mute + Volume
        body[4] = FU_MUTE | FU_VOLUME; // 0x03
        // Per-channel: Volume only
        for i in 0..ch {
            body[5 + i] = FU_VOLUME; // 0x02
        }
        body[5 + ch] = 0x00; // iFeature
        let body_len = 6 + ch;
        w.write(CS_INTERFACE, &body[..body_len])
    }

    fn write_as_descriptors(
        &self,
        w: &mut DescriptorWriter,
        cfg: &StreamConfig,
        iface: InterfaceNumber,
        ep_addr: EndpointAddress,
        is_input: bool,
    ) -> UsbResult<()> {
        // Alt 0 — stream inactive, 0 endpoints.
        w.interface(iface, 0, UAC_CLASS, UAC_SUBCLASS_STREAMING, 0x00)?;

        // Alt 1 — stream active, 1 isochronous endpoint.
        w.interface_alt(iface, 1, 1, UAC_CLASS, UAC_SUBCLASS_STREAMING, 0x00)?;

        // Terminal link: the terminal in the audio function that this
        // streaming interface is associated with.
        // - Input stream (mic): links to the USB OUT terminal in AC topology
        // - Output stream (speaker): links to the USB IN terminal in AC topology
        let term_link = if is_input {
            TERM_ID_MIC_USB
        } else {
            TERM_ID_SPEAKER_USB
        };

        // Class-Specific AS General Interface descriptor
        w.write(
            CS_INTERFACE,
            &[
                AS_GENERAL,
                term_link,
                0x01, // bDelay = 1 frame
                PCM_TAG as u8,
                (PCM_TAG >> 8) as u8, // wFormatTag = PCM
            ],
        )?;

        // Type I Format Type descriptor
        self.write_format_type(w, cfg)?;

        // Standard Endpoint descriptor
        let ep_type = if is_input {
            EndpointType::Isochronous {
                synchronization: IsoSynchronization::Asynchronous,
                usage: IsoUsage::Data,
            }
        } else {
            EndpointType::Isochronous {
                synchronization: IsoSynchronization::Adaptive,
                usage: IsoUsage::Data,
            }
        };
        w.endpoint(ep_addr, ep_type, cfg.ep_size, 1)?;

        // Class-Specific Isoc Audio Data Endpoint descriptor
        w.write(
            CS_ENDPOINT,
            &[
                EP_GENERAL, // bDescriptorSubtype
                0x00,       // bmAttributes
                0x00,       // bLockDelayUnits
                0x00, 0x00, // wLockDelay
            ],
        )
    }

    fn write_format_type(&self, w: &mut DescriptorWriter, cfg: &StreamConfig) -> UsbResult<()> {
        fn put_u24(buf: &mut [u8], pos: &mut usize, v: u32) {
            buf[*pos] = v as u8;
            *pos += 1;
            buf[*pos] = (v >> 8) as u8;
            *pos += 1;
            buf[*pos] = (v >> 16) as u8;
            *pos += 1;
        }

        let mut buf = [0u8; 64];
        let mut i = 0usize;

        buf[i] = AS_FORMAT_TYPE;
        i += 1;
        buf[i] = FORMAT_TYPE_I;
        i += 1;
        buf[i] = cfg.channels;
        i += 1;
        buf[i] = cfg.format.bytes();
        i += 1;
        buf[i] = cfg.format.bits();
        i += 1;

        match &cfg.rates {
            Rates::Continuous(min, max) => {
                buf[i] = 0x00;
                i += 1;
                put_u24(&mut buf, &mut i, *min);
                put_u24(&mut buf, &mut i, *max);
            }
            Rates::Discrete(rates) => {
                buf[i] = rates.len() as u8;
                i += 1;
                for &r in *rates {
                    put_u24(&mut buf, &mut i, r);
                }
            }
        }
        w.write(CS_INTERFACE, &buf[..i])
    }
}

// ─── UsbClass implementation ─────────────────────────────────────────────────

impl<B: UsbBus> UsbClass<B> for AudioClass<'_, B> {
    fn configuration_descriptors(&self, w: &mut DescriptorWriter) -> UsbResult<()> {
        // NOTE: IAD is intentionally omitted.  The kernel's snd-usb-audio
        // driver locates the AudioControl interface by scanning for
        // bInterfaceClass=AUDIO / bInterfaceSubClass=AUDIOCONTROL.
        // Including an IAD can sometimes cause the USB core to misplace
        // class-specific descriptors in the interface extra-buffer on
        // certain host controllers.

        // ── Audio Control (AC) Interface ────────────────────────────────────
        w.interface(self.control_iface, 0, UAC_CLASS, UAC_SUBCLASS_CONTROL, 0x00)?;
        self.write_ac_header(w)?;

        if let Some(ref cfg) = self.in_cfg {
            self.write_terminal_descriptors(
                w,
                cfg,
                TERM_ID_MIC_IN,
                TERM_ID_MIC_FU,
                TERM_ID_MIC_USB,
                true,
            )?;
        }
        if let Some(ref cfg) = self.out_cfg {
            self.write_terminal_descriptors(
                w,
                cfg,
                TERM_ID_SPEAKER_USB,
                TERM_ID_SPEAKER_FU,
                TERM_ID_SPEAKER_OUT,
                false,
            )?;
        }

        // ── Audio Streaming (AS) Interfaces ─────────────────────────────────
        if let Some(ref cfg) = self.in_cfg {
            self.write_as_descriptors(
                w,
                cfg,
                self.in_iface,
                self.in_ep.as_ref().unwrap().address(),
                true,
            )?;
        }
        if let Some(ref cfg) = self.out_cfg {
            self.write_as_descriptors(
                w,
                cfg,
                self.out_iface,
                self.out_ep.as_ref().unwrap().address(),
                false,
            )?;
        }

        Ok(())
    }

    fn control_in(&mut self, xfer: ControlIn<'_, B>) {
        let req = xfer.request();

        // Only handle Class requests to the Interface recipient.
        if req.request_type != RequestType::Class || req.recipient != Recipient::Interface {
            return;
        }

        // wIndex = (UnitID << 8) | Interface
        let unit_id = (req.index >> 8) as u8;
        // wValue = (ControlSelector << 8) | ChannelNumber
        let cs = (req.value >> 8) as u8;

        // Check if the request targets one of our Feature Units.
        let channels = self.fu_channels(unit_id);
        if channels == 0 {
            return; // Not our unit — let the request fall through.
        }

        match req.request {
            CUR_GET if cs == FU_MUTE => {
                // Mute: return 0 = unmuted
                let _ = xfer.accept_with(&[0x00]);
            }
            CUR_GET if cs == FU_VOLUME => {
                // Volume CUR = 0x0000 (max / 0 dB), 16-bit signed LE
                let _ = xfer.accept_with(&[0x00, 0x00]);
            }
            MIN_GET if cs == FU_VOLUME => {
                // Volume MIN = 0x0080 (minimum, very quiet)
                let _ = xfer.accept_with(&[0x00, 0x80]);
            }
            MAX_GET if cs == FU_VOLUME => {
                // Volume MAX = 0x0000 (0 dB, max)
                let _ = xfer.accept_with(&[0x00, 0x00]);
            }
            RES_GET if cs == FU_VOLUME => {
                // Volume RES = 0x0100 (1 dB resolution)
                let _ = xfer.accept_with(&[0x00, 0x01]);
            }
            _ => {
                let _ = xfer.reject();
            }
        }
    }

    fn control_out(&mut self, xfer: ControlOut<'_, B>) {
        let req = xfer.request();

        if req.request_type != RequestType::Class || req.recipient != Recipient::Interface {
            return;
        }

        let unit_id = (req.index >> 8) as u8;
        let cs = (req.value >> 8) as u8;

        if self.fu_channels(unit_id) == 0 {
            return;
        }

        match req.request {
            CUR_SET if cs == FU_MUTE => {
                // Accept mute change (ignore the value for now).
                let _ = xfer.accept();
            }
            CUR_SET if cs == FU_VOLUME => {
                // Accept volume change (ignore the value for now).
                let _ = xfer.accept();
            }
            _ => {
                let _ = xfer.reject();
            }
        }
    }

    fn configure(&mut self) {
        self.configured = true;
    }

    fn deconfigure(&mut self) {
        self.configured = false;
        self.in_alt = 0;
        self.out_alt = 0;
    }

    fn reset(&mut self) {
        self.configured = false;
        self.in_alt = 0;
        self.out_alt = 0;
    }

    fn alt_setting(&mut self, iface: InterfaceNumber) -> Option<u8> {
        if self.in_cfg.is_some() && self.in_iface == iface {
            return Some(self.in_alt);
        }
        if self.out_cfg.is_some() && self.out_iface == iface {
            return Some(self.out_alt);
        }
        None
    }

    fn set_alt_setting(&mut self, iface: InterfaceNumber, alt: u8) -> bool {
        if self.in_cfg.is_some() && self.in_iface == iface {
            self.in_alt = alt;
            return true;
        }
        if self.out_cfg.is_some() && self.out_iface == iface {
            self.out_alt = alt;
            return true;
        }
        false
    }
}
