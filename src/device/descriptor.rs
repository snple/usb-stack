//! Configuration descriptor builder.
//!
//! [`DescriptorWriter`] appends interface, endpoint and class-specific
//! descriptors to the configuration buffer the device stack answers
//! `GET_DESCRIPTOR(CONFIGURATION)` with.

use crate::descriptor::descriptor_type;
use crate::device::bus::{InterfaceNumber, StringIndex};
use crate::device::error::{UsbError, UsbResult};
use crate::endpoint::{EndpointAddress, EndpointType};

// ─── DescriptorWriter ─────────────────────────────────────────────────────────

/// Cursor-based USB descriptor builder.
///
/// Wraps a `&mut [u8]` buffer and advances a position pointer as descriptors
/// are appended.  The configuration-descriptor wTotalLength and bNumInterfaces
/// fields are back-patched when [`end_configuration`](Self::end_configuration)
/// is called.
pub struct DescriptorWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
    /// Byte offset of the wTotalLength field in the current configuration
    /// descriptor, if one has been opened.
    config_total_len_pos: Option<usize>,
    /// Byte offset of the bNumInterfaces field.
    config_num_ifaces_pos: Option<usize>,
    /// Running interface count for the current configuration.
    num_interfaces: u8,
}

impl<'a> DescriptorWriter<'a> {
    /// Create a new writer over `buf`.
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self {
            buf,
            pos: 0,
            config_total_len_pos: None,
            config_num_ifaces_pos: None,
            num_interfaces: 0,
        }
    }

    /// Bytes written so far.
    pub fn position(&self) -> usize {
        self.pos
    }

    // ── Low-level primitives ─────────────────────────────────────────────────

    /// Append a raw byte sequence preceded by a 2-byte header (bLength, bDescriptorType).
    ///
    /// `f` receives a mutable slice of up to 254 bytes (max descriptor body size)
    /// and should write the descriptor body, returning the body length.
    pub fn write_with(
        &mut self,
        descriptor_type: u8,
        f: impl FnOnce(&mut [u8]) -> UsbResult<usize>,
    ) -> UsbResult<()> {
        if self.pos + 2 > self.buf.len() {
            return Err(UsbError::BufferOverflow);
        }
        let body_start = self.pos + 2;
        let body_end = self.buf.len().min(body_start + 254);
        let body_len = f(&mut self.buf[body_start..body_end])?;
        let total_len = body_len + 2;
        if self.pos + total_len > self.buf.len() {
            return Err(UsbError::BufferOverflow);
        }
        self.buf[self.pos] = total_len as u8;
        self.buf[self.pos + 1] = descriptor_type;
        self.pos += total_len;
        Ok(())
    }

    /// Append a descriptor with a fixed body slice.
    pub fn write(&mut self, descriptor_type: u8, body: &[u8]) -> UsbResult<()> {
        self.write_with(descriptor_type, |buf| {
            if body.len() > buf.len() {
                return Err(UsbError::BufferOverflow);
            }
            buf[..body.len()].copy_from_slice(body);
            Ok(body.len())
        })
    }

    // ── Configuration descriptor ─────────────────────────────────────────────

    /// Begin a configuration descriptor.  Must be paired with
    /// [`end_configuration`](Self::end_configuration).
    pub fn configuration(&mut self, attributes: u8, max_power_ma: u8) -> UsbResult<()> {
        self.config_total_len_pos = Some(self.pos + 2); // offset of wTotalLength[0]
        self.config_num_ifaces_pos = Some(self.pos + 4); // offset of bNumInterfaces
        self.num_interfaces = 0;
        self.write(
            descriptor_type::CONFIGURATION,
            &[
                0,
                0, // wTotalLength — patched later
                0, // bNumInterfaces — patched later
                1, // bConfigurationValue
                0, // iConfiguration
                attributes,
                max_power_ma,
            ],
        )
    }

    /// Finalise the open configuration descriptor (patch wTotalLength and
    /// bNumInterfaces).
    pub fn end_configuration(&mut self) {
        if let (Some(tl_pos), Some(ni_pos)) =
            (self.config_total_len_pos, self.config_num_ifaces_pos)
        {
            let total = self.pos as u16;
            self.buf[tl_pos] = total as u8;
            self.buf[tl_pos + 1] = (total >> 8) as u8;
            self.buf[ni_pos] = self.num_interfaces;
        }
        self.config_total_len_pos = None;
        self.config_num_ifaces_pos = None;
    }

    // ── Interface & endpoint ─────────────────────────────────────────────────

    /// Write an Interface Association Descriptor (IAD), if the configuration was
    /// built with `composite_with_iads = true`.
    ///
    /// Always safe to call — if the outer device was not configured for IADs the
    /// call is a no-op.
    pub fn iad(
        &mut self,
        first_interface: InterfaceNumber,
        interface_count: u8,
        function_class: u8,
        function_subclass: u8,
        function_protocol: u8,
    ) -> UsbResult<()> {
        self.write(
            descriptor_type::IAD,
            &[
                first_interface.into(),
                interface_count,
                function_class,
                function_subclass,
                function_protocol,
                0, // iFunction
            ],
        )
    }

    /// Write an Interface descriptor and increment the interface count.
    pub fn interface(
        &mut self,
        number: InterfaceNumber,
        num_endpoints: u8,
        class: u8,
        subclass: u8,
        protocol: u8,
    ) -> UsbResult<()> {
        self.interface_with_string(
            number,
            num_endpoints,
            class,
            subclass,
            protocol,
            StringIndex::new(0),
        )
    }

    /// Write an Interface descriptor with a string index.
    pub fn interface_with_string(
        &mut self,
        number: InterfaceNumber,
        num_endpoints: u8,
        class: u8,
        subclass: u8,
        protocol: u8,
        string: StringIndex,
    ) -> UsbResult<()> {
        self.num_interfaces += 1;
        self.write(
            descriptor_type::INTERFACE,
            &[
                number.into(),
                0, // bAlternateSetting
                num_endpoints,
                class,
                subclass,
                protocol,
                string.into(),
            ],
        )
    }

    /// Write an Interface descriptor with a non-zero alternate setting.
    ///
    /// Does **not** increment `bNumInterfaces` — alternate settings share the
    /// same interface number and are not counted separately.
    pub fn interface_alt(
        &mut self,
        number: InterfaceNumber,
        alternate_setting: u8,
        num_endpoints: u8,
        class: u8,
        subclass: u8,
        protocol: u8,
    ) -> UsbResult<()> {
        self.write(
            descriptor_type::INTERFACE,
            &[
                number.into(),
                alternate_setting,
                num_endpoints,
                class,
                subclass,
                protocol,
                0, // iInterface
            ],
        )
    }

    /// Write an Endpoint descriptor.
    pub fn endpoint(
        &mut self,
        addr: EndpointAddress,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8,
    ) -> UsbResult<()> {
        self.write(
            descriptor_type::ENDPOINT,
            &[
                addr.raw(),
                ep_type.bm_attributes(),
                max_packet_size as u8,
                (max_packet_size >> 8) as u8,
                interval,
            ],
        )
    }

    // ── String descriptors ───────────────────────────────────────────────────

    /// Encode a UTF-8 string as a USB string descriptor (UTF-16LE with header)
    /// and write it into `buf`.  Returns the number of bytes written.
    ///
    /// Call this from the device layer when responding to GET_DESCRIPTOR(String).
    pub fn write_string(buf: &mut [u8], s: &str) -> UsbResult<usize> {
        if buf.len() < 2 {
            return Err(UsbError::BufferOverflow);
        }
        let max_chars = (buf.len() - 2) / 2;
        let mut n = 0usize;
        for c in s.chars().take(max_chars) {
            // Non-BMP code points are replaced with U+FFFD (replacement char).
            let unit = if (c as u32) <= 0xFFFF {
                c as u16
            } else {
                0xFFFD
            };
            buf[2 + n * 2] = unit as u8;
            buf[3 + n * 2] = (unit >> 8) as u8;
            n += 1;
        }
        let total = 2 + n * 2;
        buf[0] = total as u8;
        buf[1] = descriptor_type::STRING;
        Ok(total)
    }

    /// Write the language ID list descriptor (index 0) into `buf`.
    pub fn write_lang_ids(buf: &mut [u8], lang_ids: &[u16]) -> UsbResult<usize> {
        let total = 2 + lang_ids.len() * 2;
        if total > buf.len() {
            return Err(UsbError::BufferOverflow);
        }
        buf[0] = total as u8;
        buf[1] = descriptor_type::STRING;
        for (i, &id) in lang_ids.iter().enumerate() {
            buf[2 + i * 2] = id as u8;
            buf[2 + i * 2 + 1] = (id >> 8) as u8;
        }
        Ok(total)
    }
}
