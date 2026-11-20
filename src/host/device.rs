//! Enumeration and control transfers.
//!
//! A [`Device`] owns the control channel of an enumerated device: the address,
//! the control packet size and the descriptors that were read while
//! enumerating. Endpoints are turned into [`Pipe`]s on demand.

use crate::descriptor::{descriptor_type, lang_id};
use crate::endpoint::{Direction, EndpointAddress, EndpointType};
use crate::host::controller::{
    Channel, DeviceAddress, EndpointConfig, HostController, Speed, Toggle,
};
use crate::host::descriptor::{Configuration, DeviceDescriptor, EndpointDescriptor};
use crate::host::error::{HostError, HostResult};
use crate::host::pipe::Pipe;
use crate::request::Request;

/// Size of the control endpoint before the device reported its own.
const EP0_MIN_PACKET: u16 = 8;

/// Largest configuration descriptor the stack accepts.
///
/// The buffer lives inside [`Device`], so an application pays for it whether or
/// not the device has a large configuration. 256 bytes hold every HID, CDC-ACM
/// and audio control configuration; raise this when enumerating a composite
/// device fails with [`HostError::Overflow`].
pub const CONFIG_MAX: usize = 256;

/// Room for a string descriptor: thirty-two UTF-16 code units plus the header.
const STRING_BUFFER: usize = 2 + 2 * 32;

/// An enumerated device.
pub struct Device<'a, C: HostController> {
    controller: &'a C,
    control: Channel,
    address: DeviceAddress,
    speed: Speed,
    max_packet0: u16,
    descriptor: DeviceDescriptor,
    raw_config: [u8; CONFIG_MAX],
    config_length: usize,
}

impl<'a, C: HostController> Device<'a, C> {
    /// Reset the port and enumerate whatever is attached.
    pub(crate) fn enumerate(
        controller: &'a C,
        address: DeviceAddress,
        speed: Speed,
    ) -> HostResult<Self> {
        let control = controller.alloc_channel()?;
        let mut device = Self {
            controller,
            control,
            address: DeviceAddress::ZERO,
            speed,
            max_packet0: EP0_MIN_PACKET,
            descriptor: DeviceDescriptor::default(),
            raw_config: [0; CONFIG_MAX],
            config_length: 0,
        };
        device.configure_control()?;

        // The first eight bytes of the device descriptor also carry
        // bMaxPacketSize0, which every later request depends on.
        let mut head = [0u8; EP0_MIN_PACKET as usize];
        let request = Request::get_descriptor(descriptor_type::DEVICE, 0, 0, head.len() as u16);
        if device.control(&request, &mut head)? < head.len() {
            return Err(HostError::Malformed);
        }
        device.max_packet0 = match head[7] {
            8 | 16 | 32 | 64 => head[7] as u16,
            _ => return Err(HostError::Malformed),
        };
        device.configure_control()?;

        // From the status stage of SET_ADDRESS on, the device answers to its
        // new address.
        device.control(&Request::set_address(address.raw()), &mut [])?;
        device.address = address;
        controller.wait_millis(2);
        device.configure_control()?;

        let mut raw = [0u8; DeviceDescriptor::LEN];
        let request = Request::get_descriptor(descriptor_type::DEVICE, 0, 0, raw.len() as u16);
        device.control(&request, &mut raw)?;
        device.descriptor = DeviceDescriptor::parse(&raw)?;

        // The configuration is a linked list flattened into one reply: read its
        // header first to learn how long the whole thing is.
        let mut header = [0u8; Configuration::HEADER_LEN];
        let request =
            Request::get_descriptor(descriptor_type::CONFIGURATION, 0, 0, header.len() as u16);
        device.control(&request, &mut header)?;
        let total = u16::from_le_bytes([header[2], header[3]]) as usize;
        if total < Configuration::HEADER_LEN {
            return Err(HostError::Malformed);
        }
        if total > CONFIG_MAX {
            return Err(HostError::Overflow);
        }

        let mut raw_config = [0u8; CONFIG_MAX];
        let request = Request::get_descriptor(descriptor_type::CONFIGURATION, 0, 0, total as u16);
        let length = device.control(&request, &mut raw_config[..total])?;
        Configuration::parse(&raw_config[..length])?;
        device.raw_config = raw_config;
        device.config_length = length;

        Ok(device)
    }

    /// The address the device answers to.
    #[inline]
    pub fn address(&self) -> DeviceAddress {
        self.address
    }

    /// The speed the device negotiated.
    #[inline]
    pub fn speed(&self) -> Speed {
        self.speed
    }

    /// The device descriptor read while enumerating.
    #[inline]
    pub fn descriptor(&self) -> &DeviceDescriptor {
        &self.descriptor
    }

    /// The configuration descriptor together with its interface and endpoint
    /// descriptors.
    #[inline]
    pub fn configuration(&self) -> Configuration<'_> {
        // `enumerate` validated the bytes and `raw_config` is never modified
        // afterwards, so there is nothing left to check.
        Configuration::validated(&self.raw_config[..self.config_length])
    }

    /// Run one control transfer.
    ///
    /// `buffer` must hold `request.length` bytes. The returned count is the
    /// number of bytes the device actually sent, which may be less than
    /// requested.
    pub fn control(&self, request: &Request, buffer: &mut [u8]) -> HostResult<usize> {
        if (buffer.len() as u16) < request.length {
            return Err(HostError::Overflow);
        }
        control_transfer(
            self.controller,
            self.control,
            self.max_packet0,
            request,
            buffer,
        )
    }

    /// Select a configuration.
    ///
    /// The endpoints of the device are only usable after this request succeeded.
    pub fn set_configuration(&self, value: u8) -> HostResult<()> {
        self.control(&Request::set_configuration(value), &mut [])?;
        Ok(())
    }

    /// Read a string descriptor and convert it to UTF-8.
    ///
    /// Returns the number of bytes written to `out`. Strings longer than
    /// thirty-two characters and output that does not fit are truncated at a
    /// character boundary.
    pub fn read_string(&self, index: u8, out: &mut [u8]) -> HostResult<usize> {
        if index == 0 {
            return Err(HostError::Protocol);
        }
        let mut raw = [0u8; STRING_BUFFER];
        let request = Request::get_descriptor(
            descriptor_type::STRING,
            index,
            lang_id::EN_US,
            STRING_BUFFER as u16,
        );
        let length = self.control(&request, &mut raw)?;

        if length < 2 || raw[0] as usize > length || raw[1] != descriptor_type::STRING {
            return Err(HostError::Malformed);
        }
        Ok(utf16le_to_utf8(&raw[2..raw[0] as usize], out))
    }

    /// Open the endpoint described by `endpoint`.
    pub fn open(&self, endpoint: &EndpointDescriptor) -> HostResult<Pipe<'a, C>> {
        let config = EndpointConfig {
            address: endpoint.address,
            transfer: endpoint.transfer_type(),
            max_packet: endpoint.max_packet,
            interval: endpoint.interval,
        };
        let channel = self.controller.alloc_channel()?;
        let result = self.controller.configure(channel, self.address, &config);
        match result {
            Ok(()) => Ok(Pipe::new(self.controller, channel, config)),
            Err(error) => {
                self.controller.free_channel(channel);
                Err(error)
            }
        }
    }

    /// Clear the halt condition of `pipe` and restart its data toggle.
    ///
    /// Called after a transfer failed with [`HostError::Stall`].
    pub fn clear_halt(&self, pipe: &Pipe<'a, C>) -> HostResult<()> {
        self.control(&Request::clear_halt(pipe.address().raw()), &mut [])?;
        pipe.reset_toggle();
        Ok(())
    }

    fn configure_control(&self) -> HostResult<()> {
        let endpoint = EndpointConfig {
            address: EndpointAddress::EP0_OUT,
            transfer: EndpointType::Control,
            max_packet: self.max_packet0,
            interval: 0,
        };
        self.controller
            .configure(self.control, self.address, &endpoint)
    }
}

impl<C: HostController> Drop for Device<'_, C> {
    fn drop(&mut self) {
        self.controller.free_channel(self.control);
    }
}

// ─── Control transfers ───────────────────────────────────────────────────────

/// The data toggle of the `packet`-th packet of a control data stage.
///
/// The data stage always starts at DATA1; the status stage is DATA1 again, and
/// is programmed by the caller.
#[inline]
fn data_stage_toggle(packet: usize) -> Toggle {
    if packet.is_multiple_of(2) {
        Toggle::Data1
    } else {
        Toggle::Data0
    }
}

fn control_transfer<C: HostController>(
    controller: &C,
    channel: Channel,
    max_packet0: u16,
    request: &Request,
    buffer: &mut [u8],
) -> HostResult<usize> {
    let max_packet = max_packet0.max(EP0_MIN_PACKET) as usize;
    controller.setup(channel, &request.to_bytes())?;

    if request.length == 0 {
        // Without a data stage the status stage is the only one left, and it
        // always goes from the device to the host.
        controller.transfer_in(channel, Toggle::Data1, &mut [])?;
        return Ok(0);
    }

    let length = request.length as usize;
    let mut done = 0;

    match request.direction {
        Direction::In => {
            while done < length {
                let chunk = (length - done).min(max_packet);
                let received = controller.transfer_in(
                    channel,
                    data_stage_toggle(done / max_packet),
                    &mut buffer[done..done + chunk],
                )?;
                done += received;
                if received < chunk {
                    break;
                }
            }
            controller.transfer_out(channel, Toggle::Data1, &[])?;
        }
        Direction::Out => {
            while done < length {
                let chunk = (length - done).min(max_packet);
                controller.transfer_out(
                    channel,
                    data_stage_toggle(done / max_packet),
                    &buffer[done..done + chunk],
                )?;
                done += chunk;
            }
            controller.transfer_in(channel, Toggle::Data1, &mut [])?;
        }
    }
    Ok(done)
}

// ─── Strings ─────────────────────────────────────────────────────────────────

/// Convert UTF-16LE units to UTF-8, truncating at a character boundary.
fn utf16le_to_utf8(units: &[u8], out: &mut [u8]) -> usize {
    let mut written = 0;
    for pair in units.as_chunks::<2>().0 {
        // Surrogates and values above U+FFFF are not part of a USB string
        // descriptor's BMP subset; skip them rather than emit broken UTF-8.
        let code = u16::from_le_bytes([pair[0], pair[1]]);
        let decoded = match code {
            0x0000..=0x007f => [code as u8, 0, 0, 1],
            0x0080..=0x07ff => [
                0b1100_0000 | (code >> 6) as u8,
                0b1000_0000 | (code & 0x3f) as u8,
                0,
                2,
            ],
            0x0800..=0xd7ff | 0xe000..=0xffff => [
                0b1110_0000 | (code >> 12) as u8,
                0b1000_0000 | ((code >> 6) & 0x3f) as u8,
                0b1000_0000 | (code & 0x3f) as u8,
                3,
            ],
            _ => [0b1111_1101, 0, 0, 1],
        };
        let length = decoded[3] as usize;
        if written + length > out.len() {
            break;
        }
        out[written..written + length].copy_from_slice(&decoded[..length]);
        written += length;
    }
    written
}
