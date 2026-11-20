//! Bulk and interrupt pipes.
//!
//! A pipe owns one host channel and the endpoint's data toggle. Reads and
//! writes are one transfer each: they are split into packets of the endpoint's
//! maximum packet size and stop at the first short packet, which is what the
//! USB specification calls a transfer.

use core::cell::Cell;

use crate::endpoint::{EndpointAddress, EndpointType};
use crate::host::controller::{Channel, EndpointConfig, HostController, Toggle};
use crate::host::error::HostResult;

/// An endpoint of an enumerated device.
pub struct Pipe<'d, C: HostController> {
    controller: &'d C,
    channel: Channel,
    endpoint: EndpointConfig,
    /// Toggle the next packet is sent with.
    toggle: Cell<Toggle>,
}

impl<'d, C: HostController> Pipe<'d, C> {
    pub(crate) fn new(controller: &'d C, channel: Channel, endpoint: EndpointConfig) -> Self {
        Self {
            controller,
            channel,
            endpoint,
            toggle: Cell::new(Toggle::Data0),
        }
    }

    /// Endpoint number and direction.
    #[inline]
    pub fn address(&self) -> EndpointAddress {
        self.endpoint.address
    }

    /// Maximum packet size.
    #[inline]
    pub fn max_packet(&self) -> u16 {
        self.endpoint.max_packet
    }

    /// Bulk or interrupt.
    #[inline]
    pub fn transfer_type(&self) -> EndpointType {
        self.endpoint.transfer
    }

    /// Restart the data toggle at DATA0, as `CLEAR_FEATURE(ENDPOINT_HALT)`
    /// requires.
    pub(crate) fn reset_toggle(&self) {
        self.toggle.set(Toggle::Data0);
    }

    /// Receive one transfer, stopping at a short packet or when `buffer` is
    /// full.
    pub fn read(&self, buffer: &mut [u8]) -> HostResult<usize> {
        let max_packet = self.max_packet() as usize;
        let mut done = 0;

        while done < buffer.len() {
            let chunk = (buffer.len() - done).min(max_packet);
            let received = self.controller.transfer_in(
                self.channel,
                self.toggle.get(),
                &mut buffer[done..done + chunk],
            )?;
            self.advance();
            done += received;
            if received < chunk {
                break;
            }
        }
        Ok(done)
    }

    /// Send one transfer.
    ///
    /// An empty `data` sends a single zero-length packet, which is how a bulk
    /// transfer is terminated early.
    pub fn write(&self, data: &[u8]) -> HostResult<usize> {
        let max_packet = self.max_packet() as usize;
        let mut done = 0;

        while done < data.len() {
            let chunk = (data.len() - done).min(max_packet);
            self.controller.transfer_out(
                self.channel,
                self.toggle.get(),
                &data[done..done + chunk],
            )?;
            self.advance();
            done += chunk;
        }
        if data.is_empty() {
            self.controller
                .transfer_out(self.channel, self.toggle.get(), &[])?;
            self.advance();
        }
        Ok(done)
    }

    fn advance(&self) {
        self.toggle.set(match self.toggle.get() {
            Toggle::Data0 => Toggle::Data1,
            Toggle::Data1 => Toggle::Data0,
        });
    }
}

impl<C: HostController> Drop for Pipe<'_, C> {
    fn drop(&mut self) {
        self.controller.free_channel(self.channel);
    }
}
