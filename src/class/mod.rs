//! [`UsbClass`] trait and the class implementations.
//!
//! Provided classes:
//!
//! | Class | Interface |
//! |---|---|
//! | [`cdc_acm::CdcAcm`] | virtual serial port |
//! | [`hid::HidClass`] | HID (boot keyboard/mouse, custom reports, power device) |
//! | [`gs_usb::GsUsbClass`] | candleLight/GS_USB CAN adapter |
//! | [`i2c_tiny_usb::I2cTinyUsb`] | i2c-tiny-usb I2C bridge |
//! | [`uac::AudioClass`] | USB Audio Class 1.0 |
//!
//! Classes are generic over the [`UsbBus`] trait.

pub mod cdc_acm;
pub mod gs_usb;
pub mod hid;
pub mod i2c_tiny_usb;
pub mod uac;

use crate::bus::{InterfaceNumber, StringIndex, UsbBus};
use crate::control::{ControlIn, ControlOut};
use crate::descriptor::DescriptorWriter;
use crate::endpoint::EndpointAddress;
use crate::error::UsbResult;

/// USB device class trait.
///
/// All methods have default no-op implementations.  Override only the ones your
/// class needs.
pub trait UsbClass<B: UsbBus> {
    // ── Descriptors ──────────────────────────────────────────────────────────

    /// Append this class's interface and endpoint descriptors.
    fn configuration_descriptors(&self, writer: &mut DescriptorWriter) -> UsbResult<()> {
        let _ = writer;
        Ok(())
    }

    /// Return the string for a previously allocated [`StringIndex`], or `None`.
    fn string(&self, index: StringIndex, lang_id: u16) -> Option<&str> {
        let _ = (index, lang_id);
        None
    }

    // ── Control transfers ─────────────────────────────────────────────────────

    /// Handle a control IN transfer.  Accept, reject, or pass (drop without acting).
    fn control_in(&mut self, xfer: ControlIn<'_, B>) {
        let _ = xfer;
    }

    /// Handle a control OUT transfer.  Accept, reject, or pass (drop without acting).
    fn control_out(&mut self, xfer: ControlOut<'_, B>) {
        let _ = xfer;
    }

    // ── Data events ───────────────────────────────────────────────────────────

    /// Called when endpoint `addr` received an OUT packet.
    fn endpoint_out(&mut self, addr: EndpointAddress) {
        let _ = addr;
    }

    /// Called when endpoint `addr` completed an IN transfer.
    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        let _ = addr;
    }

    /// Called when endpoint `addr` received a SETUP packet (non-EP0 control).
    fn endpoint_setup(&mut self, addr: EndpointAddress) {
        let _ = addr;
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    /// Called after a USB bus reset.
    fn reset(&mut self) {}

    /// Called when the host sends SET_CONFIGURATION(1).
    fn configure(&mut self) {}

    /// Called when the host sends SET_CONFIGURATION(0) or on reset.
    fn deconfigure(&mut self) {}

    /// Called unconditionally on every [`UsbDevice::poll`](crate::device::UsbDevice::poll).
    fn poll(&mut self) {}

    // ── Alternate settings ────────────────────────────────────────────────────

    /// Return the current alternate setting for `interface`, or `None`.
    fn alt_setting(&mut self, interface: InterfaceNumber) -> Option<u8> {
        let _ = interface;
        None
    }

    /// Set the alternate setting for `interface`.  Return `true` if handled.
    fn set_alt_setting(&mut self, interface: InterfaceNumber, alt: u8) -> bool {
        let _ = (interface, alt);
        false
    }
}
