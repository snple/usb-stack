//! Host-side USB stack.
//!
//! The host stack is split the same way as the device stack: everything below
//! the controller is hardware independent, and a peripheral driver implements
//! [`HostController`].
//!
//! ```text
//!   application                  class::hid::Keyboard, your own class driver
//!   ────────────────────────
//!   Device                       enumeration, control transfers
//!   Pipe                         bulk and interrupt transfers
//!   ────────────────────────
//!   HostController               trait: port, channels, transactions
//!   ────────────────────────
//!   (a peripheral driver)        implements HostController
//! ```
//!
//! Transfers are expressed at transaction level and driven synchronously: a
//! call blocks until the controller halts the channel, so no interrupts, no DMA
//! and no `async` are involved. Enumeration is a fixed sequence of control
//! transfers ([`Host::enumerate`] runs it), and everything a device offers
//! afterwards is reachable through [`Device::control`] and [`Pipe`].
//!
//! ## What is covered
//!
//! - Root port only: hubs are not supported, so a device must be plugged into
//!   the controller's own port
//! - Control, bulk and interrupt transfers at low, full and high speed
//! - Device, configuration, interface, endpoint and string descriptors
//! - Classes: HID boot keyboards ([`class::hid::Keyboard`])
//!
//! Isochronous transfers and the hub class are not implemented.
//!
//! ## Example
//!
//! ```ignore
//! use usb_stack::host::{Host, class::hid::Keyboard};
//!
//! let bus = Host::new(&my_controller);
//!
//! loop {
//!     let device = match bus.enumerate() {
//!         Ok(device) => device,
//!         Err(error) => continue, // nothing attached, or a device that misbehaves
//!     };
//!     if let Ok(keyboard) = Keyboard::open(&device) {
//!         let mut previous = Default::default();
//!         while let Ok(report) = keyboard.read() {
//!             for key in report.new_keys(&previous) {
//!                 defmt::info!("key {}", key);
//!             }
//!             previous = report;
//!         }
//!     }
//! }
//! ```

pub mod class;
pub mod controller;
pub mod descriptor;
pub mod device;
pub mod error;
pub mod pipe;

pub use crate::request::Request;
pub use controller::{
    Channel, DeviceAddress, EndpointConfig, HostController, PortStatus, Speed, Toggle,
};
pub use descriptor::{
    Configuration, Descriptor, Descriptors, DeviceDescriptor, EndpointDescriptor, Endpoints,
    Interface, Interfaces,
};
pub use device::{CONFIG_MAX, Device};
pub use error::{HostError, HostResult};
pub use pipe::Pipe;

/// The root port of a host controller.
///
/// This is the entry point of the stack: it reports the port, resets it and
/// enumerates what is attached. Everything else hangs off the [`Device`] that
/// [`enumerate`](Self::enumerate) returns.
pub struct Host<'d, C: HostController> {
    controller: &'d C,
}

impl<'d, C: HostController> Host<'d, C> {
    /// Wrap a controller. The controller must be powered up already.
    #[inline]
    pub const fn new(controller: &'d C) -> Self {
        Self { controller }
    }

    /// Read the root port without changing anything.
    #[inline]
    pub fn port(&self) -> PortStatus {
        self.controller.port_status()
    }

    /// Power the port, drive a bus reset and report the negotiated speed.
    ///
    /// A device has to be attached; this is what enumeration starts with.
    #[inline]
    pub fn reset(&self) -> HostResult<PortStatus> {
        self.controller.reset_port()
    }

    /// Wait up to a second for a device to be attached.
    pub fn wait_for_device(&self) -> HostResult<PortStatus> {
        for _ in 0..1000 {
            let status = self.controller.port_status();
            if status.connected {
                return Ok(status);
            }
            self.controller.wait_millis(1);
        }
        Err(HostError::Timeout)
    }

    /// Reset the port and enumerate the attached device at the first free
    /// address.
    pub fn enumerate(&self) -> HostResult<Device<'d, C>> {
        self.enumerate_at(DeviceAddress::FIRST)
    }

    /// Reset the port and enumerate the attached device at `address`.
    pub fn enumerate_at(&self, address: DeviceAddress) -> HostResult<Device<'d, C>> {
        let status = self.controller.reset_port()?;
        if !status.enabled {
            return Err(HostError::Disconnected);
        }
        Device::enumerate(self.controller, address, status.speed)
    }
}
