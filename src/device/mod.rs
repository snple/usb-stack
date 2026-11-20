//! The device side of the stack.
//!
//! ```text
//!   application / class          class::cdc_acm, your own UsbClass
//!   ────────────────────────     this module
//!   core::UsbDevice              enumeration, standard requests, EP0
//!   ────────────────────────     this module
//!   bus::UsbBus                  trait: allocate, poll, read, write
//!   ────────────────────────     this module
//!   (a microcontroller crate)    implements UsbBus
//! ```
//!
//! A peripheral driver implements [`bus::UsbBus`] and re-exports this crate;
//! everything above it — the device state machine, the control pipe, descriptor
//! emission and the classes — is hardware independent.
//!
//! The vocabulary shared with the host side ([`crate::endpoint`],
//! [`crate::descriptor`], [`crate::request`]) is re-exported here, so a class
//! driver only has to import this module.

pub mod bus;
pub mod class;
pub mod control;
pub mod descriptor;
pub mod endpoint;
pub mod error;

mod core;

pub use bus::{InterfaceNumber, PollResult, StringIndex, UsbBus, UsbBusAllocator};
pub use class::UsbClass;
pub use control::{ControlIn, ControlOut, ControlPipe};
pub use core::{DeviceInfo, DeviceState, UsbDevice, UsbDeviceBuilder};
pub use descriptor::DescriptorWriter;
pub use endpoint::{Endpoint, EndpointIn, EndpointOut};
pub use error::{UsbError, UsbResult};

pub use crate::endpoint::{
    Direction, EndpointAddress, EndpointType, In, IsoSynchronization, IsoUsage, Out,
};
pub use crate::request::{Feature, Recipient, Request, RequestType, StandardRequest};
