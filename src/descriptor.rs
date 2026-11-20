//! USB descriptor vocabulary shared by both directions of the stack.
//!
//! [`descriptor_type`] holds the `bDescriptorType` codes, [`class_code`] the
//! `bDeviceClass`/`bInterfaceClass` codes and the `cdc_*` modules the CDC
//! function descriptors. The device-side builder that writes descriptors into a
//! configuration buffer lives in [`crate::device::descriptor`].

// ─── Standard descriptor type codes ──────────────────────────────────────────

/// `bDescriptorType` values (USB 2.0 §9.4).
#[allow(missing_docs)]
pub mod descriptor_type {
    pub const DEVICE: u8 = 1;
    pub const CONFIGURATION: u8 = 2;
    pub const STRING: u8 = 3;
    pub const INTERFACE: u8 = 4;
    pub const ENDPOINT: u8 = 5;
    pub const DEVICE_QUALIFIER: u8 = 6;
    pub const OTHER_SPEED_CONFIGURATION: u8 = 7;
    pub const INTERFACE_ASSOCIATION: u8 = 11;
    /// Alias of [`INTERFACE_ASSOCIATION`].
    pub const IAD: u8 = INTERFACE_ASSOCIATION;
    pub const BOS: u8 = 15;
    pub const DEVICE_CAPABILITY: u8 = 16;
    /// Alias of [`DEVICE_CAPABILITY`].
    pub const CAPABILITY: u8 = DEVICE_CAPABILITY;
}

/// USB language IDs (ISO 639).
pub mod lang_id {
    /// English (United States).
    pub const EN_US: u16 = 0x0409;
}

// ─── USB class / subclass / protocol constants ────────────────────────────────

/// USB class codes (`bDeviceClass` / `bInterfaceClass`).
#[allow(missing_docs)]
pub mod class_code {
    pub const CDC: u8 = 0x02;
    pub const CDC_DATA: u8 = 0x0A;
    pub const HID: u8 = 0x03;
    pub const MISC: u8 = 0xEF;
    pub const NONE: u8 = 0x00;
    /// Vendor-specific interface/device class.
    pub const VENDOR: u8 = 0xFF;
}

/// CDC interface subclasses (`bInterfaceSubClass`).
#[allow(missing_docs)]
pub mod cdc_subclass {
    pub const ACM: u8 = 0x02;
}

/// CDC interface protocols (`bInterfaceProtocol`).
#[allow(missing_docs)]
pub mod cdc_protocol {
    pub const AT_COMMANDS: u8 = 0x01;
    pub const NONE: u8 = 0x00;
}

/// CDC functional descriptors subtypes.
#[allow(missing_docs)]
pub mod cdc_descriptor {
    /// bDescriptorType for class-specific interface descriptors.
    pub const CS_INTERFACE: u8 = 0x24;
    pub const HEADER: u8 = 0x00;
    pub const CALL_MANAGEMENT: u8 = 0x01;
    pub const ACM: u8 = 0x02;
    pub const ABSTRACT_CONTROL_MANAGEMENT: u8 = 0x02;
    pub const UNION: u8 = 0x06;
}
