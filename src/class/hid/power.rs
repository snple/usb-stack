//! HID Power Device (UPS, power supply, battery system).
//!
//! Implements the *Universal Serial Bus Usage Tables for HID Power Devices*
//! (Power Device page `0x84` and Battery System page `0x85`).  A power device is
//! an ordinary HID device: it uses the HID descriptors and endpoints, and only
//! differs in its report descriptor and report layout.
//!
//! # Reports
//!
//! | Report ID | Type | Direction | Payload |
//! |-----------|------|-----------|---------|
//! | 1 | Input | device → host | [`PowerStatus`] present + changed status |
//! | 2 | Input | device → host | [`PowerMeasures`] voltage, current, … |
//! | 3 | Input | device → host | [`BatteryState`] battery flags and charge |
//! | 4 | Feature | both | [`PowerConfig`] configuration values |
//! | 5 | Output | host → device | [`PowerControls`] switch commands |
//!
//! Input reports are pushed to the interrupt IN endpoint by
//! [`PowerDevice::poll`], which the [`HidClass`](super::HidClass) calls on every
//! device poll.  A report whose contents did not change is not re-sent, so the
//! host sees a new input report whenever something actually changed.
//!
//! Feature writes (report 4) and output reports (report 5) are buffered and
//! picked up by the application through [`PowerDevice::take_config_write`] and
//! [`PowerDevice::take_controls`].  Feature reads return the values last passed
//! to [`PowerDevice::set_config`] — i.e. what the device actually applied, not
//! what the host requested.
//!
//! # Units
//!
//! Values follow the units in the specification's HID unit table:
//!
//! | Field | Unit | Scale |
//! |-------|------|-------|
//! | Voltage, transfer voltages | Volt (unit exponent −2) | centivolt |
//! | Current | Amp (unit exponent −2) | centiamp |
//! | Frequency | Hertz | hertz |
//! | Percent load, state of charge | none | percent |
//! | Temperature | Kelvin | kelvin |
//! | Delays | second | second |
//! | Runtime to empty | none | minute (as in Smart Battery) |
//!
//! Numeric fields are clamped to the logical range `0..=32767` declared in the
//! report descriptor.
//!
//! # Example
//!
//! ```ignore
//! use my_hal_usb::{UsbBusAllocator, UsbDeviceBuilder};
//! use my_hal_usb::class::hid::{HidClass, power::{PowerConfig, PowerDevice, PowerStatus}};
//! use my_hal_usb::MyBus;
//!
//! let alloc = UsbBusAllocator::new(MyBus::new(/* peripheral */));
//! let mut power = HidClass::new(&alloc, PowerDevice::power_supply());
//! let mut dev = UsbDeviceBuilder::new(&alloc, 0x2E8A, 0x000D)
//!     .product("Bench Power Supply")
//!     .build();
//!
//! power.device_mut().set_config(PowerConfig::default());
//! power.device_mut().set_status(
//!     PowerStatus::PRESENT | PowerStatus::GOOD | PowerStatus::USED,
//!     PowerStatus::NONE,
//! );
//!
//! loop {
//!     dev.poll(&mut [&mut power]);
//!
//!     if let Some(controls) = power.device_mut().take_controls() {
//!         // host pressed a switch in the output report
//!     }
//! }
//! ```

use crate::bus::UsbBus;
use crate::class::hid::{HidDevice, HidSpec, ReportType};
use crate::endpoint::EndpointIn;

// ─── Usage tables ─────────────────────────────────────────────────────────────

/// Power Device page (`0x84`) usage IDs.
pub mod power_usage {
    /// Index of a string describing the object.
    pub const I_NAME: u8 = 0x01;
    /// Status collection (logical).
    pub const PRESENT_STATUS: u8 = 0x02;
    /// Status-changed collection (logical).
    pub const CHANGED_STATUS: u8 = 0x03;
    /// Uninterruptible Power Supply application collection.
    pub const UPS: u8 = 0x04;
    /// Power Supply application collection.
    pub const POWER_SUPPLY: u8 = 0x05;
    /// Battery (physical collection on the Battery System page).
    pub const BATTERY: u8 = 0x12;
    /// Measured voltage.
    pub const VOLTAGE: u8 = 0x30;
    /// Measured current.
    pub const CURRENT: u8 = 0x31;
    /// Measured frequency.
    pub const FREQUENCY: u8 = 0x32;
    /// Measured load relative to the rated load.
    pub const PERCENT_LOAD: u8 = 0x35;
    /// Measured temperature.
    pub const TEMPERATURE: u8 = 0x36;
    /// Number of bad (unusable) measurements.
    pub const BAD_COUNT: u8 = 0x38;
    /// Configured (nominal) voltage.
    pub const CONFIG_VOLTAGE: u8 = 0x40;
    /// Configured (nominal) current.
    pub const CONFIG_CURRENT: u8 = 0x41;
    /// Configured (nominal) frequency.
    pub const CONFIG_FREQUENCY: u8 = 0x42;
    /// Configured (nominal) temperature.
    pub const CONFIG_TEMPERATURE: u8 = 0x46;
    /// Command: switch the output on.
    pub const SWITCH_ON_CONTROL: u8 = 0x50;
    /// Command: switch the output off.
    pub const SWITCH_OFF_CONTROL: u8 = 0x51;
    /// Command: toggle the output.
    pub const TOGGLE_CONTROL: u8 = 0x52;
    /// Voltage below which the device transfers to battery power.
    pub const LOW_VOLTAGE_TRANSFER: u8 = 0x53;
    /// Voltage above which the device transfers back to line power.
    pub const HIGH_VOLTAGE_TRANSFER: u8 = 0x54;
    /// Grace period between a shutdown request and power removal.
    pub const DELAY_BEFORE_SHUTDOWN: u8 = 0x57;
    /// Command: sound the audible alarm.
    pub const AUDIBLE_ALARM_CONTROL: u8 = 0x5A;
    /// Status flag: a power source is connected.
    pub const PRESENT: u8 = 0x60;
    /// Status flag: no failure detected.
    pub const GOOD: u8 = 0x61;
    /// Status flag: internal failure.
    pub const INTERNAL_FAILURE: u8 = 0x62;
    /// Status flag: voltage out of range.
    pub const VOLTAGE_OUT_OF_RANGE: u8 = 0x63;
    /// Status flag: frequency out of range.
    pub const FREQUENCY_OUT_OF_RANGE: u8 = 0x64;
    /// Status flag: overload.
    pub const OVERLOAD: u8 = 0x65;
    /// Status flag: over charged.
    pub const OVER_CHARGED: u8 = 0x66;
    /// Status flag: over temperature.
    pub const OVER_TEMPERATURE: u8 = 0x67;
    /// Status flag: shutdown requested.
    pub const SHUTDOWN_REQUESTED: u8 = 0x68;
    /// Status flag: shutdown imminent.
    pub const SHUTDOWN_IMMINENT: u8 = 0x69;
    /// Status flag: output is switched on.
    pub const SWITCH_ON_OFF: u8 = 0x6B;
    /// Status flag: output can be switched.
    pub const SWITCHABLE: u8 = 0x6C;
    /// Status flag: device is in use.
    pub const USED: u8 = 0x6D;
    /// Status flag: boosting.
    pub const BOOST: u8 = 0x6E;
    /// Status flag: bucking.
    pub const BUCK: u8 = 0x6F;
    /// Status flag: initialized.
    pub const INITIALIZED: u8 = 0x70;
    /// Status flag: tested.
    pub const TESTED: u8 = 0x71;
    /// Status flag: awaiting power.
    pub const AWAITING_POWER: u8 = 0x72;
    /// Status flag: communication with a module lost.
    pub const COMMUNICATION_LOST: u8 = 0x73;
    /// String index of the manufacturer.
    pub const I_MANUFACTURER: u8 = 0xFD;
    /// String index of the product.
    pub const I_PRODUCT: u8 = 0xFE;
    /// String index of the serial number.
    pub const I_SERIAL_NUMBER: u8 = 0xFF;
}

/// Battery System page (`0x85`) usage IDs.
pub mod battery_usage {
    /// Battery (physical collection).
    pub const BATTERY: u8 = 0x12;
    /// Status flag: remaining capacity below the alarm limit.
    pub const BELOW_REMAINING_CAPACITY_LIMIT: u8 = 0x42;
    /// Status flag: charging.
    pub const CHARGING: u8 = 0x44;
    /// Status flag: discharging.
    pub const DISCHARGING: u8 = 0x45;
    /// Status flag: fully charged.
    pub const FULLY_CHARGED: u8 = 0x46;
    /// Status flag: battery needs replacement.
    pub const NEED_REPLACEMENT: u8 = 0x4B;
    /// Remaining capacity as a percentage of full charge.
    pub const RELATIVE_STATE_OF_CHARGE: u8 = 0x64;
    /// Remaining capacity in the units selected by `CapacityMode`.
    pub const REMAINING_CAPACITY: u8 = 0x66;
    /// Remaining runtime at the present discharge rate, in minutes.
    pub const RUN_TIME_TO_EMPTY: u8 = 0x68;
    /// Status flag: AC (line) power present.
    pub const AC_PRESENT: u8 = 0xD0;
    /// Status flag: battery present.
    pub const BATTERY_PRESENT: u8 = 0xD1;
}

/// Report IDs used by a [`PowerDevice`].
pub mod report_id {
    /// Input report: present and changed status.
    pub const STATUS: u8 = 1;
    /// Input report: measurements.
    pub const MEASURES: u8 = 2;
    /// Input report: battery status and charge.
    pub const BATTERY: u8 = 3;
    /// Feature report: configuration values.
    pub const CONFIG: u8 = 4;
    /// Output report: switch controls.
    pub const CONTROLS: u8 = 5;
}

// ─── Constants ────────────────────────────────────────────────────────────────

/// Largest value representable by the `Logical Maximum (32767)` fields.
const LOGICAL_MAX_16: u16 = 0x7FFF;

/// Interrupt IN endpoint size; fits the largest input report including its ID.
const IN_ENDPOINT_SIZE: u16 = 16;
/// Interrupt OUT endpoint size; fits the one-byte control report plus its ID.
const OUT_ENDPOINT_SIZE: u16 = 8;
/// Polling interval of both interrupt endpoints, in milliseconds.
const POLL_INTERVAL_MS: u8 = 16;

/// Clamp a value to the logical range declared in the descriptor.
fn clamp16(value: u16) -> u16 {
    if value > LOGICAL_MAX_16 {
        LOGICAL_MAX_16
    } else {
        value
    }
}

/// Clamp a percentage to `0..=100`.
fn clamp_percent(value: u8) -> u8 {
    if value > 100 { 100 } else { value }
}

/// Copy `report` into `buf`, truncated to what the host asked for.
fn copy_report(report: &[u8], buf: &mut [u8]) -> usize {
    let len = report.len().min(buf.len());
    buf[..len].copy_from_slice(&report[..len]);
    len
}

// ─── Report descriptor ───────────────────────────────────────────────────────

/// Builds a power device report descriptor for the given application usage.
///
/// The only byte that depends on the device type is the application usage at
/// offset 3, so the descriptor is generated by a macro instead of being
/// duplicated (or patched at run time, which would prevent handing the
/// descriptor to the control pipe as a `'static` slice).
macro_rules! power_report_descriptor {
    ($app_usage:expr) => {
        &[
            // ── Application collection ─────────────────────────────────────
            0x05, 0x84, // Usage Page (Power Device)
            0x09, $app_usage, // Usage (UPS / Power Supply)
            0xA1, 0x01, // Collection (Application)
            // ── Report ID 1: status ────────────────────────────────────────
            0x85, 0x01, //   Report ID (1)
            0x09, 0x02, //   Usage (PresentStatus)
            0xA1, 0x02, //     Collection (Logical)
            0x19, 0x60, //       Usage Minimum (Present)
            0x29, 0x69, //       Usage Maximum (ShutdownImminent)
            0x15, 0x00, //       Logical Minimum (0)
            0x25, 0x01, //       Logical Maximum (1)
            0x75, 0x01, //       Report Size (1)
            0x95, 0x0A, //       Report Count (10)
            0x81, 0x02, //       Input (Data, Variable, Absolute)
            0x19, 0x6B, //       Usage Minimum (SwitchOn/Off)
            0x29, 0x73, //       Usage Maximum (CommunicationLost)
            0x95, 0x09, //       Report Count (9)
            0x81, 0x02, //       Input (Data, Variable, Absolute)
            0x75, 0x05, //       Report Size (5)
            0x95, 0x01, //       Report Count (1)
            0x81, 0x01, //       Input (Constant)      ; pad to 3 bytes
            0xC0, //     End Collection
            0x09, 0x03, //   Usage (ChangedStatus)
            0xA1, 0x02, //     Collection (Logical)
            0x75, 0x01, //       Report Size (1)
            0x19, 0x60, 0x29, 0x69, //       Usage Minimum/Maximum (Present … ShutdownImminent)
            0x95, 0x0A, 0x81, 0x02, //       Input (10 × 1 bit)
            0x19, 0x6B, 0x29, 0x73, //       Usage Minimum/Maximum (SwitchOn/Off … CommunicationLost)
            0x95, 0x09, 0x81, 0x02, //       Input (9 × 1 bit)
            0x75, 0x05, 0x95, 0x01, 0x81, 0x01, //       Input (Constant)  ; pad to 3 bytes
            0xC0, //     End Collection
            // ── Report ID 2: measurements ──────────────────────────────────
            0x85, 0x02, //   Report ID (2)
            0x05, 0x84, //   Usage Page (Power Device)
            0x09, 0x30, //   Usage (Voltage)
            0x15, 0x00, 0x26, 0xFF, 0x7F, //     Logical 0 … 32767
            0x75, 0x10, 0x95, 0x01, //           Report Size (16), Count (1)
            0x67, 0x21, 0xD1, 0xF0, 0x00, //     Unit (Volt)
            0x55, 0x0E, //                       Unit Exponent (-2)  ; centivolts
            0x81, 0x02, //     Input (Data, Variable, Absolute)
            0x09, 0x31, //   Usage (Current)
            0x67, 0x01, 0x00, 0x10, 0x00, //     Unit (Amp)
            0x55, 0x0E, //                       Unit Exponent (-2)  ; centiamps
            0x81, 0x02, //     Input (Data, Variable, Absolute)
            0x09, 0x32, //   Usage (Frequency)
            0x25, 0x64, //     Logical Maximum (100)
            0x75, 0x08, //     Report Size (8)
            0x67, 0x01, 0xF0, 0x00, 0x00, //     Unit (Hertz)
            0x55, 0x00, //                       Unit Exponent (0)
            0x81, 0x02, //     Input (Data, Variable, Absolute)
            0x09, 0x35, //   Usage (PercentLoad)
            0x65, 0x00, //     Unit (None)
            0x81, 0x02, //     Input (Data, Variable, Absolute)
            0x09, 0x36, //   Usage (Temperature)
            0x26, 0xFF, 0x7F, //     Logical Maximum (32767)
            0x75, 0x10, //     Report Size (16)
            0x67, 0x01, 0x00, 0x01, 0x00, //     Unit (Kelvin)
            0x55, 0x00, //                       Unit Exponent (0)
            0x81, 0x02, //     Input (Data, Variable, Absolute)
            // ── Report ID 3: battery ───────────────────────────────────────
            0x05, 0x85, //     Usage Page (Battery System)
            0x09, 0x12, //     Usage (Battery)
            0xA1, 0x00, //     Collection (Physical)
            0x85, 0x03, //       Report ID (3)
            0x05, 0x84, 0x09, 0x02, //       Usage (PresentStatus)
            0xA1, 0x02, //         Collection (Logical)
            0x05, 0x85, //           Usage Page (Battery System)
            0x09, 0x44, //           Usage (Charging)
            0x09, 0x45, //           Usage (Discharging)
            0x09, 0x46, //           Usage (FullyCharged)
            0x09, 0x42, //           Usage (BelowRemainingCapacityLimit)
            0x09, 0xD0, //           Usage (ACPresent)
            0x09, 0xD1, //           Usage (BatteryPresent)
            0x09, 0x4B, //           Usage (NeedReplacement)
            0x15, 0x00, 0x25, 0x01, //           Logical 0 … 1
            0x65, 0x00, //                         Unit (None)
            0x75, 0x01, 0x95, 0x07, //           Report Size (1), Count (7)
            0x81, 0x02, //             Input (Data, Variable, Absolute)
            0x95, 0x01, 0x81, 0x01, //             Input (Constant)  ; pad
            0xC0, //         End Collection
            0x05, 0x84, 0x09, 0x03, //       Usage (ChangedStatus)
            0xA1, 0x02, //         Collection (Logical)
            0x05, 0x85, //           Usage Page (Battery System)
            0x09, 0x44, 0x09, 0x45, 0x09, 0x46, 0x09, 0x42, //  Charging … BelowRemainingCapacityLimit
            0x09, 0xD0, 0x09, 0xD1, 0x09, 0x4B, //              ACPresent … NeedReplacement
            0x95, 0x07, 0x81, 0x02, //             Input (7 × 1 bit)
            0x95, 0x01, 0x81, 0x01, //             Input (Constant)  ; pad
            0xC0, //         End Collection
            0x05, 0x85, 0x09, 0x64, //       Usage (RelativeStateOfCharge)
            0x15, 0x00, 0x25, 0x64, //         Logical 0 … 100
            0x75, 0x08, 0x95, 0x01, 0x81, 0x02, //  Input (8 bits, percent)
            0x09, 0x68, //       Usage (RunTimeToEmpty)  ; minutes
            0x15, 0x00, 0x26, 0xFF, 0x7F, //     Logical 0 … 32767
            0x75, 0x10, 0x95, 0x01, 0x81, 0x02, //  Input (16 bits)
            0xC0, //     End Collection
            // ── Report ID 4: configuration ─────────────────────────────────
            0x85, 0x04, //   Report ID (4)
            0x05, 0x84, //   Usage Page (Power Device)
            0x09, 0x40, //   Usage (ConfigVoltage)
            0x15, 0x00, 0x26, 0xFF, 0x7F, //     Logical 0 … 32767
            0x75, 0x10, 0x95, 0x01, //           Report Size (16), Count (1)
            0x67, 0x21, 0xD1, 0xF0, 0x00, //     Unit (Volt)
            0x55, 0x0E, //                       Unit Exponent (-2)  ; centivolts
            0xB1, 0x02, //     Feature (Data, Variable, Absolute)
            0x09, 0x41, //   Usage (ConfigCurrent)
            0x67, 0x01, 0x00, 0x10, 0x00, //     Unit (Amp)
            0x55, 0x0E, //                       Unit Exponent (-2)  ; centiamps
            0xB1, 0x02, //     Feature (Data, Variable, Absolute)
            0x09, 0x42, //   Usage (ConfigFrequency)
            0x25, 0x64, //     Logical Maximum (100)
            0x75, 0x08, //     Report Size (8)
            0x67, 0x01, 0xF0, 0x00, 0x00, //     Unit (Hertz)
            0x55, 0x00, //                       Unit Exponent (0)
            0xB1, 0x02, //     Feature (Data, Variable, Absolute)
            0x09, 0x53, //   Usage (LowVoltageTransfer)
            0x26, 0xFF, 0x7F, //     Logical Maximum (32767)
            0x75, 0x10, //     Report Size (16)
            0x67, 0x21, 0xD1, 0xF0, 0x00, //     Unit (Volt)
            0x55, 0x0E, //                       Unit Exponent (-2)  ; centivolts
            0xB1, 0x02, //     Feature (Data, Variable, Absolute)
            0x09, 0x54, //   Usage (HighVoltageTransfer)
            0xB1, 0x02, //     Feature (Data, Variable, Absolute)
            0x09, 0x57, //   Usage (DelayBeforeShutdown)
            0x67, 0x01, 0x10, 0x00, 0x00, //     Unit (second)
            0x55, 0x00, //                       Unit Exponent (0)
            0xB1, 0x02, //     Feature (Data, Variable, Absolute)
            0x09, 0x46, //   Usage (ConfigTemperature)
            0x67, 0x01, 0x00, 0x01, 0x00, //     Unit (Kelvin)
            0xB1, 0x02, //     Feature (Data, Variable, Absolute)
            // ── Report ID 5: controls ──────────────────────────────────────
            0x85, 0x05, //   Report ID (5)
            0x05, 0x84, //   Usage Page (Power Device)
            0x09, 0x50, //   Usage (SwitchOnControl)
            0x09, 0x51, //   Usage (SwitchOffControl)
            0x09, 0x52, //   Usage (ToggleControl)
            0x09, 0x5A, //   Usage (AudibleAlarmControl)
            0x15, 0x00, 0x25, 0x01, //     Logical 0 … 1
            0x65, 0x00, //                 Unit (None)
            0x75, 0x01, 0x95, 0x04, //     Report Size (1), Count (4)
            0x91, 0x02, //     Output (Data, Variable, Absolute)
            0x75, 0x04, 0x95, 0x01, 0x91, 0x01, //     Output (Constant)  ; pad
            0xC0, // End Collection (Application)
        ]
    };
}

/// Report descriptor of a UPS ([`PowerDeviceType::Ups`]).
pub const UPS_REPORT_DESCRIPTOR: &[u8] = power_report_descriptor!(power_usage::UPS);

/// Report descriptor of a power supply ([`PowerDeviceType::PowerSupply`]).
pub const POWER_SUPPLY_REPORT_DESCRIPTOR: &[u8] =
    power_report_descriptor!(power_usage::POWER_SUPPLY);

/// Application collection usage — the device type the host sees.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PowerDeviceType {
    /// Uninterruptible power supply: has a battery and reports report 3.
    Ups,
    /// Power supply without a battery.
    PowerSupply,
}

impl PowerDeviceType {
    /// Power Device page usage ID of the application collection.
    pub const fn usage(self) -> u8 {
        match self {
            Self::Ups => power_usage::UPS,
            Self::PowerSupply => power_usage::POWER_SUPPLY,
        }
    }

    /// Report descriptor for this device type.
    pub const fn descriptor(self) -> &'static [u8] {
        match self {
            Self::Ups => UPS_REPORT_DESCRIPTOR,
            Self::PowerSupply => POWER_SUPPLY_REPORT_DESCRIPTOR,
        }
    }
}

// ─── Report payloads ─────────────────────────────────────────────────────────

/// Present-status and changed-status flags (input report 1).
///
/// Bit *n* maps to the *n*-th flag of the `PresentStatus` collection, which the
/// report descriptor declares as `Usage Minimum (Present) … Usage Maximum
/// (CommunicationLost)`.
#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PowerStatus(u32);

impl PowerStatus {
    /// Size of the packed report field, in bytes (19 flags plus 5 pad bits).
    pub const SIZE: usize = 3;

    /// No flags set.
    pub const NONE: Self = Self(0);
    /// A power source is connected.
    pub const PRESENT: Self = Self(1 << 0);
    /// The device is working correctly.
    pub const GOOD: Self = Self(1 << 1);
    /// Internal failure.
    pub const INTERNAL_FAILURE: Self = Self(1 << 2);
    /// Voltage out of range.
    pub const VOLTAGE_OUT_OF_RANGE: Self = Self(1 << 3);
    /// Frequency out of range.
    pub const FREQUENCY_OUT_OF_RANGE: Self = Self(1 << 4);
    /// Overload.
    pub const OVERLOAD: Self = Self(1 << 5);
    /// Over charged.
    pub const OVER_CHARGED: Self = Self(1 << 6);
    /// Over temperature.
    pub const OVER_TEMPERATURE: Self = Self(1 << 7);
    /// Shutdown requested.
    pub const SHUTDOWN_REQUESTED: Self = Self(1 << 8);
    /// Shutdown imminent.
    pub const SHUTDOWN_IMMINENT: Self = Self(1 << 9);
    /// The output is switched on.
    pub const SWITCH_ON_OFF: Self = Self(1 << 10);
    /// The output can be switched.
    pub const SWITCHABLE: Self = Self(1 << 11);
    /// The device is in use.
    pub const USED: Self = Self(1 << 12);
    /// Boosting.
    pub const BOOST: Self = Self(1 << 13);
    /// Bucking.
    pub const BUCK: Self = Self(1 << 14);
    /// Initialized.
    pub const INITIALIZED: Self = Self(1 << 15);
    /// Tested.
    pub const TESTED: Self = Self(1 << 16);
    /// Awaiting power.
    pub const AWAITING_POWER: Self = Self(1 << 17);
    /// Communication with a module was lost.
    pub const COMMUNICATION_LOST: Self = Self(1 << 18);

    /// Mask of the 19 defined flags.
    const MASK: u32 = (1 << 19) - 1;

    /// Raw flag bits.
    pub const fn bits(self) -> u32 {
        self.0
    }

    /// `true` if every flag in `other` is set in `self`.
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// Union of two flag sets.
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// `self` with the flags in `other` cleared.
    pub const fn without(self, other: Self) -> Self {
        Self(self.0 & !other.0)
    }

    /// Flags that differ between `self` and `previous`.
    ///
    /// Feed the result into the `ChangedStatus` half of [`PowerDevice::set_status`].
    pub const fn diff(self, previous: Self) -> Self {
        Self((self.0 ^ previous.0) & Self::MASK)
    }

    /// Pack the flags into the three report bytes.
    pub fn to_bytes(self) -> [u8; Self::SIZE] {
        let bits = (self.0 & Self::MASK).to_le_bytes();
        [bits[0], bits[1], bits[2]]
    }

    /// Unpack the three report bytes.
    pub fn from_bytes(bytes: &[u8]) -> Self {
        let mut bits = 0u32;
        for (i, byte) in bytes.iter().take(Self::SIZE).enumerate() {
            bits |= u32::from(*byte) << (8 * i);
        }
        Self(bits & Self::MASK)
    }
}

impl core::ops::BitOr for PowerStatus {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self {
        self.union(rhs)
    }
}

/// Battery status flags (input report 3).
#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BatteryStatus(u8);

impl BatteryStatus {
    /// Size of the packed report field, in bytes (7 flags plus 1 pad bit).
    pub const SIZE: usize = 1;

    /// No flags set.
    pub const NONE: Self = Self(0);
    /// The battery is charging.
    pub const CHARGING: Self = Self(1 << 0);
    /// The battery is discharging.
    pub const DISCHARGING: Self = Self(1 << 1);
    /// The battery is fully charged.
    pub const FULLY_CHARGED: Self = Self(1 << 2);
    /// Remaining capacity below the alarm limit.
    pub const BELOW_REMAINING_CAPACITY_LIMIT: Self = Self(1 << 3);
    /// AC (line) power is present.
    pub const AC_PRESENT: Self = Self(1 << 4);
    /// A battery is present.
    pub const BATTERY_PRESENT: Self = Self(1 << 5);
    /// The battery needs replacement.
    pub const NEED_REPLACEMENT: Self = Self(1 << 6);

    /// Mask of the 7 defined flags.
    const MASK: u8 = (1 << 7) - 1;

    /// Raw flag bits.
    pub const fn bits(self) -> u8 {
        self.0
    }

    /// `true` if every flag in `other` is set in `self`.
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// Union of two flag sets.
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// `self` with the flags in `other` cleared.
    pub const fn without(self, other: Self) -> Self {
        Self(self.0 & !other.0)
    }

    /// Flags that differ between `self` and `previous`.
    pub const fn diff(self, previous: Self) -> Self {
        Self((self.0 ^ previous.0) & Self::MASK)
    }

    /// Pack the flags into the report byte.
    pub const fn to_byte(self) -> u8 {
        self.0 & Self::MASK
    }

    /// Unpack the report byte.
    pub const fn from_byte(byte: u8) -> Self {
        Self(byte & Self::MASK)
    }
}

impl core::ops::BitOr for BatteryStatus {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self {
        self.union(rhs)
    }
}

/// Measurements (input report 2).
#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PowerMeasures {
    /// Output voltage in centivolts (5.00 V → 500).
    pub voltage_cv: u16,
    /// Output current in centiamps (1.25 A → 125).
    pub current_ca: u16,
    /// Line frequency in hertz; 0 when running from a battery.
    pub frequency_hz: u8,
    /// Load as a percentage of the rated load (0–100).
    pub percent_load: u8,
    /// Temperature in kelvin (25 °C → 298).
    pub temperature_k: u16,
}

impl PowerMeasures {
    /// Size of the packed report, in bytes.
    pub const SIZE: usize = 8;

    /// Pack the measurements into the report bytes.
    pub fn to_bytes(self) -> [u8; Self::SIZE] {
        let voltage = clamp16(self.voltage_cv).to_le_bytes();
        let current = clamp16(self.current_ca).to_le_bytes();
        let temperature = clamp16(self.temperature_k).to_le_bytes();
        [
            voltage[0],
            voltage[1],
            current[0],
            current[1],
            self.frequency_hz.min(100),
            clamp_percent(self.percent_load),
            temperature[0],
            temperature[1],
        ]
    }
}

/// Battery status and charge (input report 3).
///
/// Only meaningful for [`PowerDeviceType::Ups`].
#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BatteryState {
    /// Current flags.
    pub status: BatteryStatus,
    /// Flags that changed since the previous report.
    pub changed: BatteryStatus,
    /// Remaining capacity as a percentage of full charge (0–100).
    pub state_of_charge_pct: u8,
    /// Estimated runtime at the present load, in minutes.
    pub run_time_to_empty_min: u16,
}

impl BatteryState {
    /// Size of the packed report, in bytes.
    pub const SIZE: usize = 5;

    /// Pack the state into the report bytes.
    pub fn to_bytes(self) -> [u8; Self::SIZE] {
        let run_time = clamp16(self.run_time_to_empty_min).to_le_bytes();
        [
            self.status.to_byte(),
            self.changed.to_byte(),
            clamp_percent(self.state_of_charge_pct),
            run_time[0],
            run_time[1],
        ]
    }
}

/// Configuration values (feature report 4).
#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PowerConfig {
    /// Nominal output voltage in centivolts.
    pub voltage_cv: u16,
    /// Nominal output current in centiamps.
    pub current_ca: u16,
    /// Nominal line frequency in hertz.
    pub frequency_hz: u8,
    /// Voltage below which the device transfers to battery power, in centivolts.
    pub low_voltage_transfer_cv: u16,
    /// Voltage above which the device transfers back to line power, in centivolts.
    pub high_voltage_transfer_cv: u16,
    /// Grace period between a shutdown request and power removal, in seconds.
    pub delay_before_shutdown_s: u16,
    /// Nominal temperature in kelvin.
    pub temperature_k: u16,
}

impl PowerConfig {
    /// Size of the packed report, in bytes.
    pub const SIZE: usize = 13;

    /// Pack the configuration into the report bytes.
    pub fn to_bytes(self) -> [u8; Self::SIZE] {
        let voltage = clamp16(self.voltage_cv).to_le_bytes();
        let current = clamp16(self.current_ca).to_le_bytes();
        let low = clamp16(self.low_voltage_transfer_cv).to_le_bytes();
        let high = clamp16(self.high_voltage_transfer_cv).to_le_bytes();
        let delay = clamp16(self.delay_before_shutdown_s).to_le_bytes();
        let temperature = clamp16(self.temperature_k).to_le_bytes();
        [
            voltage[0],
            voltage[1],
            current[0],
            current[1],
            self.frequency_hz.min(100),
            low[0],
            low[1],
            high[0],
            high[1],
            delay[0],
            delay[1],
            temperature[0],
            temperature[1],
        ]
    }

    /// Unpack a configuration report written by the host.
    ///
    /// Returns `None` if the report is shorter than [`PowerConfig::SIZE`];
    /// bytes past the report are ignored.
    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            voltage_cv: u16::from_le_bytes([bytes[0], bytes[1]]),
            current_ca: u16::from_le_bytes([bytes[2], bytes[3]]),
            frequency_hz: bytes[4],
            low_voltage_transfer_cv: u16::from_le_bytes([bytes[5], bytes[6]]),
            high_voltage_transfer_cv: u16::from_le_bytes([bytes[7], bytes[8]]),
            delay_before_shutdown_s: u16::from_le_bytes([bytes[9], bytes[10]]),
            temperature_k: u16::from_le_bytes([bytes[11], bytes[12]]),
        })
    }
}

/// Switch commands (output report 5).
#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PowerControls(u8);

impl PowerControls {
    /// Size of the packed report, in bytes.
    pub const SIZE: usize = 1;

    /// No command.
    pub const NONE: Self = Self(0);
    /// Switch the output on.
    pub const SWITCH_ON: Self = Self(1 << 0);
    /// Switch the output off.
    pub const SWITCH_OFF: Self = Self(1 << 1);
    /// Toggle the output.
    pub const TOGGLE: Self = Self(1 << 2);
    /// Sound the audible alarm.
    pub const AUDIBLE_ALARM: Self = Self(1 << 3);

    /// Mask of the 4 defined commands.
    const MASK: u8 = (1 << 4) - 1;

    /// Raw command bits.
    pub const fn bits(self) -> u8 {
        self.0
    }

    /// `true` if every command in `other` is set in `self`.
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    /// Union of two command sets.
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// `self` with the commands in `other` cleared.
    pub const fn without(self, other: Self) -> Self {
        Self(self.0 & !other.0)
    }

    /// Pack the commands into the report byte.
    pub const fn to_byte(self) -> u8 {
        self.0 & Self::MASK
    }

    /// Unpack the report byte.
    pub const fn from_byte(byte: u8) -> Self {
        Self(byte & Self::MASK)
    }
}

impl core::ops::BitOr for PowerControls {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self {
        self.union(rhs)
    }
}

// ─── Pending input report bits ────────────────────────────────────────────────

const PENDING_STATUS: u8 = 1 << 0;
const PENDING_MEASURES: u8 = 1 << 1;
const PENDING_BATTERY: u8 = 1 << 2;
const PENDING_ALL: u8 = PENDING_STATUS | PENDING_MEASURES | PENDING_BATTERY;

// ─── PowerDevice ─────────────────────────────────────────────────────────────

/// HID Power Device: a UPS or power supply.
///
/// Wrap it in a [`HidClass`](super::HidClass) to talk to the host, and update it
/// from the main loop through
/// [`device_mut`](super::HidClass::device_mut).
#[derive(Debug)]
pub struct PowerDevice {
    device_type: PowerDeviceType,

    present: PowerStatus,
    changed: PowerStatus,
    measures: PowerMeasures,
    battery: BatteryState,
    config: PowerConfig,

    pending_input: u8,
    pending_controls: PowerControls,
    pending_config: Option<PowerConfig>,
}

impl PowerDevice {
    /// Create a power device of the given type.
    pub fn new(device_type: PowerDeviceType) -> Self {
        Self {
            device_type,
            present: PowerStatus::NONE,
            changed: PowerStatus::NONE,
            measures: PowerMeasures::default(),
            battery: BatteryState::default(),
            config: PowerConfig::default(),
            pending_input: 0,
            pending_controls: PowerControls::NONE,
            pending_config: None,
        }
    }

    /// Create a UPS ([`PowerDeviceType::Ups`]).
    pub fn ups() -> Self {
        Self::new(PowerDeviceType::Ups)
    }

    /// Create a power supply without a battery ([`PowerDeviceType::PowerSupply`]).
    pub fn power_supply() -> Self {
        Self::new(PowerDeviceType::PowerSupply)
    }

    // ── Device information ────────────────────────────────────────────────────

    /// Device type this instance was created with.
    pub fn device_type(&self) -> PowerDeviceType {
        self.device_type
    }

    // ── Status and measurements ───────────────────────────────────────────────

    /// Present status last reported to the host.
    pub fn status(&self) -> PowerStatus {
        self.present
    }

    /// Configuration last applied by the application.
    pub fn config(&self) -> PowerConfig {
        self.config
    }

    /// Update the status flags and queue input report 1 if anything changed.
    ///
    /// Pass `changed` flags (see [`PowerStatus::diff`]) to tell the host which
    /// flags just changed; pass [`PowerStatus::NONE`] for a plain refresh.
    pub fn set_status(&mut self, present: PowerStatus, changed: PowerStatus) {
        if present != self.present || changed != self.changed {
            self.present = present;
            self.changed = changed;
            self.pending_input |= PENDING_STATUS;
        }
    }

    /// Update the measurements and queue input report 2 if anything changed.
    pub fn set_measures(&mut self, measures: PowerMeasures) {
        if measures != self.measures {
            self.measures = measures;
            self.pending_input |= PENDING_MEASURES;
        }
    }

    /// Update the battery state and queue input report 3 if anything changed.
    pub fn set_battery(&mut self, battery: BatteryState) {
        if battery != self.battery {
            self.battery = battery;
            self.pending_input |= PENDING_BATTERY;
        }
    }

    /// Store the configuration the device is actually running with.
    ///
    /// This is what `Get Report` returns, so call it after applying a
    /// configuration written by the host, and whenever the configuration
    /// changes for another reason.
    pub fn set_config(&mut self, config: PowerConfig) {
        self.config = config;
    }

    /// Queue all three input reports, even if nothing changed.
    pub fn request_refresh(&mut self) {
        self.pending_input |= PENDING_ALL;
    }

    // ── Host requests ─────────────────────────────────────────────────────────

    /// Take the switch commands received since the last call.
    ///
    /// Commands are accumulated until taken, so several reports may be coalesced
    /// into one value.
    pub fn take_controls(&mut self) -> Option<PowerControls> {
        if self.pending_controls == PowerControls::NONE {
            None
        } else {
            Some(core::mem::replace(
                &mut self.pending_controls,
                PowerControls::NONE,
            ))
        }
    }

    /// Take the configuration written by the host since the last call.
    ///
    /// The application is expected to apply the values and then publish them
    /// with [`set_config`](Self::set_config); until it does, `Get Report` keeps
    /// returning the previous configuration.
    pub fn take_config_write(&mut self) -> Option<PowerConfig> {
        self.pending_config.take()
    }

    // ── Report buffers ────────────────────────────────────────────────────────

    /// Input report 1 contents, including its report ID.
    fn status_report(&self) -> [u8; 1 + 2 * PowerStatus::SIZE] {
        let mut report = [0u8; 1 + 2 * PowerStatus::SIZE];
        report[0] = report_id::STATUS;
        report[1..1 + PowerStatus::SIZE].copy_from_slice(&self.present.to_bytes());
        report[1 + PowerStatus::SIZE..].copy_from_slice(&self.changed.to_bytes());
        report
    }

    /// Input report 2 contents, including its report ID.
    fn measures_report(&self) -> [u8; 1 + PowerMeasures::SIZE] {
        let mut report = [0u8; 1 + PowerMeasures::SIZE];
        report[0] = report_id::MEASURES;
        report[1..].copy_from_slice(&self.measures.to_bytes());
        report
    }

    /// Input report 3 contents, including its report ID.
    fn battery_report(&self) -> [u8; 1 + BatteryState::SIZE] {
        let mut report = [0u8; 1 + BatteryState::SIZE];
        report[0] = report_id::BATTERY;
        report[1..].copy_from_slice(&self.battery.to_bytes());
        report
    }

    /// Feature report 4 contents, including its report ID.
    fn config_report(&self) -> [u8; 1 + PowerConfig::SIZE] {
        let mut report = [0u8; 1 + PowerConfig::SIZE];
        report[0] = report_id::CONFIG;
        report[1..].copy_from_slice(&self.config.to_bytes());
        report
    }

    /// Apply an output report (either transport) to the pending commands.
    fn apply_controls(&mut self, data: &[u8]) -> bool {
        match data.first() {
            Some(&byte) => {
                self.pending_controls = self.pending_controls.union(PowerControls::from_byte(byte));
                true
            }
            None => false,
        }
    }

    /// Drop everything the host asked for but the application has not taken yet.
    fn clear_host_requests(&mut self) {
        self.pending_controls = PowerControls::NONE;
        self.pending_config = None;
    }
}

impl<B: UsbBus> HidDevice<B> for PowerDevice {
    fn spec(&self) -> HidSpec {
        HidSpec::new(self.device_type.descriptor(), IN_ENDPOINT_SIZE)
            .out_report_size(OUT_ENDPOINT_SIZE)
            .poll_interval_ms(POLL_INTERVAL_MS)
            .report_ids(true)
    }

    /// Serve the reports the host reads on demand: the three input reports and
    /// the configuration feature report.
    fn get_report(
        &mut self,
        report_type: ReportType,
        report_id: u8,
        buf: &mut [u8],
    ) -> Option<usize> {
        match (report_type, report_id) {
            (ReportType::Input, report_id::STATUS) => Some(copy_report(&self.status_report(), buf)),
            (ReportType::Input, report_id::MEASURES) => {
                Some(copy_report(&self.measures_report(), buf))
            }
            (ReportType::Input, report_id::BATTERY) => {
                Some(copy_report(&self.battery_report(), buf))
            }
            (ReportType::Feature, report_id::CONFIG) => {
                Some(copy_report(&self.config_report(), buf))
            }
            _ => None,
        }
    }

    /// Accept a configuration write or a switch command.
    fn set_report(&mut self, report_type: ReportType, report_id: u8, data: &[u8]) -> bool {
        match (report_type, report_id) {
            (ReportType::Feature, report_id::CONFIG) => match PowerConfig::from_bytes(data) {
                Some(config) => {
                    self.pending_config = Some(config);
                    true
                }
                None => false,
            },
            (ReportType::Output, report_id::CONTROLS) => self.apply_controls(data),
            _ => false,
        }
    }

    /// Same switch commands, this time written straight to the interrupt OUT
    /// endpoint.
    fn output_report(&mut self, report_id: u8, data: &[u8]) {
        if report_id == report_id::CONTROLS {
            self.apply_controls(data);
        }
    }

    fn reset(&mut self) {
        self.pending_input = 0;
        self.clear_host_requests();
    }

    fn configure(&mut self) {
        // Give the host a snapshot of the current state without waiting for a
        // change.
        self.request_refresh();
        self.clear_host_requests();
    }

    fn deconfigure(&mut self) {
        self.pending_input = 0;
        self.clear_host_requests();
    }

    /// Send the queued input reports, status first.
    ///
    /// A report the endpoint cannot take yet
    /// ([`WouldBlock`](crate::error::UsbError::WouldBlock)) stays queued and is
    /// retried on the next poll.
    fn poll(&mut self, input: &EndpointIn<'_, B>) {
        if self.pending_input & PENDING_STATUS != 0 {
            if input.write(&self.status_report()).is_err() {
                return;
            }
            self.pending_input &= !PENDING_STATUS;
        }

        if self.pending_input & PENDING_MEASURES != 0 {
            if input.write(&self.measures_report()).is_err() {
                return;
            }
            self.pending_input &= !PENDING_MEASURES;
        }

        if self.pending_input & PENDING_BATTERY != 0 {
            if input.write(&self.battery_report()).is_err() {
                return;
            }
            self.pending_input &= !PENDING_BATTERY;
        }
    }
}
