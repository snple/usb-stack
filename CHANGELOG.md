# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0] - 2026-11-20

The crate now covers both directions of a USB link and is split accordingly.
Nothing was kept for compatibility with 0.1: import paths moved.

### Added

- Host side: `host::HostController` trait (port state, bus reset, channels,
  one packet per transaction), `host::Host` (port reset + enumeration),
  `host::Device` (control transfers, string decoding, `Pipe` creation),
  `host::Pipe` (bulk and interrupt transfers with the data toggle tracked by the
  stack), descriptor parsing (`host::Configuration`, `Interface`,
  `EndpointDescriptor`) and a HID boot keyboard driver (`host::class::hid::Keyboard`).
- `request::Request` now builds as well as parses SETUP packets, so the host
  constructs the same type the device side parses (`to_bytes`, `get_descriptor`,
  `set_address`, `set_configuration`, `set_interface`, `get_status`,
  `set_feature`, `clear_feature`, `clear_halt`, `custom`).
- `request::Request::parse` returns `Option` instead of `UsbResult`.
- Host classes live under `host::class`, mirroring `device::class`.
- `host` tests: a simulated boot keyboard exercising enumeration order, control
  stage toggles, string descriptors, pipes and STALL propagation.

### Changed

- The device stack moved under `device`: `device::bus`, `device::class`,
  `device::control`, `device::descriptor` (`DescriptorWriter`), `device::error`,
  `device::endpoint` (the `Endpoint` handles) and `device::UsbDevice`.
  `device` re-exports the shared vocabulary, so a class usually only imports
  `usb_stack::device`.
- The vocabulary both directions share lives at the crate root:
  `endpoint` (numbers, directions, transfer types), `descriptor` (descriptor and
  class codes) and `request` (SETUP packets, request types, standard requests).
- `MAX_ENDPOINTS`, `CONTROL_MAX_PACKET_SIZE` and `BULK_MAX_PACKET_SIZE` stay at
  the crate root.

## [0.1.0] - 2026-11-20

First release, extracted from the `rp2350-usb`, `ch32x035-usb` and
`esp32p4-usb` HAL crates.

### Added

- `UsbBus` trait (packet-oriented, speed agnostic) with `UsbBusAllocator` for
  endpoints, interfaces and string descriptors, and the `MAX_PACKET_SIZE`
  associated constant so classes size their endpoints per bus speed (64 bytes
  full speed, 512 high speed).
- `UsbDevice` / `UsbDeviceBuilder`: device state machine, standard control
  transfers, descriptor assembly and control-transfer dispatch.
- `ControlPipe` EP0 state machine with `ControlIn` / `ControlOut` handles;
  the EP0 packet size is configurable per device.
- `DescriptorWriter` plus standard descriptor, class, request and language-ID
  constants.
- Endpoint abstractions: `EndpointAddress`, `Direction`, `EndpointType`,
  `Endpoint` handles with `read` / `write` / `stall`.
- `UsbClass` trait and ready-made classes: CDC-ACM (`write_all`, interface
  getters), HID (boot keyboard, boot mouse, custom reports, HID Power Device),
  GS_USB (SocketCAN over USB), i2c-tiny-usb and UAC 1.0 audio.
- Optional `defmt` logging; the crate itself has no mandatory dependencies and
  is `no_std`.

[Unreleased]: https://github.com/snple/usb-stack/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/snple/usb-stack/releases/tag/v0.2.0
[0.1.0]: https://github.com/snple/usb-stack/releases/tag/v0.1.0
