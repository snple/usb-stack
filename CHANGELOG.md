# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

[Unreleased]: https://github.com/snple/usb-stack/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/snple/usb-stack/releases/tag/v0.1.0
