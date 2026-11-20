# usb-stack

[![crates.io](https://img.shields.io/crates/v/usb-stack.svg)](https://crates.io/crates/usb-stack)
[![docs.rs](https://docs.rs/usb-stack/badge.svg)](https://docs.rs/usb-stack)
[![license](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)

MCU-independent USB **device** stack core, shared by the
[`rp2350-usb`](https://github.com/snple/rp2350),
[`ch32x035-usb`](https://github.com/snple/ch32x035) and
[`esp32p4-usb`](https://github.com/snple/esp32p4) HAL crates.

The bus trait is packet-oriented and speed agnostic: a full-speed driver leaves
[`UsbBus::MAX_PACKET_SIZE`] at its 64-byte default, a high-speed one overrides
it (the ESP32-P4 HS core uses 512), and the classes size their endpoints from
it.

A microcontroller crate implements [`UsbBus`] for its USB peripheral and
re-exports this crate; everything else (device state machine, control pipe,
descriptors, classes) is hardware independent.

## Adding it to a project

```toml
[dependencies]
usb-stack = "0.1"
```

The crate is `no_std` and has no mandatory dependencies; enable the `defmt`
feature to log through `defmt`. It requires Rust 1.85 (edition 2024).

## Layers

| Module | Contents |
|--------|----------|
| `bus` | `UsbBus` trait, `UsbBusAllocator`, `PollResult` |
| `endpoint` | `EndpointAddress`, `EndpointType`, direction markers, endpoint handles |
| `control` | `ControlPipe` (EP0 state machine) and the `ControlIn` / `ControlOut` handles classes answer requests with |
| `descriptor` | `DescriptorWriter` and standard USB constants |
| `device` | `UsbDeviceBuilder` / `UsbDevice` — device state machine, descriptor serving, control-transfer dispatch |
| `class` | `UsbClass` trait, CDC-ACM, HID (boot keyboard, boot mouse, custom reports, HID Power Device), GS_USB (CAN), i2c-tiny-usb, UAC 1.0 |

## Implementing a bus

```rust,ignore
use usb_stack::{PollResult, UsbBus, Direction, EndpointAddress, EndpointType, UsbResult};

struct MyBus { /* peripheral state */ }

impl UsbBus for MyBus {
    fn alloc_ep(&mut self, dir: Direction, addr: Option<EndpointAddress>,
                ep_type: EndpointType, max_packet: u16, interval: u8)
        -> UsbResult<EndpointAddress> { /* … */ Ok(EndpointAddress::EP0_IN) }
    fn enable(&mut self) { /* attach + enable the peripheral */ }
    fn reset(&self) { /* re-init endpoints after a bus reset */ }
    fn set_device_address(&self, addr: u8) { /* … */ }
    fn poll(&self) -> PollResult { /* decode pending tokens, return masks */ }
    fn write(&self, addr: EndpointAddress, buf: &[u8]) -> UsbResult<usize> { /* … */ }
    fn read(&self, addr: EndpointAddress, buf: &mut [u8]) -> UsbResult<usize> { /* … */ }
    fn set_stalled(&self, addr: EndpointAddress, stalled: bool) { /* … */ }
    fn is_stalled(&self, addr: EndpointAddress) -> bool { /* … */ }
    fn suspend(&self) {}
    fn resume(&self) {}
}
```

The bus is packet-oriented: `poll` reports which endpoints completed an OUT
transfer, an IN transfer or received a SETUP packet, and the device layer
drives `read`/`write` from there. Implementations own their endpoint buffers
and must keep them alive for the lifetime of the bus.

## Features

- `defmt` (off by default) — log through `defmt`.

## Status

Version 0.1 — the API is still settling. Classes currently shipped: CDC-ACM,
HID (boot keyboard, boot mouse, custom reports, HID Power Device), GS_USB
(CAN), i2c-tiny-usb and UAC 1.0.

See [CHANGELOG.md](CHANGELOG.md) for the release history.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.

