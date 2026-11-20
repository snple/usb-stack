# usb-stack

[![crates.io](https://img.shields.io/crates/v/usb-stack.svg)](https://crates.io/crates/usb-stack)
[![docs.rs](https://img.shields.io/docsrs/usb-stack)](https://docs.rs/usb-stack)
[![license](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0.svg)](#license)

MCU-independent USB **device and host** stack core, shared by the
[`rp2350-usb`](https://github.com/snple/rp2350),
[`ch32x035-usb`](https://github.com/snple/ch32x035) and
[`esp32p4-usb`](https://github.com/snple/esp32p4) HAL crates.

Both directions follow the same rule: the protocol lives here, the peripheral
driver lives in the HAL crate and implements one trait. Device drivers implement
[`device::UsbBus`] (packet-oriented and speed agnostic: a full-speed driver
leaves `MAX_PACKET_SIZE` at its 64-byte default, a high-speed one overrides it),
host drivers implement [`host::HostController`] (port state, channels, one packet
per transaction).

## Adding it to a project

```toml
[dependencies]
usb-stack = "0.2"
```

The crate is `no_std` and has no mandatory dependencies; enable the `defmt`
feature to log through `defmt`. It requires Rust 1.85 (edition 2024).

## Layout

The crate is split by direction, with the vocabulary both need at the root.

| Module | Contents |
|--------|----------|
| `endpoint` | `EndpointAddress`, `Direction`, `EndpointType`, direction markers |
| `descriptor` | `bDescriptorType`, class, CDC and language-ID codes |
| `request` | `Request`: the 8-byte SETUP packet, parsed *and* built |
| `device` | `UsbBus` trait, `UsbBusAllocator`, `ControlPipe`, `UsbDevice` state machine, `DescriptorWriter`, endpoint handles, `UsbClass` and the classes |
| `host` | `HostController` trait, `Host`, `Device`, `Pipe`, descriptor parsing; `host::class` holds the host-side classes (HID boot keyboard) |

`device` and `host` both re-export the shared vocabulary, so a class driver or a
peripheral driver usually only imports one of them.

## Device side

```text
  application / class          device::class::cdc_acm, your own UsbClass
  ────────────────────────
  device::UsbDevice            enumeration, standard requests, EP0
  ────────────────────────
  device::UsbBus               trait: allocate, poll, read, write
  ────────────────────────
  (your HAL crate)             implements UsbBus
```

```rust,ignore
use usb_stack::device::{PollResult, UsbBus, UsbError, UsbResult, UsbDeviceBuilder};

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

## Host side

```text
  application / class              host::class::hid::Keyboard, your own driver
  ────────────────────────────
  host::Device / host::Pipe        enumeration, control transfers, bulk/interrupt
  ────────────────────────────
  host::HostController             trait: port, channels, transactions
  ────────────────────────────
  (your HAL crate)                 implements HostController
```

```rust,ignore
use usb_stack::host::{HostController, HostResult, PortStatus, Channel, DeviceAddress, EndpointConfig, Toggle};

struct MyHost { /* peripheral state */ }

impl HostController for MyHost {
    fn port_status(&self) -> PortStatus { /* … */ }
    fn reset_port(&self) -> HostResult<PortStatus> { /* bus reset + speed */ }
    fn alloc_channel(&self) -> HostResult<Channel> { /* … */ }
    fn free_channel(&self, channel: Channel) { /* … */ }
    fn configure(&self, channel: Channel, device: DeviceAddress,
                 endpoint: &EndpointConfig) -> HostResult<()> { /* … */ }
    fn setup(&self, channel: Channel, packet: &[u8]) -> HostResult<()> { /* … */ }
    fn transfer_in(&self, channel: Channel, toggle: Toggle,
                   buffer: &mut [u8]) -> HostResult<usize> { /* … */ }
    fn transfer_out(&self, channel: Channel, toggle: Toggle,
                    data: &[u8]) -> HostResult<()> { /* … */ }
    fn wait_millis(&self, millis: u32) { /* … */ }
}
```

The interface is transaction oriented: one call moves at most one packet, and the
protocol layer splits transfers into packets. Data toggles are tracked by the
stack (`Toggle`), not by the controller, so reprogramming a channel never
corrupts the sequence. Controllers retry NAKs internally and report a STALL as
`HostError::Stall`.

With a controller in hand, enumeration is one call:

```rust,ignore
use usb_stack::host::{Host, class::hid::Keyboard};

let host = Host::new(&my_host);
let device = host.enumerate()?;        // resets the port and reads the descriptors
device.set_configuration(1)?;
let keyboard = Keyboard::open(&device)?;   // boot protocol + interrupt endpoint
```

Hubs, isochronous transfers and SRP/HNP are not implemented; a device must be
plugged into the controller's own port.

## Classes

HID is a device class, so it has a class driver on each side and they share no
code: `device::class::hid` makes this MCU act as a keyboard or mouse, while
`host::class::hid` drives a keyboard that is plugged in.

Device classes: CDC-ACM, HID (boot keyboard, boot mouse, custom reports, HID
Power Device), GS_USB (CAN), i2c-tiny-usb, UAC 1.0.
Host classes: HID boot keyboard.

## Features

- `defmt` (off by default) — log through `defmt`.

## Status

Version 0.2 — the API is still settling, and 0.2 moved the device stack under
`device` without keeping the 0.1 paths.
