//! Enumeration smoke test: drives a CDC-ACM device through a mock bus.
//!
//! The mock implements [`UsbBus`] with plain host-side buffers, so the device
//! state machine, control pipe and descriptor emission run exactly as they do
//! on a microcontroller.

use core::cell::{Cell, RefCell};
use std::collections::VecDeque;
use std::rc::Rc;

use usb_stack::class::cdc_acm::CdcAcm;
use usb_stack::descriptor::descriptor_type;
use usb_stack::device::{DeviceState, UsbDevice, UsbDeviceBuilder};
use usb_stack::{
    Direction, EndpointAddress, EndpointType, PollResult, UsbBus, UsbBusAllocator, UsbError,
    UsbResult,
};

// ─── Mock bus ────────────────────────────────────────────────────────────────

enum Event {
    Reset,
    Setup,
}

#[derive(Default)]
struct Inner {
    next_in: Cell<u8>,
    next_out: Cell<u8>,
    events: RefCell<VecDeque<Event>>,
    setup: RefCell<Vec<u8>>,
    in_packets: RefCell<Vec<(EndpointAddress, Vec<u8>)>>,
    address: Cell<u8>,
    enabled: Cell<bool>,
    resets: Cell<u32>,
}

/// Host-side stand-in for a USB peripheral.
///
/// Holds shared state so the test keeps a handle after the allocator takes
/// ownership of the bus.
#[derive(Clone, Default)]
struct MockBus(Rc<Inner>);

impl MockBus {
    fn push(&self, event: Event) {
        self.0.events.borrow_mut().push_back(event);
    }

    /// Queue a SETUP packet followed by the matching event.
    fn setup(&self, packet: [u8; 8]) {
        *self.0.setup.borrow_mut() = packet.to_vec();
        self.push(Event::Setup);
    }

    fn last_in(&self) -> Option<Vec<u8>> {
        self.0.in_packets.borrow().last().map(|(_, d)| d.clone())
    }

    fn take_in_packets(&self) -> Vec<Vec<u8>> {
        self.0
            .in_packets
            .borrow_mut()
            .drain(..)
            .map(|(_, d)| d)
            .collect()
    }
}

impl UsbBus for MockBus {
    fn alloc_ep(
        &mut self,
        dir: Direction,
        addr: Option<EndpointAddress>,
        _ep_type: EndpointType,
        _max_packet: u16,
        _interval: u8,
    ) -> UsbResult<EndpointAddress> {
        if let Some(addr) = addr {
            return Ok(addr);
        }
        let next = match dir {
            Direction::In => &self.0.next_in,
            Direction::Out => &self.0.next_out,
        };
        let n = next.get();
        next.set(n + 1);
        Ok(EndpointAddress::new(n, dir))
    }

    fn enable(&mut self) {
        self.0.enabled.set(true);
    }

    fn reset(&self) {
        self.0.resets.set(self.0.resets.get() + 1);
        self.0.next_in.set(1);
        self.0.next_out.set(1);
    }

    fn set_device_address(&self, addr: u8) {
        self.0.address.set(addr);
    }

    fn poll(&self) -> PollResult {
        match self.0.events.borrow_mut().pop_front() {
            None => PollResult::None,
            Some(Event::Reset) => PollResult::Reset,
            Some(Event::Setup) => PollResult::Data {
                ep_out: 0,
                ep_in_complete: 0,
                ep_setup: 1,
            },
        }
    }

    fn write(&self, addr: EndpointAddress, buf: &[u8]) -> UsbResult<usize> {
        self.0.in_packets.borrow_mut().push((addr, buf.to_vec()));
        Ok(buf.len())
    }

    fn read(&self, addr: EndpointAddress, buf: &mut [u8]) -> UsbResult<usize> {
        assert!(addr.is_out(), "only EP0 OUT receives in this mock");
        let mut pending = self.0.setup.borrow_mut();
        if pending.is_empty() {
            return Err(UsbError::WouldBlock);
        }
        let n = pending.len().min(buf.len());
        buf[..n].copy_from_slice(&pending[..n]);
        pending.drain(..n);
        Ok(n)
    }

    fn set_stalled(&self, _addr: EndpointAddress, _stalled: bool) {}

    fn is_stalled(&self, _addr: EndpointAddress) -> bool {
        false
    }

    fn suspend(&self) {}

    fn resume(&self) {}
}

// ─── Test fixture ────────────────────────────────────────────────────────────

/// A CDC device on the mock bus, already reset by the host.
struct TestDevice {
    bus: MockBus,
    serial: CdcAcm<'static, MockBus>,
    dev: UsbDevice<'static, MockBus>,
}

impl TestDevice {
    fn new() -> Self {
        let bus = MockBus::default();
        // Leaked so device and class can borrow the allocator for 'static.
        let alloc: &'static UsbBusAllocator<MockBus> =
            Box::leak(Box::new(UsbBusAllocator::new(bus.clone())));
        let serial = CdcAcm::new(alloc);
        let dev = UsbDeviceBuilder::new(alloc, 0x1209, 0x0001)
            .product("usb-stack test")
            .build();
        assert!(bus.0.enabled.get(), "build() enables the peripheral");

        let mut this = Self { bus, serial, dev };
        this.bus.push(Event::Reset);
        this.poll();
        this
    }

    fn poll(&mut self) -> bool {
        self.dev.poll(&mut [&mut self.serial])
    }

    /// Deliver a SETUP packet and run one poll cycle.
    fn control_setup(&mut self, packet: [u8; 8]) {
        self.bus.setup(packet);
        self.poll();
    }
}

fn get_descriptor(desc_type: u8, length: u16) -> [u8; 8] {
    let [lo, hi] = length.to_le_bytes();
    [0x80, 0x06, 0x00, desc_type, 0x00, 0x00, lo, hi]
}

fn set_configuration(value: u8) -> [u8; 8] {
    [0x00, 0x09, value, 0x00, 0x00, 0x00, 0x00, 0x00]
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[test]
fn reset_returns_device_to_default_state() {
    let t = TestDevice::new();
    assert_eq!(t.dev.state(), DeviceState::Default);
    assert_eq!(t.bus.0.resets.get(), 1);
}

#[test]
fn device_descriptor_is_served() {
    let mut t = TestDevice::new();
    t.control_setup(get_descriptor(descriptor_type::DEVICE, 18));

    let desc = t.bus.last_in().expect("device descriptor written");
    assert_eq!(desc.len(), 18);
    assert_eq!(desc[0], 18, "bLength");
    assert_eq!(desc[1], descriptor_type::DEVICE, "bDescriptorType");
    assert_eq!(u16::from_le_bytes([desc[2], desc[3]]), 0x0200, "bcdUSB");
    assert_eq!(u16::from_le_bytes([desc[8], desc[9]]), 0x1209, "idVendor");
    assert_eq!(
        u16::from_le_bytes([desc[10], desc[11]]),
        0x0001,
        "idProduct"
    );
    assert_eq!(desc[17], 1, "bNumConfigurations");
}

#[test]
fn device_descriptor_is_truncated_to_wlength() {
    let mut t = TestDevice::new();
    t.control_setup(get_descriptor(descriptor_type::DEVICE, 8));

    let desc = t.bus.last_in().expect("device descriptor written");
    assert_eq!(desc.len(), 8, "only wLength bytes are transferred");
}

#[test]
fn configuration_descriptor_contains_cdc_interfaces() {
    let mut t = TestDevice::new();
    t.control_setup(get_descriptor(descriptor_type::CONFIGURATION, 9));

    let desc = t.bus.last_in().expect("configuration descriptor written");
    assert_eq!(desc[1], descriptor_type::CONFIGURATION);
    assert_eq!(
        desc[4], 2,
        "CDC exposes a communication and a data interface"
    );
    let total = u16::from_le_bytes([desc[2], desc[3]]);
    assert!(total > 9, "wTotalLength covers the interface descriptors");
}

#[test]
fn set_configuration_configures_device() {
    let mut t = TestDevice::new();
    assert!(!t.dev.is_configured());

    t.control_setup(set_configuration(1));

    assert_eq!(t.dev.state(), DeviceState::Configured);
    assert!(t.dev.is_configured());
    assert_eq!(
        t.bus.take_in_packets(),
        vec![Vec::<u8>::new()],
        "SET_CONFIGURATION is acknowledged with a zero-length status packet"
    );
}

#[test]
fn set_address_is_acknowledged_without_switching_address() {
    let mut t = TestDevice::new();

    t.control_setup([0x00, 0x05, 7, 0x00, 0x00, 0x00, 0x00, 0x00]);
    assert_eq!(
        t.bus.0.address.get(),
        0,
        "address only applies after the status stage"
    );
}

#[test]
fn unsupported_request_produces_no_data() {
    let mut t = TestDevice::new();
    // bRequest 0x42 is implemented neither by the device nor by CDC.
    t.control_setup([0x80, 0x42, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00]);

    assert!(
        t.bus.last_in().is_none(),
        "no data stage for a rejected request"
    );
}
