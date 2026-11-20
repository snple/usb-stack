//! The host protocol layer against a simulated boot keyboard.
//!
//! The mock implements [`HostController`] the way a real controller behaves
//! from the protocol layer's point of view: it stores a device address, answers
//! control requests, hands out reports and records every transaction so the
//! tests can check the order and the data toggles.

use core::cell::RefCell;

use usb_stack::descriptor::descriptor_type;
use usb_stack::endpoint::{EndpointAddress, EndpointType};
use usb_stack::host::class::hid::{Keyboard, Report};
use usb_stack::host::{
    Channel, DeviceAddress, EndpointConfig, Host, HostController, HostError, HostResult,
    PortStatus, Speed, Toggle,
};
use usb_stack::request::{Request, RequestType, StandardRequest};

// ─── Descriptors of the simulated device ─────────────────────────────────────

const DEVICE_DESCRIPTOR: [u8; 18] = [
    18, // bLength
    descriptor_type::DEVICE,
    0x00,
    0x02, // bcdUSB 2.00
    0x00, // bDeviceClass
    0x00, // bDeviceSubClass
    0x00, // bDeviceProtocol
    64,   // bMaxPacketSize0
    0x34,
    0x12, // idVendor 0x1234
    0x78,
    0x56, // idProduct 0x5678
    0x00,
    0x01, // bcdDevice
    1,    // iManufacturer
    2,    // iProduct
    0,    // iSerialNumber
    1,    // bNumConfigurations
];

/// One interface with a boot keyboard interrupt IN endpoint and a bulk OUT
/// endpoint, so the tests can exercise both kinds of pipe.
const CONFIG_DESCRIPTOR: [u8; 32] = [
    9, // bLength
    descriptor_type::CONFIGURATION,
    32,
    0,    // wTotalLength
    1,    // bNumInterfaces
    1,    // bConfigurationValue
    0,    // iConfiguration
    0xa0, // bmAttributes: bus powered, remote wake-up
    50,   // bMaxPower: 100 mA
    // Interface 0: HID boot keyboard
    9, // bLength
    descriptor_type::INTERFACE,
    0,    // bInterfaceNumber
    0,    // bAlternateSetting
    2,    // bNumEndpoints
    0x03, // bInterfaceClass: HID
    1,    // bInterfaceSubClass: boot
    1,    // bInterfaceProtocol: keyboard
    0,    // iInterface
    // Interrupt IN, one 8-byte report per frame
    7, // bLength
    descriptor_type::ENDPOINT,
    0x81, // bEndpointAddress
    0x03, // bmAttributes: interrupt
    8,
    0,  // wMaxPacketSize
    10, // bInterval
    // Bulk OUT
    7, // bLength
    descriptor_type::ENDPOINT,
    0x02, // bEndpointAddress
    0x02, // bmAttributes: bulk
    8,
    0, // wMaxPacketSize
    0, // bInterval
];

fn string_descriptor(index: u8) -> Option<&'static [u8]> {
    // Pre-encoded UTF-16LE string descriptors, header included.
    match index {
        1 => Some(&[
            12,
            descriptor_type::STRING,
            b'A',
            0,
            b'C',
            0,
            b'M',
            0,
            b'E',
            0,
            b'!',
            0,
        ]),
        2 => Some(&[
            18,
            descriptor_type::STRING,
            b'K',
            0,
            b'e',
            0,
            b'y',
            0,
            b'b',
            0,
            b'o',
            0,
            b'a',
            0,
            b'r',
            0,
            b'd',
            0,
        ]),
        _ => None,
    }
}

// ─── Simulated controller ────────────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Action {
    Reset,
    Configure {
        channel: usize,
        device: u8,
        max_packet: u16,
    },
    Setup {
        request: u8,
        value: u16,
        length: u16,
    },
    In {
        channel: usize,
        toggle: Toggle,
    },
    Out {
        channel: usize,
        toggle: Toggle,
        length: usize,
    },
}

struct State {
    actions: Vec<Action>,
    channels: [bool; 4],
    configs: [Option<(u8, EndpointConfig)>; 4],
    /// Address the simulated device answers to.
    address: u8,
    /// Payload of the control transfer in flight, and how much of it was sent.
    response: Vec<u8>,
    sent: usize,
    /// Reports waiting on the interrupt IN endpoint.
    reports: Vec<[u8; 8]>,
    /// Configuration value that `SET_CONFIGURATION` selected.
    configured: u8,
}

pub struct MockController {
    state: RefCell<State>,
}

impl MockController {
    fn new() -> Self {
        Self {
            state: RefCell::new(State {
                actions: Vec::new(),
                channels: [false; 4],
                configs: [None; 4],
                address: 0,
                response: Vec::new(),
                sent: 0,
                reports: Vec::new(),
                configured: 0,
            }),
        }
    }

    fn actions(&self) -> Vec<Action> {
        self.state.borrow().actions.clone()
    }

    fn queue_report(&self, report: [u8; 8]) {
        self.state.borrow_mut().reports.push(report);
    }

    fn configured(&self) -> u8 {
        self.state.borrow().configured
    }

    fn config_of(&self, channel: usize) -> Option<(u8, EndpointConfig)> {
        self.state.borrow().configs[channel]
    }
}

fn running(controller: &MockController) -> PortStatus {
    let _ = controller;
    PortStatus {
        connected: true,
        enabled: true,
        speed: Speed::Full,
        overcurrent: false,
    }
}

impl HostController for MockController {
    fn port_status(&self) -> PortStatus {
        running(self)
    }

    fn reset_port(&self) -> HostResult<PortStatus> {
        let mut state = self.state.borrow_mut();
        state.actions.push(Action::Reset);
        state.address = 0;
        drop(state);
        Ok(running(self))
    }

    fn alloc_channel(&self) -> HostResult<Channel> {
        let mut state = self.state.borrow_mut();
        match state.channels.iter().position(|used| !used) {
            Some(index) => {
                state.channels[index] = true;
                state.configs[index] = None;
                Ok(Channel::new(index as u8))
            }
            None => Err(HostError::NoChannel),
        }
    }

    fn free_channel(&self, channel: Channel) {
        let mut state = self.state.borrow_mut();
        state.channels[channel.index()] = false;
        state.configs[channel.index()] = None;
    }

    fn configure(
        &self,
        channel: Channel,
        device: DeviceAddress,
        endpoint: &EndpointConfig,
    ) -> HostResult<()> {
        let mut state = self.state.borrow_mut();
        state.actions.push(Action::Configure {
            channel: channel.index(),
            device: device.raw(),
            max_packet: endpoint.max_packet,
        });
        state.configs[channel.index()] = Some((device.raw(), *endpoint));
        Ok(())
    }

    fn setup(&self, channel: Channel, packet: &[u8]) -> HostResult<()> {
        let request = Request::parse(packet).expect("a SETUP packet is eight bytes");

        let mut state = self.state.borrow_mut();
        state.actions.push(Action::Setup {
            request: request.request,
            value: request.value,
            length: request.length,
        });

        // A real device ignores everything that is not addressed to it.
        let (address, _) = state.configs[channel.index()].ok_or(HostError::Protocol)?;
        if address != state.address {
            return Err(HostError::Timeout);
        }

        state.response.clear();
        state.sent = 0;

        match request.request {
            StandardRequest::GET_DESCRIPTOR => {
                let kind = (request.value >> 8) as u8;
                let index = request.value as u8;
                let data: &[u8] = match kind {
                    descriptor_type::DEVICE => &DEVICE_DESCRIPTOR,
                    descriptor_type::CONFIGURATION => &CONFIG_DESCRIPTOR,
                    descriptor_type::STRING => string_descriptor(index).ok_or(HostError::Stall)?,
                    _ => return Err(HostError::Stall),
                };
                let length = usize::from(request.length).min(data.len());
                state.response.extend_from_slice(&data[..length]);
                Ok(())
            }
            StandardRequest::SET_ADDRESS => {
                state.address = request.value as u8;
                Ok(())
            }
            StandardRequest::SET_CONFIGURATION => {
                state.configured = request.value as u8;
                Ok(())
            }
            _ if request.request_type == RequestType::Class => {
                // HID requests: SET_PROTOCOL and SET_IDLE carry no data.
                Ok(())
            }
            _ => Err(HostError::Stall),
        }
    }

    fn transfer_in(
        &self,
        channel: Channel,
        toggle: Toggle,
        buffer: &mut [u8],
    ) -> HostResult<usize> {
        let mut state = self.state.borrow_mut();
        state.actions.push(Action::In {
            channel: channel.index(),
            toggle,
        });
        let (_, config) = state.configs[channel.index()].ok_or(HostError::Protocol)?;

        if config.transfer == EndpointType::Control {
            let remaining = state.response.len() - state.sent;
            let length = remaining.min(buffer.len());
            buffer[..length].copy_from_slice(&state.response[state.sent..state.sent + length]);
            state.sent += length;
            Ok(length)
        } else if let Some(report) = state.reports.first().copied() {
            state.reports.remove(0);
            buffer[..report.len()].copy_from_slice(&report);
            Ok(report.len())
        } else {
            // The device has nothing new to say.
            Err(HostError::Timeout)
        }
    }

    fn transfer_out(&self, channel: Channel, toggle: Toggle, data: &[u8]) -> HostResult<()> {
        self.state.borrow_mut().actions.push(Action::Out {
            channel: channel.index(),
            toggle,
            length: data.len(),
        });
        Ok(())
    }

    fn wait_millis(&self, _millis: u32) {}
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[test]
fn enumerates_and_parses_the_descriptors() {
    let controller = MockController::new();
    let host = Host::new(&controller);

    let device = host.enumerate().expect("the device enumerates");

    assert_eq!(device.address().raw(), 1);
    assert_eq!(device.speed(), Speed::Full);

    let descriptor = device.descriptor();
    assert_eq!(descriptor.vendor_id, 0x1234);
    assert_eq!(descriptor.product_id, 0x5678);
    assert_eq!(descriptor.max_packet0, 64);
    assert_eq!(descriptor.configurations, 1);

    let configuration = device.configuration();
    assert_eq!(configuration.value(), 1);
    assert_eq!(configuration.max_power_ma(), 100);

    let interface = configuration
        .find_interface(0x03, 1, 1)
        .expect("a boot keyboard interface");
    assert_eq!(interface.number(), 0);
    assert_eq!(interface.endpoint_count(), 2);

    let endpoints: Vec<_> = interface.endpoints().collect();
    assert_eq!(endpoints.len(), 2);
    assert_eq!(endpoints[0].address.raw(), 0x81);
    assert_eq!(endpoints[0].transfer_type(), EndpointType::Interrupt);
    assert_eq!(endpoints[0].max_packet, 8);
    assert_eq!(endpoints[1].address.raw(), 0x02);
    assert_eq!(endpoints[1].transfer_type(), EndpointType::Bulk);
}

#[test]
fn assigns_the_address_before_the_second_descriptor_read() {
    let controller = MockController::new();
    let host = Host::new(&controller);
    host.enumerate().expect("the device enumerates");

    let actions = controller.actions();
    let address_index = actions
        .iter()
        .position(|action| {
            matches!(
                action,
                Action::Setup {
                    request: StandardRequest::SET_ADDRESS,
                    ..
                }
            )
        })
        .expect("SET_ADDRESS was sent");

    // Only bMaxPacketSize0 is read at address 0.
    let probes = actions[..address_index]
        .iter()
        .filter(|action| {
            matches!(
                action,
                Action::Setup {
                    request: StandardRequest::GET_DESCRIPTOR,
                    ..
                }
            )
        })
        .count();
    assert_eq!(probes, 1);

    // The channel is programmed with the new address afterwards.
    let last = actions
        .iter()
        .filter_map(|action| match action {
            Action::Configure { device, .. } => Some(*device),
            _ => None,
        })
        .next_back()
        .expect("the control channel is configured");
    assert_eq!(last, 1);
}

#[test]
fn control_stages_use_the_right_data_toggles() {
    let controller = MockController::new();
    let host = Host::new(&controller);
    host.enumerate().expect("the device enumerates");

    // A single-packet data stage runs with DATA1, and so does the zero-length
    // status stage that follows it.
    let actions = controller.actions();
    let start = actions
        .iter()
        .position(|action| {
            matches!(
                action,
                Action::Setup {
                    request: StandardRequest::GET_DESCRIPTOR,
                    value,
                    ..
                } if *value == ((descriptor_type::DEVICE as u16) << 8)
            )
        })
        .expect("the device descriptor was read");

    let toggles: Vec<Toggle> = actions[start..]
        .iter()
        .filter_map(|action| match action {
            Action::In { toggle, .. } | Action::Out { toggle, .. } => Some(*toggle),
            _ => None,
        })
        .take(2)
        .collect();
    assert_eq!(toggles, vec![Toggle::Data1, Toggle::Data1]);
}

#[test]
fn reads_a_string_descriptor() {
    let controller = MockController::new();
    let host = Host::new(&controller);
    let device = host.enumerate().expect("the device enumerates");

    let mut buffer = [0u8; 32];
    let length = device
        .read_string(2, &mut buffer)
        .expect("the product string is readable");
    assert_eq!(&buffer[..length], b"Keyboard");
}

#[test]
fn sets_the_configuration_and_opens_the_keyboard() {
    let controller = MockController::new();
    let host = Host::new(&controller);
    let device = host.enumerate().expect("the device enumerates");

    device
        .set_configuration(1)
        .expect("the configuration is accepted");
    assert_eq!(controller.configured(), 1);

    let keyboard = Keyboard::open(&device).expect("the boot keyboard opens");

    // The control channel is 0 and the interrupt endpoint gets the next one,
    // programmed for address 1.
    let (address, config) = controller
        .config_of(1)
        .expect("the interrupt channel is configured");
    assert_eq!(address, 1);
    assert_eq!(config.max_packet, 8);
    assert_eq!(keyboard.endpoint().raw(), 0x81);
}

#[test]
fn reports_new_keys_only_once() {
    let controller = MockController::new();
    let host = Host::new(&controller);
    let device = host.enumerate().expect("the device enumerates");
    device
        .set_configuration(1)
        .expect("the configuration is accepted");
    let keyboard = Keyboard::open(&device).expect("the boot keyboard opens");

    // Nothing changed on the device yet.
    assert_eq!(keyboard.read().err(), Some(HostError::Timeout));

    controller.queue_report([0, 0, 0x04, 0, 0, 0, 0, 0]);
    let report = keyboard.read().expect("a report arrives");
    assert!(report.is_pressed(0x04));
    assert_eq!(
        report.new_keys(&Report::default()).collect::<Vec<_>>(),
        [0x04]
    );

    controller.queue_report([0, 0, 0x04, 0x05, 0, 0, 0, 0]);
    let next = keyboard.read().expect("a report arrives");
    assert_eq!(next.new_keys(&report).collect::<Vec<_>>(), [0x05]);
}

#[test]
fn bulk_writes_alternate_the_data_toggle() {
    let controller = MockController::new();
    let host = Host::new(&controller);
    let device = host.enumerate().expect("the device enumerates");

    let endpoint = device
        .configuration()
        .interfaces()
        .flat_map(|interface| interface.endpoints().collect::<Vec<_>>())
        .find(|endpoint| endpoint.transfer_type() == EndpointType::Bulk)
        .expect("a bulk endpoint");
    let pipe = device.open(&endpoint).expect("the pipe opens");

    let before = controller
        .actions()
        .iter()
        .filter(|action| matches!(action, Action::Out { .. }))
        .count();

    // Twenty bytes over an eight-byte endpoint is three packets.
    pipe.write(&[0x55; 20]).expect("the transfer succeeds");

    let sent: Vec<(Toggle, usize)> = controller
        .actions()
        .into_iter()
        .filter_map(|action| match action {
            Action::Out { toggle, length, .. } => Some((toggle, length)),
            _ => None,
        })
        .skip(before)
        .collect();
    assert_eq!(
        sent,
        vec![(Toggle::Data0, 8), (Toggle::Data1, 8), (Toggle::Data0, 4)]
    );
}

#[test]
fn a_stalled_endpoint_is_reported() {
    /// Wraps the mock so that every non-control transfer stalls.
    struct Stalling {
        inner: MockController,
    }

    impl HostController for Stalling {
        fn port_status(&self) -> PortStatus {
            running(&self.inner)
        }
        fn reset_port(&self) -> HostResult<PortStatus> {
            self.inner.reset_port()
        }
        fn alloc_channel(&self) -> HostResult<Channel> {
            self.inner.alloc_channel()
        }
        fn free_channel(&self, channel: Channel) {
            self.inner.free_channel(channel)
        }
        fn configure(
            &self,
            channel: Channel,
            device: DeviceAddress,
            endpoint: &EndpointConfig,
        ) -> HostResult<()> {
            self.inner.configure(channel, device, endpoint)
        }
        fn setup(&self, channel: Channel, packet: &[u8]) -> HostResult<()> {
            self.inner.setup(channel, packet)
        }
        fn transfer_in(
            &self,
            channel: Channel,
            toggle: Toggle,
            buffer: &mut [u8],
        ) -> HostResult<usize> {
            match self.inner.config_of(channel.index()) {
                Some((_, config)) if config.transfer == EndpointType::Control => {
                    self.inner.transfer_in(channel, toggle, buffer)
                }
                _ => Err(HostError::Stall),
            }
        }
        fn transfer_out(&self, channel: Channel, toggle: Toggle, data: &[u8]) -> HostResult<()> {
            self.inner.transfer_out(channel, toggle, data)
        }
        fn wait_millis(&self, millis: u32) {
            self.inner.wait_millis(millis)
        }
    }

    let controller = Stalling {
        inner: MockController::new(),
    };
    let host = Host::new(&controller);
    let device = host.enumerate().expect("the device enumerates");
    let pipe = device
        .open(&usb_stack::host::EndpointDescriptor {
            address: EndpointAddress::in_(1),
            attributes: 0x03,
            max_packet: 8,
            interval: 10,
        })
        .expect("the pipe opens");

    let mut buffer = [0u8; 8];
    assert_eq!(pipe.read(&mut buffer).err(), Some(HostError::Stall));
}
