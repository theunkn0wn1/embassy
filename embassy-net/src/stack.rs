use core::cell::RefCell;
use core::task::Context;
use core::task::Poll;

use embassy::time::Instant;
use embassy::util::Forever;
use smoltcp::iface::{InterfaceBuilder, Neighbor, NeighborCache, Route, Routes};
use smoltcp::phy::Device as _;
use smoltcp::phy::Medium;
use smoltcp::socket::SocketSetItem;
use smoltcp::time::Instant as SmolInstant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address};

use crate::device::{Device, DeviceAdapter};
use crate::fmt::*;
use crate::{
    config::{Config, Configurator},
    device::LinkState,
};
use crate::{Interface, SocketSet};

const ADDRESSES_LEN: usize = 1;
const NEIGHBOR_CACHE_LEN: usize = 8;
const SOCKETS_LEN: usize = 2;
const LOCAL_PORT_MIN: u16 = 1025;
const LOCAL_PORT_MAX: u16 = 65535;

struct StackResources {
    addresses: [IpCidr; ADDRESSES_LEN],
    neighbor_cache: [Option<(IpAddress, Neighbor)>; NEIGHBOR_CACHE_LEN],
    sockets: [Option<SocketSetItem<'static, 'static>>; SOCKETS_LEN],
    routes: [Option<(IpCidr, Route)>; 1],
}

pub(crate) struct StackInner {
    iface: Interface,
    pub sockets: SocketSet,
    link_up: bool,
    next_local_port: u16,
    configurator: &'static mut dyn Configurator,
}

pub struct Stack {
    pub(crate) inner: RefCell<StackInner>,
}

impl Stack {
    pub async fn run(&self) {
        futures::future::poll_fn(|cx| {
            self.inner.borrow_mut().poll(cx);
            Poll::<()>::Pending
        })
        .await
    }
}

impl StackInner {
    pub fn get_local_port(&mut self) -> u16 {
        let res = self.next_local_port;
        self.next_local_port = if res >= LOCAL_PORT_MAX {
            LOCAL_PORT_MIN
        } else {
            res + 1
        };
        res
    }

    fn poll(&mut self, cx: &mut Context<'_>) {
        self.iface.device_mut().device.register_waker(cx.waker());

        let timestamp = instant_to_smoltcp(Instant::now());
        if let Err(e) = self.iface.poll(&mut self.sockets, timestamp) {
            warn!("poll err {:?}, rescheduling poll", e);
            cx.waker().wake_by_ref();
        }

        // Update link up
        let old_link_up = self.link_up;
        self.link_up = self.iface.device_mut().device.link_state() == LinkState::Up;

        // Print when changed
        if old_link_up != self.link_up {
            if self.link_up {
                info!("Link up!");
            } else {
                info!("Link down!");
            }
        }

        let medium = self.iface.device().capabilities().medium;

        if old_link_up || self.link_up {
            if let Some(config) =
                self.configurator
                    .poll(&mut self.iface, &mut self.sockets, timestamp)
            {
                let (addr, gateway) = match config {
                    Config::Up(config) => (config.address.into(), Some(config.gateway)),
                    Config::Down => (IpCidr::new(Ipv4Address::UNSPECIFIED.into(), 32), None),
                };

                self.iface.update_ip_addrs(|addrs| {
                    let curr_addr = &mut addrs[0];
                    if *curr_addr != addr {
                        info!("IPv4 address: {:?} -> {:?}", *curr_addr, addr);
                        *curr_addr = addr;
                    }
                });

                if medium == Medium::Ethernet {
                    self.iface.routes_mut().update(|r| {
                        let cidr = IpCidr::new(IpAddress::v4(0, 0, 0, 0), 0);
                        let curr_gateway = r.get(&cidr).map(|r| r.via_router);

                        if curr_gateway != gateway.map(|a| a.into()) {
                            info!("IPv4 gateway: {:?} -> {:?}", curr_gateway, gateway);
                            if let Some(gateway) = gateway {
                                r.insert(cidr, Route::new_ipv4_gateway(gateway)).unwrap();
                            } else {
                                r.remove(&cidr);
                            }
                        }
                    });
                }
            }
        }
    }
}

static STACK: Forever<Stack> = Forever::new();
static STACK_RESOURCES: Forever<StackResources> = Forever::new();

pub fn init(
    device: &'static mut dyn Device,
    configurator: &'static mut dyn Configurator,
) -> &'static Stack {
    let res = STACK_RESOURCES.put(StackResources {
        addresses: [IpCidr::new(Ipv4Address::UNSPECIFIED.into(), 32)],
        neighbor_cache: [None; NEIGHBOR_CACHE_LEN],
        sockets: [None; SOCKETS_LEN],
        routes: [None; 1],
    });

    let ethernet_addr = EthernetAddress([0x02, 0x02, 0x02, 0x02, 0x02, 0x02]);

    let medium = device.capabilities().medium;

    let mut b = InterfaceBuilder::new(DeviceAdapter::new(device));
    b = b.ip_addrs(&mut res.addresses[..]);

    if medium == Medium::Ethernet {
        b = b.ethernet_addr(ethernet_addr);
        b = b.neighbor_cache(NeighborCache::new(&mut res.neighbor_cache[..]));
        b = b.routes(Routes::new(&mut res.routes[..]));
    }

    let iface = b.finalize();

    let sockets = SocketSet::new(&mut res.sockets[..]);

    let local_port = loop {
        let mut res = [0u8; 2];
        embassy::rand::rand(&mut res);
        let port = u16::from_le_bytes(res);
        if port >= LOCAL_PORT_MIN && port <= LOCAL_PORT_MAX {
            break port;
        }
    };

    let stack = StackInner {
        iface,
        sockets,
        link_up: false,
        configurator,
        next_local_port: local_port,
    };
    let stack = STACK.put(Stack {
        inner: RefCell::new(stack),
    });

    stack
}

fn instant_to_smoltcp(instant: Instant) -> SmolInstant {
    SmolInstant::from_millis(instant.as_millis() as i64)
}
