#![feature(type_alias_impl_trait)]

use embassy::executor::{task, Executor};
use embassy::io::{AsyncBufReadExt, AsyncWriteExt};
use embassy::time::{Duration, Timer};
use embassy::util::Forever;
use embassy_net::*;
use heapless::Vec;
use log::*;

static DEVICE: Forever<TunTapDevice> = Forever::new();
static CONFIG: Forever<StaticConfigurator> = Forever::new();

#[task]
async fn net_task() {
    embassy_net::run().await
}

#[task]
async fn main_task(executor: &'static Executor) {
    // Init network device
    let device = TunTapDevice::new("tap0").unwrap();

    // Static IP configuration
    let config = StaticConfigurator::new(UpConfig {
        address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 69, 1), 24),
        dns_servers: Vec::new(),
        gateway: Ipv4Address::new(192, 168, 69, 100),
    });

    // Init network stack
    embassy_net::init(DEVICE.put(device), CONFIG.put(config));

    // Launch network task
    executor.spawn(net_task()).unwrap();

    Timer::after(Duration::from_secs(1)).await;

    // Then we can use it!
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut socket = TcpSocket::new(&mut rx_buffer, &mut tx_buffer);

    socket.set_timeout(Some(embassy_net::SmolDuration::from_secs(10)));

    let remote_endpoint = (Ipv4Address::new(192, 168, 69, 100), 8000);
    info!("connecting to {:?}...", remote_endpoint);
    let r = socket.connect(remote_endpoint).await;
    if let Err(e) = r {
        warn!("connect error: {:?}", e);
        return;
    }
    info!("connected!");
    loop {
        let r = socket.write_all(b"Hello!\n").await;
        if let Err(e) = r {
            warn!("write error: {:?}", e);
            return;
        }
    }
}

fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Debug)
        .filter_module("async_io", log::LevelFilter::Info)
        .format_timestamp_nanos()
        .init();

    let executor = embassy_std::init();
    executor.spawn(main_task(executor)).unwrap();
    embassy_std::run(executor);
}
