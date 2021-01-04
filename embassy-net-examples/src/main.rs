use embassy::io::{AsyncBufReadExt, AsyncWriteExt};
use embassy::util::Forever;
use embassy_net::*;
use env_logger::builder;
use futures::pin_mut;
use heapless::consts::*;
use heapless::Vec;
use log::*;

static DEVICE: Forever<TunTapDevice> = Forever::new();
static CONFIG: Forever<StaticConfigurator> = Forever::new();

#[async_std::main]
async fn main() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Debug)
        .filter_module("async_io", log::LevelFilter::Info)
        .format_timestamp_nanos()
        .init();
    embassy_std::init();
    info!("Hi!");

    // Init network device
    let device = TunTapDevice::new("tap0").unwrap();

    // Init network stack with a static IP address
    let config = StaticConfigurator::new(UpConfig {
        address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 69, 1), 24),
        dns_servers: Vec::new(),
        gateway: Ipv4Address::new(192, 168, 69, 100),
    });
    let stack = embassy_net::init(DEVICE.put(device), CONFIG.put(config));
    async_std::task::spawn_local(stack.run());

    // Then we can use it!
    let socket: TcpSocket<U4096, U4096> = TcpSocket::new(stack);
    pin_mut!(socket);

    socket
        .as_mut()
        .set_timeout(Some(embassy_net::SmolDuration::from_secs(10)));

    let remote_endpoint = (Ipv4Address::new(192, 168, 69, 100), 8000);
    info!("connecting to {:?}...", remote_endpoint);
    let r = socket.as_mut().connect(remote_endpoint).await;
    if let Err(e) = r {
        warn!("connect error: {:?}", e);
        return;
    }
    info!("connected!");
    loop {
        let r = socket.as_mut().write_all(b"Hello!\n").await;
        if let Err(e) = r {
            warn!("write error: {:?}", e);
            return;
        }
    }
}
