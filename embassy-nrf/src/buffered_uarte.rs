//! HAL interface to the UARTE peripheral
//!
//! See product specification:
//!
//! - nrf52832: Section 35
//! - nrf52840: Section 6.34
use core::cmp::min;
use core::marker::PhantomPinned;
use core::ops::Deref;
use core::pin::Pin;
use core::ptr;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::{Context, Poll};
use core::{cell::UnsafeCell, marker::PhantomData};
use embassy::io::{AsyncBufRead, AsyncWrite, Result};
use embassy::util::WakerRegistration;
use embedded_hal::digital::v2::OutputPin;

use crate::fmt::{assert, panic, todo, *};
use crate::hal::gpio::Port as GpioPort;
use crate::interrupt::{self, CriticalSection, OwnedInterrupt};
use crate::pac::uarte0;
use crate::ring_buffer::RingBuffer;

// Re-export SVD variants to allow user to directly set values
pub use crate::hal::uarte::Pins;
pub use uarte0::{baudrate::BAUDRATE_A as Baudrate, config::PARITY_A as Parity};

#[derive(Copy, Clone, Debug, PartialEq)]
enum RxState {
    Idle,
    Receiving,
    ReceivingReady,
    Stopping,
}
#[derive(Copy, Clone, Debug, PartialEq)]
enum TxState {
    Idle,
    Transmitting(usize),
}

/// Interface to a UARTE instance
///
/// This is a very basic interface that comes with the following limitations:
/// - The UARTE instances share the same address space with instances of UART.
///   You need to make sure that conflicting instances
///   are disabled before using `Uarte`. See product specification:
///     - nrf52832: Section 15.2
///     - nrf52840: Section 6.1.2
pub struct BufferedUarte<'a, T: Instance> {
    reg: PeripheralRegistration<T::State<'a>>,
    wtf: PhantomData<&'a ()>,
}

// public because it needs to be used in Instance::{get_state, set_state}, but
// should not be used outside the module
#[doc(hidden)]
pub struct UarteState<'a, T: Instance> {
    inner: T,
    irq: T::Interrupt,

    rx: RingBuffer<'a>,
    rx_state: RxState,
    rx_waker: WakerRegistration,

    tx: RingBuffer<'a>,
    tx_state: TxState,
    tx_waker: WakerRegistration,
}

#[cfg(any(feature = "52833", feature = "52840"))]
fn port_bit(port: GpioPort) -> bool {
    match port {
        GpioPort::Port0 => false,
        GpioPort::Port1 => true,
    }
}

impl<'a, T: Instance> BufferedUarte<'a, T> {
    pub fn new(
        uarte: T,
        irq: T::Interrupt,
        rx_buffer: &'a mut [u8],
        tx_buffer: &'a mut [u8],
        mut pins: Pins,
        parity: Parity,
        baudrate: Baudrate,
    ) -> Self {
        // Select pins
        uarte.psel.rxd.write(|w| {
            let w = unsafe { w.pin().bits(pins.rxd.pin()) };
            #[cfg(any(feature = "52833", feature = "52840"))]
            let w = w.port().bit(port_bit(pins.rxd.port()));
            w.connect().connected()
        });
        pins.txd.set_high().unwrap();
        uarte.psel.txd.write(|w| {
            let w = unsafe { w.pin().bits(pins.txd.pin()) };
            #[cfg(any(feature = "52833", feature = "52840"))]
            let w = w.port().bit(port_bit(pins.txd.port()));
            w.connect().connected()
        });

        // Optional pins
        uarte.psel.cts.write(|w| {
            if let Some(ref pin) = pins.cts {
                let w = unsafe { w.pin().bits(pin.pin()) };
                #[cfg(any(feature = "52833", feature = "52840"))]
                let w = w.port().bit(port_bit(pin.port()));
                w.connect().connected()
            } else {
                w.connect().disconnected()
            }
        });

        uarte.psel.rts.write(|w| {
            if let Some(ref pin) = pins.rts {
                let w = unsafe { w.pin().bits(pin.pin()) };
                #[cfg(any(feature = "52833", feature = "52840"))]
                let w = w.port().bit(port_bit(pin.port()));
                w.connect().connected()
            } else {
                w.connect().disconnected()
            }
        });

        // Enable UARTE instance
        uarte.enable.write(|w| w.enable().enabled());

        // Enable interrupts
        uarte.intenset.write(|w| w.endrx().set().endtx().set());

        // Configure
        let hardware_flow_control = pins.rts.is_some() && pins.cts.is_some();
        uarte
            .config
            .write(|w| w.hwfc().bit(hardware_flow_control).parity().variant(parity));

        // Configure frequency
        uarte.baudrate.write(|w| w.baudrate().variant(baudrate));

        BufferedUarte {
            reg: PeripheralRegistration::new(
                irq,
                UarteState {
                    inner: uarte,
                    irq,

                    rx: RingBuffer::new(rx_buffer),
                    rx_state: RxState::Idle,
                    rx_waker: WakerRegistration::new(),

                    tx: RingBuffer::new(tx_buffer),
                    tx_state: TxState::Idle,
                    tx_waker: WakerRegistration::new(),
                },
            ),
            wtf: PhantomData,
        }
    }
}

impl<'a, T: Instance> Drop for BufferedUarte<'a, T> {
    fn drop(&mut self) {
        // stop DMA before dropping, because DMA is using the buffer in `self`.
        todo!()
    }
}

impl<'a, T: Instance> AsyncBufRead for BufferedUarte<'a, T> {
    fn poll_fill_buf(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Result<&[u8]>> {
        self.reg.with(|s| s.poll_fill_buf(cx))
    }

    fn consume(self: Pin<&mut Self>, amt: usize) {
        self.reg.with(|s| s.consume(amt))
    }
}

impl<'a, T: Instance> AsyncWrite for BufferedUarte<'a, T> {
    fn poll_write(self: Pin<&mut Self>, cx: &mut Context<'_>, buf: &[u8]) -> Poll<Result<usize>> {
        self.reg.with(|s| s.poll_write(cx, buf))
    }
}

impl<'a, T: Instance> UarteState<'a, T> {
    pub fn start(self: Pin<&mut Self>) {
        self.irq.set_handler(|| unsafe {
            interrupt::free(|cs| T::get_state(cs).as_mut().unwrap().on_interrupt());
        });

        self.irq.pend();
        self.irq.enable();
    }

    fn poll_fill_buf(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Result<&[u8]>> {
        let this = unsafe { self.get_unchecked_mut() };

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // before any DMA action has started
        compiler_fence(Ordering::SeqCst);
        trace!("poll_read");

        // We have data ready in buffer? Return it.
        let buf = this.rx.pop_buf();
        if buf.len() != 0 {
            trace!("  got {:?} {:?}", buf.as_ptr() as u32, buf.len());
            return Poll::Ready(Ok(buf));
        }

        trace!("  empty");

        if this.rx_state == RxState::ReceivingReady {
            trace!("  stopping");
            this.rx_state = RxState::Stopping;
            this.inner.tasks_stoprx.write(|w| unsafe { w.bits(1) });
        }

        this.rx_waker.register(cx.waker());
        Poll::Pending
    }

    fn consume(self: Pin<&mut Self>, amt: usize) {
        let this = unsafe { self.get_unchecked_mut() };
        trace!("consume {:?}", amt);
        this.rx.pop(amt);
        this.irq.pend();
    }

    fn poll_write(self: Pin<&mut Self>, cx: &mut Context<'_>, buf: &[u8]) -> Poll<Result<usize>> {
        let this = unsafe { self.get_unchecked_mut() };

        trace!("poll_write: {:?}", buf.len());

        let tx_buf = this.tx.push_buf();
        if tx_buf.len() == 0 {
            trace!("poll_write: pending");
            this.tx_waker.register(cx.waker());
            return Poll::Pending;
        }

        let n = min(tx_buf.len(), buf.len());
        tx_buf[..n].copy_from_slice(&buf[..n]);
        this.tx.push(n);

        trace!("poll_write: queued {:?}", n);

        // Conservative compiler fence to prevent optimizations that do not
        // take in to account actions by DMA. The fence has been placed here,
        // before any DMA action has started
        compiler_fence(Ordering::SeqCst);

        this.irq.pend();

        Poll::Ready(Ok(n))
    }

    fn on_interrupt(&mut self) {
        trace!("irq: start");
        let mut more_work = true;
        while more_work {
            more_work = false;
            match self.rx_state {
                RxState::Idle => {
                    trace!("  irq_rx: in state idle");

                    if self.inner.events_rxdrdy.read().bits() != 0 {
                        trace!("  irq_rx: rxdrdy?????");
                        self.inner.events_rxdrdy.reset();
                    }

                    if self.inner.events_endrx.read().bits() != 0 {
                        panic!("unexpected endrx");
                    }

                    let buf = self.rx.push_buf();
                    if buf.len() != 0 {
                        trace!("  irq_rx: starting {:?}", buf.len());
                        self.rx_state = RxState::Receiving;

                        // Set up the DMA read
                        self.inner.rxd.ptr.write(|w|
                            // The PTR field is a full 32 bits wide and accepts the full range
                            // of values.
                            unsafe { w.ptr().bits(buf.as_ptr() as u32) });
                        self.inner.rxd.maxcnt.write(|w|
                            // We're giving it the length of the buffer, so no danger of
                            // accessing invalid memory. We have verified that the length of the
                            // buffer fits in an `u8`, so the cast to `u8` is also fine.
                            //
                            // The MAXCNT field is at least 8 bits wide and accepts the full
                            // range of values.
                            unsafe { w.maxcnt().bits(buf.len() as _) });
                        trace!("  irq_rx: buf {:?} {:?}", buf.as_ptr() as u32, buf.len());

                        // Enable RXRDY interrupt.
                        self.inner.events_rxdrdy.reset();
                        self.inner.intenset.write(|w| w.rxdrdy().set());

                        // Start UARTE Receive transaction
                        self.inner.tasks_startrx.write(|w|
                            // `1` is a valid value to write to task registers.
                            unsafe { w.bits(1) });
                    }
                }
                RxState::Receiving => {
                    trace!("  irq_rx: in state receiving");
                    if self.inner.events_rxdrdy.read().bits() != 0 {
                        trace!("  irq_rx: rxdrdy");

                        // Disable the RXRDY event interrupt
                        // RXRDY is triggered for every byte, but we only care about whether we have
                        // some bytes or not. So as soon as we have at least one, disable it, to avoid
                        // wasting CPU cycles in interrupts.
                        self.inner.intenclr.write(|w| w.rxdrdy().clear());

                        self.inner.events_rxdrdy.reset();

                        self.rx_waker.wake();
                        self.rx_state = RxState::ReceivingReady;
                        more_work = true; // in case we also have endrx pending
                    }
                }
                RxState::ReceivingReady | RxState::Stopping => {
                    trace!("  irq_rx: in state ReceivingReady");

                    if self.inner.events_rxdrdy.read().bits() != 0 {
                        trace!("  irq_rx: rxdrdy");
                        self.inner.events_rxdrdy.reset();
                    }

                    if self.inner.events_endrx.read().bits() != 0 {
                        let n: usize = self.inner.rxd.amount.read().amount().bits() as usize;
                        trace!("  irq_rx: endrx {:?}", n);
                        self.rx.push(n);

                        self.inner.events_endrx.reset();

                        self.rx_waker.wake();
                        self.rx_state = RxState::Idle;
                        more_work = true; // start another rx if possible
                    }
                }
            }
        }

        more_work = true;
        while more_work {
            more_work = false;
            match self.tx_state {
                TxState::Idle => {
                    trace!("  irq_tx: in state Idle");
                    let buf = self.tx.pop_buf();
                    if buf.len() != 0 {
                        trace!("  irq_tx: starting {:?}", buf.len());
                        self.tx_state = TxState::Transmitting(buf.len());

                        // Set up the DMA write
                        self.inner.txd.ptr.write(|w|
                            // The PTR field is a full 32 bits wide and accepts the full range
                            // of values.
                            unsafe { w.ptr().bits(buf.as_ptr() as u32) });
                        self.inner.txd.maxcnt.write(|w|
                            // We're giving it the length of the buffer, so no danger of
                            // accessing invalid memory. We have verified that the length of the
                            // buffer fits in an `u8`, so the cast to `u8` is also fine.
                            //
                            // The MAXCNT field is 8 bits wide and accepts the full range of
                            // values.
                            unsafe { w.maxcnt().bits(buf.len() as _) });

                        // Start UARTE Transmit transaction
                        self.inner.tasks_starttx.write(|w|
                            // `1` is a valid value to write to task registers.
                            unsafe { w.bits(1) });
                    }
                }
                TxState::Transmitting(n) => {
                    trace!("  irq_tx: in state Transmitting");
                    if self.inner.events_endtx.read().bits() != 0 {
                        self.inner.events_endtx.reset();

                        trace!("  irq_tx: endtx {:?}", n);
                        self.tx.pop(n);
                        self.tx_waker.wake();
                        self.tx_state = TxState::Idle;
                        more_work = true; // start another tx if possible
                    }
                }
            }
        }
        trace!("irq: end");
    }
}

mod private {
    pub trait Sealed {}

    impl Sealed for crate::pac::UARTE0 {}
    #[cfg(any(feature = "52833", feature = "52840", feature = "9160"))]
    impl Sealed for crate::pac::UARTE1 {}
}

pub trait Instance: Deref<Target = uarte0::RegisterBlock> + Sized + private::Sealed {
    type Interrupt: OwnedInterrupt + 'static;
    type State<'a>: PeripheralState<Interrupt = Self::Interrupt> + 'a;
}

impl Instance for crate::pac::UARTE0 {
    type Interrupt = interrupt::UARTE0_UART0Interrupt;
}

impl<'a> PeripheralState for UarteState<'a, crate::pac::UARTE0> {
    type Interrupt = interrupt::UARTE0_UART0Interrupt;
    fn storage<'b>() -> &'b PeripheralStorage<Self> {
        static STORAGE: PeripheralStorage<UarteState<'static, crate::pac::UARTE0>> =
            PeripheralStorage::uninit();
        &STORAGE
    }
}

// ===========================
// here be dragons
use core::mem;
use core::mem::MaybeUninit;

pub(crate) struct PeripheralStorage<T>(MaybeUninit<UnsafeCell<T>>);
impl<T> PeripheralStorage<T> {
    pub const fn uninit() -> Self {
        Self(MaybeUninit::uninit())
    }

    unsafe fn as_mut_ptr(&self) -> *mut T {
        (*self.0.as_ptr()).get()
    }

    unsafe fn as_mut(&self) -> &mut T {
        &mut *self.as_mut_ptr()
    }

    unsafe fn write(&self, val: T) {
        ptr::write(self.as_mut_ptr(), val)
    }

    unsafe fn drop_in_place(&self) {
        ptr::drop_in_place(self.as_mut_ptr())
    }

    unsafe fn read(&self) -> T {
        ptr::read(self.as_mut_ptr())
    }
}

trait PeripheralState: Sized {
    type Interrupt: OwnedInterrupt;
    fn storage<'a>() -> &'a PeripheralStorage<Self>;
}

struct PeripheralRegistration<P: PeripheralState> {
    irq: P::Interrupt,
    not_send: PhantomData<*mut P>,
}

impl<P: PeripheralState> PeripheralRegistration<P> {
    fn new(irq: P::Interrupt, state: P) -> Self {
        let storage = P::storage();
        irq.enable();

        // safety:
        // - No other PeripheralRegistration can already exist because we have the owned interrupt
        // - therefore, storage is uninitialized
        // - therefore it's safe to overwrite it without dropping the previous contents

        unsafe { storage.write(state) }
        Self {
            irq,
            not_send: PhantomData,
        }
    }

    fn with<R>(&mut self, f: impl FnOnce(&mut P) -> R) -> R {
        self.irq.disable();
        compiler_fence(Ordering::SeqCst);

        // safety:
        // - If a PeripheralRegistration instance exists, P::storage() is initialized.
        // - It's OK to get a &mut to it since the irq is disabled.
        let r = f(unsafe { P::storage().as_mut() });

        compiler_fence(Ordering::SeqCst);
        self.irq.enable();

        r
    }

    fn irq(&mut self) -> &mut P::Interrupt {
        &mut self.irq
    }

    fn free(self) -> (P::Interrupt, P) {
        let irq = self.irq;
        mem::forget(self);
        let storage = P::storage();
        (irq, unsafe { storage.read() })
    }
}

impl<P: PeripheralState> Drop for PeripheralRegistration<P> {
    fn drop(&mut self) {
        let storage = P::storage();
        unsafe { storage.drop_in_place() };
    }
}

/*

impl Peripheral for crate::pac::UARTE1 {
    type State = UarteState<crate::pac::UARTE1>;
    fn storage() -> &'static PeripheralStorage<Self::State, Self::Interrupt> {
        static STORAGE: PeripheralStorage<
            UarteState<crate::pac::UARTE1>,
            interrupt::UARTE1Interrupt,
        > = PeripheralStorage::new();
        &STORAGE
    }
}

*/
