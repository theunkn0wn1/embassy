use core::sync::atomic::{compiler_fence, Ordering};

use defmt::{assert, *};

use crate::pac;
use pac::dma::vals;

pub struct Dma<T: Channel> {
    inner: T,
}

impl<T: Channel> Dma<T> {
    pub fn copy(inner: T, from: &[u32], to: &mut [u32]) {
        assert!(from.len() == to.len());

        unsafe {
            let p = inner.regs();

            p.read_addr().write_value(from.as_ptr() as u32);
            p.write_addr().write_value(to.as_mut_ptr() as u32);
            p.trans_count().write_value(from.len() as u32);

            compiler_fence(Ordering::SeqCst);

            p.ctrl_trig().write(|w| {
                w.set_data_size(vals::DataSize::SIZE_WORD);
                w.set_incr_read(true);
                w.set_incr_write(true);
                w.set_chain_to(inner.number());
                w.set_en(true);
            });

            while p.ctrl_trig().read().busy() {}

            compiler_fence(Ordering::SeqCst);
        }
    }
}

mod sealed {
    use super::*;

    pub trait Channel {
        fn number(&self) -> u8;

        fn regs(&self) -> pac::dma::Channel {
            pac::DMA.ch(self.number() as _)
        }
    }
}

pub trait Channel: sealed::Channel {}

pub struct AnyChannel {
    number: u8,
}

impl Channel for AnyChannel {}
impl sealed::Channel for AnyChannel {
    fn number(&self) -> u8 {
        self.number
    }
}

macro_rules! channel {
    ($name:ident, $num:expr) => {
        pub struct $name {
            pub(crate) _private: (),
        }

        impl Channel for $name {}
        impl sealed::Channel for $name {
            fn number(&self) -> u8 {
                $num
            }
        }
    };
}

channel!(Channel0, 0);
channel!(Channel1, 1);
channel!(Channel2, 2);
channel!(Channel3, 3);
channel!(Channel4, 4);
channel!(Channel5, 5);
channel!(Channel6, 6);
channel!(Channel7, 7);
channel!(Channel8, 8);
channel!(Channel9, 9);
channel!(Channel10, 10);
channel!(Channel11, 11);
