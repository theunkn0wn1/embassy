#![no_std]
#![feature(generic_associated_types)]
#![feature(asm)]
#![feature(type_alias_impl_trait)]
#![feature(never_type)]

pub use rp2040_pac2 as pac;

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;
pub(crate) mod util;

pub mod interrupt;

pub mod gpio;
pub mod init;
pub mod pll;
pub mod resets;
