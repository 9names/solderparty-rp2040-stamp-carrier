#![no_std]

pub use rp2040_hal as hal;

#[cfg(feature = "rt")]
extern crate cortex_m_rt;
#[cfg(feature = "rt")]
pub use cortex_m_rt::entry;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[cfg(feature = "boot2")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_GD25Q64CS;

pub use hal::pac;

hal::bsp_pins!(
    Gpio0 { name: tx },
    Gpio1 { name: rx },
    Gpio2 { name: gpio2 },
    Gpio3 { name: gpio3 },
    Gpio4 { name: sda },
    Gpio5 { name: scl },
    Gpio6 { name: gpio6 },
    Gpio7 { name: gpio7 },
    Gpio8 { name: gpio8 },
    Gpio9 { name: gpio9 },
    Gpio10 { name: gpio10 },
    Gpio11 { name: gpio11 },
    Gpio12 { name: gpio12 },
    Gpio13 { name: gpio13 },
    Gpio14 { name: gpio14 },
    Gpio15 { name: gpio15 },
    Gpio16 { name: cipo },
    Gpio17 { name: cs },
    Gpio18 { name: sck },
    Gpio19 { name: copi },
    Gpio20 { name: led },
    Gpio21 { name: neopixel },
    Gpio22 { name: gpio22 },
    Gpio23 { name: gpio23 },
    Gpio24 { name: gpio24 },
    Gpio25 { name: gpio25 },
    Gpio26 { name: a0 },
    Gpio27 { name: a1 },
    Gpio28 { name: a2 },
    Gpio29 { name: a3 },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
pub mod prelude {
    pub use crate as bsp;
    pub use crate::Pins;
    pub use crate::BOOT2_FIRMWARE as _;
    pub use crate::XOSC_CRYSTAL_FREQ;
    pub use core::iter::once;
    pub use cortex_m_rt::entry;
    pub use embedded_hal::adc::OneShot;
    pub use embedded_hal::timer::CountDown;
    pub use embedded_time::duration::Extensions;
    pub use embedded_time::fixed_point::FixedPoint;
    pub use embedded_hal::digital::v2::ToggleableOutputPin;
    pub use rp2040_hal::{
        clocks::{init_clocks_and_plls, Clock},
        pac,
        pio::PIOExt,
        prelude::*,
        timer::Timer,
        watchdog::Watchdog,
        Sio,
    };
    pub use smart_leds::SmartLedsWrite;
    pub use ws2812_pio::Ws2812;
    
}
