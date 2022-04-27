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

rp2040_hal::bsp_pins!(
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
    pub use embedded_hal::digital::v2::ToggleableOutputPin;
    pub use embedded_hal::timer::CountDown;
    pub use embedded_time::duration::Extensions;
    pub use embedded_time::fixed_point::FixedPoint;
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

/// Provides access to board peripherals on stamp-carrier
#[allow(non_snake_case)]
#[allow(unused)]
pub struct Board {
    pub pins: Pins,
    /// Core peripheral: Cache and branch predictor maintenance operations
    pub CBP: pac::CBP,

    /// Core peripheral: CPUID
    pub CPUID: pac::CPUID,

    /// Core peripheral: Debug Control Block
    pub DCB: pac::DCB,

    /// Core peripheral: Data Watchpoint and Trace unit
    pub DWT: pac::DWT,

    /// Core peripheral: Flash Patch and Breakpoint unit
    pub FPB: pac::FPB,

    /// Core peripheral: Instrumentation Trace Macrocell
    pub ITM: pac::ITM,

    /// Core peripheral: Memory Protection Unit
    pub MPU: pac::MPU,

    /// Core peripheral: Nested Vector Interrupt Controller
    pub NVIC: pac::NVIC,

    /// Core peripheral: System Control Block
    pub SCB: pac::SCB,

    /// Core peripheral: SysTick Timer
    pub SYST: pac::SYST,

    /// Core peripheral: Trace Port Interface Unit
    pub TPIU: pac::TPIU,

    // PAC peripherals

    // pub ADC: pac::ADC,
    pub BUSCTRL: pac::BUSCTRL,
    // pub CLOCKS: pac::CLOCKS,
    pub DMA: pac::DMA,
    pub I2C0: pac::I2C0,
    pub I2C1: pac::I2C1,
    // pub IO_BANK0: pac::IO_BANK0,
    pub IO_QSPI: pac::IO_QSPI,
    pub PIO0: pac::PIO0,
    pub PIO1: pac::PIO1,
    // pub PLL_SYS: pac::PLL_SYS,
    // pub PLL_USB: pac::PLL_USB,
    pub PPB: pac::PPB,
    pub PSM: pac::PSM,
    pub PWM: pac::PWM,
    pub RESETS: pac::RESETS,
    pub ROSC: pac::ROSC,
    pub RTC: pac::RTC,
    // pub SIO: pac::SIO,
    pub SPI0: pac::SPI0,
    pub SPI1: pac::SPI1,
    pub SYSCFG: pac::SYSCFG,
    pub SYSINFO: pac::SYSINFO,
    pub TBMAN: pac::TBMAN,
    // pub TIMER: pac::TIMER,
    pub UART0: pac::UART0,
    pub UART1: pac::UART1,
    pub USBCTRL_DPRAM: pac::USBCTRL_DPRAM,
    pub USBCTRL_REGS: pac::USBCTRL_REGS,
    pub VREG_AND_CHIP_RESET: pac::VREG_AND_CHIP_RESET,
    // pub WATCHDOG: pac::WATCHDOG,
    pub XIP_CTRL: pac::XIP_CTRL,
    pub XIP_SSI: pac::XIP_SSI,
    // pub XOSC: pac::XOSC,

    // HAL drivers
    pub clocks: rp2040_hal::clocks::ClocksManager,
    pub adc: rp2040_hal::Adc,
    pub timer: rp2040_hal::Timer,
}

impl Board {
    pub fn take() -> Option<Self> {
        Some(Self::new(
            pac::Peripherals::take()?,
            pac::CorePeripherals::take()?,
        ))
    }

    pub fn new(mut p: pac::Peripherals, cp: pac::CorePeripherals) -> Self {
        use prelude::*;
        let mut watchdog = Watchdog::new(p.WATCHDOG);
        let sio = Sio::new(p.SIO);

        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            p.XOSC,
            p.CLOCKS,
            p.PLL_SYS,
            p.PLL_USB,
            &mut p.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let pins = bsp::Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

        let adc = bsp::hal::Adc::new(p.ADC, &mut p.RESETS);

        let timer = Timer::new(p.TIMER, &mut p.RESETS);

        Self {
            pins,

            // Core peripherals
            CBP: cp.CBP,
            CPUID: cp.CPUID,
            DCB: cp.DCB,
            DWT: cp.DWT,
            FPB: cp.FPB,
            ITM: cp.ITM,
            MPU: cp.MPU,
            NVIC: cp.NVIC,
            SCB: cp.SCB,
            SYST: cp.SYST,
            TPIU: cp.TPIU,

            // RP2040 peripherals
            // ADC: pac.ADC,
            BUSCTRL: p.BUSCTRL,
            // CLOCKS: pac.CLOCKS,
            DMA: p.DMA,
            I2C0: p.I2C0,
            I2C1: p.I2C1,
            // IO_BANK0: pac.IO_BANK0,
            IO_QSPI: p.IO_QSPI,
            PIO0: p.PIO0,
            PIO1: p.PIO1,
            // PLL_SYS: pac.PLL_SYS,
            // PLL_USB: pac.PLL_USB,
            PPB: p.PPB,
            PSM: p.PSM,
            PWM: p.PWM,
            RESETS: p.RESETS,
            ROSC: p.ROSC,
            RTC: p.RTC,
            // SIO: pac.SIO,
            SPI0: p.SPI0,
            SPI1: p.SPI1,
            SYSCFG: p.SYSCFG,
            SYSINFO: p.SYSINFO,
            TBMAN: p.TBMAN,
            // TIMER: p.TIMER,
            UART0: p.UART0,
            UART1: p.UART1,
            USBCTRL_DPRAM: p.USBCTRL_DPRAM,
            USBCTRL_REGS: p.USBCTRL_REGS,
            VREG_AND_CHIP_RESET: p.VREG_AND_CHIP_RESET,
            // WATCHDOG: pac.WATCHDOG,
            XIP_CTRL: p.XIP_CTRL,
            XIP_SSI: p.XIP_SSI,
            // XOSC: pac.XOSC,
            clocks,
            adc,
            timer,
        }
    }
}

// TODO: Remove once HAL includes this.
/// Reset spinlocks on startup.
///
/// # Safety
/// Call this before anything else in fn main(). Never call it again.
pub unsafe fn spinlock_reset() {
    // Using raw pointers to avoid taking peripherals accidently at startup
    const SIO_BASE: u32 = 0xd0000000;
    const SPINLOCK0_PTR: *mut u32 = (SIO_BASE + 0x100) as *mut u32;
    const SPINLOCK_COUNT: usize = 32;
    for i in 0..SPINLOCK_COUNT {
        SPINLOCK0_PTR.wrapping_add(i).write_volatile(1);
    }
}
