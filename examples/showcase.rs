//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use solderparty_rp2040_stamp_carrier::prelude::*;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    delay.delay_ms(500);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    // Enable ADC
    let mut adc = bsp::hal::Adc::new(pac.ADC, &mut pac.RESETS);
    // Enable the temperature sense channel
    // let mut temperature_sensor = adc.enable_temp_sensor();
    // Configure GPIO26 as an ADC input
    let mut adc_pin_0 = pins.a0.into_floating_input();
    // Configure GPIO27 as an ADC input
    let mut adc_pin_1 = pins.a1.into_floating_input();
    // Configure GPIO28 as an ADC input
    let mut adc_pin_2 = pins.a2.into_floating_input();
    let mut adc_pin_3 = pins.a3.into_floating_input();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        // The onboard NeoPixel is attached to GPIO pin #21 on the RP2040 Stamp.
        pins.neopixel.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    // Infinite colour wheel loop
    let mut n: u8 = 128;
    loop {
        // let temp_sens_adc_counts: u16 = adc.read(&mut temperature_sensor).unwrap();
        let pin_adc_counts: u16 = adc.read(&mut adc_pin_0).unwrap();
        let pin_adc_counts1: u16 = adc.read(&mut adc_pin_1).unwrap();
        let pin_adc_counts2: u16 = adc.read(&mut adc_pin_2).unwrap();
        let pin_adc_counts3: u16 = adc.read(&mut adc_pin_3).unwrap();
        info!(
            "ADC readings: Pin: {:02}, pin1 {:02}, pin2 {:02}, pin3 {:02}\r\n",
            pin_adc_counts, pin_adc_counts1, pin_adc_counts2, pin_adc_counts3
        );
        ws.write(smart_leds::brightness(once(wheel(n)), 32))
            .unwrap();
        n = n.wrapping_add(1);
        delay.delay_ms(25);
    }
}

/// Convert a number from `0..=255` to an RGB color triplet.
///
/// The colours are a transition from red, to green, to blue and back to red.
fn wheel(mut wheel_pos: u8) -> smart_leds::RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        // No green in this sector - red and blue only
        (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    } else if wheel_pos < 170 {
        // No red in this sector - green and blue only
        wheel_pos -= 85;
        (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    } else {
        // No blue in this sector - red and green only
        wheel_pos -= 170;
        (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    }
}

// End of file
