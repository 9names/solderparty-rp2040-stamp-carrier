//! Showcase some of the functionality of the Solder Party RP2040 Stamp Carrier

#![no_std]
#![no_main]

use bsp::hal::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use solderparty_rp2040_stamp_carrier::prelude::*;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut board = bsp::Board::take().unwrap();
    let pins = board.pins;
    let mut led = pins.led.into_push_pull_output();

    // Configure GPIO26 as an ADC input
    let mut adc_pin_0 = pins.a0.into_floating_input();
    // Configure GPIO27 as an ADC input
    let mut adc_pin_1 = pins.a1.into_floating_input();
    // Configure GPIO28 as an ADC input
    let mut adc_pin_2 = pins.a2.into_floating_input();
    // Configure GPIO29 as an ADC input
    let mut adc_pin_3 = pins.a3.into_floating_input();

    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = board.PIO0.split(&mut board.RESETS);
    let mut ws = Ws2812::new(
        // The onboard NeoPixel is attached to GPIO pin #21 on the RP2040 Stamp.
        pins.neopixel.into_mode(),
        &mut pio,
        sm0,
        board.clocks.peripheral_clock.freq(),
        board.timer.count_down(),
    );

    // Configure the Timer peripheral in count-down modeMER, &mut board.RESETS);
    let mut count_down = board.timer.count_down();

    // Infinite colour wheel loop
    let mut n: u8 = 128;
    loop {
        // capture data from every ADC channel
        let pin_adc_counts: u16 = board.adc.read(&mut adc_pin_0).unwrap();
        let pin_adc_counts1: u16 = board.adc.read(&mut adc_pin_1).unwrap();
        let pin_adc_counts2: u16 = board.adc.read(&mut adc_pin_2).unwrap();
        let pin_adc_counts3: u16 = board.adc.read(&mut adc_pin_3).unwrap();
        info!(
            "ADC readings: Pin: {:02}, pin1 {:02}, pin2 {:02}, pin3 {:02}\r\n",
            pin_adc_counts, pin_adc_counts1, pin_adc_counts2, pin_adc_counts3
        );
        // Rotate through the colours on the RGB LED
        ws.write(smart_leds::brightness(once(wheel(n)), 32))
            .unwrap();
        n = n.wrapping_add(1);
        let _ = led.toggle();

        // Sleep for 25ms
        count_down.start(25.millis());
        let _ = nb::block!(count_down.wait());
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
