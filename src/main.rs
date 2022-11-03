#![no_std]
#![no_main]

// Neopixel Rainbow
// This only functions when the --release version is compiled. Using the debug
// version leads to slow pulse durations which results in a straight white LED
// output.
//
// // Needs to be compiled with --release for the timing to be correct

#[cfg(not(feature = "use_semihosting"))]
use panic_halt as _;
#[cfg(feature = "use_semihosting")]
use panic_semihosting as _;

use bsp::hal;
use bsp::pac;
use circuit_playground_express as bsp;

use bsp::entry;
use hal::adc::Adc;
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::prelude::*;
use hal::timer::TimerCounter;

use pac::{CorePeripherals, Peripherals};

use smart_leds::{
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};
use ws2812_timer_delay as ws2812;

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );
    let pins = bsp::Pins::new(peripherals.PORT);
    let mut delay = Delay::new(core.SYST, &mut clocks);

    let gclk0 = clocks.gclk0();
    let timer_clock = clocks.tcc2_tc3(&gclk0).unwrap();
    let mut timer = TimerCounter::tc3_(&timer_clock, peripherals.TC3, &mut peripherals.PM);
    timer.start(3.mhz());

    let neopixel_pin: bsp::NeoPixel = pins.d8.into();
    let mut neopixel = ws2812::Ws2812::new(timer, neopixel_pin);

    
    let mut adc = Adc::adc(peripherals.ADC, &mut peripherals.PM, &mut clocks);
    let mut a1 : hal::gpio::v2::Pin<hal::gpio::v2::pin::PA05, hal::gpio::v2::Alternate<hal::gpio::v2::B>> = pins.a1.into();
    let mut a2 : hal::gpio::v2::Pin<hal::gpio::v2::pin::PA06, hal::gpio::v2::Alternate<hal::gpio::v2::B>> = pins.a2.into();

    loop {
        // to span the full hue range, data / 16, but this maps to 0-240 deg i.e.red to blue

        let data1: u16 = adc.read(&mut a1).unwrap();
        let hue1: u8 = (data1 *15 / 360).try_into().unwrap();  
        let rgb1 = hsv2rgb(Hsv {
            hue: hue1,
            sat: 255,
            val: 2,
        });

        let data2: u16 = adc.read(&mut a2).unwrap();
        let hue2: u8 = (data2 *15 / 360).try_into().unwrap();  
        let rgb2= hsv2rgb(Hsv {
            hue: hue2,
            sat: 255,
            val: 2,
        });
        
        
        let mut colors = [rgb1; 10];
        for i in 0..3 { colors[i] = rgb2; }
        for i in 8..10 { colors[i] = rgb2; }

        neopixel.write(colors.iter().cloned()).unwrap();
        delay.delay_ms(100u16);
    }
}


