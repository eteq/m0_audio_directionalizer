#![no_std]
#![no_main]

#[cfg(feature = "use_halt")]
use panic_halt as _;
#[cfg(feature = "use_semihosting")]
use panic_semihosting as _;
use panic_persist;

use bsp::hal;
use bsp::pac;
use circuit_playground_express as bsp;

use bsp::entry;
use hal::adc::Adc;
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::prelude::*;
use hal::timer::TimerCounter;
use hal::gpio::v2::pin;

use pac::{CorePeripherals, Peripherals, PM, DAC};

use smart_leds::{
    RGB,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};
use ws2812_timer_delay as ws2812;

use libm;

// adjusts hue in response to the two analog inputs

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


    // Setup UART peripheral.
    let (rx_pin, tx_pin) = (pins.a6, pins.a7);
    let mut uart = bsp::uart(
        &mut clocks,
        115200.hz(),
        peripherals.SERCOM4,
        &mut peripherals.PM,
        rx_pin,
        tx_pin,
    );


    // Check if there was a panic message, if so, send to UART
    if let Some(msg) = panic_persist::get_panic_message_bytes() {
        loop {
            for byte in msg {
                nb::block!(uart.write(*byte)).ok();
            }
            write_message(&mut uart, b"\rreset to stop showing this");
            delay.delay_ms(1000u16);
        }
    }


    // disable the speaker on startup
    let mut speaker_enable = pins.d11.into_push_pull_output();
    speaker_enable.set_low().expect("couldn't turn off speaker!");

    let neopixel_pin: bsp::NeoPixel = pins.d8.into();
    let mut neopixel = ws2812::Ws2812::new(timer, neopixel_pin);

    
    // let mut adc = Adc::adc(peripherals.ADC, &mut peripherals.PM, &mut clocks);
    // let mut a1 : hal::gpio::v2::Pin<hal::gpio::v2::pin::PA05, hal::gpio::v2::Alternate<hal::gpio::v2::B>> = pins.a1.into();
    // let mut a2 : hal::gpio::v2::Pin<hal::gpio::v2::pin::PA06, hal::gpio::v2::Alternate<hal::gpio::v2::B>> = pins.a2.into();
    neopixel_hue(&mut neopixel, &[30_u8; 10], 255, 2).expect("failed to start neopixels");

    
        // Write out a message on start up.
    
        loop {
            write_message(&mut uart, b"blork!");
            delay.delay_ms(500u16);
            write_message(&mut uart, b"blork!");
            delay.delay_ms(500u16);
            panic!("arg");
        }
}

fn neopixel_hue<S: SmartLedsWrite>(neopixel: &mut S, huearr: &[u8; 10], sat: u8, val: u8) -> Result<(), S::Error>
    where <S as SmartLedsWrite>::Color: From<RGB<u8>> {

    let rgbarr: [RGB<u8>; 10] = core::array::from_fn( |i| {
        hsv2rgb(Hsv {
            hue: huearr[i],
            sat: sat,
            val: val,
        })
    });

    neopixel.write(rgbarr.iter().cloned())

}

fn write_message(uart: &mut bsp::Uart, msg: &[u8]) {

    for byte in msg {
        nb::block!(uart.write(*byte)).expect("uart writing failed");
    }

    nb::block!(uart.write('\r' as u8)).expect("uart writing failed");
    nb::block!(uart.write('\n' as u8)).expect("uart writing failed");
}