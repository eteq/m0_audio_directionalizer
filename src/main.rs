#![no_std]
#![no_main]

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


    // disable the speaker on startup
    let mut speaker_enable = pins.d11.into_push_pull_output();
    speaker_enable.set_low().expect("couldn't turn off speaker!");

    let neopixel_pin: bsp::NeoPixel = pins.d8.into();
    let mut neopixel = ws2812::Ws2812::new(timer, neopixel_pin);

    
    // let mut adc = Adc::adc(peripherals.ADC, &mut peripherals.PM, &mut clocks);
    // let mut a1 : hal::gpio::v2::Pin<hal::gpio::v2::pin::PA05, hal::gpio::v2::Alternate<hal::gpio::v2::B>> = pins.a1.into();
    // let mut a2 : hal::gpio::v2::Pin<hal::gpio::v2::pin::PA06, hal::gpio::v2::Alternate<hal::gpio::v2::B>> = pins.a2.into();
    neopixel_hue(&mut neopixel, &[10_u8; 10], 255, 2).expect("failed to start neopixels");


    let mut _speaker_out: pin::Pin<_, pin::AlternateB> = pins.a0.into_alternate();
    let mut dac = peripherals.DAC;
    init_dac(&mut dac, &mut peripherals.PM, &mut clocks);
    //speaker_enable.set_high().expect("couldn't turn on speaker!");

    
    
    let mut i = 0;
    loop {
        let val_10bit = 0x2f * (i%2);//(val * (0x3ff as f32)) as u16;

        while dac.status.read().syncbusy().bit_is_set() {}
        dac.data.write(|w| unsafe { w.bits(val_10bit) });
        
        delay.delay_us(800_u16); 
        i += 1;
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

fn init_dac(dac: &mut DAC, pm: &mut PM, clocks: &mut GenericClockController) {
    pm.apbcmask.modify(|_, w| w.dac_().set_bit());

    let gclk0 = clocks.gclk0();
    clocks.dac(&gclk0).expect("dac clock setup failed");
    while dac.status.read().syncbusy().bit_is_set() {}

    // reset because you never know...
    dac.ctrla.modify(|_, w| w.swrst().set_bit());
    while dac.status.read().syncbusy().bit_is_set() {}


    dac.ctrlb.modify(|_, w| {
        w.eoen().set_bit();
        w.refsel().int1v()
    });
    while dac.status.read().syncbusy().bit_is_set() {}


    // enable
    dac.ctrla.modify(|_, w| w.enable().set_bit());
    while dac.status.read().syncbusy().bit_is_set() {}
}