#![no_std]
#![no_main]

use panic_persist;

use cortex_m::peripheral::NVIC;

use bsp::hal;
use bsp::pac;
use circuit_playground_express as bsp;

use bsp::entry;
use hal::clock::{GenericClockController, ClockGenId, ClockSource};
use hal::delay::Delay;
use hal::prelude::*;
use hal::timer::TimerCounter;

use pac::{CorePeripherals, Peripherals, interrupt};

use smart_leds::{
    RGB,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};
use ws2812_timer_delay as ws2812;

use nb::block;

use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use bsp::hal::usb::UsbBus;


const NPIX : usize = 10;

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut corep = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );
    
    let pins = bsp::Pins::new(peripherals.PORT);
    let mut delay = Delay::new(corep.SYST, &mut clocks);

    let gclk0 = clocks.gclk0();

    // start up RTC
    let rtc_timer_clock = clocks.configure_gclk_divider_and_source(ClockGenId::GCLK2, 8, ClockSource::OSC8M, false).unwrap();
    let rtc_clock = clocks.rtc(&rtc_timer_clock).unwrap();
    let rtc = hal::rtc::Rtc::count32_mode(peripherals.RTC, rtc_clock.freq(), &mut peripherals.PM);
    

    // Setup UART peripheral.
    let mut uart = bsp::uart(
        &mut clocks,
        460800.hz(),
        peripherals.SERCOM4,
        &mut peripherals.PM,
        pins.a6,
        pins.a7,
    );


    // Check if there was a panic message, if so, send to UART
    if let Some(msg) = panic_persist::get_panic_message_bytes() {
        loop {
            for byte in msg {
                block!(uart.write(*byte)).ok();
            }
            write_message_line(&mut uart, b"\rreset to stop showing this");
            delay.delay_ms(1000u16);
        }
    }


    // disable the speaker on startup
    let mut speaker_enable = pins.d11.into_push_pull_output();
    speaker_enable.set_low().expect("couldn't turn off speaker!");

    let npx_timer_clock = clocks.tcc2_tc3(&gclk0).unwrap();
    let mut npx_timer = TimerCounter::tc3_(&npx_timer_clock, peripherals.TC3, &mut peripherals.PM);
    npx_timer.start(3.mhz());

    let neopixel_pin: bsp::NeoPixel = pins.d8.into();

    let mut neopixel = ws2812::Ws2812::new(npx_timer, neopixel_pin);

    neopixel_hue(&mut neopixel, &[30_u8; NPIX], 255, 2).expect("failed to start neopixels");

    unsafe {
        USB_ALLOCATOR = Some(bsp::usb_allocator(
            peripherals.USB,
            &mut clocks,
            &mut peripherals.PM,
            pins.usb_dm,
            pins.usb_dp,
        ));
        let bus_allocator = USB_ALLOCATOR.as_ref().unwrap();

        USB_SERIAL = Some(SerialPort::new(&bus_allocator));
        USB_BUS = Some(
            UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(USB_CLASS_CDC)
                .build(),
        );
    }

    unsafe {
        corep.NVIC.set_priority(interrupt::USB, 1);
        NVIC::unmask(interrupt::USB);
    }

    loop {
        if unsafe { TRIGGER } {
            neopixel_hue(&mut neopixel, &[90_u8; NPIX], 255, 2).expect("failed to start neopixels");
            delay.delay_ms(100u32);
            neopixel_hue(&mut neopixel, &[30_u8; NPIX], 255, 2).expect("failed to start neopixels");
            delay.delay_ms(100u32);
        }
        unsafe { TRIGGER = false; }

        delay.delay_ms(1u32);

    }

}

static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
static mut TRIGGER: bool = false;

static mut LAST_CHAR: u8 = b'0';

#[interrupt]
fn USB() {
    unsafe {
        USB_BUS.as_mut().map(|usb_dev| {
            USB_SERIAL.as_mut().map(|serial| {
                usb_dev.poll(&mut [serial]);
                let mut buf = [0u8; 64];

                if let Ok(count) = serial.read(&mut buf) {
                    // data present

                    // echo back
                    for (i, c) in buf.iter().enumerate() {
                        if i >= count {
                            break;
                        }
                        serial.write(&[c.clone()]).ok();
                        if *c == b'\n' || *c == b'\r' {
                            match LAST_CHAR {
                                b'r' => { TRIGGER = true; },
                                 _ => {}
                            }
                        } 
                        LAST_CHAR = *c;
                    }
                };
            });
        });
    };
}


fn neopixel_hue<S: SmartLedsWrite>(neopixel: &mut S, huearr: &[u8; NPIX], sat: u8, val: u8) -> Result<(), S::Error>
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
        block!(uart.write(*byte)).expect("uart writing failed");
    }
}

fn write_message_line(uart: &mut bsp::Uart, msg: &[u8]) {

    write_message(uart, msg);
    write_message(uart, b"\r\n");

}
