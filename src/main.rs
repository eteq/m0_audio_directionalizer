#![no_std]
#![no_main]

use panic_persist;

use bsp::hal;
use bsp::pac;
use circuit_playground_express as bsp;

use bsp::entry;
//use hal::adc::Adc;
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::prelude::*;
use hal::timer::TimerCounter;
use hal::sercom::v2::spi;
use hal::time::MegaHertz;

use pac::{CorePeripherals, Peripherals};

use smart_leds::{
    RGB,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};
use ws2812_timer_delay as ws2812;

use nb::block;
use numtoa::NumToA;

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
    let mut uart = bsp::uart(
        &mut clocks,
        115200.hz(),
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



    //set up SPI for flash - defaults to 48 MHz and mode=0, which is fine for the flash chip
    let (mut flash_spi, mut flash_cs) = flash_spi_master2(
        &mut clocks, 
        peripherals.SERCOM3, 
        &mut peripherals.PM, 
        pins.flash_sck, 
        pins.flash_mosi, 
        pins.flash_miso, 
        pins.flash_cs);

        let mut idbuffer = [0u8; 3];
        flash_read(&mut flash_spi, &mut flash_cs, 0x9f as u8, -1, &mut idbuffer);
        if idbuffer[0] != 0x01 || idbuffer[1] != 0x40 || idbuffer[2] != 0x15 {
            panic!("Failed to read ID info from flash chip!");
        }

        let mut numtoascratch = [0u8; 20];
        let mut flashdata = [0u8; 3];
        flash_read(&mut flash_spi, &mut flash_cs, 0x03, 0, &mut flashdata);
        write_message(&mut uart, b"pre-buffer:");
        for i in 0..flashdata.len() {write_message(&mut uart, flashdata[i].numtoa(16, &mut numtoascratch));}

    
    flash_command(&mut flash_spi, &mut flash_cs, 0x06, -1);  // Write enable
    flash_command(&mut flash_spi, &mut flash_cs, 0xD8, 0);  // Erase block 0
    let n = wait_for_flash(&mut flash_spi, &mut flash_cs);
    write_message(&mut uart, b"block erase took this many cycles:");
    write_message(&mut uart, n.numtoa(10, &mut numtoascratch));

    let mut outbuff = [0u8; 4];

    write_message(&mut uart, b"pre addr 0:");
    flash_read(&mut flash_spi, &mut flash_cs, 0x03, 0, &mut outbuff);
    for i in 0..outbuff.len() { write_message(&mut uart, outbuff[i].numtoa(10, &mut numtoascratch)); }

    write_message(&mut uart, b"pre addr 1:");
    flash_read(&mut flash_spi, &mut flash_cs, 0x03, 1, &mut outbuff);
    for i in 0..outbuff.len() { write_message(&mut uart, outbuff[i].numtoa(10, &mut numtoascratch)); }

    write_message(&mut uart, b"pre addr 256:");
    flash_read(&mut flash_spi, &mut flash_cs, 0x03, 256, &mut outbuff);
    for i in 0..outbuff.len() { write_message(&mut uart, outbuff[i].numtoa(10, &mut numtoascratch)); }

    let buf1 = [1, 2, 3u8];
    let buf2 = [4, 5, 6u8];
    let buf3 = [7, 8, 9u8];

    flash_write_page(&mut flash_spi, &mut flash_cs, 3, &buf2);
    flash_write_page(&mut flash_spi, &mut flash_cs, 0, &buf1);
    flash_write_page(&mut flash_spi, &mut flash_cs, 256, &buf3);
    

    let m = wait_for_flash(&mut flash_spi, &mut flash_cs);
    write_message(&mut uart, b"write took this many cycles:");
    write_message(&mut uart, m.numtoa(10, &mut numtoascratch));

    write_message(&mut uart, b"post addr 0:");
    flash_read(&mut flash_spi, &mut flash_cs, 0x03, 0, &mut outbuff);
    for i in 0..outbuff.len() { write_message(&mut uart, outbuff[i].numtoa(10, &mut numtoascratch)); }

    write_message(&mut uart, b"post addr 1:");
    flash_read(&mut flash_spi, &mut flash_cs, 0x03, 1, &mut outbuff);
    for i in 0..outbuff.len() { write_message(&mut uart, outbuff[i].numtoa(10, &mut numtoascratch)); }

    write_message(&mut uart, b"post addr 256:");
    flash_read(&mut flash_spi, &mut flash_cs, 0x03, 256, &mut outbuff);
    for i in 0..outbuff.len() { write_message(&mut uart, outbuff[i].numtoa(10, &mut numtoascratch)); }


    flash_command(&mut flash_spi, &mut flash_cs, 0x06, -1);  // Write enable
    flash_command(&mut flash_spi, &mut flash_cs, 0x20, 0x1000);  // Erase sector 1
    let n = wait_for_flash(&mut flash_spi, &mut flash_cs);

    write_message(&mut uart, b"postpost addr 0:");
    flash_read(&mut flash_spi, &mut flash_cs, 0x03, 0, &mut outbuff);
    for i in 0..outbuff.len() { write_message(&mut uart, outbuff[i].numtoa(10, &mut numtoascratch)); }

    write_message(&mut uart, b"postpost addr 1:");
    flash_read(&mut flash_spi, &mut flash_cs, 0x03, 1, &mut outbuff);
    for i in 0..outbuff.len() { write_message(&mut uart, outbuff[i].numtoa(10, &mut numtoascratch)); }

    write_message(&mut uart, b"postpost addr 256:");
    flash_read(&mut flash_spi, &mut flash_cs, 0x03, 256, &mut outbuff);
    for i in 0..outbuff.len() { write_message(&mut uart, outbuff[i].numtoa(10, &mut numtoascratch)); }


    loop {
        neopixel_hue(&mut neopixel, &[85_u8; 10], 255, 2).expect("failed to start neopixels");
        delay.delay_ms(500u16);
        neopixel_hue(&mut neopixel, &[190_u8; 10], 255, 2).expect("failed to start neopixels");
        delay.delay_ms(500u16);
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
        block!(uart.write(*byte)).expect("uart writing failed");
    }

    block!(uart.write('\r' as u8)).expect("uart writing failed");
    block!(uart.write('\n' as u8)).expect("uart writing failed");
}


// this is the same as what's in the BSP but down-rated a bit on the speed because the default 48 MHz didn't work...
fn flash_spi_master2(
    clocks: &mut GenericClockController,
    sercom3: pac::SERCOM3,
    pm: &mut pac::PM,
    sck: impl Into<bsp::FlashSck>,
    mosi: impl Into<bsp::FlashMosi>,
    miso: impl Into<bsp::FlashMiso>,
    cs: impl Into<bsp::FlashCs>,
) -> (bsp::FlashSpi, bsp::FlashCs) {
    let gclk0 = clocks.gclk0();
    let clock = clocks.sercom3_core(&gclk0).unwrap();
    let freq = clock.freq();
    let (sck, mosi, miso, mut cs) = (sck.into(), mosi.into(), miso.into(), cs.into());
    let pads = spi::Pads::default().data_in(miso).data_out(mosi).sclk(sck);
    let spi = spi::Config::new(pm, sercom3, pads, freq)
        .baud(MegaHertz(10))
        .spi_mode(spi::MODE_0)
        .enable();

    cs.set_high().unwrap();

    (spi, cs)
}


fn flash_read(flash_spi: &mut bsp::FlashSpi, flash_cs: &mut bsp::FlashCs, command: u8, address: i32, outbuffer: &mut[u8]) {
    // negative address means no address, as addresses are only 24 bit
    // addresses are byte-addresses

    flash_cs.set_low().expect("failed to set flash cs pin high");

    block!(flash_spi.send(command)).expect("flash send failed");
    block!(flash_spi.read()).expect("flash read failed");
    if address >= 0 {
        for i in (0..3).rev() {
            let offset = i*8;
            block!(flash_spi.send(((address >> offset) & 0xff) as u8)).expect("flash send failed");
            block!(flash_spi.read()).expect("flash read failed");
        }
    }

    for i in 0..outbuffer.len() {
        block!(flash_spi.send(1)).expect("flash send failed");
        outbuffer[i] = block!(flash_spi.read()).expect("flash read failed");
    }

    flash_cs.set_high().expect("failed to set flash cs pin high");
}


fn flash_command(flash_spi: &mut bsp::FlashSpi, flash_cs: &mut bsp::FlashCs, command: u8, address: i32) {
    // negative address means no address, as addresses are only 24 bit

    flash_cs.set_low().expect("failed to set flash cs pin high");

    block!(flash_spi.send(command)).expect("flash send failed");
    block!(flash_spi.read()).expect("flash read failed"); // consume
    if address >= 0 {
        for i in (0..3).rev() {
            let offset = i*8;
            block!(flash_spi.send(((address >> offset) & 0xff) as u8)).expect("flash send failed");
            block!(flash_spi.read()).expect("flash read failed"); // consume
        }
    }

    flash_cs.set_high().expect("failed to set flash cs pin high");
}


fn is_flash_busy(flash_spi: &mut bsp::FlashSpi, flash_cs: &mut bsp::FlashCs) -> bool{
    let mut buf = [0; 1];
    flash_read(flash_spi, flash_cs, 0x05, -1, &mut buf);
    (buf[0] & 1) == 1
}


fn wait_for_flash(flash_spi: &mut bsp::FlashSpi, flash_cs: &mut bsp::FlashCs) -> u32 {

    flash_cs.set_low().expect("failed to set flash cs pin high");

    block!(flash_spi.send(0x05)).expect("flash send failed");
    block!(flash_spi.read()).expect("flash read failed");

    let mut i = -1i32;
    let mut status = 1u8;
    while (status & 1) == 1 {
        block!(flash_spi.send(0x05)).expect("flash send failed");
        status = block!(flash_spi.read()).expect("flash read failed");
        i += 1;
    }

    flash_cs.set_high().expect("failed to set flash cs pin high");

    i as u32
}


fn flash_write_page(flash_spi: &mut bsp::FlashSpi, flash_cs: &mut bsp::FlashCs, address: i32, towrite: &[u8]) {
    // addresses are byte-addresses
    if address < 0 { panic!("must give a non-negative address for writing!"); }
    if towrite.len() < 1 || towrite.len() > 256 { panic!("page must be 0 to 256 bytes!");}
    
    flash_command(flash_spi, flash_cs, 0x06, -1);  // write enable
    
    flash_cs.set_low().expect("failed to set flash cs pin high");

    block!(flash_spi.send(0x02)).expect("flash send failed");
    block!(flash_spi.read()).expect("flash read failed"); // consume
    for i in (0..3).rev() {
        let offset = i*8;
        block!(flash_spi.send(((address >> offset) & 0xff) as u8)).expect("flash send failed");
        block!(flash_spi.read()).expect("flash read failed"); // consume
    }
    for i in 0..towrite.len() {
        block!(flash_spi.send(towrite[i])).expect("flash send failed");
        block!(flash_spi.read()).expect("flash read failed"); // consume

    }
    flash_cs.set_high().expect("failed to set flash cs pin high");
}