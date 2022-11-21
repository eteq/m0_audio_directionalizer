#![no_std]
#![no_main]

use core::ptr;
use panic_persist;

use bsp::hal;
use bsp::pac;
use circuit_playground_express as bsp;

use bsp::entry;
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::prelude::*;
use hal::timer::TimerCounter;
use hal::sercom::v2::spi;
use hal::time::MegaHertz;

use hal::gpio::v2 as gpio;

use pac::{CorePeripherals, Peripherals,gclk};

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

    // set up the input pins
    let left_button = pins.d4.into_pull_down_input(); // 1 is pressed
    let right_button = pins.d5.into_pull_down_input(); // 1 is pressed
    let slide_switch = pins.d7.into_pull_up_input(); //1 is pointing to left


    let npx_timer_clock = clocks.tcc2_tc3(&gclk0).unwrap();
    let mut npx_timer = TimerCounter::tc3_(&npx_timer_clock, peripherals.TC3, &mut peripherals.PM);
    npx_timer.start(3.mhz());

    let neopixel_pin: bsp::NeoPixel = pins.d8.into();

    let mut neopixel = ws2812::Ws2812::new(npx_timer, neopixel_pin);

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

        /*  
        interface plan:
        * neopixels yellow for record mode if slide is 0, green if slide is 1
        * if in record mode, left button triggers chip erase -> white neopixels until done.
        * if in record mode, right button triggers record -> blue neopixels
        * if in green mode, either button triggers data dump over uart -> magenta neopixels

        plan on ~ 20ksamp / sec, 16-bit samples x 2 -> 32-bit samples, 80 kB/sec, well within the 365 kB/s limit 
        set up ADC as 12 bit resolution, single-ended, 1/2 VDDANA, gain=2, free-running, which yields 8 clocks per conversion
        350 ks / sec limit for 20 kHz x 2 channels -> up to 8 conversions per sample.  Try boosting to 13 bits via AVGCTRL.SAMPLENUM = 0x2, AVGCTRL.ADJRES = 0x1
        20 *2 * 4 -> 160 kConversions -> 1.28 MHz before division.  Closesest matches are 96M / (4*19) or 48/9/4 (both DIV4 with another gclk)
        *OR* can do 96 MHz / 15 / DIV4 / 10 clocks per conversion (requires a sampletime of 2.5 clock cycles?) -> 160 kconvs -> 20 khz per channel
        A1->PA05->AIN5
        A2->PA06->AIN6
        A3->PA07->AIN7

        in interrupt, catch result and drop it in a buffer that is 256 bytes long. switch to other buffer when full
        in main loop, notice when a buffer is full and write it to flash as a full page
        stop if record button is released
        */

    // set up ADC with a1/a2 pins - note a1 and a2 are not directly used after this, and init_adc is currently hard-coded to use them
    let mut _a1 : gpio::Pin<gpio::pin::PA05, gpio::Alternate<gpio::B>> = pins.a1.into();
    let mut _a2 : gpio::Pin<gpio::pin::PA06, gpio::Alternate<gpio::B>> = pins.a2.into();
    let adc = init_adc(peripherals.ADC, &mut peripherals.PM, &mut clocks);    
    

    loop {
        if slide_switch.is_low().unwrap() {
            //record mode
            neopixel_hue(&mut neopixel, &[35_u8; 10], 255, 2).unwrap();
            if left_button.is_high().unwrap() {
                // chip erase
                neopixel_hue(&mut neopixel, &[0_u8; 10], 0, 2).unwrap();

                write_message_line(&mut uart, b"starting flash erase");
                flash_command(&mut flash_spi, &mut flash_cs, 0x06, -1); // write enable
                flash_command(&mut flash_spi, &mut flash_cs, 0x60, -1); //chip erase
                wait_for_flash(&mut flash_spi, &mut flash_cs);
                write_message_line(&mut uart, b"flash erased!");
            }
            if right_button.is_high().unwrap() {
                //record!

                if !is_page_erased(&mut flash_spi, &mut flash_cs, 0) {
                    neopixel_hue(&mut neopixel, &[0_u8; 10], 255, 2).unwrap();
                    write_message_line(&mut uart, b"cannot record - flash not empty!");
                    while right_button.is_high().unwrap() { }
                } else {
                    neopixel_hue(&mut neopixel, &[170_u8; 10], 255, 2).unwrap();
                    write_message_line(&mut uart, b"starting recording");
                    // TODO: start ADC
                    neopixel_hue(&mut neopixel, &[170_u8; 10], 255, 2).unwrap();

                    while right_button.is_high().unwrap() || true { 
                        // whatever saving happens here

                        while adc.status.read().syncbusy().bit_is_set() {}
                        let res = adc.result.read().bits();
                        write_message_line(&mut uart, b"adc result:");
                        write_message_line(&mut uart, res.numtoa(10, &mut numtoascratch));
                        delay.delay_ms(500u16);

                    }
                    // TODO: stop ADC

                    write_message_line(&mut uart, b"recording completed");
                }
                delay.delay_ms(100u16); // for debouncing
            }
        } else {
            // writeback mode
            neopixel_hue(&mut neopixel, &[82_u8; 10], 255, 2).unwrap();
            if left_button.is_high().unwrap() || right_button.is_high().unwrap() {
                // dump flash to uart - magenta
                neopixel_hue(&mut neopixel, &[215_u8; 10], 255, 2).unwrap();

                let nbytes = 2*1024*1024;

                write_message(&mut uart, b"Reading back ");
                write_message(&mut uart, nbytes.numtoa(16, &mut numtoascratch));
                write_message_line(&mut uart, b" bytes");

                flash_cs.set_low().expect("failed to set flash cs pin high");
            
                block!(flash_spi.send(0x03)).expect("flash send failed");
                block!(flash_spi.read()).expect("flash read failed");
                // address 0
                for _ in (0..3).rev() {
                    block!(flash_spi.send(0)).expect("flash send failed");
                    block!(flash_spi.read()).expect("flash read failed");
                }
                
                
                for _ in 0..nbytes {
                    let byte_to_transfer: u8;

                    block!(flash_spi.send(1)).expect("flash send failed");
                    byte_to_transfer = block!(flash_spi.read()).expect("flash read failed");
                    block!(uart.write(byte_to_transfer)).expect("uart write failed");
                }
            
                flash_cs.set_high().expect("failed to set flash cs pin high");
            }
        }
        delay.delay_ms(5u16);
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
}

fn write_message_line(uart: &mut bsp::Uart, msg: &[u8]) {

    write_message(uart, msg);
    write_message(uart, b"\r\n");

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


#[allow(dead_code)]
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


fn is_page_erased(flash_spi: &mut bsp::FlashSpi, flash_cs: &mut bsp::FlashCs, address: i32) -> bool {
    let mut outbuffer = [0u8; 256];
    flash_read(flash_spi, flash_cs, 0x03, address, &mut outbuffer);
    for i in 0..outbuffer.len() {
        if outbuffer[i] != 0xff { return false }
    }
    true
}


fn init_adc(adc: bsp::pac::ADC, 
            pm: &mut bsp::pac::PM, 
            clocks: &mut GenericClockController) -> bsp::pac::ADC {
    pm.apbcmask.modify(|_, w| w.adc_().set_bit());

    //set up 96 MHz clock / 15
    let gclk2 = clocks.configure_gclk_divider_and_source(gclk::clkctrl::GEN_A::GCLK2, 
        15, 
        gclk::genctrl::SRC_A::DPLL96M, 
        true).unwrap();

    clocks.adc(&gclk2).expect("adc clock setup failed");
    while adc.status.read().syncbusy().bit_is_set() {}

    // reset should not be necessary but just in case
    adc.ctrla.modify(|_, w| w.swrst().set_bit());
    while adc.status.read().syncbusy().bit_is_set() {}

    adc.ctrlb.modify(|_, w| {
        w.prescaler().div4();
        w.ressel()._16bit();
        w.freerun().set_bit()
    });
    while adc.status.read().syncbusy().bit_is_set() {}

    adc.sampctrl.modify(|_, w| unsafe { w.samplen().bits(4) }); //sample length
    while adc.status.read().syncbusy().bit_is_set() {}

    adc.avgctrl.modify(|_, w| {
        w.samplenum()._4();
        unsafe { w.adjres().bits(1) }
    });

    adc.refctrl.modify(|_, w| w.refsel().intvcc1() ); // 1/2 VDDANA
    while adc.status.read().syncbusy().bit_is_set() {}

    // load calibration data from NVM cal area
    let regval1: u32;
    let regval2: u32;
    unsafe {
        let addr1: *const u32 = (0x806020u32) as *const _;  
        let addr2: *const u32 = (0x806024u32) as *const _;  
        regval1 = ptr::read(addr1);
        regval2 = ptr::read(addr2);
    }
    let linearity_cal = (((regval2 & 0x7) << 5) | ((regval1 & 0xf8000000) >> 27)) as u8;
    let bias_cal = ((regval2 & 0b111000) >> 3) as u8;

    adc.calib.modify(|_, w|  {
        unsafe {
            w.linearity_cal().bits(linearity_cal);
            w.bias_cal().bits(bias_cal)
        }
    });
    while adc.status.read().syncbusy().bit_is_set() {}


    adc.inputctrl.modify(|_, w|  {
        w.muxneg().gnd(); // No negative input (internal gnd)
        w.gain().div2(); // offsets 1/2 VDDANA
        w.muxpos().pin5(); // pin5 is A1
        unsafe { w.inputscan().bits(1) }  // 1 cycles through pin 5/6 / A1/A2, 0 is just A1
    });
    while adc.status.read().syncbusy().bit_is_set() {}

    // power up
    adc.ctrla.modify(|_, w| w.enable().set_bit());
    while adc.status.read().syncbusy().bit_is_set() {}

    // trigger now to start freerunning going continuously
    adc.swtrig.modify(|_, w| w.start().set_bit());
    while adc.status.read().syncbusy().bit_is_set() {}

    adc
}
