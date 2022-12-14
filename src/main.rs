#![no_std]
#![no_main]

use core::ptr;
use panic_persist;

use cortex_m::peripheral::NVIC;
use cortex_m::interrupt::free as disable_interrupts;

use bsp::hal;
use bsp::pac;
use circuit_playground_express as bsp;

use bsp::entry;
use hal::clock::{GenericClockController, ClockGenId, ClockSource};
use hal::delay::Delay;
use hal::prelude::*;
use hal::timer::TimerCounter;
use hal::sercom::v2::spi;
use hal::time::MegaHertz;

use hal::gpio::v2 as gpio;

use pac::{CorePeripherals, Peripherals, interrupt};

use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC, UsbError};
use bsp::hal::usb::UsbBus;

use smart_leds::{
    RGB,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};
use ws2812_timer_delay as ws2812;

use nb::block;
use numtoa::NumToA;

const NPIX : usize = 10;

// ugh globals not concurrency safe, RTIC if need anything fancier
#[cfg(not(feature = "log-time"))]
const DATA_SIZE: usize = 128; 
#[cfg(feature = "log-time")]
const DATA_SIZE: usize = 126;  // a page is 256 bytes, so this should fit that - overhead of u32 rtc count at the start of the page
static mut N_OVERRUN:usize = 0;
static mut DATA1: [u16; DATA_SIZE] = [0u16; DATA_SIZE];
static mut DATA1_IDX: usize = 0;
static mut DATA2: [u16; DATA_SIZE] = [0u16; DATA_SIZE];
static mut DATA2_IDX: usize = 0;

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
    

    // set up USB
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

        corep.NVIC.set_priority(interrupt::USB, 1);
        NVIC::unmask(interrupt::USB);
    }


    // Check if there was a panic message, if so, send to USB serial
    if let Some(msg) = panic_persist::get_panic_message_bytes() {
        loop {
            write_message(msg);
            write_message_line(b"\rreset to stop showing this");
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

    neopixel_hue(&mut neopixel, &[30_u8; NPIX], 255, 2).expect("failed to start neopixels");

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
        * if in green mode, either button triggers data dump -> magenta neopixels

        plan on ~ 20ksamp / sec, 16-bit samples x 2 -> 32-bit samples, 80 kB/sec, well within the 365 kB/s limit 
        https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/ gives timing calcs.  To get 22.05 kHz per channel:
        48 MHz / 17 & DIV4 & 12-bit & DIV2 gain & SAMPLEN=0, &16 samples & no offset corren . But that gave ~8 k per sec so something's very wrong there
        Alternatives:
        48 MHz / DIV4 / gclockdiv=8 / 4 accumulations / 8.5 clocks per conversion = 22.058 kHz per channel (within noise of clock to 22.05 kHz)
        48 MHz / DIV4 / gclockdiv=10 / 2 accumulations / 12.5 clocks per conversion = 24 kHz per channel
        48 MHz / DIV4 / gclockdiv=6 / 4 accumulations / 10.5 clocks per conversion = 23.81 kHz per channel (within noise of clock to 24 kHz)
        48 MHz / DIV4 / gclockdiv=7 / 4 accumulations / 9 clocks per conversion = 23.81 kHz per channel (within noise of clock to 24 kHz)

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

    unsafe { corep.NVIC.set_priority(interrupt::ADC, 1); }
    let adc = init_adc(peripherals.ADC, &mut peripherals.PM, &mut clocks);    
    

    loop {
        if slide_switch.is_low().unwrap() {
            //record mode
            neopixel_hue(&mut neopixel, &[35_u8; NPIX], 255, 2).unwrap();
            if left_button.is_high().unwrap() {
                // chip erase
                neopixel_hue(&mut neopixel, &[0_u8; NPIX], 0, 2).unwrap();

                write_message_line(b"starting flash erase");
                flash_command(&mut flash_spi, &mut flash_cs, 0x06, -1); // write enable
                flash_command(&mut flash_spi, &mut flash_cs, 0x60, -1); //chip erase
                wait_for_flash(&mut flash_spi, &mut flash_cs);
                write_message_line(b"flash erased!");
            }
            if right_button.is_high().unwrap() {
                //record!

                if !is_page_erased(&mut flash_spi, &mut flash_cs, 0) {
                    neopixel_hue(&mut neopixel, &[0_u8; NPIX], 255, 2).unwrap();
                    write_message_line(b"cannot record - flash not empty!");
                    while right_button.is_high().unwrap() { }
                } else {
                    neopixel_hue(&mut neopixel, &[170_u8; NPIX], 255, 2).unwrap();
                    write_message_line(b"starting recording");
                    
                    // reset the global state before starting the interrupt
                    unsafe { 
                        N_OVERRUN = 0;
                        DATA1_IDX = 0;
                        DATA2_IDX = 0;
                    }

                    let start_time = rtc.count32();
                    // Clear the interrupt flag just before unmasking
                    adc.intflag.modify(|_, w| w.resrdy().set_bit());
                    unsafe{ NVIC::unmask(interrupt::ADC); }


                    neopixel_hue(&mut neopixel, &[170_u8; NPIX], 255, 2).unwrap();

                    let mut pages_written = 0usize;
                    let mut button_counts = -10isize;  //negative means it has to be low that long
                    while button_counts < 10 && pages_written < 8191 {  // leave page 0 for the metadata

                        let whichdata: usize;
                        unsafe {
                            if (DATA1_IDX >= DATA_SIZE) && (DATA2_IDX > 0){
                                whichdata = 1;
                            } else if (DATA2_IDX >= DATA_SIZE) && (DATA1_IDX > 0) {
                                whichdata = 2;
                            } else {
                                whichdata = 0;
                            }
                        }

                        if (whichdata > 0) && (whichdata < 3) {
                            let data_arr: &mut [u16; DATA_SIZE];
                            unsafe {
                                if whichdata == 1 {
                                    data_arr = &mut DATA1;
                                    disable_interrupts(|_| { DATA1_IDX = 0; });
                                } else {
                                    data_arr = &mut DATA2;
                                    disable_interrupts(|_| { DATA2_IDX = 0; });
                                }
                            }
                            let mut write_buffer = [0u8; 256];
                            for i in 0..data_arr.len() {
                                let j = i*2;
                                write_buffer[j] = ((data_arr[i] >> 8) & 0xff) as u8;
                                write_buffer[j + 1] = (data_arr[i] & 0xff) as u8;
                            }
                            #[cfg(feature = "log-time")] 
                            {
                                let tcount = rtc.count32();
                                write_buffer[252] = ((tcount >> 24) & 0xff) as u8;
                                write_buffer[253] = ((tcount >> 16) & 0xff) as u8;
                                write_buffer[254] = ((tcount >> 8) & 0xff) as u8;
                                write_buffer[255] = (tcount & 0xff) as u8;
                            }

                            flash_write_page(&mut flash_spi, &mut flash_cs, ((pages_written + 1) as isize)*256, &write_buffer);

                            pages_written += 1;

                        }
                        if button_counts < 0 {
                            if right_button.is_low().unwrap() { button_counts += 1; }
                        } else {
                            if right_button.is_high().unwrap() { button_counts += 1; }
                        }
                        delay.delay_ms(1u16);
                    }
                    
                    // mask to stop responding to the free-running ADC
                    NVIC::mask(interrupt::ADC);
                    let stop_time = rtc.count32();

                    let no = unsafe { N_OVERRUN };

                    let freqeff = ((pages_written*DATA_SIZE) as f32)*1e6_f32/((stop_time - start_time) as f32);

                    // turn off the neopixels for fast "it's done" response
                    neopixel_hue(&mut neopixel, &[170_u8; NPIX], 255, 0).unwrap();

                    // write info to the first page. first 2 bytes give a number of pages, next 2 bytes are best guess at sampling frequency
                    let write_buffer: [u8; 4] = [((pages_written >> 8) & 0xff) as u8, 
                                                 (pages_written & 0xff) as u8,
                                                 (((freqeff as u16) >> 8) & 0xff) as u8, 
                                                 ((freqeff as u16) & 0xff) as u8
                                                ];
                    flash_write_page(&mut flash_spi, &mut flash_cs, 0, &write_buffer);

                    write_message_line(b"Recording completed.");
                    write_message(b"kB written:");
                    write_message_line((pages_written/4).numtoa(10, &mut numtoascratch));
                    write_message(b"Number of overruns:");
                    write_message_line(no.numtoa(10, &mut numtoascratch));

                    write_message(b"Sampling frequency:");
                    write_message_line((freqeff as u32).numtoa(10, &mut numtoascratch));

                    // let the button come up
                    button_counts = -10;
                    while button_counts < 0 { 
                        if right_button.is_low().unwrap() { button_counts += 1; }
                        delay.delay_ms(1u16); 
                    }
                }
                delay.delay_ms(20u16); // for debouncing
            }
        } else {
            // writeback mode
            neopixel_hue(&mut neopixel, &[82_u8; NPIX], 255, 2).unwrap();
            let dosize = unsafe { SIZE_TRIGGER };
            let doread = left_button.is_high().unwrap() || right_button.is_high().unwrap() || unsafe { READ_TRIGGER };
            
            if doread || dosize {
                // dump flash - magenta
                neopixel_hue(&mut neopixel, &[215_u8; NPIX], 255, 2).unwrap();

                // read first two bytes to work out how many bytes to read
                flash_cs.set_low().expect("failed to set flash cs pin low");
                block!(flash_spi.send(0x03)).expect("flash send failed");
                block!(flash_spi.read()).expect("flash read failed");
                // address 0
                for _ in (0..3).rev() {
                    block!(flash_spi.send(0)).expect("flash send failed");
                    block!(flash_spi.read()).expect("flash read failed");
                }
                block!(flash_spi.send(0x03)).expect("flash send failed");
                let nbytes1 = block!(flash_spi.read()).unwrap();
                block!(flash_spi.send(0x03)).expect("flash send failed");
                let nbytes2 = block!(flash_spi.read()).unwrap();
                flash_cs.set_high().expect("failed to set flash cs pin high");

                //let nbytes = 2*1024*1024;
                let npages: usize = (nbytes2 as usize) + ((nbytes1 as usize) << 8);
                let nbytes = npages*256;

                if dosize {
                    write_message_line(nbytes.numtoa(10, &mut numtoascratch));
                    unsafe { SIZE_TRIGGER = false; }
                }

                if doread {
                    flash_cs.set_low().expect("failed to set flash cs pin low");
                
                    block!(flash_spi.send(0x03)).expect("flash send failed");
                    block!(flash_spi.read()).expect("flash read failed");
                    // address 256, skipping the first page
                    block!(flash_spi.send(0)).expect("flash send failed");
                    block!(flash_spi.read()).expect("flash read failed");
                    block!(flash_spi.send(1)).expect("flash send failed");
                    block!(flash_spi.read()).expect("flash read failed");
                    block!(flash_spi.send(0)).expect("flash send failed");
                    block!(flash_spi.read()).expect("flash read failed");
                    
                    
                    let mut send_buffer = [0u8; 128];
                    for i in 0..nbytes {
                        let byte_to_transfer: u8;

                        block!(flash_spi.send(1)).expect("flash send failed");
                        byte_to_transfer = block!(flash_spi.read()).expect("flash read failed");
                        //block!(uart.write(byte_to_transfer)).expect("uart write failed");
                        send_buffer[i%128] = byte_to_transfer;
                        if i%128 == 127 {
                            write_message(&send_buffer);
                        }
                    }
                    // also manage the tail of the buffer that was read but not written let
                    if nbytes%128 != 0 {
                        write_message(&send_buffer[0..(nbytes%128)]);
                    }
                
                    flash_cs.set_high().expect("failed to set flash cs pin high");

                    unsafe { READ_TRIGGER = false; }
                }
            } 
        }
        delay.delay_ms(5u16);
    }
}


static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
static mut READ_TRIGGER: bool = false;
static mut SIZE_TRIGGER: bool = false;

static mut LAST_CHAR_USB: u8 = b'0';

#[interrupt]
fn USB() {
    unsafe {
        USB_BUS.as_mut().map(|usb_dev| {
            USB_SERIAL.as_mut().map(|serial| {
                usb_dev.poll(&mut [serial]);
                let mut buf = [0u8; 64];

                if let Ok(count) = serial.read(&mut buf) {
                    // data present

                    for (i, c) in buf.iter().enumerate() {
                        if i >= count {
                            break;
                        }
                        // uncomment this to echo back
                        //serial.write(&[c.clone()]).ok();
                        if *c == b'\n' || *c == b'\r' {
                            match LAST_CHAR_USB {
                                b'r' => { READ_TRIGGER = true; },
                                b's' => { SIZE_TRIGGER = true; },
                                 _ => {}
                            }
                        } 
                        LAST_CHAR_USB = *c;
                    }
                };
            });
        });
    };
}


#[interrupt]
fn ADC() {
    let adcreg;
    unsafe{ adcreg = bsp::pac::ADC::ptr().as_ref().unwrap(); }

    if adcreg.intflag.read().resrdy().bit_is_set() {
        // conversion completed
        disable_interrupts(|_| {
            unsafe {
                let i: &mut usize;
                let dataarr: &mut [u16; DATA_SIZE];

                // use data2 if it has been started but not full, or data1 is full, or 
                if DATA1_IDX >= DATA_SIZE {
                    if DATA2_IDX >= DATA_SIZE {
                        panic!("both buffers are full, ahh!!");
                    } else {
                        i = &mut DATA2_IDX;
                        dataarr = &mut DATA2;
                    }
                } else if (DATA2_IDX > 0) && (DATA2_IDX < DATA_SIZE) {
                    i = &mut DATA2_IDX;
                    dataarr = &mut DATA2;
                } else {
                    i = &mut DATA1_IDX;
                    dataarr = &mut DATA1;
                }

                dataarr[*i] = adcreg.result.read().bits();
                *i += 1;
            }
        });

        // clear the data ready interrupt
        adcreg.intflag.write(|w| w.resrdy().set_bit());
    }

    // note overrun does *not* trigger the interrupt as of this comment - this is bookkeeping for when the data are recorded
    if adcreg.intflag.read().overrun().bit_is_set() {
        // an overrun occurred since the last conversion, record it and clear

        disable_interrupts(|_| unsafe {N_OVERRUN += 1;});
        // clear the interrupt
        adcreg.intflag.write(|w| w.overrun().set_bit());
    }
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



fn write_message(msg: &[u8]) {
    let serial = unsafe { USB_SERIAL.as_mut().unwrap() };
    let mut written: usize = 0;
    while written < msg.len() {
        match serial.write(msg) {
            Err(UsbError::WouldBlock) => { },  // keep trying
            Err(_err) => { panic!("USB error"); }
            Ok(count) => { written += count; }
        }
    }
}

fn write_message_line(msg: &[u8]) {
    write_message(msg);
    write_message(b"\r\n");

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


fn flash_read(flash_spi: &mut bsp::FlashSpi, flash_cs: &mut bsp::FlashCs, command: u8, address: isize, outbuffer: &mut[u8]) {
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


fn flash_command(flash_spi: &mut bsp::FlashSpi, flash_cs: &mut bsp::FlashCs, command: u8, address: isize) {
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

    let mut i = -1isize;
    let mut status = 1u8;
    while (status & 1) == 1 {
        block!(flash_spi.send(0x05)).expect("flash send failed");
        status = block!(flash_spi.read()).expect("flash read failed");
        i += 1;
    }

    flash_cs.set_high().expect("failed to set flash cs pin high");

    i as u32
}


fn flash_write_page(flash_spi: &mut bsp::FlashSpi, flash_cs: &mut bsp::FlashCs, address: isize, towrite: &[u8]) {
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


fn is_page_erased(flash_spi: &mut bsp::FlashSpi, flash_cs: &mut bsp::FlashCs, address: isize) -> bool {
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


    //48 MHz / DIV4 / gclockdiv=8 / 4 accumulations / 8.5 clocks per conversion = 22.058 kHz per channel (within noise of clock to 22.05 kHz)
    // 8.5 clocks per conversion = samplelen of 1
    // this still doesn't match, so just go empirical to get the numbers below.  Seems to average 47.7 kHz -> 23.85 kHz per channel, within noise of 48/24

    let gclk_for_adc = clocks.configure_gclk_divider_and_source(ClockGenId::GCLK3, 
        7, 
        ClockSource::DFLL48M, 
        true).unwrap();

    clocks.adc(&gclk_for_adc).expect("adc clock setup failed");
    wait_for_adc_sync(&adc);

    // reset should not be necessary but just in case
    adc.ctrla.modify(|_, w| w.swrst().set_bit());
    wait_for_adc_sync(&adc);

    adc.ctrlb.modify(|_, w| {
        w.prescaler().div4();
        w.ressel()._16bit();
        w.freerun().set_bit()
    });
    wait_for_adc_sync(&adc);

    adc.sampctrl.modify(|_, w| unsafe { w.samplen().bits(4) }); //sample length
    wait_for_adc_sync(&adc);

    adc.avgctrl.modify(|_, w| {
        w.samplenum()._4()
    });

    adc.refctrl.modify(|_, w| w.refsel().intvcc1() ); // 1/2 VDDANA
    wait_for_adc_sync(&adc);

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
    wait_for_adc_sync(&adc);


    adc.inputctrl.modify(|_, w|  {
        w.muxneg().gnd(); // No negative input (internal gnd)
        w.gain().div2(); // offsets 1/2 VDDANA
        w.muxpos().pin5(); // pin5 is A1
        unsafe { w.inputscan().bits(1) }  // 1 cycles through pin 5/6 / A1/A2, 0 is just A1
    });
    wait_for_adc_sync(&adc);

    // Make sure the interrupt is *not* globally set since we want the "turn on" to come from unmasking the interrupt
    NVIC::mask(interrupt::ADC);

    // set the "ready" interrupt
    adc.intenset.write(|w| w.resrdy().set_bit());
    wait_for_adc_sync(&adc);

    // power up
    adc.ctrla.modify(|_, w| w.enable().set_bit());
    wait_for_adc_sync(&adc);

    // trigger now to start freerunning going continuously
    adc.swtrig.modify(|_, w| w.start().set_bit());
    wait_for_adc_sync(&adc);

    adc
}

#[inline]
fn wait_for_adc_sync(adc: &bsp::pac::ADC) {
    while adc.status.read().syncbusy().bit_is_set() {};
}