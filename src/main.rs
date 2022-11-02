#![no_std]
#![no_main]

use panic_probe as _;
use defmt_rtt as _;

use cortex_m_rt::entry;

use stm32h7xx_hal::{
    prelude::*,
    stm32,
};
use stm32h7xx_hal::usb_hs::{UsbBus, USB2};
use stm32h7xx_hal::rcc::rec::UsbClkSel;
use usb_device::{
    prelude::*,
    UsbError,
};

use usbd_serial;

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let pwrcfg = dp.PWR.constrain().freeze();
    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc
        .sys_ck(80.MHz())
        .pclk1(48.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must run");
    ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::HSI48);
    
    let (pin_dm, pin_dp) = {
        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        (gpioa.pa11.into_alternate(), gpioa.pa12.into_alternate())
    };
    
    let usb = USB2::new(
        dp.OTG2_HS_GLOBAL,
        dp.OTG2_HS_DEVICE,
        dp.OTG2_HS_PWRCLK,
        pin_dm,
        pin_dp,
        ccdr.peripheral.USB2OTG,
        &ccdr.clocks,
    );
    
    let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });

    defmt::debug!("USB Bus set up");

    let mut serial = usbd_serial::SerialPort::new(&usb_bus);
    
    let mut usb_dev =
        UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Olin College of Engineering")
            .product("STM32H7 Dev Board")
            .serial_number("FEEDBEEF")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

    // let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    // let mut led = gpioe.pe3.into_push_pull_output();

    let mut cmd_buf = [0u8; 64];
    let mut cmd_buf_idx = 0;

    let mut cli = Runner::new(&ROOT_MENU, &mut cmd_buf, serial);

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut read_buf = [0u8; 64];
        
        match serial.read(&mut read_buf) {
            Ok(1) => {
                serial.write(&read_buf).unwrap();
                match read_buf[0] {
                    b'\n' => {
                        let cmd: &str = core::str::from_utf8(&cmd_buf).unwrap();
                        process(cmd, &mut serial).unwrap();
                        cmd_buf_idx = 0;
                    },
                    _ => {
                        cmd_buf[cmd_buf_idx] = read_buf[0];
                        cmd_buf_idx += 1;
                    },
                }
            },
            Ok(count) if count > 1 => {
                if count != 1 {
                    serial.write(b"Count isn't one").unwrap();
                }
                serial.write(&read_buf).unwrap();
                // let mut write_offset = 0;
                //
                // while write_offset < count {
                //     match serial.write(&read_buf[write_offset..count]) {
                //         Ok(len) if len > 0 => {
                //             write_offset += len;
                //         }
                //         _ => {}
                //     }
                // }
            }
            _ => {}
        }
    }
}

fn process<B: usb_device::bus::UsbBus>(cmd: &str, serial: &mut usbd_serial::SerialPort<B>) -> Result<(), ()> {
    defmt::dbg!("Received command: {}", cmd);
    
    match cmd {
        "help" => {
            serial.write(b"Help message\n").unwrap();
        },
        _ => {
            serial.write(b"Unknown command: ").unwrap();
            serial.write(cmd.as_bytes()).unwrap();
        },
    }
    Ok(())
}
