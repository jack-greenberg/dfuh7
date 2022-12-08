#![no_std]
#![no_main]

use panic_probe as _;
use defmt_rtt as _;

use cortex_m_rt::entry;
use cortex_m::asm;

use stm32h7xx_hal::{
    prelude::*,
    stm32,
    interrupt,
};
use stm32h7xx_hal::xspi::{
    Qspi,
    QspiWord,
};
use stm32h7xx_hal::rcc::rec::UsbClkSel;
use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::usb_hs::{UsbBus, USB2};
use usb_device::{
    prelude::*,
    bus::UsbBusAllocator,
};
use usbd_dfu::*;

use core::cell::RefCell;
use cortex_m::interrupt::{free as interrupt_free, Mutex};

mod qspi;
use qspi::wait_for_write_completion;


static mut USB_BUS: Option<UsbBusAllocator<UsbBus<USB2>>> = None;
static mut USB_DEVICE: Mutex<RefCell<Option<UsbDevice<UsbBus<USB2>>>>> = Mutex::new(RefCell::new(None));
static mut USB_DFU: Mutex<RefCell<Option<DFUClass<UsbBus<USB2>, QspiFlash>>>> = Mutex::new(RefCell::new(None));

static mut EP_MEMORY_1: [u32; 1024] = [0; 1024];

pub struct QspiFlash {
    qspi: Qspi<stm32::QUADSPI>,
    buffer: [u8; 64],
}

impl QspiFlash {
    fn new(qspi: Qspi<stm32::QUADSPI>) -> Self {
        Self {
            qspi,
            buffer: [0u8; 64],
        }
    }
}

static mut is_programming: bool = false;

const KEY_START_APP: u32 = 0x20f2ab92;

impl DFUMemIO for QspiFlash {
    const INITIAL_ADDRESS_POINTER: u32 = 0x9000_0100;
    const PROGRAM_TIME_MS: u32 = 7; // TODO: time it takes to program 64 bytes
    const ERASE_TIME_MS: u32 = 50; // TODO
    const FULL_ERASE_TIME_MS: u32 = 50 * 112; // TODO

    const MEM_INFO_STRING: &'static str = "@QSPI/0x90000100/7*1Mg"; // TODO?
    const HAS_DOWNLOAD: bool = true;
    const HAS_UPLOAD: bool = true;
    const TRANSFER_SIZE: u16 = 64;

    const MANIFESTATION_TOLERANT: bool = false;

    // 32 bytes read/write max

    fn store_write_buffer(&mut self, src: &[u8]) -> Result<(), ()> {
        unsafe {
            is_programming = true;
        }

        assert!(src.len() <= Self::TRANSFER_SIZE.into());
        self.buffer.copy_from_slice(src);
        Ok(())
    }

    /*
     * Each QSPI transaction is at most 32 bytes,
     * but max read length is 64 bytes. We need to return a buffer of up to 64 bytes.
     */
    fn read(&mut self, address: u32, size: usize) -> Result<&[u8], DFUMemError> {
        let mut buf = [0u8; 32]; // 32 is the size of the HW FIFO

        let flash_top = Self::INITIAL_ADDRESS_POINTER + (8 * 1024 * 1024);

        if address >= flash_top {
            return Ok(&[]);
        }

        for i in 0..(size / 32) {
            self.qspi.read_extended(
                QspiWord::U8(qspi::W25Q64_Instruction::Read as u8),
                QspiWord::U24(address + (i*32) as u32),
                QspiWord::None,
                0,
                &mut buf,
            ).unwrap();

            self.buffer[i*32..(i*32 + buf.len())].copy_from_slice(&buf);
        }

        Ok(&self.buffer)
    }

    fn program(&mut self, address: u32, size: usize) -> Result<(), DFUMemError> {
        // Checks
        if address < 0x9000_0000 {
            return Err(DFUMemError::Address);
        }

        if address >= 0x9000_0000 + 0x800000 {
            return Err(DFUMemError::Address);
        }

        self.qspi.write_extended(
            QspiWord::U8(qspi::W25Q64_Instruction::WriteEnable as u8),
            QspiWord::None,
            QspiWord::None,
            &[],
        ).unwrap();

        for i in 0..(self.buffer.len() / 32) {
            self.qspi.write_extended(
                QspiWord::U8(qspi::W25Q64_Instruction::WriteEnable as u8),
                QspiWord::None,
                QspiWord::None,
                &[],
            ).unwrap();

            wait_for_write_completion(&mut self.qspi);

            self.qspi.write_extended(
                QspiWord::U8(qspi::W25Q64_Instruction::Write as u8),
                QspiWord::U24(address + (i*32) as u32),
                QspiWord::None,
                &self.buffer[i*32..(i+1)*32],
            ).unwrap();

            wait_for_write_completion(&mut self.qspi);
        }

        Ok(())
    }

    fn erase(&mut self, address: u32) -> Result<(), DFUMemError> {
        self.qspi.write_extended(
            QspiWord::U8(qspi::W25Q64_Instruction::WriteEnable as u8),
            QspiWord::None,
            QspiWord::None,
            &[],
        ).unwrap();

        self.qspi.write_extended(
            QspiWord::U8(qspi::W25Q64_Instruction::SectorErase as u8),
            QspiWord::U24(address),
            QspiWord::None,
            &[],
        ).unwrap();
        wait_for_write_completion(&mut self.qspi);

        Ok(())
    }

    fn erase_all(&mut self) -> Result<(), DFUMemError> {
        Err(DFUMemError::Unknown)
    }

    fn manifestation(&mut self) -> Result<(), DFUManifestationError> {
        defmt::info!("Manifestation");
        Ok(())
    }

    fn usb_reset(&mut self) {
        defmt::debug!("USB reset command issued");
        unsafe {
            if is_programming {
                // Write KEY_START_APP
                is_programming = false;
                defmt::info!("Was programming, now RESET");
                start_app();
            }
        }
    }
}

#[inline(never)]
fn start_app() {
    defmt::info!("Jumping to application");
    let app_addr = 0x9000_0100 as *const u32;

    unsafe {
        cortex_m::asm::bootload(app_addr);
    }
}

fn get_boot_key(qspi: &mut Qspi<stm32::QUADSPI>) -> u32 {
    let p: u32 = 0x9000_0000;

    let mut buf = [0u8; 4];

    qspi.read_extended(
        QspiWord::U8(qspi::W25Q64_Instruction::Read as u8),
        QspiWord::U24(p),
        QspiWord::None,
        0,
        &mut buf,
    ).unwrap();

    return u32::from_le_bytes(buf);
}

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();

    let pwrcfg = dp.PWR.constrain().freeze();
    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc
        .sys_ck(80.MHz())
        .pclk1(48.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must run");
    ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::HSI48);

    defmt::trace!("Clocks initialized");
    
    // Initialize USB
    let (pin_dm, pin_dp) = {
        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        (gpioa.pa11.into_alternate::<10>(), gpioa.pa12.into_alternate::<10>())
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

    unsafe {
        USB_BUS = Some(UsbBus::new(usb, &mut EP_MEMORY_1));
    }

    defmt::trace!("USB initialized");

    // Initialize QSPI
    let (
        io0,
        io1,
        io2,
        io3,
        clk,
        _cs
    ) = {
        let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
        let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);

        (
            gpiod.pd11.into_alternate().speed(Speed::VeryHigh),
            gpiod.pd12.into_alternate().speed(Speed::VeryHigh),
            gpioe.pe2.into_alternate().speed(Speed::VeryHigh),
            gpiod.pd13.into_alternate().speed(Speed::VeryHigh),
            gpiob.pb2.into_alternate().speed(Speed::VeryHigh),
            gpiob.pb6.into_alternate::<10>().speed(Speed::VeryHigh)
        )
    };

    let mut qspi = dp.QUADSPI.bank1(
        (clk, io0, io1, io2, io3),
        3.MHz(),
        &ccdr.clocks,
        ccdr.peripheral.QSPI,
    );


    let key = get_boot_key(&mut qspi);

    if key == KEY_START_APP {
        defmt::info!("Start app key found, jumping to application");
        start_app();
    } else {
        defmt::info!("Start app key not found, staying in updater");
    }

    let qspi_flash = QspiFlash::new(qspi);

    let mut dfu = unsafe { DFUClass::new(USB_BUS.as_ref().unwrap(), qspi_flash) };

    let usb_dev = unsafe { UsbDeviceBuilder::new(&USB_BUS.as_ref().unwrap(), UsbVidPid(0xf055, 0xdf11))
        .manufacturer("Jack Greenberg")
        .product("DFUH7")
        .serial_number("0xFEEDBEEF")
        .device_release(0x0200)
        .self_powered(false)
        .max_power(250)
        .max_packet_size_0(64)
        .build()
    };
    
    unsafe {
        interrupt_free(|cs| {
            USB_DFU.borrow(cs).replace(Some(dfu));
            USB_DEVICE.borrow(cs).replace(Some(usb_dev));
        });
    }

    defmt::trace!("QSPI initialized");

    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::OTG_HS);
    }

    unsafe { cortex_m::interrupt::enable() };
    
    loop {
        unsafe {
            interrupt_free(|cs| {
                if let (Some(device), Some(dfu)) = (
                    USB_DEVICE.borrow(cs).borrow_mut().as_mut(),
                    USB_DFU.borrow(cs).borrow_mut().as_mut(),
                ) {
                    device.poll(&mut [dfu]);
                }
            })
        }
    }
}

#[interrupt]
unsafe fn OTG_HS() {
    defmt::trace!("USB INT!");
    interrupt_free(|cs| {
        if let (Some(device), Some(dfu)) = (
            USB_DEVICE.borrow(cs).borrow_mut().as_mut(),
            USB_DFU.borrow(cs).borrow_mut().as_mut(),
        ) {
            device.poll(&mut [dfu]);
        }
    });
}
