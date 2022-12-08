// From datasheet
pub enum W25Q64_Instruction {
    WriteEnable = 0x06,
    // WriteDisable = 0x04,
    Read = 0x03,
    Write = 0x02,
    ReadStatusRegister1 = 0x05,
    SectorErase = 0x20,
    // BlockErase32K = 0x52,
    // BlockErase64K = 0xD8,
}

use stm32h7xx_hal::pac;
use stm32h7xx_hal::xspi::{
    Qspi,
    QspiWord,
};

pub fn wait_for_write_completion(qspi: &mut Qspi<pac::QUADSPI>) {
    loop {
        let mut status: [u8; 1] = [0xFF; 1];
        qspi.read_extended(
            QspiWord::U8(W25Q64_Instruction::ReadStatusRegister1 as u8),
            QspiWord::None,
            QspiWord::None,
            0,
            &mut status,
        )
        .unwrap();

        if status[0] & 0x01 == 0 {
            break;
        }
    }
}
