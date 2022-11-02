# Diagnostics Shell

A diagnostics shell will allow for the user to exercise the full capabilities of
the WeAct STM32H7 development board. The shell will be able to set/clear/read
any GPIO pin or ADC if the pin is configured like that, read/write (Q)SPI flash,
interact with the SD card.

A stretch goal would be to also have the text output to the terminal (involving
writing a mini graphics driver).

A further stretch goal would be having the code boot from QSPI instead of normal
flash, and have a bootloader in the normal flash.
