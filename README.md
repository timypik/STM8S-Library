STM8S Library
============

Library contains:
* board - clock and WDT system init, IRQ priority control, defines for fast pin configuration and etc
* badc - ADC2 control system
* can - CAN control system
* eeprom - EEPROM file system with CRC16 check
* rs485 - init and control RS485 interface (use UART3 and TIM3)
* sys_timer - software 1 ms timer, use as system timer (use TIM4)
* ucon - universal uart interface (use UART1 and sys_timer)
* uid - uniq ID interface
* uowi - 1-wire interface through UART (usr UART1)
* irq - IRQ system
* libstm8 - Standard Peripheral Library from STMicroelectronics (changed)
