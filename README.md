# K21_mkrzero
## OLD implementation of K21 device

This version of code uses:
* Arduino IDE
* MPR121 touch sensor for input
* SD card to write LOG and read audio files by using SPI
* Bluetooth to transffering letter (HID mode)
* I2S to play audio over separate audio amplifier

This approach was **discontinued** because it served it's purpose - concept was working - our own PCB was needded with SAMD21 device
Also PC side driver/app is needed as simple HID can't commnicate anytheng else but Latin symbols and numbers.

TO DO:
* comment more
* clean up code
* fix bugs
