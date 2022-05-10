# mat-clog
RPi Pico data logger (C-language version)

Code for a data logger using a Raspberry Pi Pico board.
The logger consists of:
1. Pico
1. SSD1306 OLED display
1. BME280 temperature and humidity sensor
1. VL53L1X laser distance measurement sensor
1. VEML6030 ambient light sensor
1. SD card reader

The SD card FAT filesystem read/write routines are from the no-OS-FatFS-SD-SPI-RPi-Pico repo.
