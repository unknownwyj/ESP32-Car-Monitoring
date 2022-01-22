# ESP32-Car-Monitoring
## What is this ?
This is a custom all in one logging car battery, temperature and Battery voltage.

## Features
Monitor and shows data on OLED display. Forever Accurate Clock display with GPS data.

## Hardware
* ESP32
* ADS1115
* DS18b20
* I2C OLED display
* SDCARD holder and SDCARD
* GPS module neo-6m

## PIN used
I2C pin 22-SCL,21-SDA => ADS1115 and OLED display

SPI ( 18 -> CLK, 19 -> MISO, 23 -> MOSI, 5 -> CS ) => SDcard

Serial2 pin 16-RXD,17-TXD => GPS neo-6m

pin 33 => Button external pullup req

OneWire pin 4 => DS18B20 Temp sensor

