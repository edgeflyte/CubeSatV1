## Install RPI Pico Board Library
- Open Arduino IDE.
- Go to `File` > `Preferences`.
- In "Additional Board Manager URLs", add:
  ```https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json```
- Go to Tools > Board > Boards Manager.
- Search for "pico" or "RP2040".
- Click the core by Earle Philhower.
- Click the dropdown next to "Install" and select version 3.6.0.
- Click Install.

## Install Libraries
Open "Library Manager" inside of the Arduino IDE.
Search for and install the following libraries:
- "SD" by Arduino, Sparkfun
- "RF24" by TMRh20
- "FastIMU" by LiquidCGS
- "Adafruit SHT31 Library" by Adafruit
- "Adafruit GPS Library" by Adafruit
- "Adafruit SHT4x Library" by Adafruit
- "sensirion-sps" by Johannes Winkelmann
- "DFRobot_SCD4X" by DFRobot
- "Adafruit BMP085 Library" by Adafruit
- "SparkFun Indoor Air Quality Sensor - ENS160" by SparkFun

## Select Board and Settings
Go to Tools and select:
- Board: Raspberry Pi Pico/RP2040 > Raspberry Pi Pico
- Upload Method: Default (UF2)
- Port: Appears after first upload (CDC serial)
- CPU Speed: 125 MHz (default)
- Optimize: Small (-Os)
- Flash Size: 2MB
- Debug Level: None

## Connect via USB (BOOTSEL Mode)
- Hold BOOTSEL and plug in your Pico with a USB Micro cable.
- Release BOOTSEL — Pico shows as RPI-RP2 drive.
- Using the MCB.ino code, click Upload in Arduino IDE.


