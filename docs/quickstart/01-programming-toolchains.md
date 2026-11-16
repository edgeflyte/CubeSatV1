# Program a module

How to build and load each board in suite **2.1.0**. The version is stored
in that module's `version.h`. The folder name and the sketch name match:
`MCB/MCB.ino`, `BMS/BMS.ino`, `ASB/ASB.ino`, `GS/GS.ino`. Leave the version
number out of the folder name.

## What you need

- Arduino IDE 2.x, or [Arduino CLI](https://arduino.github.io/arduino-cli/).
- A USB cable with data wires. A charge-only cable will not show a port.
- The board package for the module you are programming.

Add these addresses under File → Preferences → Additional boards manager
URLs, or with `arduino-cli config add board_manager.additional_urls`:

| Module | Boards URL |
|--------|------------|
| Flight computer | `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json` |
| Battery and atmospheric boards | `https://mcudude.github.io/MiniCore/package_MCUdude_MiniCore_index.json` |
| Ground station | `https://adafruit.github.io/arduino-board-index/package_adafruit_index.json` |

## Flight computer (Raspberry Pi Pico)

1. Install **Raspberry Pi Pico/RP2040** by Earle Philhower. Leave the
   Arduino Mbed OS RP2040 package unselected. This sketch compiles only
   for the Pico core.
2. Board: **Raspberry Pi Pico**.
3. Install libraries: `RF24`, `Adafruit SHT4x`, `Adafruit SHT31`, `FastIMU`,
   `MicroNMEA`. `SPI`, `Wire`, `EEPROM`, and `SD` come with the core.
4. Open `MCB/MCB.ino`. If Arduino offers to move the sketch so the name
   matches the folder, choose **Cancel**. Accepting that dialog puts the
   sketch in a nested folder and the rest of the flight-computer files
   stop compiling.
5. Upload. If no port appears, hold **BOOTSEL** while plugging in the Pico,
   then copy the `.uf2` or upload from the IDE.

From the command line:

```text
arduino-cli compile -b rp2040:rp2040:rpipico --output-dir build/mcb MCB
```

## Battery and atmospheric boards (ATmega328PB)

1. Install **MiniCore**.
2. Board: **ATmega328**. Variant: **328PB**. Clock: **Internal 8 MHz**.
3. The battery firmware measures against a **3.3 V** supply. The 8 MHz
   internal clock matches that board. The example atmospheric template
   uses the same clock.
4. Burn the MiniCore bootloader once per chip before a serial upload.
   ISP programming does not need that bootloader. Use MiniCore's
   burn-bootloader action for the clock you selected.
5. The battery sketch has no extra libraries. The atmospheric sketch needs
   `Adafruit SHT31`, `Adafruit SHT4x`, `SparkFun ENS160`, `Adafruit BMP085`,
   `sps30`, and `DFRobot_SCD4X`.
6. Open `BMS/BMS.ino` or `ASB/ASB.ino` and upload.

From the command line:

```text
arduino-cli compile -b MiniCore:avr:328:clock=8MHz_internal,variant=modelPB --output-dir build/bms BMS
arduino-cli compile -b MiniCore:avr:328:clock=8MHz_internal,variant=modelPB --output-dir build/asb ASB
```

## Ground station (Feather M0)

1. Install **Adafruit SAMD Boards**.
2. Board: **Adafruit Feather M0**.
3. Library: `RF24`.
4. Open `GS/GS.ino` and upload. If the port disappears, double-tap reset
   to show the bootloader.

From the command line:

```text
arduino-cli compile -b adafruit:samd:adafruit_feather_m0 --output-dir build/gs GS
```

## Confirm the program

| Module | Baud | What you should see |
|--------|------|---------------------|
| Flight computer | 115200 | Boot log, then `version` prints `2.1.0` |
| Battery board | 9600 | `#BMS, Battery Management System v2.1.0 Initializing...` |
| Atmospheric board | 9600 | `#ASB, Atmospheric Sensing Board v2.1.0 Initializing...` |
| Ground station | 115200 | `8601 GROUND STATION v2.1.0` |

On every module, send `#?EF?#`. The third field of the reply is `20100`.
Field order is in [Identify a module](06-device-identification.md).

Upload and radio problems are in [Troubleshooting](../troubleshooting.md).
