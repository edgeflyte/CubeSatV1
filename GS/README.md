# Uploading Bootloader to ATmega328P

## Required Hardware
- Arduino UNO (or similar) as ISP programmer
- Target board with ATmega328P
- 6 jumper wires

## Setup Arduino as ISP
1. Connect Arduino to computer via USB
2. Open Arduino IDE
3. Select `Arduino UNO` from Tools > Board menu
4. Open File > Examples > ArduinoISP > ArduinoISP
5. Upload sketch to Arduino

## Wiring Connections
Connect Arduino to target board's ATmega328P as follows:

| Arduino Pin | Target Board Pin | Function |
|------------|------------------|----------|
| 10 | RESET | RESET |
| 11 | MOSI | MOSI |
| 12 | MISO | MISO |
| 13 | SCK | SCK |
| 5V | VCC | Power |
| GND | GND | Ground |

## Uploading Bootloader
1. Open Arduino IDE
2. Install MiniCore board package:
   - Open File > Preferences
   - Add `https://mcudude.github.io/MiniCore/package_MCUdude_MiniCore_index.json` to Additional Boards Manager URLs
   - Open Tools > Board > Boards Manager
   - Search for "MiniCore" and install
3. Select board settings:
   - Select Programmer's Port from Tools > Port menu (e.g. COM3 (Arduino UNO)
   - Select `ATmega328` from Tools > Board > MiniCore
   - Set clock to `8 MHz internal`
   - Set BOD to `2.7V`
   - Set Variant to `328PB`
   - Set Bootloader to `Yes (UART 0)`
4. Select `Arduino as ISP` from Tools > Programmer
5. Click Tools > Burn Bootloader

## Uploading Code
After bootloader is installed:

1. Connect the Ground Station to the USB port of the Computer
2. Select board settings:
   - Select Programmer's Port from Tools > Port menu (e.g. COM4)
   - Select `ATmega328` from Tools > Board > MiniCore
   - Set clock to `8 MHz internal`
   - Set BOD to `2.7V`
   - Set Variant to `328PB`
   - Set Bootloader to `Yes (UART 0)`
3. Select the corresponding Port from Tools > Port menu
4. Upload the Ground Station code normally

## Troubleshooting
- Verify all connections are secure
- Ensure proper power supply voltage (3.3V)
- Check if connections are not shorted

## Notes
- The bootloader enables serial programming without needing an ISP programmer for future uploads
- Once bootloader is installed, the chip can be programmed like a regular Arduino
- The process only needs to be done once per board

