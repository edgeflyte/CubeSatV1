# Troubleshooting

Suite **2.1.0**. Programming steps are in
[Program a module](quickstart/01-programming-toolchains.md).

## Programming and upload

| What you see | What it usually means | What to do |
|--------------|----------------------|------------|
| The flight computer sketch will not compile and mentions RP2040 | The selected board is the Mbed core, or it is not a Pico | Tools → Board → **Raspberry Pi Pico** under Raspberry Pi RP2040 Boards (Earle Philhower) |
| The IDE renamed the sketch folder and the extra source files disappeared | It nested a second `MCB` folder | Choose Cancel on the rename dialog. `MCB.ino` sits next to the other MCB source files |
| The Pico never shows a port | It is not in bootloader mode, or the cable has no data wires | Hold BOOTSEL while plugging in. A drive named `RPI-RP2` appears. Copy the `.uf2` onto it |
| Battery or atmospheric upload fails with no sync | The bootloader, clock, or chip variant does not match | Board **ATmega328**, variant **328PB**, clock **Internal 8 MHz**. Burn the MiniCore bootloader for that clock, or program over ISP |
| Compiling the atmospheric or battery sketch on Windows crashes inside `avr/power.h`, or mentions writing to `nul` | The Arduino AVR compiler that ships with the IDE is 32-bit and crashes on this sketch | Run `powershell -File tools/install-avr-gcc-x64.ps1`, then compile again. Restart the IDE if the log still shows compiler `7.3.0-atmel3.6.1-arduino7` |
| Battery voltages are scaled wrong | The board is not running at 3.3 V | Power the battery board at 3.3 V. The firmware assumes a 3.3 V reference |
| The ground station port disappears | The Feather M0 is not in the bootloader | Double-tap reset. A port appears for a few seconds. Board: Adafruit Feather M0 |
| The banner is not `2.1.0` | An older firmware image is running | Send `#?EF?#`. The third field is `20100` for this suite and `20002` for 2.0.2. Program the board again from this folder |

## Radio

| What you see | What it usually means | What to do |
|--------------|----------------------|------------|
| Ground station prints `Radio initialization failed` and the red LED blinks | The radio did not start after five tries | Check CE D10, CSN D9, MOSI D11, MISO D12, SCK D13, and 3.3 V. The sketch stays in that loop until the wiring is fixed |
| `[RADIO] Chip disconnected` | The radio stopped answering | `radio` prints whether the chip is connected. Wait 2 seconds, or send `radio reset` |
| The same `[HEX]` line three times, then `[RADIO] RX recovered: stuck packet` | The radio returned one packet over and over | It clears on its own. Three of these within 5 seconds restart the radio |
| `[HEX]` lines arrive but `stats` calls them invalid | The frame does not start with `#1` or `#R`, or the sender id is 0 | `debug` prints the reason. The first two bytes are `23 31` or `23 52` for a 2.1.0 frame. Older frames that start with `*` or `%` are ignored |
| Frames from the wrong satellite are dropped | A sender filter is set | `stats` shows the invalid count rising. `device 0` accepts every sender |
| No frames, and both radios look powered | Channel or address does not match | Both ends use channel 62 unless you changed it. The address bytes are `45 46 45 46 30` |
| `tx` returns `TX ERR` | The line was not 64 hex characters, or a directed message got no reply | Four zero bytes in the destination field send a broadcast, which does not wait for a reply. Any other destination needs the flight computer in range on the same channel |
| `TX OK`, and the flight computer does nothing | `TX OK` only means the radio accepted the send | Watch for a later `[HEX]` line of type `0008`. Reboot, restart, and emergency power-off need `A5` as the first data byte |
| The two radios stop hearing each other after a channel change | Only one side moved | The flight computer prints `[RADIO] channel`. The ground station prints `[RADIO] Channel set to`. Set both to the same channel from 0 to 125 |

## SATBUS

| What you see | What it usually means | What to do |
|--------------|----------------------|------------|
| No `$ASB` or `$BMS` lines on the flight computer | Wiring, baud, or ground | Atmospheric and battery transmit pins go to flight-computer GPIO 5. GPIO 4 goes to the module receive pins. Common ground, 9600 baud, 8 data bits, no parity, 1 stop bit, 3.3 V |
| Garbled sentences | Two boards are driving the wire, or the baud rate differs | Do not open a serial monitor on a module's data port while it is also wired to the flight computer |
| Atmospheric numbers look like raw counts | The scale has not been applied | Temperature is the wire value divided by 100, then minus 40. Altitude is the wire value minus 100. Pressure is the wire value times 10. See [SATBUS](quickstart/04-satbus-protocol.md) |
| A `$ASB` sentence is ignored | The packet id is not one the flight computer knows | Ids `0` and `1` are the kit sentences. Ids `3` and `4` are accepted as the same two sentences. Any other id is dropped |
| A battery channel command does nothing | The sentence does not match | The flight computer sends `&BMS,CH1,1` style text. The channel is `CH1` through `CH4` and the value is `0` or `1`. `&BMS,CFG,` takes a number from 0 to 15 |
| Solar readings sit near 3.3 V | The panel is open and the converter tops out at the board voltage | That ceiling is the measurement limit, not a higher panel voltage |

## Sensors

| What you see | What it usually means | What to do |
|--------------|----------------------|------------|
| Atmospheric board prints `humidity sensor: NONE` | Neither humidity sensor answered | At boot the scan lists `DTC 0x44` or `0x45` when a sensor is present. Check the I2C pull-ups |
| Temperature 1 stays still while temperature 2 moves | The humidity sensor failed and the pressure sensor is still running | Temperature 1 comes from the humidity sensor. Temperature 2 comes from the BMP180 |
| Particulate fields stay at zero | The particulate sensor is still warming up, or it was not found | It starts sending after it reports data ready. Negative readings are reported as 0 |
| Air-quality or CO2 fields stay at 0 | That sensor did not start, or it has no new sample | The ENS160 is at I2C address `0x53`. Readings are skipped until the sensor is present. The CO2 sensor waits until it has a sample |
| Flight computer IMU or humidity sensor missing | Startup failed | Identify status is `ERR` when both the humidity sensor and the IMU are down. Status flag bits are listed in the [radio guide](quickstart/03-telemetry-radio-protocol.md) |

## SD card on the flight computer

| What you see | What it usually means | What to do |
|--------------|----------------------|------------|
| No log file | The card is not mounted, or logging is off | Chip select is GPIO 17. Files are named `log_0001.csv`, using the boot count |
| Logging stops during a run | Ten writes in a row failed | Reseat the card and reboot. After that streak the logger closes the file |
| A log row is split across columns | An event string contained a comma or a newline | Keep event text free of commas |

## Identification and settings

| What you see | What it usually means | What to do |
|--------------|----------------------|------------|
| `#?EF?#` gets no answer | Wrong baud, or the letters were lowercased | 115200 on the flight computer and ground station, 9600 on the battery and atmospheric boards. The probe is case-sensitive |
| Ground station identify serial is `0016`, and `serial` prints `8601-0042` | Those are two different labels | Radio and identify use `0016`. The `serial` command prints the unit label |
| `#G` returns `E7` | The version byte is not `01` | The frame starts with `#G01`. See [Settings](quickstart/05-parameter-protocol.md) |
| A `#G` channel change does not retune the radio | The setting is saved for the next boot | Reboot, or send radio command `0x8200` to retune now |
| A multi-byte `#G` value looks reversed | The bytes were sent least-significant first | Multi-byte values are big-endian. Serial `00000043` is sent as `00000043` |
| `reboot` on USB does nothing | The confirmation word is missing | The reply says `Requires: reboot confirm`. Send `reboot confirm`. On the radio the confirmation byte is `A5` |
