# Flight computer (MCB) · v2.1.0

The flight computer of the EdgeFlyte CubeSat v1. It reads the onboard
sensors, takes data from the expansion boards, sends radio telemetry,
handles commands, logs to the SD card, and relays messages for the mesh.

- **Board:** Raspberry Pi Pico
- **Sketch:** `MCB/MCB.ino`
- **Model:** `1001` · **Serial on a new board:** `0043` (you can store another with settings code `0100`)
- **Firmware:** `2.1.0` (identify field `20100`)

## Hardware

- **Onboard sensors:** temperature and humidity (SHT41 or SHT31), 9-axis IMU, GPS (NMEA on the GPS port).
- **Radio:** nRF24L01+, 2.4 GHz, channel 62, 1 Mbps, low power, 32-byte payload.
- **Expansion:** atmospheric board, battery board, and an optional charge controller on SATBUS.

### Pins (Raspberry Pi Pico)

| Function | GPIO | Function | GPIO |
|----------|------|----------|------|
| I2C data | 12 | I2C clock | 13 |
| SPI MOSI | 19 | SPI MISO | 16 |
| SPI clock | 18 | SD card select | 17 |
| Radio CE | 11 | Radio CSN | 15 |
| Radio IRQ | 14 | Buzzer | 3 |
| GPS TX/RX | 0 / 1 | SATBUS TX/RX | 4 / 5 |
| LED 1 / 2 / 3 | 6 / 7 / 8 | Built-in LED | 25 |
| Remove-before-flight | 2 | BLE enable | 20 |
| Analog 0 / 1 / 2 | 26 / 27 / 28 | Unused | 9, 10, 21, 22 |

GPS is 9600 baud. SATBUS is 9600 baud, 8 data bits, no parity, 1 stop bit.
USB is 115200 baud.

## Program the board

1. Install the Raspberry Pi Pico boards package by Earle Philhower.
2. Install libraries: `RF24`, `Adafruit SHT4x`, `Adafruit SHT31`, `FastIMU`, `MicroNMEA`.
3. Open `MCB/MCB.ino`, select **Raspberry Pi Pico**, and upload.
   If Arduino asks to move the sketch, choose **Cancel**.

See [Program a module](../docs/quickstart/01-programming-toolchains.md).

## Talking to it

- **Identify:** send `#?EF?#` over USB. See [Identify a module](../docs/quickstart/06-device-identification.md).
- **Settings:** `#G` reads and writes serial, radio channel, and intervals. Saved values return on the next boot. See [Settings](../docs/quickstart/05-parameter-protocol.md).
- **Text commands:** `version`, `uptime`, `params`, `config reset`, `log`, `telem`, `mode`, `power chN on` or `off`, `reboot`.
- **Radio and SATBUS:** [Radio link](../docs/protocols/telemetry-radio.md) and [SATBUS](../docs/protocols/satbus-v1.md).

### Commands in 2.1.0

- System commands start with `80`, power with `81`, radio with `82`, sensors with `83`, and debug with `8F`. A code with no action answers "Unknown command".
- You can set the channel (`8200`), power (`8201`), and fast telemetry interval (`8204`), save those three (`8205`), sound the buzzer (`800B`), and send a short message (`0900`).
- Reboot, restart, and emergency power-off need the confirmation byte `A5` on the radio, or the word `confirm` on USB.
- There is no password. Any command on the USB port or the radio runs. Keep those links limited to people you trust.

## Upgrading from 2.0.2

Existing telemetry bytes stay the same. Short messages and the radio
commands above are new. Two earlier corrections are still in effect:

- Altitude from the atmospheric board is about 100 m lower than on 1.x firmware. That lower number is the corrected altitude.
- Messages from other radios are decoded with the proper byte order, so sender, destination, and message type match what was transmitted.

See [CHANGELOG.md](CHANGELOG.md) and [Troubleshooting](../docs/troubleshooting.md).

## License

EdgeFlyte Non-Commercial / Educational License 1.0 — see [LICENSE](../LICENSE).
