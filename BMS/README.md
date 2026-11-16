# Battery board (BMS) · v2.1.0

Measures the battery pack and two solar inputs, switches four power
outputs, and reports to the flight computer over SATBUS.

- **Board:** ATmega328PB, MiniCore
- **Sketch:** `BMS.ino`
- **Model:** `2001` · **Serial:** `0001`
- **Firmware:** `2.1.0` (identify field `20100`)

## What it does

- Pack voltage on A0, through a divider that doubles the pin reading, against 3.3 V
- Solar inputs on A1 and A2
- Four power outputs on D5, D6, D7, and D8
- LEDs on D2, D3, and D4 (activity, command, low battery)
- Sends a `$BMS` sentence about every 250 ms
- Accepts `&BMS,CH1,1` through `&BMS,CH4,0`, `&BMS,RST,1`, and `&BMS,CFG,` with a number from 0 to 15

## Pins

| Function | Pin |
|----------|-----|
| Pack sense | A0 |
| Solar 1 / solar 2 | A1 / A2 |
| LEDs | D2 / D3 / D4 |
| Power outputs 1–4 | D5 / D6 / D7 / D8 |
| SATBUS | Hardware serial at 9600. The transmit pin is released between sentences |

## Program the board

1. Install MiniCore and select **ATmega328PB**, internal 8 MHz.
2. Burn the bootloader if you upload over serial. See [Program a module](../docs/quickstart/01-programming-toolchains.md).
3. Open `BMS.ino` and upload.

## Sentences

- **Telemetry:** `$BMS,<err>,<output1..4>,<pack volts>,<percent>,<solar1>,<solar2>*` — [SATBUS](../docs/protocols/satbus-v1.md)
- **Commands:** `&BMS,CH1,1` through `&BMS,CH4,0`, and `&BMS,RST,1`
- **Identify:** `#?EF?#` answers `#!EF!:2001,...`

## License

EdgeFlyte Non-Commercial / Educational License 1.0 — see [LICENSE](../LICENSE).
