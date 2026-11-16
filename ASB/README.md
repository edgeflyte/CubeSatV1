# Atmospheric board (ASB) · v2.1.0

Temperature, humidity, pressure, altitude, air quality, CO2, and
particulate matter, reported to the flight computer over SATBUS.

- **Board:** ATmega328PB, MiniCore
- **Sketch:** `ASB.ino` (uses an SHT4x humidity sensor when it answers, otherwise an SHT31)
- **Model:** `3001` · **Serial:** `0001`
- **Firmware:** `2.1.0` (identify field `20100`)

## Sensors

| Sensor | What it contributes |
|--------|---------------------|
| SHT4x or SHT31 | Temperature 1 and humidity. Chosen at startup |
| BMP180 | Temperature 2, pressure, altitude |
| ENS160 | Air-quality index, TVOC, equivalent CO2 |
| SCD4x | CO2 |
| SPS30 | Particulate matter, on a second sentence |

## Pins

| Function | Pin |
|----------|-----|
| Status LED | D2 |
| I2C | The hardware I2C pins |
| SATBUS | Hardware serial at 9600. The transmit pin is released between sentences |

## Program the board

1. Install MiniCore and select **ATmega328PB**, internal 8 MHz.
2. Libraries: `Adafruit SHT31`, `Adafruit SHT4x`, `SparkFun ENS160`, `Adafruit BMP085`, `sps30`, `DFRobot_SCD4X`.
3. Open `ASB.ino` and upload.

See [Program a module](../docs/quickstart/01-programming-toolchains.md).

## Sentences

- `$ASB,0,...*` is the environment sentence. `$ASB,1,...*` is particulate matter.
- Altitude on the wire is meters plus 100. The flight computer subtracts 100.
- Identify: `#?EF?#`.
- Layout: [Packet specification](ASB_PACKET_SPECIFICATION.md) and [SATBUS](../docs/protocols/satbus-v1.md).
- Adding a sensor: [Custom sensor integration](Custom-Sensor-Integration.md). A separate template is in [examples/ASB_Framework](../examples/ASB_Framework/). Match the kit packet specification before that template flies with the stock flight computer.

## License

EdgeFlyte Non-Commercial / Educational License 1.0 — see [LICENSE](../LICENSE).
