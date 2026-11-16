[![EdgeFlyte](https://edgeflyte.com/edgeflyte_logo.png)](https://edgeflyte.com/)

# EdgeFlyte CubeSat v1 — Firmware Suite

**Suite version:** 2.1.0 ("Orbit") · **License:** EdgeFlyte Non-Commercial / Educational 1.0

[![1U EDU Classic](https://edgeflyte.com/uploads/1792368887980-Vd6hrxhFj9k.png)](https://edgeflyte.com/products/1001)

Firmware for the [1U EDU Classic](https://edgeflyte.com/products/1001), the EdgeFlyte CubeSat v1 kit. This repository is the firmware for every module in the kit, released together as version 2.1.0.

## Modules

| Module | What it does | Board |
|--------|----------------|-------|
| **MCB** | Flight computer. Reads onboard sensors, talks to the other boards, sends radio telemetry, logs to the SD card | Raspberry Pi Pico |
| **BMS** | Battery board. Measures the pack and solar inputs, switches four power outputs | ATmega328PB (MiniCore) |
| **ASB** | Atmospheric board. Temperature, humidity, pressure, air quality, CO2, and particulate matter | ATmega328PB (MiniCore) |
| **GS 8601** | Ground station. Bridges the radio to a computer over USB, and can send one radio message back | Adafruit Feather M0 |

## What 2.1.0 adds

- The ground station checks incoming radio frames and can send a frame when you type `tx` followed by 64 hex characters.
- Short text messages can pass between satellites and ground stations.
- You can change the flight computer's radio channel, transmit power, and telemetry interval.
- Reboot, restart, and emergency power-off wait for a confirmation.
- The flight computer reads the atmospheric board's current sentences.

## Start here

- [Program a module](docs/quickstart/01-programming-toolchains.md)
- [Connect the modules](docs/quickstart/02-interfacing-modules.md)
- [Read the radio link](docs/quickstart/03-telemetry-radio-protocol.md)
- [Read SATBUS sentences](docs/quickstart/04-satbus-protocol.md)
- [Change settings on the flight computer](docs/quickstart/05-parameter-protocol.md)
- [Identify a module](docs/quickstart/06-device-identification.md)

Byte-level layouts are in [docs/protocols/](docs/protocols/).

## Also in this folder

- [Troubleshooting](docs/troubleshooting.md)
- [Model numbers and serials](docs/device-identity-registry.md)
- [How versions work](VERSIONING.md)
- [License](LICENSE)

## Working with older firmware

2.1.0 still understands telemetry and commands from 2.0.x and 1.x radios.
Older software ignores the new short-message type. Reboot, restart, and
emergency power-off now require confirmation. See [VERSIONING.md](VERSIONING.md).

## License

Source-available under the **EdgeFlyte Non-Commercial / Educational License 1.0**
— free for personal, educational, and research use. Commercial use requires a
separate agreement. See [LICENSE](LICENSE).

[1U EDU Classic](https://edgeflyte.com/products/1001) · [Buy the kit](https://store.edgeflyte.com/products/cubesat-kit) · [Support](https://edgeflyte.com/support)
