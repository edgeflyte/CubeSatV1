# Versions

The current kit firmware is **2.1.0** ("Orbit").

Four boards ship together under that suite version. Each board is programmed
on its own.

| Module | Board | Sketch |
|--------|-------|--------|
| MCB — flight computer | Raspberry Pi Pico | `MCB/MCB.ino` |
| BMS — battery board | ATmega328PB | `BMS/BMS.ino` |
| ASB — atmospheric board | ATmega328PB | `ASB/ASB.ino` |
| GS 8601 — ground station | Adafruit Feather M0 | `GS/GS.ino` |

The folder name and the sketch name match (`MCB` and `MCB.ino`). The version
number stays in each module's `version.h`. Leave it out of the folder name.

## How the number is built

`MAJOR.MINOR.PATCH`

- **MAJOR** — a radio, SATBUS, settings, or identify layout that older
  software cannot read, or a pin or saved-settings change that means every
  board in the kit should be programmed again.
- **MINOR** — new messages, commands, or sensors that older software can
  ignore.
- **PATCH** — fixes that keep the same layouts.

2.0.x kept the same radio bytes as 1.x boards already in the field.
2.1.x keeps those layouts and adds short messages, which older software
ignores. Reboot, restart, and emergency power-off now require confirmation.
A host that sends those three commands without confirmation receives a
rejection.

## Where you see the version

Each module has one version string, `2.1.0`. The identify response uses the
integer `MAJOR × 10000 + MINOR × 100 + PATCH`, which is **20100** for this
suite. The text command `version` on the flight computer and the ground
station prints `2.1.0`.

| Version | Identify integer |
|---------|------------------|
| 2.0.2 | `20002` |
| 2.1.0 | `20100` |

## Names

- 2.0.x — "Horizon"
- 2.1.x — "Orbit"

The name follows the minor version. It does not change how the boards talk.

How a suite is built and tagged is recorded in
[docs/maintainer/release-process.md](docs/maintainer/release-process.md).
