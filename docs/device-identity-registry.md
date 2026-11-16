# Device identity

How each CubeSat v1 board identifies itself on firmware **2.1.0**.
The identify command is documented in
[Device identification](quickstart/06-device-identification.md).

## Model numbers

| Module | Model number |
|--------|--------------|
| Flight computer (MCB) | `1001` |
| Battery board (BMS) | `2001` |
| Atmospheric board (ASB) | `3001` |
| Ground station (GS 8601) | `8601` |

## Serial numbers

The identify response prints the serial in hexadecimal.

| Module | What you see on a new board |
|--------|-----------------------------|
| MCB | `0043` |
| BMS | `0001` |
| ASB | `0001` |
| GS 8601 | `0016` |

On the flight computer you can store a different serial with the settings
command for code `0100`. The new serial is used after the next boot.

On the ground station, identify always prints `0016`. The text command
`serial` prints `8601-0042`. That second string is the label printed for
the unit. Software that filters radio traffic uses `0016`.

## Firmware version

The identify field is a single integer:

```text
MAJOR × 10000 + MINOR × 100 + PATCH
```

| Version string | Identify field |
|----------------|----------------|
| 2.0.2 | `20002` |
| 2.1.0 | `20100` |

`version` on the flight computer and the ground station prints the string
`2.1.0`. Banners on the battery and atmospheric boards print it at startup.

## Identify fields

`MODEL`, `SERIAL`, `FW_VERSION`, `STATUS`, `UPTIME`, `BOOT_COUNT`, `TEMP`,
`VOLT`, `FREE_RAM`, `CHIP_ID`.

Which fields are measured on each board is in the
[identification guide](quickstart/06-device-identification.md).

## What each board can do

| | MCB | BMS | ASB | GS 8601 |
|--|-----|-----|-----|---------|
| Answers `#?EF?#` | Yes | Yes | Yes | Yes |
| Settings command `#G` | Yes | | | |
| Radio transmit | Yes | Through the flight computer | Through the flight computer | Yes, with `tx` |
| Radio receive | Yes | Through the flight computer | Through the flight computer | Yes |
