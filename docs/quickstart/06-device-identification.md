# Identify a module (`#?EF?#`)

Every module in suite **2.1.0** answers the same probe on its serial port.
The layout is in [the identify specification](../protocols/ef-identify.md).
Model numbers and serials are in
[Device identity](../device-identity-registry.md).

## Probe

Send this line exactly. Case matters. A trailing newline is fine.

```text
#?EF?#
```

| Module | Port | Baud |
|--------|------|------|
| Flight computer | USB | 115200 |
| Ground station | USB | 115200 |
| Battery board | SATBUS data port | 9600 |
| Atmospheric board | SATBUS data port | 9600 |

The battery and atmospheric boards drive the transmit pin for the reply,
then release it. On a shared SATBUS wire, send the probe when the bus is
quiet enough for the reply to fit.

## Response

```text
#!EF!:<MODEL>,<SERIAL>,<FW_VERSION>,<STATUS>,<UPTIME>,<BOOT_COUNT>,<TEMP>,<VOLT>,<FREE_RAM>,<CHIP_ID>#
```

| Field | Meaning |
|-------|---------|
| MODEL | `1001` flight computer, `2001` battery, `3001` atmospheric, `8601` ground station |
| SERIAL | Hexadecimal |
| FW_VERSION | `MAJOR × 10000 + MINOR × 100 + PATCH`. **2.1.0 is `20100`** |
| STATUS | `OK` or `ERR` |
| UPTIME | Seconds since boot |
| BOOT_COUNT | See the notes for that board |
| TEMP | Degrees C, one decimal, or a fixed stand-in |
| VOLT | Volts, two decimals, or a fixed stand-in |
| FREE_RAM | Bytes, or `0` when the board does not measure it |
| CHIP_ID | Hex chip id, or `00000000` when the board does not read one |

## What each board fills in

**Flight computer.** The serial is the last four hex digits of the stored
serial, so a new board prints `0043`. Status is `ERR` when the chip
temperature is outside −20 °C to 60 °C, voltage is outside 3.0 V to 5.5 V,
both the humidity sensor and the IMU are absent, or the low-battery flag
is set. Temperature, voltage, free memory, and the Pico's chip id are
measured. Boot count is saved and increases across reboots.

```text
#!EF!:1001,0043,20100,OK,<uptime>,<boots>,<temp>,<volt>,<memory>,<chip>#
```

**Ground station.** The identify serial is always `0016`. The text command
`serial` prints `8601-0042`, which is the unit label, not this field.
Status is `ERR` when the radio is disconnected, chip temperature is
outside −20 °C to 85 °C, or measured supply voltage is outside 3.0 V to
3.6 V. Boot count increases until power is removed, then starts at 1
again. The chip id is 32 hex digits.

```text
#!EF!:8601,0016,20100,OK,<uptime>,<boots>,<temp>,<volt>,<memory>,<32 hex digits>#
```

**Battery board.** Status is `OK` only when the pack is above the warning
level (`err` is 0). Temperature is always `0.0`. Voltage is the pack
voltage. Free memory is `0`. Chip id is `00000000`. Boot count stays at 1.

```text
#!EF!:2001,0001,20100,OK,<uptime>,1,0.0,<pack volts>,0,00000000#
```

**Atmospheric board.** Status is always `OK`. Temperature is the humidity
sensor reading. Voltage is always `0.00`. Free memory is `0`. Chip id is
`00000000`. Boot count stays at 1.

```text
#!EF!:3001,0001,20100,OK,<uptime>,1,<temperature>,0.00,0,00000000#
```

## Using it

- Open a serial port, send `#?EF?#`, and match the model number to find the board.
- For this suite the version field is `20100`. `20002` is firmware 2.0.2. `1` is an older ground station.
- `ERR` on the flight computer or ground station uses the checks above. `ERR` on the battery board means the pack is at or below 20%.
