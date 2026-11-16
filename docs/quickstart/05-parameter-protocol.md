# Settings command (`#G`)

USB serial on the **flight computer only**, 115200 baud, 8 data bits,
no parity, 1 stop bit. The radio does not accept this command.
Suite **2.1.0**.

## Frame

```text
#G<VV><CC><PPPP>[VALUE_HEX]
```

| Field | Width | Meaning |
|-------|-------|---------|
| `VV` | 2 hex digits | Protocol version. Only `01` is accepted |
| `CC` | 2 hex digits | Command |
| `PPPP` | 4 hex digits | Setting code |
| VALUE | an even number of hex digits | Value, high byte first |

| Code | Direction | Meaning |
|------|-----------|---------|
| `00` | You → flight computer | Read |
| `7F` | Flight computer → you | Read result |
| `FF` | You → flight computer | Write |
| `FE` | Flight computer → you | Write result |
| `01` | Flight computer → you | Error. The value byte is the error code |

The frame has no line ending inside it. The flight computer answers
with one line.

### Error codes

| Code | Meaning |
|------|---------|
| `E0` | Unknown setting |
| `E1` | Read only |
| `E2` | Value length does not match |
| `E3` | Out of range, or the value was rejected |
| `E4` | Not permitted |
| `E5` | The saved copy could not be written |
| `E6` | Hardware unavailable |
| `E7` | Protocol version is not `01` |
| `E8` | Malformed packet |

## Examples

Read the firmware version (code `0002`). The value is the ASCII bytes
of `2.1.0`.

```text
#G01000002
#G017F0002322E312E30
```

Read the serial (code `0001`). A new board answers `00000043`.

```text
#G01000001
#G017F000100000043
```

Save radio channel 62 (code `0200`, value `3E`):

```text
#G01FF02003E
#G01FE02003E
```

That write is stored for the next boot. It does not retune the radio
that is already running. Reboot, or send radio command `8200`, to use
the new channel now. A channel stored this way is the one used on the
next boot.

## Groups

| Codes | Access | Contents |
|-------|--------|----------|
| `00xx` | read | Serial, version, uptime, boot count, mode, status, free memory |
| `01xx` | read/write, saved | Serial override, intervals, default mode |
| `0200`–`021F` | read/write, saved | Channel, power, rate, payload size, 5-byte addresses |
| `0220`–`022F` | read | Packet counters, radio-present flag |
| `03xx` | mixed | Which sensors are present, and IMU calibration slots |
| `04xx` | read | Live flight-computer sensor and GPS values |
| `0500`–`050F` | read/write | LEDs, buzzer, BLE enable |
| `0510`–`052F` | read | Remove-before-flight pin and analog inputs 0–2 |
| `0600`–`060F` | read/write, saved | Logging on/off and interval |
| `0610` | read | Current log filename |
| `0700`–`070F` | read/write, saved | Mesh relay, mesh SD log |
| `0710`–`072F` | read | Mesh counters |
| `08xx` | read | Latest battery-board values |
| `09xx` | read | Latest atmospheric-board values |
| `0Axx` | read | Latest charge-controller values |

The full code list is the `PARAM_` names in `MCB/ParameterHandler.h`.

## Limits on a write

| Code | Limit |
|------|-------|
| `0101` fast telemetry | 100–60000 ms |
| `0102` slow telemetry | 100–300000 ms |
| `0103` sensor interval | 50–10000 ms |
| `0104` heartbeat | 100–10000 ms |
| `0105` GPS interval | 100–10000 ms |
| `0111` SATBUS poll | 10–5000 ms |
| `0110` default mode | 0–5 |
| `0200` channel | 0–125 |
| `0201` power | 0–3. `0` is low power |
| `0202` data rate | 0–2. `0` is 1 Mbps |
| `0203` payload | 1–32 |
| `0601` log interval | 100–300000 ms |

Turning logging on, or changing a log interval, takes effect immediately.
LED, buzzer, and BLE writes take effect immediately. Other writes change
the saved configuration. Radio channel, power, and the fast telemetry
interval also return on the next boot when you send radio command `8205`.

The saved block starts with the bytes `CB 26`. If those bytes are missing,
the next boot writes the factory settings. The text command `config reset`
restores those settings and sets the boot count back to zero.

Floating-point values are the raw IEEE bits, high byte first, not decimal text.
