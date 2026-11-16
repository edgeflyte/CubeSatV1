# Settings command `#G`

Firmware suite **2.1.0**. USB serial on the flight computer only, at
115200 baud. The radio does not accept this command. Ranges and limits
are in the [settings guide](../quickstart/05-parameter-protocol.md).

## Frame

`#G<VV><CC><PPPP>[VALUE_HEX]`

| Field | Meaning |
|-------|---------|
| VV | `01` protocol version |
| CC | `00` read · `FF` write · `7F` read result · `FE` write result · `01` error |
| PPPP | setting code, four hex digits |
| VALUE | value bytes, high byte first |

## Examples

```
#G01000002           read version (code 0002)
#G017F0002322E312E30  answer "2.1.0"
#G01FF02003E         save radio channel 62
```

## Key ranges

| Range | Purpose |
|-------|---------|
| `0x00xx` | Read-only system values |
| `0x01xx` | Saved configuration |
| `0x02xx` | Radio |
| `0x03xx` | Sensors |
| `0x04xx` | Live readings |
| `0x05xx` | Pins |
| `0x06xx` | Logging |
| `0x07xx` | Mesh |
| `0x08xx` | Battery board |
| `0x09xx` | Atmospheric board |
| `0x0Axx` | Charge controller |

Writes that set an interval are limited (fast telemetry is 100–60000 ms).
Saved settings are stored as a block that starts with the bytes `CB 26`.
