# Radio link

A working guide for suite **2.1.0**. Byte layouts are in
[the radio specification](../protocols/telemetry-radio.md).
Multi-byte integers and floating-point numbers are **big-endian**
(the high byte comes first).

## Link

| Setting | Value |
|---------|-------|
| Radio | nRF24L01+ |
| Channel | 62, unless you change it |
| Rate | 1 Mbps |
| Power | Low (level 0) |
| Address | `45 46 45 46 30` |
| Acknowledgements | On |

The flight computer sends the header plus the bytes it is using.
The ground station always prints **32** bytes. Unused bytes at the end
of a short frame are zeros.

## 32-byte frame

| Bytes | Field |
|-------|-------|
| 0–1 | `#1` from the sender, or `#R` if another satellite relayed it |
| 2–5 | Sender id, 4 bytes |
| 6–9 | Destination id, 4 bytes. All zeros means everyone on the channel |
| 10–11 | Message type |
| 12–31 | Data, at most 20 bytes |

The ground station counts a buffer as valid when it starts with `#1` or
`#R` and the sender id is not zero. `device` followed by a hex id keeps
a single sender. `device 0` keeps all of them.

A relayed frame is marked `#R`. Command messages (`8` in the first type
digit) are not relayed.

## Example: system status `0001`

Eleven data bytes:

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | Operating mode |
| 1–2 | 2 | Status flags |
| 3–6 | 4 | Uptime, seconds |
| 7–10 | 4 | Boot count |

Flag bits, from the low bit upward: humidity sensor, IMU, GPS, radio,
BLE, atmospheric board, battery board, charge controller,
remove-before-flight pin inserted, low battery, error.

This example uses sender `00000043`, a broadcast destination, mode `2`
(normal operation), flags `000B` (humidity, IMU, and radio), uptime 1000,
and boot count 3. The last nine bytes are the unused part of the
ground-station print. It is an example, not a capture from hardware.

```text
[HEX] 23310000004300000000000102000B000003E800000003000000000000000000
```

| Bytes | Meaning |
|-------|---------|
| `23 31` | `#1` |
| `00 00 00 43` | Sender `43` |
| `00 00 00 00` | Broadcast |
| `00 01` | System status |
| `02` | Mode 2, normal operation |
| `00 0B` | Flags |
| `00 00 03 E8` | Uptime 1000 |
| `00 00 00 03` | Boot count 3 |

## Other message types

Decode tables for command replies (`0008`), battery (`010F`), atmosphere
(`030F`), particulate (`031F`), flight-computer temperature (`0320`),
GPS (`041F`), and IMU (`05FF`) are in the radio specification.
GPS latitude and longitude are degrees times 10,000,000.
IMU acceleration is times 1000. Gyro and magnetometer are times 10.

`0900` is a short text message: message id, fragment index, fragment
count (1–4), flags, then up to 16 text bytes. Bit 0 of the flags means
the sender is a ground station. The flight computer prints a finished
message as `[MSG] From 0x` and the sender id.

## Sending from the ground station

Build the same 32 bytes and send them as 64 hex characters.

```text
tx 233100000016000000430001...
```

- Bytes 2–5 are the sender id. The ground station's radio id is `0016`.
- Bytes 6–9 are the destination. All zeros is a broadcast and is sent
  without waiting for a hardware reply. Any other destination waits for
  that reply.
- `TX OK` or `TX ERR` is only that radio result. The flight computer
  answers a command later with message type `0008`.

Commands that need confirmation put `A5` in the first data byte
(byte 12 of the frame):

| Type | Action |
|------|--------|
| `8001` | Reboot |
| `8002` | Restart |
| `8103` | Emergency power off |
| `8200` | Set channel. The next byte is 0–125 |
| `8201` | Set power. The next byte is 0–3 |
| `8204` | Set the fast telemetry interval, milliseconds, limited to 100–60000 |
| `8205` | Save channel, power, and fast interval so they return on the next boot |
| `800B` | Buzzer: frequency in Hz, then duration in milliseconds |

On the flight computer's USB port the same reboot is the line
`reboot confirm`.

## Where to look next

- Packet names and layouts: [telemetry dictionary](../telemetry_dictionary.json)
- Ground-station commands: [GS README](../../GS/README.md)
