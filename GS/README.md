# Ground station 8601 · v2.1.0

Receives nRF24L01+ packets from the CubeSat and prints them on USB.
A computer can also send one 32-byte frame back onto the radio.

- **Board:** Adafruit Feather M0 (8601 hardware)
- **Sketch:** `GS.ino`
- **Model:** `8601` · **Radio id:** `0016` · **Unit label:** `8601-0042`
- **Firmware:** `2.1.0` (identify field `20100`)
- **Uplink:** `tx` followed by 64 hex characters sends one 32-byte frame

The 8701 ground station is a different product and is not in this suite.

## What you see

- Radio channel 62 unless you change it, 32-byte payloads, address `45 46 45 46 30`
- Each received payload is one line: `[HEX]` and 64 hex characters
- A frame counts when it starts with `#1` or `#R` and the sender id is not zero
- Status LEDs, a buzzer, identify, and the commands below

## Pins

| Function | Pin |
|----------|-----|
| Green / red / blue / yellow / white LEDs | D2 / D3 / D4 / D5 / D13 |
| Radio CSN / CE | D9 / D10 |
| Buzzer | D8 |
| Extra serial RX/TX | D0 / D1 |

The radio SPI pins are MOSI D11, MISO D12, and clock D13.
D13 is also the white LED, so that LED follows the clock while the radio is active.

## Serial commands

USB is **115200** baud, 8 data bits, no parity, 1 stop bit. One command
per line. `#?EF?#` and `tx` keep their capital letters. Other commands
are accepted in any case.

| Command | Effect |
|---------|--------|
| `help` or `h` | Print the command list |
| `stats` or `s` | Session counts |
| `debug` or `d` | Extra lines for rejected frames |
| `hex` | Turn `[HEX]` lines on or off (on by default) |
| `device` and an id | Accept only that sender, in hex. `0` accepts all |
| `reset` or `r` | Clear the session counts |
| `radio` | Channel, payload size, power, rate, and whether the chip answers |
| `radio reset` | Restart the radio (at most once every 2 seconds) |
| `piezo` | Turn the buzzer on or off |
| `leds` | Turn the status LEDs on or off |
| `version` or `v` | Print `2.1.0` |
| `serial` | Print `8601-0042` |
| `tx` and 64 hex characters | Send 32 bytes. The reply is `TX OK` or `TX ERR` |
| `channel` and a number 0–125 | Set this station's channel, then listen again |
| `#?EF?#` | Identification response |

`TX OK` means a directed send was acknowledged by the other radio, or a
broadcast was handed to the radio. It is not the flight computer's command
reply (`0008`). A destination of all zeros is a broadcast and does not
wait for an acknowledgement.

## Program the board

1. Install Adafruit SAMD boards and select **Adafruit Feather M0**.
2. Install the `RF24` library.
3. Open `GS.ino` and upload. Double-tap reset if the bootloader port does not appear.

See [Program a module](../docs/quickstart/01-programming-toolchains.md).

## More detail

- [Radio link](../docs/protocols/telemetry-radio.md)
- [Identify a module](../docs/protocols/ef-identify.md)

## License

EdgeFlyte Non-Commercial / Educational License 1.0 — see [LICENSE](../LICENSE).
