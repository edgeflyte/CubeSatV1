# Telemetry radio protocol

Firmware suite **2.1.0**. A worked example is in the
[radio guide](../quickstart/03-telemetry-radio-protocol.md).

## Link defaults

| Param | Value |
|-------|-------|
| Channel | 62 |
| Data rate | 1 Mbps |
| Power | Low |
| Payload | 32 bytes |
| Address (5 bytes) | `45 46 45 46 30` |
| Replies | On |

## On-wire frame (32 bytes)

| Offset | Size | Field |
|--------|------|-------|
| 0–1 | 2 | `#1` from the sender, or `#R` if another node relayed it |
| 2–5 | 4 | Sender id |
| 6–9 | 4 | Destination id. `0` means everyone on the channel |
| 10–11 | 2 | Message type |
| 12–31 | up to 20 | Data |

Multi-byte numbers are high byte first. `f32` is an IEEE-754 float.

## Message types you will see

| Type | Data |
|------|------|
| `0001` System status | mode (1 byte), flags (2), uptime in seconds (4), boot count (4) |
| `0008` Command reply | command (2), status (`01` = ok), action, parameter, value, message length, then up to 13 text bytes |
| `010F` Battery | error (1), packed output states (1), pack volts (float), percent (float), solar 1 and solar 2 as hundredths of a volt (2 each) |
| `030F` Environment | temperature 1, temperature 2, humidity, and pressure as floats, then air-quality index and TVOC, eCO2, and CO2 as 2-byte counts. The radio field stops at 20 bytes |
| `031F` Particulate | PM1, PM2.5, PM4, PM10, and typical size, as floats |
| `0320` Flight-computer climate | temperature and humidity, as floats |
| `041F` GPS | latitude and longitude as degrees × 10,000,000 (4 bytes each), altitude (float), satellite count, HDOP × 10, fix |
| `05FF` IMU | acceleration × 1000 (three signed 2-byte values), gyro × 10 (three), magnetometer × 10 (three) |

## Commands

The frame is the same. The destination is the flight computer's serial, or all zeros for everyone on the channel.

Reboot, restart, and emergency power-off need `A5` as the first data byte:

| Type | Action |
|------|--------|
| `8001` | Reboot |
| `8002` | Restart |
| `8103` | Emergency power off |
| `8200` | Channel, next byte 0–125 |
| `8201` | Power, next byte 0–3 |
| `8204` | Fast telemetry interval, next 2 bytes are milliseconds, limited to 100–60000 |
| `8205` | Save the live channel, power, and fast interval for the next boot |
| `800B` | Buzzer. Next 2 bytes are Hz (`0000` stops it; otherwise 20–20000). The 2 bytes after that are duration in milliseconds, 10–30000 |

Every command gets a `0008` reply addressed to the sender. That reply goes out before a channel or power change is applied. `TX OK` from the ground station only means the radio accepted the send.

## Short messages (`0900`)

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | Message id |
| 1 | 1 | Fragment index, starting at 0 |
| 2 | 1 | Fragment count, 1–4 |
| 3 | 1 | Flags. Bit 0 means the sender is a ground station |
| 4–19 | up to 16 | Text |

A message can be 64 characters across 4 fragments. A relay marks the frame `#R`. Command messages are not relayed.

Destination `0` reaches every radio on the channel, including other ground stations. A specific ground-station serial reaches that station. Ground stations do not send a `0008` reply.

## Ground station USB

- **115200** baud, 8 data bits, no parity, 1 stop bit
- Each received frame is one line: `[HEX]` and 64 hex characters
- Identify: `#?EF?#` answers `#!EF!:8601,<serial>,20100,...#`
- Send: `tx` and 64 hex characters. The reply is `TX OK` or `TX ERR`
