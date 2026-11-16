# Device Identification `#?EF?#`

Version: applies to Firmware suite **2.1.0**.

## Probe

Host sends exact string (case-sensitive):

```
#?EF?#
```

## Response

```
#!EF!:<MODEL>,<SERIAL>,<FW_VERSION>,<STATUS>,<UPTIME>,<BOOT_COUNT>,<TEMP>,<VOLT>,<FREE_RAM>,<CHIP_ID>#
```

`FW_VERSION` = `MAJOR*10000 + MINOR*100 + PATCH`. For 2.1.0 this is **`20100`**.

## Models

| Module | Model | Serial on a new board |
|--------|-------|------------------------|
| Flight computer | `1001` | `0043` |
| Battery board | `2001` | `0001` |
| Atmospheric board | `3001` | `0001` |
| Ground station | `8601` | `0016` |

Typical USB VID (host OS): Adafruit Feather M0 `0x239A`, Raspberry Pi Pico `0x2E8A`.
