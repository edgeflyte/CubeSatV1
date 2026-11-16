# Connect the modules

How the 2.1.0 boards connect. Pin numbers match the firmware.
This repository does not define connector names or cable colors.

```text
  Atmospheric board  --UART 9600-->  Flight computer  <--UART 9600--  Battery board
                                      |  radio
                                      v
                                 Ground station  --USB 115200-->  computer
```

A solar charge controller, when fitted, shares the same data wire as the
atmospheric and battery boards. The flight computer understands `$SCC`
sentences. This suite does not include charge-controller firmware.

## Power

The battery board switches four regulator outputs on D5, D6, D7, and D8.
If a pattern has been saved, that pattern is restored at boot. Otherwise
every output turns on.

The flight computer requests a change with `&BMS,CH1,1` through
`&BMS,CH4,0`, restores the boot pattern with `&BMS,RST,1`, and stores a
pattern with `&BMS,CFG,` followed by a number from 0 to 15.

Battery voltage is measured on A0 through a divider that doubles the pin
voltage, against a 3.3 V reference. Solar channels are A1 and A2 and are
reported up to 3.3 V. Keep the grounds of the battery board, atmospheric
board, flight computer, and ground station connected. The data link uses
3.3 V logic.

## Data wire (SATBUS)

| | |
|--|--|
| Speed | 9600 baud, 8 data bits, no parity, 1 stop bit |
| Flight computer | TX GPIO 4, RX GPIO 5 |
| Atmospheric and battery boards | Hardware serial at 9600 |

The atmospheric and battery boards drive the transmit pin only while they
are sending a sentence. The flight computer is the board that sends
`&BMS` commands. Do not tie two transmit pins together unless each board
releases the wire between sentences.

Sentences that start with `$` are data. Sentences that start with `&` are
commands. Both end with `*`. Details are in [SATBUS](04-satbus-protocol.md).

## Radio link

Both radios are nRF24L01+ modules on SPI. The default channel is **62**,
at 1 Mbps, low power, 32-byte payload, address `45 46 45 46 30`.

| Signal | Flight computer | Ground station |
|--------|-----------------|----------------|
| CE | GPIO 11 | D10 |
| CSN | GPIO 15 | D9 |
| SCK | GPIO 18 | D13 |
| MOSI | GPIO 19 | D11 |
| MISO | GPIO 16 | D12 |
| IRQ | GPIO 14 | not used |

Ground-station D13 is also the white activity LED, so that LED follows
the SPI clock while the radio is in use.

The flight computer can change its own channel, power, and fast telemetry
interval with radio commands `0x8200`, `0x8201`, and `0x8204`. The ground
station changes only its own channel with `channel` and a number from 0
to 125. Both ends use the same channel and address, or no frames arrive.

## USB serial

| Module | Baud | Role |
|--------|------|------|
| Flight computer | 115200 | Log, text commands, settings, identify |
| Flight computer GPS | 9600 on GPIO 0 and 1 | NMEA in |
| Ground station | 115200 | `[HEX]` downlink, `tx` uplink, identify |
| Battery and atmospheric boards | 9600 | Data sentences, and identify on the bench |

The flight computer's BLE enable pin is GPIO 20. Local configuration uses
the USB serial line.

## Pins already in use

| Module | Pin | Use |
|--------|-----|-----|
| Flight computer | GPIO 2 | Remove-before-flight sense |
| Flight computer | GPIO 3 | Buzzer |
| Flight computer | GPIO 6 / 7 / 8 | LED 1 / 2 / 3 |
| Flight computer | GPIO 17 | SD card chip select |
| Flight computer | GPIO 12 / 13 | I2C data / clock at 400 kHz |
| Flight computer | GPIO 26 / 27 / 28 | Analog inputs 0 / 1 / 2 |
| Atmospheric board | D2 | Status LED |
| Battery board | D2 / D3 / D4 | Activity, command, and low-battery LEDs |

Unused flight-computer pins in this firmware are GPIO 9, 10, 21, and 22.
