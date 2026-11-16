# SATBUS v1

Firmware suite **2.1.0**. Worked examples are in the
[SATBUS guide](../quickstart/04-satbus-protocol.md).

## Physical

- 9600 baud, 8 data bits, no parity, 1 stop bit, 3.3 V
- Flight computer transmit GPIO 4, receive GPIO 5

## Grammar

- Data: `$MODULE,fields...,*`
- Command: `&MODULE,CMD,VALUE`

## Atmospheric sentences

| Sentence | Id | Content |
|----------|----|---------|
| `$ASB,0,...` | 0 | Environment. Id 3 is the same sentence |
| `$ASB,1,...` | 1 | Particulate matter. Id 4 is the same sentence |

### Environmental encoding (`$ASB,0`)

| Field | On the wire | How to read it |
|-------|--------------|--------------|
| T1, T2 | `(°C + 40) × 100` | `/100 − 40` |
| RH | `% × 10` | `/10` |
| P | Pa `/ 10` | `×10` |
| Altitude | `m + 100` | `− 100` |
| AQI, TVOC, eCO2, CO2 | raw | raw |

### PM encoding (`$ASB,1`)

| Field | On the wire | How to read it |
|-------|--------------|--------------|
| MC* | `μg/m³ × 10` | `/10` |
| NC* | `/cm³ × 10` | `/10` |
| TPS | `μm × 100` | `/100` |

## Battery board

`$BMS,ERR,LDO1,LDO2,LDO3,LDO4,BATV,BATPCT,SOL1,SOL2*`

Commands from the flight computer: `&BMS,CH1,1` through `&BMS,CH4,0`,
`&BMS,RST,1`, and `&BMS,CFG,` with a number from 0 to 15.
