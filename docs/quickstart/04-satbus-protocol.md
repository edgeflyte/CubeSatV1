# SATBUS

How the atmospheric board, battery board, and an optional charge
controller talk to the flight computer. Suite **2.1.0**. The sentence
layouts are also in [the SATBUS specification](../protocols/satbus-v1.md).

## Wire and framing

- 9600 baud, 8 data bits, no parity, 1 stop bit, 3.3 V.
- Flight computer transmit is GPIO 4. Receive is GPIO 5.
- Data sentences start with `$` and end with `*`.
- Command sentences start with `&`.
- Fields are separated by commas. These sentences have no checksum.
- The atmospheric and battery boards release the transmit pin between
  sentences so they do not hold the wire.

## Atmospheric board

The kit firmware sends two sentences. The flight computer also accepts
the older ids `3` and `4`, with the same fields and the same scales.

| Sentence | Contents |
|----------|----------|
| `$ASB,0,...*` | Environment |
| `$ASB,1,...*` | Particulate matter |
| `$ASB,3,...*` | Same fields as id 0 |
| `$ASB,4,...*` | Same fields as id 1 |

The example in `examples/ASB_Framework` still sends ids `3` and `4` and
uses older scales. The kit sketch is `ASB/ASB.ino`. It sends ids `0` and
`1`. Match [the packet specification](../../ASB/ASB_PACKET_SPECIFICATION.md)
before a modified board flies with the stock flight computer.

### Environment

```text
$ASB,0,T1,T2,RH,P,ALT,AQI,TVOC,ECO2,CO2,*
```

| Field | On the wire | Back to engineering units |
|-------|-------------|---------------------------|
| T1, T2 | `(°C + 40) × 100`, 5 digits | divide by 100, then subtract 40 |
| RH | `% × 10`, 4 digits | divide by 10 |
| P | `Pa / 10`, 5 digits | multiply by 10 |
| ALT | `m + 100`, 5 digits | subtract 100 |
| AQI, TVOC, eCO2, CO2 | integers | unchanged |

Example for 25.0 °C, 24.5 °C, 40.0 %RH, 101325 Pa, 100 m, AQI 1,
TVOC 12, eCO2 400, CO2 420:

```text
$ASB,0,06500,06450,0400,10132,00200,1,000012,00400,00420,*
```

Pressure is divided by 10 before it is sent, so 101325 Pa goes out as
10132 and comes back as 101320 Pa. Altitude 100 m goes out as 200 and
comes back as 100 m.

### Particulate matter

```text
$ASB,1,MC1,MC25,MC4,MC10,NC05,NC1,NC25,NC4,NC10,TPS,*
```

| Field | On the wire | Back to engineering units |
|-------|-------------|---------------------------|
| Mass concentrations | `µg/m³ × 10` | divide by 10. Values below 0 become 0 |
| Number concentrations | `particles/cm³ × 10` | divide by 10. Values below 0 become 0 |
| Typical particle size | `µm × 100` | divide by 100. Values below 0 become 0 |

Number bins other than 0.5 µm are the difference from the next-smaller
bin, held at 0 when the difference would be negative, then scaled.

The environment line is sent every 250 ms. The particulate line is sent
on its own 250 ms timer, and only after at least 80 ms have passed since
the environment line.

## Battery board

About every 250 ms:

```text
$BMS,<err>,<ldo1>,<ldo2>,<ldo3>,<ldo4>,<batV>,<batPct>,<sol1>,<sol2>*
```

| Field | Meaning |
|-------|---------|
| err | `0` above 20%, `1` at or below 20%, `2` at or below 5% |
| ldo1–ldo4 | `1` on or `0` off |
| batV | Pack volts, two decimal places. Pin volts times 2, limited to 0–6.6 V |
| batPct | 0–100, from 4.0 V to 5.6 V (four NiMH cells, 1.0 V to 1.4 V each) |
| sol1, sol2 | Panel volts, limited to 0–3.3 V |

Commands the flight computer sends:

| Line | Effect |
|------|--------|
| `&BMS,CH1,1` through `&BMS,CH4,0` | Turn that output on or off |
| `&BMS,RST,1` | Restore the outputs that were on at boot |
| `&BMS,CFG,` and a number 0–15 | Store the boot pattern. Bit 0 is channel 1 |

A stored pattern is used only when the first saved byte is `B5` and the
second byte is `00` through `0F`.

## Charge controller

The flight computer accepts these sentences. This suite has no
charge-controller sketch.

| Sentence | Contents |
|----------|----------|
| `$SCC,1,V0,V1,V2,V3,V4,V5,*` | Six voltages, in volts |
| `$SCC,2,T0,…,T7,*` | Eight temperatures, in °C |
| `$SCC,3,VOUT,IOUT,MODE,*` | Output volts, output current, mode |

## Adding a sensor

Edit `ASB/ASB.ino`. The walkthrough is
[Custom sensor integration](../../ASB/Custom-Sensor-Integration.md).
The flight computer accepts atmospheric ids `0`, `1`, `3`, and `4`.
A new id, or a new module name, is ignored until the flight computer
firmware is updated to recognize it. Radio packets for the existing
sentences are type `030F` (environment) and `031F` (particulate).
A radio data field holds at most 20 bytes, so a longer SATBUS sentence
does not automatically fit in one radio frame.
