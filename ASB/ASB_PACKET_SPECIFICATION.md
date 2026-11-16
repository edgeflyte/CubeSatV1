# Atmospheric board packet specification

Suite **2.1.0**. This is the format sent by `ASB/ASB.ino` at 9600 baud,
8 data bits, no parity, 1 stop bit.

The example template in `examples/ASB_Framework` sends older ids and
scales. Use this document when a board talks to the stock flight computer.

## Sentences

| Sentence | When |
|----------|------|
| `$ASB,0,...*` | Environment, every 250 ms |
| `$ASB,1,...*` | Particulate matter, every 250 ms, and at least 80 ms after the environment sentence |

The flight computer also accepts ids `3` and `4` and decodes them with
the scales below.

## Environment (`$ASB,0`)

```text
$ASB,0,T1,T2,RH,P,ALT,AQI,TVOC,ECO2,CO2,*
```

Worked example for 25.00 °C, 24.50 °C, 40.0 %RH, 101325 Pa, 100 m,
AQI 1, TVOC 12, eCO2 400, CO2 420:

```text
$ASB,0,06500,06450,0400,10132,00200,1,000012,00400,00420,*
```

| Field | Name | Units | On the wire | Back to engineering units | Source |
|-------|------|-------|-------------|---------------------------|--------|
| T1 | Temperature 1 | °C | `(°C + 40) × 100`, 5 digits | `/ 100 − 40` | SHT4x, or SHT31 if that is the sensor that answered |
| T2 | Temperature 2 | °C | `(°C + 40) × 100`, 5 digits | `/ 100 − 40` | BMP180 |
| RH | Humidity | %RH | `% × 10`, 4 digits | `/ 10` | Same sensor as T1 |
| P | Pressure | Pa | `Pa / 10`, 5 digits | `× 10` | BMP180 |
| ALT | Altitude | m | `m + 100`, 5 digits | `− 100` | BMP180 |
| AQI | Air-quality index | 1–5 | integer, 1 digit | unchanged | ENS160 |
| TVOC | Total VOC | ppb | integer, 6 digits | unchanged | ENS160 |
| ECO2 | Equivalent CO2 | ppm | integer, 5 digits | unchanged | ENS160 |
| CO2 | CO2 | ppm | integer, 5 digits | unchanged | SCD4x |

Pressure loses the last digit: 101325 Pa is sent as 10132 and reads back
as 101320 Pa. Altitude 100 m is sent as 200.

Air-quality index: 1 excellent, 2 good, 3 moderate, 4 poor, 5 unhealthy.

## Particulate matter (`$ASB,1`)

```text
$ASB,1,MC1,MC25,MC4,MC10,NC05,NC1,NC25,NC4,NC10,TPS,*
```

| Field | Name | Units | On the wire | Back to engineering units |
|-------|------|-------|-------------|---------------------------|
| MC1, MC25, MC4, MC10 | Mass, 1.0 / 2.5 / 4.0 / 10 µm | µg/m³ | `× 10`, 5 digits | `/ 10` |
| NC05 | Number, 0.5 µm | /cm³ | `× 10`, 6 digits | `/ 10` |
| NC1, NC25, NC4, NC10 | Number difference from the next-smaller bin | /cm³ | `× 10`, 6 digits | `/ 10` |
| TPS | Typical particle size | µm | `× 100`, 3 digits | `/ 100` |

A negative mass, count, or size is sent as 0. Number differences that
would be negative are sent as 0. All of these fields come from the SPS30.
The sentence starts after the sensor reports data ready.

## Reading a sentence

```python
def parse_environment(packet):
    fields = packet.strip().strip("*").split(",")
    if fields[0] != "$ASB" or fields[1] != "0":
        return None
    return {
        "t1_c": int(fields[2]) / 100 - 40,
        "t2_c": int(fields[3]) / 100 - 40,
        "rh_percent": int(fields[4]) / 10,
        "pressure_pa": int(fields[5]) * 10,
        "altitude_m": int(fields[6]) - 100,
        "aqi": int(fields[7]),
        "tvoc_ppb": int(fields[8]),
        "eco2_ppm": int(fields[9]),
        "co2_ppm": int(fields[10]),
    }


def parse_particulate(packet):
    fields = packet.strip().strip("*").split(",")
    if fields[0] != "$ASB" or fields[1] != "1":
        return None
    return {
        "pm1_ug": int(fields[2]) / 10,
        "pm25_ug": int(fields[3]) / 10,
        "pm4_ug": int(fields[4]) / 10,
        "pm10_ug": int(fields[5]) / 10,
        "nc05_per_cm3": int(fields[6]) / 10,
        "nc1_diff": int(fields[7]) / 10,
        "nc25_diff": int(fields[8]) / 10,
        "nc4_diff": int(fields[9]) / 10,
        "nc10_diff": int(fields[10]) / 10,
        "typical_size_um": int(fields[11]) / 100,
    }
```

Treat ids `3` and `4` with these same conversions when the sender is the
kit firmware or a custom board that follows this page. The example
framework does not follow this page until you change it.

## Adding a field

Keep `$ASB,0` and `$ASB,1` as they are. Add a new sentence id, document
its order and scale, and teach the flight computer to accept that id.
The stock flight computer drops every atmospheric id other than `0`, `1`,
`3`, and `4`. A radio data field holds at most 20 bytes.
