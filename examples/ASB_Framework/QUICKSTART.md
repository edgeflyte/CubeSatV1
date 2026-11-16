# Atmospheric board template

A starting point for a modified atmospheric-board sketch. For the firmware
that ships with the kit, program `ASB/ASB.ino`. Use this folder when you
want switches for each sensor, or empty functions where your own code runs.

The kit sketch sends `$ASB,0` and `$ASB,1` with the scales in
[the packet specification](../../ASB/ASB_PACKET_SPECIFICATION.md).
This template still sends `$ASB,3` and `$ASB,4`, encodes altitude without
the +100 m offset, multiplies particulate mass by 100, and divides number
concentration by 10. The flight computer accepts ids `3` and `4`, and it
decodes them with the kit scales. Change this template to match the packet
specification before you fly it with the stock flight computer.

## What you can edit at the top of the file

- One switch per sensor. Turn a sensor off to leave it out of the build.
- Four functions: `onBoot`, `onTick`, `onBeforeTransmit`, `onAfterTransmit`.
- A worked example for adding a sensor, at the bottom of `ASB_Framework.ino`.

## Hardware and libraries

- CubeSat v1 atmospheric board (ATmega328PB in the kit; this template was written against an ATmega328P)
- A USB or ISP programmer
- Arduino IDE or Arduino CLI
- MiniCore, clock **Internal 8 MHz**

Install a library only for the sensors you leave enabled:

| Sensor | Library |
|--------|---------|
| SHT4x | `Adafruit SHT4x Library` |
| BMP180 | `Adafruit BMP085 Library` |
| ENS160 | `SparkFun Indoor Air Quality Sensor - ENS160` |
| SCD4x | `DFRobot_SCD4X` |
| SPS30 | `Sensirion SPS30` |

## First upload

1. Open `ASB_Framework.ino`.
2. Select the board and port.
3. Upload.
4. Open a serial monitor at **9600** baud.

You will see `$ASB,3` about once a second. `$ASB,4` starts several seconds
later, after the particulate sensor warms up. Those ids are this template's
ids, not the kit sketch's ids.

## Common edits

All of these are in the top of `ASB_Framework.ino`.

Leave a sensor out of the build:

```cpp
#define USE_SPS30 0
```

Print human-readable values before each sentence. Turn this off when only
sentences should be on the wire:

```cpp
#define DEBUG_OUTPUT 1
```

Change the sentence rate:

```cpp
#define PACKET_INTERVAL_MS 500
```

Use your own sentence name when the contents differ from the kit:

```cpp
#define PACKET_ID_MAIN "$MYBUILD,0"
#define PACKET_ID_PM   "$MYBUILD,1"
```

Update the flight computer so it recognizes that name.

## Your code

| Function | When it runs | Use it for |
|----------|----------------|------------|
| `onBoot()` | Once, after the sensors start | Pin setup, a banner |
| `onTick()` | Every pass through the loop | A heartbeat LED, a button |
| `onBeforeTransmit()` | After the sensors are read, before the sentence | A calibration offset, a filter |
| `onAfterTransmit()` | After the sentence is sent | A log line, a blink |

```cpp
static void onBeforeTransmit() {
  data.t1 += 0.5f;
}
```

Keep `onTick()` short. A delay there holds up the sensor reads and the sentence timer.

## Add a sensor

Section 10 of `ASB_Framework.ino` is a complete example.

1. Set `#define USE_MYSENSOR 1` in the configuration block.
2. Wrap the library include and the object in `#if USE_MYSENSOR`.
3. Add a field for the reading.
4. Write `initMySensor()` and `readMySensor()`, and call them from `initAllSensors()` and `readSensors()`.
5. Append the value in `transmitMainPacket()`, or add another transmit function.

Someone without that sensor can still build the sketch by setting `USE_MYSENSOR` to `0`.

## Sentence builders

`transmitMainPacket()` sends the environment sentence.
`transmitPMPacket()` sends the particulate sentence.
Edit them to add, remove, or reorder fields, and change the sentence id
when you do. The kit field order and scales are in the packet specification.

## Order of operations

```text
setup()
  Wire.begin(), Serial.begin(9600)
  release the transmit pin
  initAllSensors()
  onBoot()

loop()
  onTick()
  particulate warm-up
  retry any sensor that failed (every 10 s)
  every PACKET_INTERVAL_MS:
    readSensors()
    onBeforeTransmit()
    transmitPackets()
    onAfterTransmit()
```

A sensor can drop off the bus and return. The next retry starts it again.

## If something looks wrong

| What you see | What to check |
|--------------|----------------|
| No output | 9600 baud and the port |
| `$ASB,3` and never `$ASB,4` | Particulate sensor. Wait at least 8 seconds after boot |
| Temperature or humidity stuck at the empty-sensor value | SHT4x wiring and I2C |
| Compile error, library not found | Install that library, or set its `USE_` switch to `0` |
| The sketch does not fit | Turn off unused sensors and set `DEBUG_OUTPUT` to `0` |
| The flight computer shows the wrong altitude or dust numbers | This template's scales differ from the kit. Match the packet specification |

## Files

- `ASB_Framework.ino` — the template
- `QUICKSTART.md` — this file
- `../../ASB/ASB.ino` — the kit firmware
- `../../ASB/ASB_PACKET_SPECIFICATION.md` — kit sentence format
