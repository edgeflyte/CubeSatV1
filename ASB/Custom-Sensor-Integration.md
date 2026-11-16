# Add a sensor to the atmospheric board

This guide edits the kit sketch `ASB/ASB.ino`. That sketch reads the
onboard sensors and sends `$ASB,0` and `$ASB,1` over the data port at
9600 baud. Sentence layouts are in
[ASB_PACKET_SPECIFICATION.md](ASB_PACKET_SPECIFICATION.md).

A separate starting point, with on/off switches for each stock sensor,
is [examples/ASB_Framework](../examples/ASB_Framework/). That template
still uses older sentence ids. Match the packet specification before it
flies with the stock flight computer.

## How a sensor is wired into the sketch

1. Include the library and create the sensor object near the other sensors.
2. Add a presence flag to `baseSens` (`bs`) and fields to `baseData` (`bd`).
3. Start it in `setup()` after `Wire.begin()`.
4. Read it from `loop()` only while the flag is true, and clear the flag when the reading is invalid.
5. Send scaled integers in a sentence. Leave `$ASB,0` and `$ASB,1` unchanged.

The board drives its transmit pin only while a sentence is going out,
then releases the pin so the flight computer can use the wire.

## 1. Library and object

```cpp
#include "YourSensorLibrary.h"

YourSensorClass mySensor;
```

Sensors on the shared I2C bus use the `Wire` object started in `setup()`.
Pass `&Wire` when the library asks for it, as `DFRobot_SCD4X` does.

## 2. Readings

`bd` holds the values that go out on the wire:

```cpp
struct baseData {
  float t1, t2, p1, rh, pa1;
  uint32_t aqi, tvoc, eco2;
  uint16_t co2;
  int ensFlags;
};
```

Add a field that matches the reading, for example `float lux;`.

## 3. Presence flag and startup

`bs` records which sensors answered:

```cpp
struct baseSens {
  bool sht3x, sht4x, bmp180, ens160, sps30, scd40;
};
```

Add `bool mySensor` there, then a small starter:

```cpp
static void initMySensor() {
  bs.mySensor = mySensor.begin();
}
```

Call `initMySensor()` from `setup()` after `Wire.begin()`.

Some sensors need a reset and a short wait before they measure. The
ENS160 block in `setup()` is that pattern. The CO2 block can take a
pressure reading from the BMP180 before it starts.

## 4. Read on a timer

`loop()` already polls sensors on 1000 ms, 500 ms, and 200 ms timers.
Add your read next to the sensor it most resembles:

```cpp
void getMySensor() {
  if (!bs.mySensor) return;
  bd.lux = mySensor.readLux();
  if (isnan(bd.lux) || bd.lux < 0.0f) {
    bs.mySensor = false;
  }
}
```

Clear the flag when the device or the bus has failed. If the sensor
simply has no new sample yet, skip the update and leave the flag true.
The ENS160 and SCD4x reads in this sketch do that.

The SPS30 is slower. `pollsps()` waits until the sensor says data is
ready, and it does not send particulate data before that. Copy that
pattern for a sensor that needs a warm-up or a start command.

## 5. Send a sentence

`$ASB,0` and `$ASB,1` stay as they are, so existing software keeps
working. Add a new id and document the field order and scale.

The stock flight computer accepts atmospheric ids `0`, `1`, `3`, and `4`
only. Any other id is ignored until the flight computer sketch is
updated to accept it.

```cpp
void txMyPkt() {
  if (!bs.mySensor) return;
  UCSR0B |= _BV(TXEN0);
  char bu[80];
  sprintf(bu, "$ASB,5,%05d,*", (int)(bd.lux * 100));
  Serial.println(bu);
  delay(30);
  UCSR0B &= ~bit(TXEN0);
}
```

Call it from `loop()` on its own timer, with a gap after the other
sentences, the way `txPM2Pkt()` waits 80 ms after the environment line.
Keep the send short. Do the sensor read in the poll function.

Scale so the receiver can undo it. Temperature in the kit sentences is
`(°C + 40) × 100`. Particulate mass is `µg/m³ × 10`.

## Checklist

| Task | Where |
|------|--------|
| Include and object | Top of `ASB.ino` |
| Reading | `baseData bd` |
| Presence flag | `baseSens bs` |
| Start | `setup()`, after `Wire.begin()` |
| Read | A `get...` function called from `loop()` |
| Send | A new `$ASB` id, with the transmit pin released afterward |
| Flight computer | Accept that id, or the sentence is dropped |

Baud is 9600. Environment and particulate sentences each go out about
every 250 ms.
