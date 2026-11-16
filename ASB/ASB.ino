// EdgeFlyte CubeSat v1 — ASB Firmware 2.1.0
// ==========================================
// ©Copyright 2023-2026 EdgeFlyte, LLC.
// All Rights Reserved.
//
// One sketch. At startup it uses an SHT4x when that sensor answers,
// and otherwise an SHT31.
//

#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_SHT4x.h"
#include "SparkFun_ENS160.h"
#include <Adafruit_BMP085.h>
#include <sps30.h>
#include <DFRobot_SCD4X.h>
#include "version.h"

Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

SparkFun_ENS160 ens;
Adafruit_BMP085 bmp;
struct sps30_measurement m;
DFRobot_SCD4X SCD4X(&Wire, SCD4X_I2C_ADDR);

#define LED     2
#define DEVICE_MODEL "3001"
#define DEVICE_SERIAL_HEX 0x0001

bool ist[128];

struct baseSens {
  bool sht3x, sht4x, bmp180, ens160, sps30, scd40;
};
baseSens bs;

struct baseData {
  float t1, t2, p1, rh, pa1;
  uint32_t aqi, tvoc, eco2;
  uint16_t co2;
  int ensFlags;
};
baseData bd;

uint32_t bootCount = 1;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(1, INPUT);
  UCSR0B &= ~_BV(TXEN0);

  bootLED();

  scani2c();

  UCSR0B |= _BV(TXEN0);
  Serial.print(F("#ASB, Atmospheric Sensing Board v"));
  Serial.print(FIRMWARE_VERSION);
  Serial.println(F(" Initializing..."));

  for (int i = 0; i < 128; i++) {
    if (ist[i]) {
      Serial.print(F("DTC 0x"));
      if (i < 16) Serial.print(F("0"));
      Serial.println(i, HEX);
    }
  }
  delay(100);
  UCSR0B &= ~bit(TXEN0);

  if (ist[0x77]) bs.bmp180 = true;
  // SHT31 default 0x44; SHT4x also commonly 0x44 — probe libraries below
  if (ist[0x44] || ist[0x45]) bs.sht3x = true;
  if (ist[0x53]) bs.ens160 = true;

  sensirion_i2c_init();

  if (sps30_probe() != 0) {
    bs.sps30 = false;
  } else {
    bs.sps30 = true;
    uint16_t ret = sps30_set_fan_auto_cleaning_interval_days(4);
    (void)ret;
    ret = sps30_start_measurement();
    (void)ret;
  }

  if (bmp.begin()) {
    bs.bmp180 = true;
    flashLED();
  }
  if (ens.begin()) {
    bs.ens160 = true;
    flashLED();
  }

  // Prefer SHT4x when available; fall back to SHT31
  bs.sht4x = false;
  bs.sht3x = false;
  if (sht4.begin()) {
    bs.sht4x = true;
    sht4.setPrecision(SHT4X_LOW_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
    flashLED();
  } else if (sht31.begin(0x44) || sht31.begin(0x45)) {
    bs.sht3x = true;
    flashLED();
  }

  if (SCD4X.begin()) {
    bs.scd40 = true;
    flashLED();
  }

  delay(2000);
  if (bs.scd40) {
    SCD4X.enablePeriodMeasure(SCD4X_STOP_PERIODIC_MEASURE);
    SCD4X.setTempComp(4.0);
    if (bs.bmp180) {
      SCD4X.setSensorAltitude(bmp.readAltitude());
    }
    SCD4X.enablePeriodMeasure(SCD4X_START_PERIODIC_MEASURE);
  }

  if (bs.ens160 && ens.setOperatingMode(SFE_ENS160_RESET)) {
    delay(100);
    ens.setOperatingMode(SFE_ENS160_STANDARD);
    delay(100);
    bd.ensFlags = ens.getFlags();
  }

  UCSR0B |= _BV(TXEN0);
  Serial.print(F("#ASB, humidity sensor: "));
  if (bs.sht4x) Serial.println(F("SHT4x"));
  else if (bs.sht3x) Serial.println(F("SHT31"));
  else Serial.println(F("NONE"));
  Serial.println(F("#ASB, Initialized."));
  delay(50);
  UCSR0B &= ~bit(TXEN0);
}

uint32_t lowPollRate = 0, highPollRate = 0, fastPollRate = 0, txEnvTime = 0, txPmTime = 0;

void loop() {
  if (Serial.available() > 0) {
    String rs = Serial.readStringUntil('\n');
    rs.trim();
    if (rs == "#?EF?#") {
      handleEdgeFlyteIdentification();
    }
  }

  if (lowPollRate + 1000UL <= millis()) {
    getHumidityTemp();
    getSCD4x();
    lowPollRate = millis();
  }

  if (highPollRate + 500UL <= millis()) {
    getBMP180();
    highPollRate = millis();
  }

  // SPS30 produces a new sample about once a second, and the ENS160 flags
  // a new TVOC on the same order. Checking every 200 ms picks the sample
  // up as soon as it is ready instead of waiting out the slow poll.
  if (fastPollRate + 200UL <= millis()) {
    pollsps();
    getENS160();
    fastPollRate = millis();
  }

  // Environmental packet carries TVOC. Particulate packet is staggered
  // so the two lines are not stacked on the SATBUS UART.
  if (txEnvTime + 250UL <= millis()) {
    txMainPkt();
    txEnvTime = millis();
  }

  if (txPmTime + 250UL <= millis() && millis() - txEnvTime > 80UL) {
    txPM2Pkt();
    txPmTime = millis();
  }
}


void bootLED() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED, 1);
    delay(50);
    digitalWrite(LED, 0);
    delay(50);
  }
}

void flashLED() {
  digitalWrite(LED, 1);
  delay(100);
  digitalWrite(LED, 0);
  delay(500);
}

void scani2c() {
  for (byte a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    byte e = Wire.endTransmission();
    ist[a] = (e == 0);
  }
}

void txMainPkt() {
  UCSR0B |= _BV(TXEN0);
  digitalWrite(LED, 1);

  char bu[150];
  //                  | T1 | T2 | RH | P1 | PA |AQI |TVOC|ECO2|CO2
  // Altitude: encode altitude_m + 100 (MCB subtracts 100 on decode)
  sprintf(bu, "$ASB,0,%05d,%05d,%04d,%05d,%05d,%01d,%06d,%05d,%05d,*",
          (int)((bd.t1 + 40) * 100), (int)((bd.t2 + 40) * 100), (int)(bd.rh * 10),
          (int)(bd.p1 / 10), (int)(bd.pa1 + 100),
          (int)bd.aqi, (int)bd.tvoc, (int)bd.eco2, (int)bd.co2);
  Serial.println(bu);
  delay(30);
  UCSR0B &= ~bit(TXEN0);
  digitalWrite(LED, 0);
}

void txPM2Pkt() {
  digitalWrite(LED, 1);
  UCSR0B |= _BV(TXEN0);
  char bu[150];
  float nc1 = m.nc_1p0 - m.nc_0p5;
  float nc25 = m.nc_2p5 - m.nc_1p0;
  float nc4 = m.nc_4p0 - m.nc_2p5;
  float nc10 = m.nc_10p0 - m.nc_4p0;
  if (nc1 < 0) nc1 = 0;
  if (nc25 < 0) nc25 = 0;
  if (nc4 < 0) nc4 = 0;
  if (nc10 < 0) nc10 = 0;
  sprintf(bu, "$ASB,1,%05d,%05d,%05d,%05d,%06d,%06d,%06d,%06d,%06d,%03d,*",
          (int)(m.mc_1p0 * 10), (int)(m.mc_2p5 * 10), (int)(m.mc_4p0 * 10), (int)(m.mc_10p0 * 10),
          (int)(m.nc_0p5 * 10), (int)(nc1 * 10), (int)(nc25 * 10),
          (int)(nc4 * 10), (int)(nc10 * 10),
          (int)(m.typical_particle_size * 100));
  Serial.println(bu);
  delay(30);
  UCSR0B &= ~bit(TXEN0);
  digitalWrite(LED, 0);
}

void getHumidityTemp() {
  float t = NAN;
  float h = NAN;

  if (bs.sht4x) {
    sensors_event_t humidity, temp;
    sht4.getEvent(&humidity, &temp);
    t = (float)temp.temperature;
    h = (float)humidity.relative_humidity;
  } else if (bs.sht3x) {
    t = sht31.readTemperature();
    h = sht31.readHumidity();
  }

  if (!isnan(t)) bd.t1 = t;
  if (!isnan(h)) bd.rh = h;
}

void getBMP180() {
  if (!bs.bmp180) return;
  bd.t2 = bmp.readTemperature();
  bd.p1 = bmp.readPressure();
  bd.pa1 = bmp.readAltitude();
}

void getENS160() {
  if (!bs.ens160) return;
  if (ens.checkDataStatus()) {
    bd.aqi = ens.getAQI();
    bd.tvoc = ens.getTVOC();
    bd.eco2 = ens.getECO2();
    bd.ensFlags = ens.getFlags();
  }
}

void pollsps() {
  if (!bs.sps30) return;

  uint16_t data_ready;
  int16_t ret;

  ret = sps30_read_data_ready(&data_ready);
  if (ret < 0) {
    return;
  } else if (!data_ready) {
    return;
  }
  ret = sps30_read_measurement(&m);
  if (ret < 0) {
    return;
  }
  // The SPS30 reports small negative concentrations while the fan settles.
  // Mass and number concentrations are physical quantities and stay at zero.
  if (m.mc_1p0 < 0) m.mc_1p0 = 0;
  if (m.mc_2p5 < 0) m.mc_2p5 = 0;
  if (m.mc_4p0 < 0) m.mc_4p0 = 0;
  if (m.mc_10p0 < 0) m.mc_10p0 = 0;
  if (m.nc_0p5 < 0) m.nc_0p5 = 0;
  if (m.nc_1p0 < 0) m.nc_1p0 = 0;
  if (m.nc_2p5 < 0) m.nc_2p5 = 0;
  if (m.nc_4p0 < 0) m.nc_4p0 = 0;
  if (m.nc_10p0 < 0) m.nc_10p0 = 0;
  if (m.typical_particle_size < 0) m.typical_particle_size = 0;
}

void getSCD4x() {
  if (!bs.scd40) return;
  if (SCD4X.getDataReadyStatus()) {
    DFRobot_SCD4X::sSensorMeasurement_t data;
    SCD4X.readMeasurement(&data);
    bd.co2 = data.CO2ppm;
  }
}

void handleEdgeFlyteIdentification() {
  UCSR0B |= _BV(TXEN0);
  delay(50);
  Serial.print(F("#!EF!:"));
  Serial.print(DEVICE_MODEL);
  Serial.print(F(","));
  if (DEVICE_SERIAL_HEX < 0x1000) Serial.print(F("0"));
  if (DEVICE_SERIAL_HEX < 0x100) Serial.print(F("0"));
  if (DEVICE_SERIAL_HEX < 0x10) Serial.print(F("0"));
  Serial.print(DEVICE_SERIAL_HEX, HEX);
  Serial.print(F(","));
  Serial.print(FIRMWARE_VERSION_INT);
  Serial.print(F(","));
  Serial.print(F("OK"));
  Serial.print(F(","));
  Serial.print(millis() / 1000UL);
  Serial.print(F(","));
  Serial.print(bootCount);
  Serial.print(F(","));
  Serial.print(bd.t1, 1);
  Serial.print(F(","));
  Serial.print(F("0.00"));
  Serial.print(F(","));
  Serial.print(F("0"));
  Serial.print(F(","));
  Serial.print(F("00000000"));
  Serial.println(F("#"));
  delay(50);
  UCSR0B &= ~bit(TXEN0);
}
