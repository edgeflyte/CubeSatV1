// =============================================================================
//  ASB Framework - Custom Build Template for the CubeSat v1 Air Sensor Board
// =============================================================================
//
//  This file is a starting point for a custom atmospheric-board sketch.
//  The kit firmware is ASB/ASB.ino ($ASB,0 and $ASB,1). This template
//  still sends $ASB,3 and $ASB,4 with older scales. Match
//  ASB/ASB_PACKET_SPECIFICATION.md before it flies with the stock
//  flight computer. You can:
//
//    * Enable / disable individual sensors
//    * Change the packet rate
//    * Add your own sensors (just follow the MY_SENSOR example)
//    * Change what gets transmitted (custom packet format)
//    * Add custom behavior on boot, every loop, or every packet
//
//  See QUICKSTART.md next to this file for a walkthrough.
//
//  Settings and hooks are at the top. Sentence builders and sensor
//  drivers follow.
//
//  -------------------------------------------------------------------------
//  Hardware target: CubeSat v1 atmospheric board. The kit uses an
//  ATmega328PB. This template was written against an ATmega328P.
//  UART: 9600 baud. The transmit pin is enabled only while a sentence
//  is going out, then released so the data wire is free.
//  -------------------------------------------------------------------------

#include "Arduino.h"
#include <Wire.h>
#include <math.h>

// =============================================================================
// 1. USER CONFIGURATION
// =============================================================================
// Edit the values in this section to change firmware behavior. You should not
// need to touch anything below section 1 for most customizations.

// ---- Feature toggles --------------------------------------------------------
// Set to 1 to include a sensor in the build, 0 to exclude it completely.
// Disabling a sensor removes its library include, its variables, and its I2C
// traffic, which saves flash/RAM on the ATmega328P.
#define USE_SHT4X    1   // Temperature + humidity (Adafruit SHT4x)
#define USE_BMP180   1   // Temperature + pressure + altitude (BMP180/BMP085)
#define USE_ENS160   1   // Air quality: AQI / TVOC / eCO2 (SparkFun ENS160)
#define USE_SCD4X    1   // True CO2 (DFRobot SCD4x)
#define USE_SPS30    1   // Particulate matter (Sensirion SPS30)

// ---- Debug output -----------------------------------------------------------
// When 1, a human-readable dump is printed before each packet. Leave at 0 for
// production builds so only packets are transmitted.
#define DEBUG_OUTPUT 0

// ---- Timing -----------------------------------------------------------------
// How often to read sensors and transmit packets (milliseconds).
#define PACKET_INTERVAL_MS          1000

// How often to retry sensors that failed to initialize or dropped off the bus.
#define SENSOR_REINIT_INTERVAL_MS   10000

// SPS30 takes a few seconds to spin up its fan and settle.
#define SPS30_PROBE_INTERVAL_MS     5000
#define SPS30_WARMUP_MS             8000

// ---- UART / Serial ----------------------------------------------------------
#define SERIAL_BAUD 9600

// ---- Packet identifiers -----------------------------------------------------
// This template sends $ASB,3 and $ASB,4. The kit sketch sends $ASB,0
// and $ASB,1. Change these when a custom build should not look like
// either, and teach the flight computer the new ids.
#define PACKET_ID_MAIN "$ASB,3"
#define PACKET_ID_PM   "$ASB,4"


// =============================================================================
// 2. CONDITIONAL INCLUDES / SENSOR INSTANCES
// =============================================================================
// You should not need to edit this section unless you're adding a new sensor
// library. If you do add a sensor, put its #include and its instance here,
// wrapped in a #if USE_YOUR_SENSOR block.

#if USE_SHT4X
  #include "Adafruit_SHT4x.h"
  Adafruit_SHT4x sht4 = Adafruit_SHT4x();
  bool sht4_ok = false;
#endif

#if USE_BMP180
  #include <Adafruit_BMP085.h>
  Adafruit_BMP085 bmp;
  bool bmp_ok = false;
#endif

#if USE_ENS160
  #include "SparkFun_ENS160.h"
  SparkFun_ENS160 ens;
  bool ens_ok = false;
#endif

#if USE_SCD4X
  #include <DFRobot_SCD4X.h>
  DFRobot_SCD4X SCD4X(&Wire, SCD4X_I2C_ADDR);
  bool scd4x_ok = false;
#endif

#if USE_SPS30
  #include <sps30.h>
  struct sps30_measurement m;
  bool sps30_ready = false;
  uint32_t sps30_start = 0;
  uint32_t sps30_last_probe = 0;
#endif


// =============================================================================
// 3. SHARED DATA
// =============================================================================
// All sensor readings collect into one struct so the packet builder stays
// simple. Add fields here if you add a new sensor.

struct SensorData {
  float    t1;    // SHT4x temperature (deg C)
  float    t2;    // BMP180 temperature (deg C)
  float    rh;    // SHT4x humidity (%RH)
  float    p1;    // BMP180 pressure (Pa)
  float    pa1;   // BMP180 altitude (m)
  uint32_t aqi;   // ENS160 air quality index (1-5)
  uint32_t tvoc;  // ENS160 TVOC (ppb)
  uint32_t eco2;  // ENS160 eCO2 (ppm)
  uint16_t co2;   // SCD4x CO2 (ppm), falls back to eCO2
};
SensorData data;

// Timekeeping for the main loop
static uint32_t packet_time = 0;
static uint32_t sensor_reinit_time = 0;


// =============================================================================
// 4. USER HOOKS
// =============================================================================
// These are the easiest places to add your own behavior without having to
// understand the rest of the firmware. The stock implementations are empty.

// Called once after all sensors are initialized, before the main loop starts.
// Good place to: set pin modes, print a banner, initialize an SD card, etc.
static void onBoot() {
  // Example:
  // pinMode(LED_BUILTIN, OUTPUT);
  // Serial.println("# ASB custom build ready");
}

// Called every loop iteration, BEFORE sensors are read or packets are sent.
// Keep this fast (no delay()) -- it runs hundreds of times per second.
static void onTick() {
  // Example: toggle a heartbeat LED, poll a button, etc.
}

// Called once per PACKET_INTERVAL_MS, AFTER sensors have been read but
// BEFORE packets are transmitted. Good place to modify `data` (e.g. apply
// calibration offsets) or trigger auxiliary outputs.
static void onBeforeTransmit() {
  // Example:
  // data.t1 += 0.5f;  // calibration offset
}

// Called once per PACKET_INTERVAL_MS, AFTER packets have been transmitted.
// Good place to: log to SD, blink an LED, etc.
static void onAfterTransmit() {
  // Example:
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}


// =============================================================================
// 5. SETUP / LOOP
// =============================================================================

void setup() {
  Wire.begin();
  Serial.begin(SERIAL_BAUD);

  uartTxOff();

  initAllSensors();

  onBoot();
}

void loop() {
  const uint32_t now = millis();

  onTick();

#if USE_SPS30
  updateSPS30State(now);
#endif

  if ((now - sensor_reinit_time) >= SENSOR_REINIT_INTERVAL_MS) {
    sensor_reinit_time = now;
    reinitMissingSensors();
  }

  if ((now - packet_time) >= PACKET_INTERVAL_MS) {
    packet_time = now;
    readSensors();
    onBeforeTransmit();
    transmitPackets();
    onAfterTransmit();
  }
}


// =============================================================================
// 6. SENSOR INIT
// =============================================================================
// Each sensor has an initXxx() function that sets its _ok flag on success.
// reinitMissingSensors() retries any that failed -- this is how hot-plug
// recovery works.

#if USE_SHT4X
static void initSHT4x() {
  if (sht4.begin()) {
    sht4.setPrecision(SHT4X_LOW_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
    sht4_ok = true;
  }
}
#endif

#if USE_BMP180
static void initBMP() {
  if (bmp.begin()) bmp_ok = true;
}
#endif

#if USE_ENS160
static void initENS160() {
  if (ens.begin()) {
    ens.setOperatingMode(SFE_ENS160_RESET);
    delay(250);
    ens.setOperatingMode(SFE_ENS160_STANDARD);
    ens_ok = true;
  }
}
#endif

#if USE_SCD4X
static void initSCD4X() {
  if (SCD4X.begin()) {
    SCD4X.enablePeriodMeasure(SCD4X_STOP_PERIODIC_MEASURE);
    SCD4X.setTempComp(4.0);
    #if USE_BMP180
      if (bmp_ok) SCD4X.setSensorAltitude(bmp.readAltitude());
    #endif
    SCD4X.enablePeriodMeasure(SCD4X_START_PERIODIC_MEASURE);
    scd4x_ok = true;
  }
}
#endif

#if USE_SPS30
// Probe the SPS30 and start a measurement (spins up the fan).
// Returns true if a measurement was successfully started.
static bool trySPS30Start() {
  if (sps30_probe() != 0) return false;
  sps30_set_fan_auto_cleaning_interval_days(4);
  if (sps30_start_measurement() != 0) return false;
  sps30_start = millis();
  return true;
}

// Drive the SPS30 state machine: probe until present, then warm up, then
// flip sps30_ready so the main reader starts pulling measurements.
static void updateSPS30State(uint32_t now) {
  if (sps30_ready) return;

  if (sps30_start != 0 && (now - sps30_start) > SPS30_WARMUP_MS) {
    sps30_ready = true;
  } else if (sps30_start == 0 &&
             (now - sps30_last_probe) >= SPS30_PROBE_INTERVAL_MS) {
    sps30_last_probe = now;
    if (trySPS30Start()) sps30_start = now;
  }
}
#endif

static void initAllSensors() {
#if USE_SPS30
  sensirion_i2c_init();
#endif
#if USE_SHT4X
  initSHT4x();
#endif
#if USE_BMP180
  initBMP();
#endif
#if USE_ENS160
  initENS160();
#endif
#if USE_SCD4X
  initSCD4X();
#endif
#if USE_SPS30
  if (trySPS30Start()) sps30_last_probe = millis();
#endif
}

static void reinitMissingSensors() {
#if USE_SHT4X
  if (!sht4_ok)  initSHT4x();
#endif
#if USE_BMP180
  if (!bmp_ok)   initBMP();
#endif
#if USE_ENS160
  if (!ens_ok)   initENS160();
#endif
#if USE_SCD4X
  if (!scd4x_ok) initSCD4X();
#endif
}


// =============================================================================
// 7. SENSOR READ
// =============================================================================
// Pulls one reading per enabled sensor into the shared `data` struct.
// If a sensor returns a clearly bad value, its _ok flag is cleared so
// reinitMissingSensors() will try to bring it back later.

void readSensors() {
#if USE_SHT4X
  if (sht4_ok) {
    sensors_event_t humidity, temp;
    sht4.getEvent(&humidity, &temp);
    data.t1 = temp.temperature;
    data.rh = humidity.relative_humidity;
    if (isnan(data.t1) || isnan(data.rh)) sht4_ok = false;
  }
#endif

#if USE_BMP180
  if (bmp_ok) {
    data.t2  = bmp.readTemperature();
    data.p1  = bmp.readPressure();
    data.pa1 = bmp.readAltitude();
    if (isnan(data.t2) || data.t2 < -40.0f || data.t2 > 85.0f) bmp_ok = false;
  }
#endif

#if USE_ENS160
  if (ens_ok && ens.checkDataStatus()) {
    data.aqi  = ens.getAQI();
    data.tvoc = ens.getTVOC();
    data.eco2 = ens.getECO2();
  }
#endif

#if USE_SCD4X
  if (scd4x_ok && SCD4X.getDataReadyStatus()) {
    DFRobot_SCD4X::sSensorMeasurement_t scd_data;
    SCD4X.readMeasurement(&scd_data);
    if (scd_data.CO2ppm >= 400 && scd_data.CO2ppm <= 5000) {
      data.co2 = scd_data.CO2ppm;
    } else {
      data.co2 = (data.eco2 > 400) ? data.eco2 : 410;
    }
  } else {
    data.co2 = (data.eco2 > 400) ? data.eco2 : 410;
  }
#else
  data.co2 = (data.eco2 > 400) ? data.eco2 : 410;
#endif

#if USE_SPS30
  if (sps30_ready) {
    uint16_t data_ready;
    int16_t ret = sps30_read_data_ready(&data_ready);
    if (ret < 0) {
      sps30_ready = false;
      sps30_start = 0;
    } else if (data_ready && sps30_read_measurement(&m) < 0) {
      sps30_ready = false;
      sps30_start = 0;
    }
  }
#endif
}


// =============================================================================
// 8. PACKET TRANSMISSION
// =============================================================================
// The main packet format matches the stock firmware so existing MCB
// parsers keep working. If you want a totally different packet format, change
// transmitMainPacket() -- that's the recommended place for custom protocols.

static void uartTxOn()  { UCSR0B |=  _BV(TXEN0); }
static void uartTxOff() { UCSR0B &= ~_BV(TXEN0); }

void transmitPackets() {
  uartTxOn();

#if DEBUG_OUTPUT
  printDebugData();
#endif

  transmitMainPacket();

#if USE_SPS30
  if (sps30_ready) transmitPMPacket();
#endif

  delay(100); // allow the last byte to shift out before disabling TX
  uartTxOff();
}

// ---- Main environmental packet ---------------------------------------------
// Format: $ASB,3,T1,T2,RH,P1,PA,AQI,TVOC,ECO2,CO2,*
// See ASB_PACKET_SPECIFICATION.md for scaling and field details.
void transmitMainPacket() {
  Serial.print(PACKET_ID_MAIN);                    Serial.print(",");
  Serial.print((int)((data.t1 + 40) * 100));       Serial.print(",");
  Serial.print((int)((data.t2 + 40) * 100));       Serial.print(",");
  Serial.print((int)(data.rh * 10));               Serial.print(",");
  Serial.print((int)(data.p1 / 10));               Serial.print(",");
  Serial.print((int)(data.pa1));                   Serial.print(",");
  Serial.print((int)data.aqi);                     Serial.print(",");
  Serial.print((int)data.tvoc);                    Serial.print(",");
  Serial.print((int)data.eco2);                    Serial.print(",");
  Serial.print((int)data.co2);
  Serial.println(",*");
}

#if USE_SPS30
// ---- Particulate matter packet ---------------------------------------------
// Format: $ASB,4,PM1,PM25,PM4,PM10,NC05,NC1,NC25,NC4,NC10,TPS,*
void transmitPMPacket() {
  Serial.print(PACKET_ID_PM);                                Serial.print(",");
  Serial.print((int)(m.mc_1p0  * 100));                      Serial.print(",");
  Serial.print((int)(m.mc_2p5  * 100));                      Serial.print(",");
  Serial.print((int)(m.mc_4p0  * 100));                      Serial.print(",");
  Serial.print((int)(m.mc_10p0 * 100));                      Serial.print(",");
  Serial.print((int)(m.nc_0p5  / 10));                       Serial.print(",");
  Serial.print((int)((m.nc_1p0  - m.nc_0p5) / 10));          Serial.print(",");
  Serial.print((int)((m.nc_2p5  - m.nc_1p0) / 10));          Serial.print(",");
  Serial.print((int)((m.nc_4p0  - m.nc_2p5) / 10));          Serial.print(",");
  Serial.print((int)((m.nc_10p0 - m.nc_4p0) / 10));          Serial.print(",");
  Serial.print((int)(m.typical_particle_size * 100));
  Serial.println(",*");
}
#endif


// =============================================================================
// 9. DEBUG OUTPUT (optional)
// =============================================================================
// Enabled by setting DEBUG_OUTPUT to 1 at the top of the file.

#if DEBUG_OUTPUT
void printDebugData() {
  Serial.println("=== SENSOR DATA DEBUG ===");
  #if USE_SHT4X
    Serial.print("SHT4x Temperature: "); Serial.print(data.t1); Serial.println(" C");
    Serial.print("Humidity:          "); Serial.print(data.rh); Serial.println(" %RH");
  #endif
  #if USE_BMP180
    Serial.print("BMP180 Temperature: "); Serial.print(data.t2);  Serial.println(" C");
    Serial.print("Pressure:           "); Serial.print(data.p1);  Serial.println(" Pa");
    Serial.print("Altitude:           "); Serial.print(data.pa1); Serial.println(" m");
  #endif
  #if USE_ENS160
    Serial.print("AQI:  "); Serial.println(data.aqi);
    Serial.print("TVOC: "); Serial.print(data.tvoc); Serial.println(" ppb");
    Serial.print("eCO2: "); Serial.print(data.eco2); Serial.println(" ppm");
  #endif
  #if USE_SCD4X
    Serial.print("CO2:  "); Serial.print(data.co2);  Serial.println(" ppm");
  #endif
  #if USE_SPS30
    if (sps30_ready) {
      Serial.println("--- PM ---");
      Serial.print("PM1.0:  "); Serial.print(m.mc_1p0);  Serial.println(" ug/m3");
      Serial.print("PM2.5:  "); Serial.print(m.mc_2p5);  Serial.println(" ug/m3");
      Serial.print("PM4.0:  "); Serial.print(m.mc_4p0);  Serial.println(" ug/m3");
      Serial.print("PM10.0: "); Serial.print(m.mc_10p0); Serial.println(" ug/m3");
    } else {
      Serial.println("--- SPS30 NOT READY ---");
    }
  #endif
  Serial.println("=== PACKET OUTPUT ===");
}
#endif


// =============================================================================
// 10. EXAMPLE: ADDING YOUR OWN SENSOR
// =============================================================================
// To add a new sensor, copy this template and adapt:
//
//   #define USE_MYSENSOR 1
//
//   #if USE_MYSENSOR
//     #include <MySensorLib.h>
//     MySensor mySensor;
//     bool mysensor_ok = false;
//
//     static void initMySensor() {
//       if (mySensor.begin()) mysensor_ok = true;
//     }
//
//     static void readMySensor() {
//       if (!mysensor_ok) return;
//       float v = mySensor.read();
//       // Store v somewhere useful (add a field to SensorData, or
//       // print it in transmitMainPacket / a new packet type).
//     }
//   #endif
//
// Then call initMySensor() from initAllSensors(), readMySensor() from
// readSensors(), and (if applicable) add an _ok check to
// reinitMissingSensors(). Put any output in transmitMainPacket() or add a
// new packet function (remember to also update your MCB parser
// if you change packet formats).
