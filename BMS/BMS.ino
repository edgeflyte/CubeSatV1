// EdgeFlyte CubeSat v1 — BMS Firmware 2.1.0
// ==========================================
// ©Copyright 2025-2026 EdgeFlyte, LLC.
// All Rights Reserved.
//
// Solar inputs A1 and A2 are configured as inputs so the readings stay stable.
//
// Hardware Requirements:
// - ATmega328PB (MiniCore)
// - Battery voltage sense on A0 (divider ×2 vs 3.3 V Vref)
// - Solar panels on A1 / A2

#include "version.h"
#include <EEPROM.h>

#define VSENSE    A0
#define SP1       A1
#define SP2       A2

// LEDs
#define L1        2
#define L2        3
#define L3        4

// LDO / Channel Outs
#define LDO1      5
#define LDO2      6
#define LDO3      7
#define LDO4      8

// ADC: 10-bit full scale. Vref = DEFAULT (AVCC = 3.3 V on MiniCore BMS).
// A pin can never read above this. An open solar input floats up to the
// rail, so the reported voltage stops at VREF rather than running away.
#define ADC_MAX   1023.0f
#define VREF      3.3f
#define ADC_SAMPLES 8

// Batt Chem (4-cell NiMH pack)
const int NUM_CELLS = 4;
const float MIN_VOLTAGE_PER_CELL = 1.0;
const float MAX_VOLTAGE_PER_CELL = 1.4;

const float vsense_offset = 0;
const float sp1_offset = 0;
const float sp2_offset = 0;

// Device identity
#define DEVICE_MODEL "2001"
#define DEVICE_SERIAL_HEX 0x0001

// Sys Variables
bool activeonBoot_L1 = true;
bool activeonBoot_L2 = true;
bool activeonBoot_L3 = true;
bool activeonBoot_L4 = true;

bool active_LDO1 = false;
bool active_LDO2 = false;
bool active_LDO3 = false;
bool active_LDO4 = false;

bool disabled_LDO1 = false;
bool disabled_LDO2 = false;
bool disabled_LDO3 = false;
bool disabled_LDO4 = false;

bool rememberStates = false;  // Write LDO States to EEPROM

// Boot mask lives in EEPROM so a stored configuration survives power loss.
// Bit 0 = channel 1 ... bit 3 = channel 4. 1 = on after reboot.
#define BMS_EEPROM_MAGIC      0xB5
#define BMS_EEPROM_MAGIC_ADDR 0
#define BMS_EEPROM_MASK_ADDR  1

int err = 0;
uint32_t bootCount = 1;

// System Sensor Variables
float batV, batPct;
float sol1, sol2;

void setup() {
  initPins();

  Serial.begin(9600);
  pinMode(1, INPUT);
  UCSR0B &= ~_BV(TXEN0);

  Serial.print(F("#BMS, Battery Management System v"));
  Serial.print(FIRMWARE_VERSION);
  Serial.println(F(" Initializing..."));
  sequenceLEDS();

  analogReference(DEFAULT);
  loadBootMask();
  if (activeonBoot_L1) setLDO(1, 1);
  if (activeonBoot_L2) setLDO(2, 1);
  if (activeonBoot_L3) setLDO(3, 1);
  if (activeonBoot_L4) setLDO(4, 1);

  Serial.println(F("#BMS, Battery Management System Initialized."));
}


unsigned long sensT = 0, updateT = 0;

void loop() {
  lights();

  if (millis() - sensT >= 200UL) {
    batV = getBatV();
    batPct = getBatPct(batV);
    sol1 = getSol(1);
    sol2 = getSol(2);
    if (batPct <= 5) err = 2;
    else if (batPct <= 20) err = 1;
    else err = 0;
    sensT = millis();
  }


  if (millis() - updateT >= 250UL) {
    digitalWrite(L1, 1);
    UCSR0B |= _BV(TXEN0);
    delay(5);
    char bu[20];
    Serial.print(F("$BMS,"));
    Serial.print(err);
    Serial.print(F(","));
    Serial.print(active_LDO1);
    Serial.print(F(","));
    Serial.print(active_LDO2);
    Serial.print(F(","));
    Serial.print(active_LDO3);
    Serial.print(F(","));
    Serial.print(active_LDO4);
    Serial.print(F(","));
    dtostrf(batV, -1, 2, bu);
    Serial.print(bu);
    Serial.print(F(","));
    dtostrf(batPct, -3, 2, bu);
    Serial.print(bu);
    Serial.print(F(","));
    dtostrf(sol1, -1, 2, bu);
    Serial.print(bu);
    Serial.print(F(","));
    dtostrf(sol2, -1, 2, bu);
    Serial.print(bu);
    Serial.println(F("*"));

    digitalWrite(L1, 0);
    updateT = millis();
    delay(20);
    UCSR0B &= ~bit(TXEN0);
  }


  if (Serial.available() > 0) {
    digitalWrite(L2, 1);
    String rs = Serial.readStringUntil('\n');
    rs.trim();

    if (rs == "#?EF?#") {
      handleEdgeFlyteIdentification();
    } else if (rs.startsWith("&BMS,")) {
      String es = rs.substring(5);
      es.trim();

      if (es.equals("CH1,1")) {
        Serial.println(F("#Set CH1 ON..."));
        if (setLDO(1, 1)) Serial.println(F("#OK"));
      }
      if (es.equals("CH1,0")) {
        Serial.println(F("#Set CH1 OFF..."));
        if (setLDO(1, 0)) Serial.println(F("#OK"));
      }
      if (es.equals("CH2,1")) {
        Serial.println(F("#Set CH2 ON..."));
        if (setLDO(2, 1)) Serial.println(F("#OK"));
      }
      if (es.equals("CH2,0")) {
        Serial.println(F("#Set CH2 OFF..."));
        if (setLDO(2, 0)) Serial.println(F("#OK"));
      }
      if (es.equals("CH3,1")) {
        Serial.println(F("#Set CH3 ON..."));
        if (setLDO(3, 1)) Serial.println(F("#OK"));
      }
      if (es.equals("CH3,0")) {
        Serial.println(F("#Set CH3 OFF..."));
        if (setLDO(3, 0)) Serial.println(F("#OK"));
      }
      if (es.equals("CH4,1")) {
        Serial.println(F("#Set CH4 ON..."));
        if (setLDO(4, 1)) Serial.println(F("#OK"));
      }
      if (es.equals("CH4,0")) {
        Serial.println(F("#Set CH4 OFF..."));
        if (setLDO(4, 0)) Serial.println(F("#OK"));
      }

      if (es.equals("RST,1")) {
        if (rst()) Serial.println(F("#OK"));
      }

      if (es.startsWith("CFG,")) {
        if (storeBootMask(es.substring(4))) Serial.println(F("#OK"));
        else Serial.println(F("#ERR"));
      }
    } else if (rs.length() > 0) {
      Serial.println(F("#BMS?"));
    }
    digitalWrite(L2, 0);
  }
}

// Average the 10-bit converter. The first conversion after a channel
// change is discarded so the sample-and-hold has settled.
float readPinVolts(uint8_t pin) {
  analogRead(pin);
  delayMicroseconds(250);
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
    sum += (uint16_t)analogRead(pin);
    delayMicroseconds(150);
  }
  float counts = (float)sum / (float)ADC_SAMPLES;
  if (counts > ADC_MAX) counts = ADC_MAX;
  float volts = (counts / ADC_MAX) * VREF;
  if (volts < 0.0f) volts = 0.0f;
  if (volts > VREF) volts = VREF;
  return volts;
}

float getBatV() {
  // Sense divider doubles the pack voltage onto a pin that still tops out at VREF.
  float iv = readPinVolts(VSENSE) * 2.0f;
  iv = iv + vsense_offset;
  if (iv < 0.0f) iv = 0.0f;
  if (iv > VREF * 2.0f) iv = VREF * 2.0f;
  iv = (iv < 0.1f) ? 0.0f : iv;
  return iv;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Uses already-sampled voltage to avoid redundant ADC read.
float getBatPct(float v) {
  float bp = mapFloat(v,
                      (MIN_VOLTAGE_PER_CELL * NUM_CELLS),
                      (MAX_VOLTAGE_PER_CELL * NUM_CELLS),
                      0.0f, 100.0f);
  if (bp < 0.0f) bp = 0.0f;
  if (bp > 100.0f) bp = 100.0f;
  return bp;
}

float getSol(int pan) {
  float v = 0.0f;
  if (pan == 1) {
    v = readPinVolts(SP1) + sp1_offset;
  } else if (pan == 2) {
    v = readPinVolts(SP2) + sp2_offset;
  }
  // Open panels float up to the 3.3 V rail. The converter cannot report more.
  if (v < 0.0f) v = 0.0f;
  if (v > VREF) v = VREF;
  v = (v < 0.1f) ? 0.0f : v;
  return v;
}


void initPins() {
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(L3, OUTPUT);
  pinMode(LDO1, OUTPUT);
  pinMode(LDO2, OUTPUT);
  pinMode(LDO3, OUTPUT);
  pinMode(LDO4, OUTPUT);
  // Stable solar readings when panels are open-circuit (from bms misc notes)
  pinMode(SP1, INPUT);
  pinMode(SP2, INPUT);
}

void sequenceLEDS() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(L1, HIGH);
    delay(100);
    digitalWrite(L1, LOW);
    digitalWrite(L2, HIGH);
    delay(100);
    digitalWrite(L2, LOW);
    digitalWrite(L3, HIGH);
    delay(100);
    digitalWrite(L3, LOW);
  }
  digitalWrite(L1, LOW);
  digitalWrite(L2, LOW);
  digitalWrite(L3, LOW);
}

bool setLDO(int ch, bool state) {
  if (ch == 1) {
    digitalWrite(LDO1, state);
    active_LDO1 = state;
    return true;
  }
  if (ch == 2) {
    digitalWrite(LDO2, state);
    active_LDO2 = state;
    return true;
  }
  if (ch == 3) {
    digitalWrite(LDO3, state);
    active_LDO3 = state;
    return true;
  }
  if (ch == 4) {
    digitalWrite(LDO4, state);
    active_LDO4 = state;
    return true;
  }
  return false;
}

bool rst() {
  // Soft reset of LDO channels to boot defaults
  setLDO(1, activeonBoot_L1);
  setLDO(2, activeonBoot_L2);
  setLDO(3, activeonBoot_L3);
  setLDO(4, activeonBoot_L4);
  err = 0;
  return true;
}

void loadBootMask() {
  if (EEPROM.read(BMS_EEPROM_MAGIC_ADDR) != BMS_EEPROM_MAGIC) return;
  uint8_t mask = EEPROM.read(BMS_EEPROM_MASK_ADDR);
  if (mask > 0x0F) return;
  activeonBoot_L1 = mask & 0x01;
  activeonBoot_L2 = mask & 0x02;
  activeonBoot_L3 = mask & 0x04;
  activeonBoot_L4 = mask & 0x08;
  rememberStates = true;
}

bool storeBootMask(const String& digits) {
  if (digits.length() == 0) return false;
  for (unsigned i = 0; i < digits.length(); i++) {
    if (!isDigit(digits[i])) return false;
  }
  int mask = digits.toInt();
  if (mask < 0 || mask > 15) return false;
  EEPROM.update(BMS_EEPROM_MAGIC_ADDR, (uint8_t)BMS_EEPROM_MAGIC);
  EEPROM.update(BMS_EEPROM_MASK_ADDR, (uint8_t)mask);
  activeonBoot_L1 = mask & 0x01;
  activeonBoot_L2 = mask & 0x02;
  activeonBoot_L3 = mask & 0x04;
  activeonBoot_L4 = mask & 0x08;
  rememberStates = true;
  return true;
}


unsigned long l3t = 0;
bool l3s = false;

void lights() {
  if (!err) {
    return;
  }

  unsigned long interval = (err == 2) ? 500UL : 2000UL;
  if (millis() - l3t >= interval) {
    l3s = !l3s;
    digitalWrite(L3, l3s);
    l3t = millis();
  }
}

/**
 * #!EF!:<MODEL>,<SERIAL>,<FW_VERSION>,<STATUS>,<UPTIME>,<BOOT_COUNT>,<TEMP>,<VOLT>,<FREE_RAM>,<CHIP_ID>#
 */
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
  Serial.print((err == 0) ? F("OK") : F("ERR"));
  Serial.print(F(","));
  Serial.print(millis() / 1000UL);
  Serial.print(F(","));
  Serial.print(bootCount);
  Serial.print(F(","));
  Serial.print(F("0.0"));  // No die temp on ATmega328PB
  Serial.print(F(","));
  Serial.print(batV, 2);
  Serial.print(F(","));
  Serial.print(F("0"));    // Free RAM not reported
  Serial.print(F(","));
  Serial.print(F("00000000"));
  Serial.println(F("#"));
  delay(50);
  UCSR0B &= ~bit(TXEN0);
}
