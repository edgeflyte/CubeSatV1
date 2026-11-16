/*
 * ============================================================================
 * MCB firmware 2.1.0 — EdgeFlyte CubeSat v1
 * ============================================================================
 * 
 * Main Control Board (MCB) firmware for the EdgeFlyte CubeSat Educational Kit
 * 
 * Hardware Platform: Raspberry Pi Pico (RP2040)
 * 
 * Onboard Sensors:
 *   - SHT41/SHT31: Temperature & Humidity
 *   - MPU9250: 9-axis IMU (Accelerometer, Gyroscope, Magnetometer)
 *   - GPS Module: Position & Time (UART)
 * 
 * Communication Modules:
 *   - NRF24L01: 2.4GHz Radio Transceiver (Primary Telemetry)
 *   - AT-09 BLE: Bluetooth Low Energy (Local Configuration)
 * 
 * Expansion Modules (via SATBUS v1):
 *   - ASB: Atmospheric Sensing Board
 *   - BMS: Battery Management System
 *   - SCC: Solar Charge Controller
 * 
 * Created for educational purposes - designed to teach:
 *   - Embedded systems programming
 *   - Satellite communication protocols
 *   - Sensor integration and data processing
 *   - State machine design patterns
 *   - Real-time system concepts
 * 
 * ============================================================================
 */

// ============================================================================
// LIBRARY INCLUDES
// ============================================================================

#if !defined(ARDUINO_ARCH_RP2040)
#error "MCB firmware targets RP2040 (Raspberry Pi Pico). In Arduino IDE select Tools > Board > Raspberry Pi Pico under 'Raspberry Pi RP2040 Boards' (Earle Philhower arduino-pico core). Do not use Arduino Mbed OS RP2040."
#endif

#include <Wire.h>              // I2C Communication
#include <SPI.h>               // SPI Communication (NRF24L01, SD Card)
#include <EEPROM.h>            // Configuration storage
#include <pico/unique_id.h>   // RP2040 unique board ID (safe wrapper)
#include <RF24.h>              // NRF24L01 Radio Library
#include <Adafruit_SHT4x.h>    // SHT4x Temperature/Humidity Sensor
#include <Adafruit_SHT31.h>    // SHT31 Temperature/Humidity Sensor (fallback)
#include "FastIMU.h"           // MPU9250 9-axis IMU (FastIMU library)
#include <MicroNMEA.h>         // GPS NMEA Parser


// Local Project Headers
#include "SatbusParser.h"      // SATBUS Protocol Parsers
#include "TelemetryRadio.h"    // Telemetry Dictionary & Transmission
#include "CommandHandler.h"    // Command Processing
#include "DataLogger.h"        // SD Card Data Logging
#include "ParameterHandler.h"  // #G Protocol Parameter Get/Set
#include "version.h"           // Single-sourced firmware version (2.1.0)

// ============================================================================
// PIN DEFINITIONS - Raspberry Pi Pico
// ============================================================================

// I2C Bus
#define PIN_I2C_SDA         12
#define PIN_I2C_SCL         13

// SPI Bus
#define PIN_SPI_MOSI        19
#define PIN_SPI_MISO        16
#define PIN_SPI_SCK         18

// NRF24L01 Radio
#define PIN_NRF_CE          11
#define PIN_NRF_CSN         15
#define PIN_NRF_IRQ         14

// SD Card
#define PIN_SD_CS           17

// UART Interfaces
#define PIN_GPS_TX          0
#define PIN_GPS_RX          1
#define PIN_SATBUS_TX       4
#define PIN_SATBUS_RX       5

// Status Indicators
#define PIN_LED_BUILTIN     25
#define PIN_LED1            6
#define PIN_LED2            7
#define PIN_LED3            8
#define PIN_LED_STATUS      PIN_LED_BUILTIN

// Control Pins
#define PIN_RBF_SENSE       2
#define PIN_BUZZER          3
#define PIN_BLE_EN          20

// Analog Inputs
#define PIN_ADC0            26
#define PIN_ADC1            27
#define PIN_ADC2            28

// Additional GPIO
#define PIN_GP9             9
#define PIN_GP10            10
#define PIN_GP21            21
#define PIN_GP22            22

// ============================================================================
// SERIAL PORT DEFINITIONS
// ============================================================================

#define Serial_GPS      Serial1
#define Serial_SATBUS   Serial2
#define Serial_BLE      Serial

// ============================================================================
// SYSTEM CONFIGURATION
// ============================================================================

// Default Timing Values (used if EEPROM not configured)
// These are now stored in SystemConfig and loaded from EEPROM
#define DEFAULT_TELEMETRY_FAST      1000
#define DEFAULT_TELEMETRY_SLOW      5000
#define DEFAULT_SENSOR_INTERVAL     500
#define DEFAULT_SATBUS_POLL         100
#define DEFAULT_HEARTBEAT           1000
#define DEFAULT_GPS_INTERVAL        1250
#define WATCHDOG_TIMEOUT            8000

// Default Radio Configuration (used if EEPROM not configured)
#define DEFAULT_RADIO_CHANNEL       62
#define DEFAULT_RADIO_PA_LEVEL      RF24_PA_LOW
#define DEFAULT_RADIO_DATA_RATE     RF24_1MBPS
#define DEFAULT_RADIO_PAYLOAD       32
#define RADIO_RETRY_COUNT           3
#define RADIO_RETRY_DELAY           15
#define RADIO_REINIT_INTERVAL_MS    15000   // Min interval between re-init attempts when radio is down
#define RADIO_REINIT_PERIODIC_MS   15000   // Periodic re-init when up (keep radio from drifting into bad state)

// I2C Configuration
#define I2C_CLOCK_SPEED             400000

// Device Identification (default serial can be changed via #G protocol).
// The authoritative default serial is SystemConfig::serialNumber (0x00000043)
// in ParameterHandler.h. Firmware version is single-sourced in version.h
// (FIRMWARE_VERSION / FIRMWARE_VERSION_INT).

// EdgeFlyte Identification
#define EF_MODEL_NUMBER             "1001"

// EEPROM Configuration
#define EEPROM_SIZE                 512

// ============================================================================
// SYSTEM STATE MACHINE
// ============================================================================

enum class SystemMode : uint8_t {
    BOOT,
    SAFE,
    NOMINAL,
    LOW_POWER,
    SCIENCE,
    EMERGENCY
};

// SystemStatus is defined in TelemetryRadio.h

// ============================================================================
// GLOBAL OBJECTS & VARIABLES
// ============================================================================

// Hardware Objects - Temperature/Humidity (supports both SHT41 and SHT31)
Adafruit_SHT4x      sht4x;
Adafruit_SHT31      sht31;
bool                useSHT41 = false;   // True if SHT41 detected, false for SHT31

// Hardware Objects - IMU (FastIMU library for MPU9250)
MPU9250             imu;
calData             imuCalibration = {0};   // IMU calibration data
uint8_t             imuI2cAddress = 0;
AccelData           accelData;
GyroData            gyroData;
MagData             magData;
// Removed from every gyro sample. Captured at boot while the CubeSat is still.
float               gyroBiasX = 0.0f;
float               gyroBiasY = 0.0f;
float               gyroBiasZ = 0.0f;

// Hardware Objects - GPS & Radio
char                nmeaBuffer[100];
MicroNMEA           nmea(nmeaBuffer, sizeof(nmeaBuffer));
RF24                radio(PIN_NRF_CE, PIN_NRF_CSN);

// Module Objects
SatbusParser        satbus;
TelemetryRadio      telemetry;
CommandHandler      commands;
MeshNetwork         mesh;
DataLogger          dataLog;
static const char*  logFilename = nullptr;  // Pointer to DataLogger's filename

// Configuration & Parameter Handler
SystemConfig        sysConfig;          // Persistent configuration
ParameterHandler    params;             // #G protocol handler

// System State
SystemMode          currentMode = SystemMode::BOOT;
SystemStatus        systemStatus = {0};
uint32_t            bootCount = 0;
uint32_t            uptimeSeconds = 0;

// Timing Variables (non-blocking delays using millis())
uint32_t            lastSensorRead = 0;
uint32_t            lastTelemetryTx = 0;
uint32_t            lastSatbusPoll = 0;
uint32_t            lastAsbTx = 0;
uint32_t            lastBmsTx = 0;
uint32_t            lastHeartbeat = 0;
uint32_t            lastSecondTick = 0;
uint32_t            lastGPSRead = 0;
uint32_t            lastRadioReinitAttempt = 0;  // For rate-limiting radio re-init when down
uint32_t            lastRadioPeriodicReinit = 0; // For periodic re-init when up (reconnection recovery)

// UART Passthrough State
bool                uartPassthroughActive = false;
uint8_t             uartPassthroughTarget = 0;  // 1=GPS, 2=SATBUS

// Sensor Data Storage
SensorData          localSensors;     // MCB onboard sensors
ASBData             asbData;          // Atmospheric Sensing Board
BMSData             bmsData;          // Battery Management System
SCCData             sccData;          // Solar Charge Controller
GPSData             gpsData;          // GPS Position/Time

// Mesh short-message reassembly (0x0900)
struct MsgAssembly {
    uint32_t fromId;
    uint8_t  msgId;
    uint8_t  fragCount;
    uint8_t  receivedMask;
    char     text[TELEM_MSG_MAX_CHARS + 1];
    uint32_t lastMs;
    bool     active;
};
#define MSG_LOG_ENTRIES 8
struct MsgLogEntry {
    uint32_t fromId;
    uint8_t  msgId;
    uint8_t  flags;
    char     text[TELEM_MSG_MAX_CHARS + 1];
    uint32_t timestamp;
    bool     valid;
};
MsgAssembly msgAssembly;
MsgLogEntry msgLog[MSG_LOG_ENTRIES];
uint8_t msgLogNext = 0;

void handleMeshMessage(TelemetryPacket& packet);

// ============================================================================
// SETUP FUNCTION - Runs Once at Startup
// ============================================================================

void setup() {
    // -----------------------------------------------------------------------
    // Phase 1: Basic Hardware Initialization (GPIO)
    // -----------------------------------------------------------------------
    
    // Initialize status LEDs for visual feedback
    pinMode(PIN_LED_BUILTIN, OUTPUT);
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);
    digitalWrite(PIN_LED_BUILTIN, HIGH);  // LED on during boot
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PIN_LED2, LOW);
    digitalWrite(PIN_LED3, LOW);
    
    // Initialize control pins
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_RBF_SENSE, INPUT);  // RBF pin (no pullup - external circuit)
    pinMode(PIN_BLE_EN, OUTPUT);
    pinMode(PIN_SD_CS, OUTPUT);
    pinMode(PIN_NRF_CSN, OUTPUT);
    pinMode(PIN_NRF_CE, OUTPUT);
    pinMode(PIN_NRF_IRQ, INPUT);
    
    // Deselect SPI devices initially
    digitalWrite(PIN_SD_CS, HIGH);
    digitalWrite(PIN_NRF_CSN, HIGH);
    
    // -----------------------------------------------------------------------
    // Phase 2: Serial Communications
    // -----------------------------------------------------------------------
    
    Serial.begin(115200);
    delay(100);
    
    Serial_GPS.begin(9600);
    
    Serial_SATBUS.setRX(PIN_SATBUS_RX);
    Serial_SATBUS.setTX(PIN_SATBUS_TX);
    Serial_SATBUS.begin(9600);
    
    // -----------------------------------------------------------------------
    // Phase 3: SPI Bus Initialization
    // -----------------------------------------------------------------------
    
    SPI.setRX(PIN_SPI_MISO);
    SPI.setTX(PIN_SPI_MOSI);
    SPI.setSCK(PIN_SPI_SCK);
    SPI.begin();
    
    // -----------------------------------------------------------------------
    // Phase 4: I2C Bus Initialization
    // -----------------------------------------------------------------------
    
    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.begin();
    Wire.setClock(I2C_CLOCK_SPEED);
    
    // -----------------------------------------------------------------------
    // Phase 5: EEPROM & Configuration
    // -----------------------------------------------------------------------
    
    EEPROM.begin(EEPROM_SIZE);
    
    // Initialize parameter handler and load configuration from EEPROM
    params.begin(&sysConfig);
    
    // Set up parameter handler data pointers
    params.setSensorDataPtr(&localSensors);
    params.setGPSDataPtr(&gpsData);
    params.setSystemStatusPtr(&systemStatus);
    params.setTelemetryPtr(&telemetry);
    params.setDataLoggerPtr(&dataLog);
    params.setMeshPtr(&mesh);
    params.setRadioPtr(&radio);
    params.setUptimePtr(&uptimeSeconds);
    params.setBootCountPtr(&bootCount);
    
    // Set up SATBUS module data pointers
    params.setBMSDataPtr(&bmsData);
    params.setASBDataPtr(&asbData);
    params.setSCCDataPtr(&sccData);
    
    // Load and increment boot count
    loadBootCount();
    incrementBootCount();
    
    // -----------------------------------------------------------------------
    // Phase 6: Print Boot Banner
    // -----------------------------------------------------------------------
    
    printBootBanner();
    
    // -----------------------------------------------------------------------
    // Phase 7: Sensor Initialization
    // -----------------------------------------------------------------------
    
    initializeSensors();
    
    // -----------------------------------------------------------------------
    // Phase 8: Radio Initialization
    // -----------------------------------------------------------------------
    
    initializeRadio(false);  // false = boot init (verbose)
    lastRadioPeriodicReinit = millis();  // Start periodic re-init timer after first init
    
    // -----------------------------------------------------------------------
    // Phase 9: Module Initialization
    // -----------------------------------------------------------------------
    
    // Initialize SATBUS parser
    satbus.begin(&Serial_SATBUS);
    Serial.println(F("[BOOT] SATBUS parser initialized"));
    
    // Initialize telemetry system (use serial from config)
    telemetry.begin(&radio, sysConfig.serialNumber);
    telemetry.setLedPins(PIN_LED1, PIN_LED3);  // LED1 = success, LED3 = failure
    Serial.println(F("[BOOT] Telemetry system initialized"));
    
    // Set telemetry statistics pointers for parameter handler
    params.setPacketsSentPtr(telemetry.getPacketsSentPtr());
    params.setPacketsFailedPtr(telemetry.getPacketsFailedPtr());
    
    // Initialize command handler (use serial from config)
    commands.begin(sysConfig.serialNumber);
    Serial.println(F("[BOOT] Command handler initialized"));
    
    // Initialize mesh network (use serial from config)
    mesh.begin(&radio, &telemetry, sysConfig.serialNumber);
    mesh.setSDPin(PIN_SD_CS);
    mesh.setRelayEnabled(sysConfig.meshRelayEnabled);
    mesh.setLogToSD(sysConfig.meshLogToSD);
    Serial.println(F("[BOOT] Mesh network initialized"));
    
    // Set parameter handler mode pointer
    params.setModePtr((uint8_t*)&currentMode);
    
    // Initialize data logger (use config settings)
    if (dataLog.begin(PIN_SD_CS, bootCount)) {
        dataLog.setEnabled(sysConfig.loggingEnabled);
        dataLog.setSensorInterval(sysConfig.logInterval);
        dataLog.setGPSInterval(sysConfig.logInterval);
        logFilename = dataLog.getFilename();  // Store pointer for parameter handler
        params.setLogFilenamePtr(&logFilename);
        Serial.print(F("[BOOT] Data logger initialized: "));
        Serial.println(dataLog.getFilename());
    } else {
        Serial.println(F("[BOOT] Data logger: SD card not available"));
    }
    
    // Initialize BLE module
    initializeBLE();
    
    // -----------------------------------------------------------------------
    // Phase 10: Check RBF State and Set Initial Mode
    // -----------------------------------------------------------------------
    
    systemStatus.rbfInserted = (digitalRead(PIN_RBF_SENSE) == LOW);
    
    if (systemStatus.rbfInserted) {
        currentMode = SystemMode::SAFE;
        Serial.println(F("[BOOT] RBF inserted - entering SAFE mode"));
    } else {
        currentMode = SystemMode::NOMINAL;
        Serial.println(F("[BOOT] RBF removed - entering NOMINAL mode"));
    }
    
    // -----------------------------------------------------------------------
    // Phase 11: Boot Complete
    // -----------------------------------------------------------------------
    
    tone(PIN_BUZZER, 1000, 100);
    
    digitalWrite(PIN_LED_BUILTIN, LOW);  // LED off - boot complete
    Serial.print(F("[BOOT] System ready. Boot count: "));
    Serial.println(bootCount);
    Serial.println(F("[BOOT] ===== BOOT SEQUENCE COMPLETE ====="));
    Serial.println();
}

// ============================================================================
// MAIN LOOP - Runs Continuously
// ============================================================================

void loop() {
    uint32_t currentMillis = millis();
    
    // -----------------------------------------------------------------------
    // Task 0: UART Passthrough Mode (bypasses normal operations)
    // -----------------------------------------------------------------------
    
    if (uartPassthroughActive) {
        handleUARTPassthrough();
        return;  // Skip all other tasks in passthrough mode
    }
    
    // -----------------------------------------------------------------------
    // Task 1: Update System Uptime
    // -----------------------------------------------------------------------
    
    if (currentMillis - lastSecondTick >= 1000) {
        lastSecondTick = currentMillis;
        uptimeSeconds++;
    }
    
    // -----------------------------------------------------------------------
    // Task 2: Process Incoming SATBUS Data
    // -----------------------------------------------------------------------
    
    if (currentMillis - lastSatbusPoll >= sysConfig.satbusPollInterval) {
        lastSatbusPoll = currentMillis;
        processSatbusData();
    }
    
    // -----------------------------------------------------------------------
    // Task 3: Process GPS Data
    // -----------------------------------------------------------------------
    
    processGPSData();
    
    // -----------------------------------------------------------------------
    // Task 4: Read Local Sensors
    // -----------------------------------------------------------------------
    
    if (currentMillis - lastSensorRead >= sysConfig.sensorReadInterval) {
        lastSensorRead = currentMillis;
        readLocalSensors();
    }
    
    // -----------------------------------------------------------------------
    // Task 5: Process Incoming Commands
    // -----------------------------------------------------------------------
    
    processIncomingCommands();
    
    // -----------------------------------------------------------------------
    // Task 6: Transmit Telemetry (continual broadcast for link reconnection)
    // -----------------------------------------------------------------------
    
    uint32_t telemetryInterval = getTelemetryInterval();
    if (currentMillis - lastTelemetryTx >= telemetryInterval) {
        lastTelemetryTx = currentMillis;
        transmitTelemetry();  // Broadcast sent without ACK so link can reconnect when restored
    }

    // ASB particulate and TVOC are sampled faster than the rest of telemetry.
    // Send that pair on its own 250 ms cadence so the ground station plot
    // follows the sensor instead of the 1 s system broadcast.
    if (systemStatus.radioNRF && systemStatus.satbusASB &&
        currentMillis - lastAsbTx >= 250) {
        lastAsbTx = currentMillis;
        telemetry.sendASBData(asbData);
    }

    // Battery and solar are sampled on the BMS every 200 ms. Radio them on
    // the same 250 ms cadence so the power plot is not stuck on the 1 s broadcast.
    if (systemStatus.radioNRF && systemStatus.satbusBMS &&
        currentMillis - lastBmsTx >= 250) {
        lastBmsTx = currentMillis;
        telemetry.sendBMSData(bmsData);
    }
    
    // -----------------------------------------------------------------------
    // Task 7: Update Status LED (Heartbeat)
    // -----------------------------------------------------------------------
    
    if (currentMillis - lastHeartbeat >= sysConfig.heartbeatInterval) {
        lastHeartbeat = currentMillis;
        updateHeartbeat();
    }
    
    // -----------------------------------------------------------------------
    // Task 8: Check System Health
    // -----------------------------------------------------------------------
    
    checkSystemHealth();
    
    // -----------------------------------------------------------------------
    // Task 9: Update Mesh Network
    // -----------------------------------------------------------------------
    
    mesh.update();
    
    // -----------------------------------------------------------------------
    // Task 10: Update Data Logger
    // -----------------------------------------------------------------------
    
    dataLog.update(localSensors, gpsData);
    
    // -----------------------------------------------------------------------
    // Task 11: Handle Mode-Specific Operations
    // -----------------------------------------------------------------------
    
    handleModeOperations();
}

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

void printBootBanner() {
    Serial.println();
    Serial.println(F("╔════════════════════════════════════════════════════════════╗"));
    Serial.println(F("║     MCB SP26 Release - Educational CubeSat Firmware        ║"));
    Serial.println(F("║                                                            ║"));
    Serial.print(F("║     Firmware Version: "));
    Serial.print(FIRMWARE_VERSION);
    Serial.println(F("                          ║"));
    Serial.print(F("║     Serial Number:    0x"));
    // Pad serial number for alignment
    if (sysConfig.serialNumber < 0x10000000) Serial.print(F("0"));
    if (sysConfig.serialNumber < 0x1000000) Serial.print(F("0"));
    if (sysConfig.serialNumber < 0x100000) Serial.print(F("0"));
    if (sysConfig.serialNumber < 0x10000) Serial.print(F("0"));
    if (sysConfig.serialNumber < 0x1000) Serial.print(F("0"));
    if (sysConfig.serialNumber < 0x100) Serial.print(F("0"));
    if (sysConfig.serialNumber < 0x10) Serial.print(F("0"));
    Serial.print(sysConfig.serialNumber, HEX);
    Serial.println(F("                       ║"));
    Serial.println(F("║                                                            ║"));
    Serial.println(F("║     Platform: Raspberry Pi Pico (RP2040)                   ║"));
    Serial.println(F("╚════════════════════════════════════════════════════════════╝"));
    Serial.println();
}

/**
 * Average the gyro after the part is actually producing samples.
 * update() leaves the last reading in place when no new sample is ready,
 * so zeros from the first moments after init must not become the bias.
 */
static bool averageStationaryGyro(int samples, float& outX, float& outY, float& outZ) {
    float sumX = 0.0f;
    float sumY = 0.0f;
    float sumZ = 0.0f;
    int count = 0;

    for (int i = 0; i < samples; i++) {
        imu.update();
        imu.getGyro(&gyroData);
        bool ready = (gyroData.gyroX != 0.0f || gyroData.gyroY != 0.0f || gyroData.gyroZ != 0.0f);
        if (ready && i >= 15) {
            sumX += gyroData.gyroX;
            sumY += gyroData.gyroY;
            sumZ += gyroData.gyroZ;
            count++;
        }
        delay(10);
    }

    if (count < 20) return false;
    outX = sumX / count;
    outY = sumY / count;
    outZ = sumZ / count;
    return true;
}

/**
 * The CubeSat is sitting still when it boots.
 * FastIMU stores the gyro bias in the sensor frame and only applies it when
 * init() is given a valid calibration. A second average then removes whatever
 * is still left on the axes the dashboard shows.
 */
void calibrateGyroBias() {
    if (!systemStatus.sensorIMU || imuI2cAddress == 0) return;

    Serial.println(F("[IMU] Hold still — measuring gyro bias"));

    imuCalibration.magScale[0] = 1.0f;
    imuCalibration.magScale[1] = 1.0f;
    imuCalibration.magScale[2] = 1.0f;
    imu.calibrateAccelGyro(&imuCalibration);
    // Keep accelerometer scale as-is. This pass is only a gyro-zero.
    imuCalibration.accelBias[0] = 0.0f;
    imuCalibration.accelBias[1] = 0.0f;
    imuCalibration.accelBias[2] = 0.0f;
    imuCalibration.magScale[0] = 1.0f;
    imuCalibration.magScale[1] = 1.0f;
    imuCalibration.magScale[2] = 1.0f;
    imuCalibration.valid = true;
    imu.init(imuCalibration, imuI2cAddress);

    Serial.print(F("[IMU] Sensor-frame gyro bias °/s  X "));
    Serial.print(imuCalibration.gyroBias[0], 2);
    Serial.print(F("  Y "));
    Serial.print(imuCalibration.gyroBias[1], 2);
    Serial.print(F("  Z "));
    Serial.println(imuCalibration.gyroBias[2], 2);

    delay(400);
    float biasX = 0.0f;
    float biasY = 0.0f;
    float biasZ = 0.0f;
    if (!averageStationaryGyro(120, biasX, biasY, biasZ)) {
        gyroBiasX = 0.0f;
        gyroBiasY = 0.0f;
        gyroBiasZ = 0.0f;
        Serial.println(F("[IMU] Output trim skipped — gyro was not ready"));
        return;
    }

    // A hard spin at boot is not drift.
    const float limit = 400.0f;
    if (fabs(biasX) > limit || fabs(biasY) > limit || fabs(biasZ) > limit) {
        gyroBiasX = 0.0f;
        gyroBiasY = 0.0f;
        gyroBiasZ = 0.0f;
        Serial.println(F("[IMU] Output trim discarded — CubeSat was moving"));
        return;
    }

    gyroBiasX = biasX;
    gyroBiasY = biasY;
    gyroBiasZ = biasZ;
    Serial.print(F("[IMU] Output gyro trim °/s  X "));
    Serial.print(gyroBiasX, 2);
    Serial.print(F("  Y "));
    Serial.print(gyroBiasY, 2);
    Serial.print(F("  Z "));
    Serial.println(gyroBiasZ, 2);
}

/**
 * Initialize all onboard sensors
 */
void initializeSensors() {
    Serial.println(F("[BOOT] Initializing sensors..."));
    
    // Temperature/Humidity Sensor (SHT41 primary, SHT31 fallback)
    Serial.print(F("  - SHT4x: "));
    if (sht4x.begin()) {
        sht4x.setPrecision(SHT4X_HIGH_PRECISION);
        sht4x.setHeater(SHT4X_NO_HEATER);
        useSHT41 = true;
        systemStatus.sensorSHT = true;
        Serial.println(F("OK (SHT41)"));
    } else {
        Serial.println(F("not found, trying SHT31..."));
        Serial.print(F("  - SHT31: "));
        if (sht31.begin(0x44)) {
            useSHT41 = false;
            systemStatus.sensorSHT = true;
            Serial.println(F("OK (SHT31)"));
        } else {
            systemStatus.sensorSHT = false;
            Serial.println(F("FAIL - No temperature sensor found"));
        }
    }
    
    // IMU Sensor (MPU9250 - supports 0x68 and 0x69)
    Serial.print(F("  - MPU9250: "));
    Wire.beginTransmission(0x68);
    uint8_t i2cError = Wire.endTransmission();
    
    if (i2cError == 0) {
        imu.init(imuCalibration, 0x68);
        imuI2cAddress = 0x68;
        systemStatus.sensorIMU = true;
        Serial.println(F("OK (address 0x68)"));
    } else {
        // Try alternative I2C address (0x69)
        Wire.beginTransmission(0x69);
        i2cError = Wire.endTransmission();
        
        if (i2cError == 0) {
            imu.init(imuCalibration, 0x69);
            imuI2cAddress = 0x69;
            systemStatus.sensorIMU = true;
            Serial.println(F("OK (address 0x69)"));
        } else {
            systemStatus.sensorIMU = false;
            Serial.println(F("FAIL - No IMU found"));
        }
    }

    calibrateGyroBias();
    
    // GPS Module
    Serial.println(F("  - GPS: Waiting for fix..."));
    systemStatus.sensorGPS = false;
}

/**
 * Apply radio config (channel, pipes, rate, listen). Used after begin() or as recovery without begin().
 * RF_CH is only latched in standby. Writing it while CE is high leaves the radio on the old channel.
 */
static void applyRadioConfig() {
    radio.stopListening();
    delay(2);
    radio.setPALevel((rf24_pa_dbm_e)sysConfig.radioPowerLevel);
    radio.setPayloadSize(sysConfig.radioPayloadSize);
    radio.setRetries(RADIO_RETRY_COUNT, RADIO_RETRY_DELAY);
    radio.setAutoAck(true);
    radio.disableDynamicPayloads();
    radio.enableAckPayload();
    uint8_t txAddr[6], rxAddr[6];
    memcpy(txAddr, sysConfig.radioTxAddress, 5);
    txAddr[5] = 0;
    memcpy(rxAddr, sysConfig.radioRxAddress, 5);
    rxAddr[5] = 0;
    radio.openWritingPipe(txAddr);
    radio.openReadingPipe(1, rxAddr);
    radio.setDataRate((rf24_datarate_e)sysConfig.radioDataRate);
    radio.setChannel(sysConfig.radioChannel);
    if (radio.getChannel() != sysConfig.radioChannel) {
        radio.setChannel(sysConfig.radioChannel);
    }
    radio.flush_tx();
    radio.flush_rx();
    radio.startListening();
}

/**
 * Initialize NRF24L01 radio module
 * Uses configuration values from sysConfig (loaded from EEPROM).
 * When reinit is true: power-cycle, retry begin() up to 3x, then try config-only recovery if needed.
 */
void initializeRadio(bool reinit) {
    if (!reinit) {
        Serial.print(F("[BOOT] Initializing NRF24L01 radio... "));
    } else {
        Serial.print(F("[RADIO] Re-initializing NRF24L01... "));
    }
    
    bool ok = false;
    
    if (reinit) {
        // CE low then high resets NRF24 state machine; then power cycle
        digitalWrite(PIN_NRF_CE, LOW);
        delay(2);
        digitalWrite(PIN_NRF_CE, HIGH);
        delay(2);
        radio.powerDown();
        delay(5);
        radio.powerUp();
        delay(5);
    }
    
    for (int attempt = 0; attempt < (reinit ? 3 : 1) && !ok; attempt++) {
        if (attempt > 0) {
            delay(100);
        }
        ok = radio.begin(&SPI);
    }
    
    if (ok) {
        applyRadioConfig();
        systemStatus.radioNRF = true;
        Serial.println(F("OK"));
        if (!reinit) {
            Serial.print(F("         Channel: "));
            Serial.println(radio.getChannel());
            Serial.print(F("         Power Level: "));
            Serial.println(radio.getPALevel());
            Serial.print(F("         Data Rate: "));
            Serial.println(radio.getDataRate());
            Serial.print(F("         Chip Connected: "));
            Serial.println(radio.isChipConnected() ? "Yes" : "No");
        }
        return;
    }
    
    // Re-init only: try config-only recovery (chip may still respond to SPI)
    if (reinit) {
        radio.powerDown();
        delay(5);
        radio.powerUp();
        delay(5);
        applyRadioConfig();
        if (radio.isChipConnected()) {
            systemStatus.radioNRF = true;
            Serial.println(F("OK (config recovery)"));
            return;
        }
    }
    
    if (!reinit) {
        systemStatus.radioNRF = false;
    }
    Serial.println(F("FAIL - Check SPI connection"));
}

// ============================================================================
// SENSOR READING FUNCTIONS
// ============================================================================

/**
 * Read data from all local (MCB onboard) sensors
 */
void readLocalSensors() {
    // Temperature and Humidity
    if (systemStatus.sensorSHT) {
        if (useSHT41) {
            sensors_event_t humidity, temp;
            if (sht4x.getEvent(&humidity, &temp)) {
                localSensors.temperature = temp.temperature;
                localSensors.humidity = humidity.relative_humidity;
            }
        } else {
            float t = sht31.readTemperature();
            float h = sht31.readHumidity();
            if (!isnan(t) && !isnan(h)) {
                localSensors.temperature = t;
                localSensors.humidity = h;
            }
        }
    }
    
    // IMU Data
    if (systemStatus.sensorIMU) {
        imu.update();
        imu.getAccel(&accelData);
        imu.getGyro(&gyroData);
        imu.getMag(&magData);
        
        // Accelerometer (g)
        localSensors.accelX = accelData.accelX;
        localSensors.accelY = accelData.accelY;
        localSensors.accelZ = accelData.accelZ;
        
        // Gyroscope (degrees/second), with the boot-time stationary bias removed
        localSensors.gyroX = gyroData.gyroX - gyroBiasX;
        localSensors.gyroY = gyroData.gyroY - gyroBiasY;
        localSensors.gyroZ = gyroData.gyroZ - gyroBiasZ;
        
        // Magnetometer (uT)
        localSensors.magX = magData.magX;
        localSensors.magY = magData.magY;
        localSensors.magZ = magData.magZ;
    }
}

void processGPSData() {
    while (Serial_GPS.available() > 0) {
        char c = Serial_GPS.read();
        nmea.process(c);
    }
    
    if (nmea.isValid()) {
        gpsData.latitude = nmea.getLatitude() / 1000000.0;
        gpsData.longitude = nmea.getLongitude() / 1000000.0;
        
        long altitude;
        if (nmea.getAltitude(altitude)) {
            gpsData.altitude = altitude / 1000.0;
        }
        
        gpsData.satellites = nmea.getNumSatellites();
        gpsData.hdop = nmea.getHDOP() / 10.0;
        gpsData.hasValidFix = true;
        systemStatus.sensorGPS = true;
        
        gpsData.year = nmea.getYear();
        gpsData.month = nmea.getMonth();
        gpsData.day = nmea.getDay();
        gpsData.hour = nmea.getHour();
        gpsData.minute = nmea.getMinute();
        gpsData.second = nmea.getSecond();
    } else {
        gpsData.hasValidFix = false;
    }
}

// ============================================================================
// SATBUS COMMUNICATION
// ============================================================================

void processSatbusData() {
    // Check for incoming SATBUS packets
    while (Serial_SATBUS.available()) {
        char c = Serial_SATBUS.read();
        
        // Feed character to parser
        SatbusPacket packet = satbus.parseChar(c);
        
        // If a complete packet was received, process it
        if (packet.isValid) {
            processSatbusPacket(packet);
        }
    }
}

void processSatbusPacket(SatbusPacket& packet) {
    switch (packet.moduleType) {
        case SATBUS_MODULE_ASB:
            handleASBPacket(packet);
            systemStatus.satbusASB = true;
            break;
            
        case SATBUS_MODULE_BMS:
            handleBMSPacket(packet);
            systemStatus.satbusBMS = true;
            break;
            
        case SATBUS_MODULE_SCC:
            handleSCCPacket(packet);
            systemStatus.satbusSCC = true;
            break;
            
        default:
            // Unknown module - log for debugging with raw data
            Serial.print(F("[SATBUS] Unknown module type "));
            Serial.print(packet.moduleType);
            Serial.print(F(", raw: "));
            // Print first 20 chars of raw data for debugging
            for (int i = 0; i < 20 && packet.rawData[i] != '\0'; i++) {
                char c = packet.rawData[i];
                if (c >= 32 && c <= 126) {
                    Serial.print(c);
                } else {
                    Serial.print(F("[0x"));
                    Serial.print((uint8_t)c, HEX);
                    Serial.print(F("]"));
                }
            }
            Serial.println();
            break;
    }
}

void handleASBPacket(SatbusPacket& packet) {
    // Fielded ASB boards emit $ASB,0 (env) and $ASB,1 (PM).
    // Older docs used ids 3/4 — accept both.
    if (packet.packetId == 0 || packet.packetId == 3) {
        satbus.parseASBEnvironmental(packet, asbData);
        dataLog.logASBData(asbData);
    } else if (packet.packetId == 1 || packet.packetId == 4) {
        satbus.parseASBParticulate(packet, asbData);
        dataLog.logASBData(asbData);
    }
}

void handleBMSPacket(SatbusPacket& packet) {
    satbus.parseBMSStatus(packet, bmsData);
    dataLog.logBMSData(bmsData);
    
    // Check for low battery condition
    if (bmsData.errorCode == 2) {  // Critical battery
        systemStatus.lowBattery = true;
        dataLog.logError("Critical battery", bmsData.errorCode);
    } else if (bmsData.errorCode == 1) {  // Low battery warning
        systemStatus.lowBattery = true;
    } else {
        systemStatus.lowBattery = false;
    }
}

void handleSCCPacket(SatbusPacket& packet) {
    if (packet.packetId == 1) {
        satbus.parseSCCVoltages(packet, sccData);
    } else if (packet.packetId == 2) {
        satbus.parseSCCTemperatures(packet, sccData);
    } else if (packet.packetId == 3) {
        satbus.parseSCCOutput(packet, sccData);
        dataLog.logSCCData(sccData);  // Log when all data available
    }
}

// ============================================================================
// EDGEFLYTE IDENTIFICATION COMMAND
// ============================================================================

/**
 * Get RP2040 unique chip ID as 32-character hex string
 */
void getChipID(char* buffer) {
    // RP2040 has a 64-bit unique ID in flash (8 bytes = 16 hex chars)
    // We'll pad to 32 chars for the EF protocol spec
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    
    // Format as 32-char hex (16 chars from ID + 16 zeros for padding)
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
        sprintf(buffer + (i * 2), "%02X", id.id[i]);
    }
    // Pad remaining chars with zeros to reach 32 total
    for (int i = PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2; i < 32; i++) {
        buffer[i] = '0';
    }
    buffer[32] = '\0';
}

/**
 * Read device temperature
 * Returns temperature in Celsius
 */
float readInternalTemp() {
    // Use onboard SHT sensor if available (more accurate for device temp)
    if (systemStatus.sensorSHT) {
        return localSensors.temperature;
    }
    // Fallback to RP2040 internal temp sensor via Arduino-Pico core
    return analogReadTemp();
}

/**
 * Read system voltage
 * Returns voltage in Volts
 */
float readInternalVoltage() {
    // If BMS is connected, use its battery voltage reading
    if (systemStatus.satbusBMS && bmsData.batteryVoltage > 0) {
        return bmsData.batteryVoltage;
    }
    // Otherwise return nominal 3.3V (RP2040 operating voltage)
    return 3.30f;
}

/**
 * Determine system status for EF identify response
 * Returns "OK" if all systems healthy, "ERR" otherwise
 */
const char* getEFSystemStatus() {
    // Check critical systems
    bool healthy = true;
    
    // Check temperature (internal MCU temp should be < 60°C)
    float temp = readInternalTemp();
    if (temp > 60.0f || temp < -20.0f) healthy = false;
    
    // Check voltage (VSYS should be 3.0-5.5V typical)
    float volt = readInternalVoltage();
    if (volt < 3.0f || volt > 5.5f) healthy = false;
    
    // Check if critical sensors failed
    if (!systemStatus.sensorSHT && !systemStatus.sensorIMU) healthy = false;
    
    // Check for low battery condition
    if (systemStatus.lowBattery) healthy = false;
    
    return healthy ? "OK" : "ERR";
}

/**
 * Handle EdgeFlyte Identification Command (#?EF?#)
 * 
 * Response format:
 * #!EF!:<MODEL>,<SERIAL>,<FW_VERSION>,<STATUS>,<UPTIME>,<BOOT_COUNT>,<TEMP>,<VOLT>,<FREE_RAM>,<CHIP_ID>#
 */
void handleMeshMessage(TelemetryPacket& packet) {
    if (packet.dataLength < 4) {
        return;
    }

    uint8_t msgId = packet.data[0];
    uint8_t fragIdx = packet.data[1];
    uint8_t fragCount = packet.data[2];
    uint8_t flags = packet.data[3];

    if (fragCount == 0 || fragCount > TELEM_MSG_MAX_FRAGS || fragIdx >= fragCount) {
        return;
    }

    uint8_t textLen = packet.dataLength - 4;
    if (textLen > TELEM_MSG_FRAG_BYTES) {
        textLen = TELEM_MSG_FRAG_BYTES;
    }

    // Start or continue assembly for this (from, msgId)
    if (!msgAssembly.active ||
        msgAssembly.fromId != packet.transmitterId ||
        msgAssembly.msgId != msgId ||
        msgAssembly.fragCount != fragCount) {
        msgAssembly.active = true;
        msgAssembly.fromId = packet.transmitterId;
        msgAssembly.msgId = msgId;
        msgAssembly.fragCount = fragCount;
        msgAssembly.receivedMask = 0;
        memset(msgAssembly.text, 0, sizeof(msgAssembly.text));
    }
    msgAssembly.lastMs = millis();

    // Deduplicate fragment copies (relay #R / retries)
    if (msgAssembly.receivedMask & (1u << fragIdx)) {
        return;
    }

    memcpy(&msgAssembly.text[fragIdx * TELEM_MSG_FRAG_BYTES], &packet.data[4], textLen);
    msgAssembly.receivedMask |= (1u << fragIdx);

    uint8_t expectMask = (1u << fragCount) - 1;
    if (msgAssembly.receivedMask != expectMask) {
        return;
    }

    // Complete — log and print on USB
    MsgLogEntry& entry = msgLog[msgLogNext];
    msgLogNext = (msgLogNext + 1) % MSG_LOG_ENTRIES;
    entry.fromId = packet.transmitterId;
    entry.msgId = msgId;
    entry.flags = flags;
    entry.timestamp = millis();
    entry.valid = true;
    strncpy(entry.text, msgAssembly.text, TELEM_MSG_MAX_CHARS);
    entry.text[TELEM_MSG_MAX_CHARS] = '\0';
    msgAssembly.active = false;

    Serial.print(F("[MSG] From 0x"));
    Serial.print(entry.fromId, HEX);
    if (flags & TELEM_MSG_FLAG_GROUND) {
        Serial.print(F(" (GS)"));
    }
    Serial.print(F(": "));
    Serial.println(entry.text);
}

void handleEFIdentifyCommand() {
    char chipID[33];
    getChipID(chipID);
    
    // Get serial number as last 2 bytes (4 hex chars)
    uint16_t serialShort = (uint16_t)(sysConfig.serialNumber & 0xFFFF);
    
    // Read system values
    float temp = readInternalTemp();
    float volt = readInternalVoltage();
    uint32_t freeRam = rp2040.getFreeHeap();
    const char* status = getEFSystemStatus();
    
    // Build response
    // #!EF!:<MODEL>,<SERIAL>,<FW_VERSION>,<STATUS>,<UPTIME>,<BOOT_COUNT>,<TEMP>,<VOLT>,<FREE_RAM>,<CHIP_ID>#
    Serial.print(F("#!EF!:"));
    Serial.print(EF_MODEL_NUMBER);
    Serial.print(',');
    
    // Print serial as 4-char hex (padded with leading zeros)
    if (serialShort < 0x1000) Serial.print('0');
    if (serialShort < 0x100) Serial.print('0');
    if (serialShort < 0x10) Serial.print('0');
    Serial.print(serialShort, HEX);
    Serial.print(',');
    
    Serial.print(FIRMWARE_VERSION_INT);
    Serial.print(',');
    Serial.print(status);
    Serial.print(',');
    Serial.print(uptimeSeconds);
    Serial.print(',');
    Serial.print(bootCount);
    Serial.print(',');
    Serial.print(temp, 1);
    Serial.print(',');
    Serial.print(volt, 2);
    Serial.print(',');
    Serial.print(freeRam);
    Serial.print(',');
    Serial.print(chipID);
    Serial.println('#');
}

// ============================================================================
// COMMAND PROCESSING
// ============================================================================

static uint8_t radioRxFaults = 0;
static uint32_t radioRxFaultWindow = 0;

static void noteRadioRxFault(const __FlashStringHelper* reason) {
    uint32_t now = millis();
    if (radioRxFaultWindow == 0 || (now - radioRxFaultWindow) > 5000UL) {
        radioRxFaults = 0;
        radioRxFaultWindow = now;
    }
    radioRxFaults++;
    if (radioRxFaults == 1) {
        Serial.print(F("[RADIO] RX recovered: "));
        Serial.println(reason);
    }
    if (radioRxFaults < 3) return;

    radioRxFaults = 0;
    radioRxFaultWindow = now;
    if (now - lastRadioReinitAttempt < 3000UL) return;
    lastRadioReinitAttempt = now;
    Serial.println(F("[RADIO] RX still stuck, resetting radio"));
    initializeRadio(true);
}

// Read one payload. A corrupt length or the same bytes repeating inside 80 ms
// is a wedged RX FIFO, not a new packet. Returns false after dropping it.
static bool takeRadioPacket(uint8_t* buf, uint8_t len) {
    static uint8_t lastPacket[TELEM_MAX_PACKET_SIZE];
    static bool haveLast = false;
    static uint8_t repeatCount = 0;
    static uint32_t lastRepeatMs = 0;

    uint8_t width = radio.getDynamicPayloadSize();
    if (width == 0 || width > len) {
        if (width > len) radio.flush_rx();
        noteRadioRxFault(F("bad payload width"));
        return false;
    }

    memset(buf, 0, len);
    radio.read(buf, width);

    uint32_t now = millis();
    bool same = haveLast && memcmp(buf, lastPacket, len) == 0;
    if (same && (now - lastRepeatMs) < 80UL) {
        repeatCount++;
        lastRepeatMs = now;
        if (repeatCount >= 3) {
            radio.flush_rx();
            repeatCount = 0;
            noteRadioRxFault(F("stuck packet"));
        }
        return false;
    }

    memcpy(lastPacket, buf, len);
    haveLast = true;
    repeatCount = 0;
    lastRepeatMs = now;
    return true;
}

void processIncomingCommands() {
    // Check for radio packets (radio stays in RX; no stop/start toggle to avoid destabilizing chip)
    if (systemStatus.radioNRF && radio.available()) {
        uint8_t drained = 0;
        while (radio.available() && drained < 3) {
            uint8_t rxBuffer[TELEM_MAX_PACKET_SIZE];
            if (!takeRadioPacket(rxBuffer, TELEM_MAX_PACKET_SIZE)) break;
            drained++;

            // Decode the wire format (12-byte big-endian header + data) into the
            // packet struct. A raw struct read does not match the serialized
            // layout (padding + native endianness), so decode explicitly.
            TelemetryPacket rxPacket;
            if (telemetry.deserializePacket(rxBuffer, TELEM_MAX_PACKET_SIZE, rxPacket)) {
                // Check if this is a relay packet (#R preamble)
                bool isRelay = (rxPacket.preamble[0] == '#' && rxPacket.preamble[1] == 'R');

                // Check if packet is from another CubeSat
                if (rxPacket.transmitterId != sysConfig.serialNumber) {
                    mesh.processPacket(rxPacket, isRelay);
                }

                // Process packets addressed to us (or broadcast)
                if (rxPacket.destinationId == sysConfig.serialNumber ||
                    rxPacket.destinationId == 0x00000000) {
                    if (rxPacket.packetType == TELEM_MSG_TEXT) {
                        handleMeshMessage(rxPacket);
                    } else if ((rxPacket.packetType & 0x8000) != 0) {
                        CommandResult result = commands.processPacket(rxPacket);
                        handleCommandResult(result, &rxPacket);
                    }
                }
            }
        }
    }
    
    // Check for BLE/Serial commands (local interface)
    if (Serial.available()) {
        String cmdLine = Serial.readStringUntil('\n');
        cmdLine.trim();
        
        if (cmdLine.length() > 0) {
            // Check for EdgeFlyte Identification Command (#?EF?#)
            if (cmdLine == "#?EF?#") {
                handleEFIdentifyCommand();
            }
            // Check for #G parameter protocol packets (USB Serial only)
            else if (cmdLine.startsWith("#G")) {
                params.processPacket(cmdLine.c_str(), cmdLine.length());
            }
            // Handle params command to show all parameters
            else if (cmdLine.equalsIgnoreCase("params")) {
                params.printAllParameters();
            }
            // Handle config reset command
            else if (cmdLine.equalsIgnoreCase("config reset")) {
                Serial.println(F("[CONFIG] Resetting configuration to defaults..."));
                params.resetToDefaults();
                Serial.println(F("[CONFIG] Done. Reboot to apply all changes."));
            }
            // Handle config save command
            else if (cmdLine.equalsIgnoreCase("config save")) {
                if (params.saveConfig()) {
                    Serial.println(F("[CONFIG] Configuration saved to EEPROM"));
                } else {
                    Serial.println(F("[CONFIG] Failed to save configuration"));
                }
            }
            else {
                // Regular text command
                CommandResult result = commands.processTextCommand(cmdLine);
                handleCommandResult(result, nullptr);
            }
        }
    }
}

/**
 * Radio ACK for an uplinked command (TELEM_CMD_ACK / 0x0008).
 *
 * Sent before handleCommandResult applies channel or PA changes, so the
 * frame leaves on the channel the ground station is still listening on.
 * Payload (20 bytes, zero padded):
 *   0-1  echoed command type, uint16 BE
 *   2    1 = accepted, 0 = rejected
 *   3    CommandAction
 *   4    parameter
 *   5    value
 *   6    message length (0-13)
 *   7-19 message bytes
 * Destination is the uplink transmitter (GS serial). 0 stays broadcast.
 */
void transmitCommandAck(const TelemetryPacket& cmd, const CommandResult& result) {
    if (!systemStatus.radioNRF) return;

    uint8_t payload[TELEM_MAX_WIRE_DATA_SIZE] = {0};
    payload[0] = (uint8_t)((cmd.packetType >> 8) & 0xFF);
    payload[1] = (uint8_t)(cmd.packetType & 0xFF);
    payload[2] = result.success ? 1 : 0;
    payload[3] = (uint8_t)result.action;
    payload[4] = result.parameter;
    payload[5] = result.value;

    uint8_t msgLen = 0;
    while (msgLen < 13 && result.message[msgLen] != '\0') {
        payload[7 + msgLen] = (uint8_t)result.message[msgLen];
        msgLen++;
    }
    payload[6] = msgLen;

    TelemetryPacket ack = telemetry.buildPacket(TELEM_CMD_ACK, payload, TELEM_MAX_WIRE_DATA_SIZE);
    uint32_t replyTo = cmd.transmitterId;
    if (replyTo == 0 || replyTo == 0xFFFFFFFF) {
        replyTo = TELEM_BROADCAST_ADDR;
    }
    ack.destinationId = replyTo;
    telemetry.transmit(ack);
}

void handleCommandResult(CommandResult& result, const TelemetryPacket* uplink) {
    if (uplink != nullptr) {
        transmitCommandAck(*uplink, result);
    }

    if (!result.success) {
        Serial.print(F("[CMD] Error: "));
        Serial.println(result.message);
        return;
    }
    
    // Handle specific command actions
    switch (result.action) {
        case CMD_ACTION_REBOOT:
            Serial.println(F("[CMD] Rebooting system..."));
            delay(100);
            rp2040.reboot();
            break;

        case CMD_ACTION_RESTART:
            Serial.println(F("[CMD] Restarting application..."));
            delay(100);
            rp2040.reboot();
            break;
            
        case CMD_ACTION_SET_MODE:
            setSystemMode((SystemMode)result.parameter);
            break;

        case CMD_ACTION_RADIO_CHANNEL:
            sysConfig.radioChannel = result.parameter;
            if (sysConfig.radioChannel > 125) sysConfig.radioChannel = 125;
            applyRadioConfig();
            Serial.print(F("[RADIO] Channel set to "));
            Serial.print(sysConfig.radioChannel);
            Serial.print(F(" (chip "));
            Serial.print(radio.getChannel());
            Serial.println(F(")"));
            break;

        case CMD_ACTION_RADIO_POWER:
            sysConfig.radioPowerLevel = result.parameter;
            applyRadioConfig();
            Serial.print(F("[RADIO] Power level set to "));
            Serial.println(sysConfig.radioPowerLevel);
            break;

        case CMD_ACTION_SAVE_RADIO:
            if (params.saveConfig()) {
                Serial.print(F("[RADIO] Saved channel "));
                Serial.print(sysConfig.radioChannel);
                Serial.print(F(" power "));
                Serial.print(sysConfig.radioPowerLevel);
                Serial.print(F(" interval "));
                Serial.print(sysConfig.telemetryIntervalFast);
                Serial.println(F(" ms"));
            } else {
                Serial.println(F("[CMD] Error: radio save failed"));
            }
            break;

        case CMD_ACTION_TELEM_INTERVAL:
            {
                uint32_t interval = ((uint32_t)result.parameter << 8) | result.value;
                if (interval < 100) interval = 100;
                if (interval > 60000) interval = 60000;
                sysConfig.telemetryIntervalFast = interval;
                Serial.print(F("[TELEM] Fast interval set to "));
                Serial.print(interval);
                Serial.println(F(" ms"));
            }
            break;
            
        case CMD_ACTION_BMS_CHANNEL:
            // Send command to BMS via SATBUS
            satbus.sendBMSCommand(result.parameter, result.value);
            break;

        case CMD_ACTION_BMS_STORE:
            satbus.sendBMSStore(result.parameter);
            Serial.print(F("[BMS] Boot mask stored: 0x"));
            Serial.println(result.parameter, HEX);
            break;
            
        case CMD_ACTION_SEND_TELEMETRY:
            transmitTelemetry();
            break;
            
        case CMD_ACTION_PRINT_STATUS:
            printSystemStatus();
            break;
            
        case CMD_ACTION_PRINT_SENSORS:
            printSensorData();
            break;
            
        case CMD_ACTION_PRINT_GPS:
            printGPSData();
            break;
            
        case CMD_ACTION_PRINT_IMU:
            printIMUData();
            break;
            
        case CMD_ACTION_PRINT_ENV:
            printEnvData();
            break;
            
        case CMD_ACTION_PRINT_POWER:
            printPowerData();
            break;
            
        case CMD_ACTION_PRINT_TELEM_STATS:
            printTelemStats();
            break;
            
        case CMD_ACTION_PRINT_MEMORY:
            printMemoryStats();
            break;
            
        case CMD_ACTION_PRINT_MESH:
            mesh.printStatus();
            break;
            
        case CMD_ACTION_PRINT_CUBESAT:
            {
                // Reconstruct serial from parameter and value
                uint32_t serial = result.parameter | (result.value << 8);
                // For longer serials, we'd need more bytes - this handles up to 16-bit
                // For full 32-bit, user should use the mesh.printCubesatData directly
                mesh.printCubesatData(serial);
            }
            break;
            
        case CMD_ACTION_MESH_RELAY:
            if (result.parameter == 0xFF) {
                Serial.print(F("[MESH] Relay is "));
                Serial.println(mesh.isRelayEnabled() ? "enabled" : "disabled");
            } else {
                mesh.setRelayEnabled(result.parameter != 0);
                Serial.print(F("[MESH] Relay "));
                Serial.println(mesh.isRelayEnabled() ? "enabled" : "disabled");
            }
            break;
            
        case CMD_ACTION_MESH_LOG:
            if (result.parameter == 0xFF) {
                Serial.print(F("[MESH] SD Logging is "));
                Serial.println(mesh.isLogToSD() ? "enabled" : "disabled");
            } else {
                mesh.setLogToSD(result.parameter != 0);
                if (result.parameter != 0) {
                    if (!mesh.initSD()) {
                        Serial.println(F("[MESH] Warning: SD card not initialized"));
                    }
                }
                Serial.print(F("[MESH] SD Logging "));
                Serial.println(mesh.isLogToSD() ? "enabled" : "disabled");
            }
            break;
            
        case CMD_ACTION_PRINT_LOG_STATUS:
            dataLog.printStatus();
            break;
            
        case CMD_ACTION_LOG_ENABLE:
            dataLog.setEnabled(result.parameter != 0);
            Serial.print(F("[LOG] Data logging "));
            Serial.println(dataLog.isEnabled() ? "enabled" : "disabled");
            if (result.parameter != 0) {
                dataLog.logEvent("Logging enabled by user", 0);
            }
            break;
            
        case CMD_ACTION_LOG_INTERVAL:
            {
                uint32_t interval = (result.parameter << 8) | result.value;
                // Handle larger intervals passed differently
                if (interval < 100) {
                    // Small value means it was truncated, use default multiplier
                    interval = interval * 1000;
                }
                dataLog.setSensorInterval(interval);
                dataLog.setGPSInterval(interval);
                Serial.print(F("[LOG] Interval set to "));
                Serial.print(interval);
                Serial.println(F(" ms"));
            }
            break;
            
        case CMD_ACTION_LOG_FLUSH:
            dataLog.flush();
            Serial.println(F("[LOG] Buffer flushed to SD card"));
            break;
            
        case CMD_ACTION_BUZZER:
            setBuzzer(result.aux0, result.aux1);
            Serial.print(F("[BUZZ] "));
            Serial.println(result.message);
            break;

        case CMD_ACTION_UART_PASSTHROUGH:
            {
                // parameter: 1=GPS, 2=SATBUS
                // value: 0=off, 1=on, 0xFF=query
                if (result.value == 0xFF) {
                    // Query mode
                    Serial.print(F("[UART] Passthrough status: "));
                    if (uartPassthroughActive) {
                        Serial.print(F("ACTIVE on "));
                        Serial.println(uartPassthroughTarget == 1 ? "GPS" : "SATBUS");
                    } else {
                        Serial.println(F("INACTIVE"));
                    }
                } else if (result.value == 1) {
                    // Enable passthrough
                    uartPassthroughActive = true;
                    uartPassthroughTarget = result.parameter;
                    Serial.print(F("[UART] Passthrough ENABLED on "));
                    Serial.print(result.parameter == 1 ? "GPS" : "SATBUS");
                    Serial.println(F(" - Type 'exit' to disable"));
                } else {
                    // Disable passthrough
                    uartPassthroughActive = false;
                    uartPassthroughTarget = 0;
                    Serial.println(F("[UART] Passthrough DISABLED"));
                }
            }
            break;
            
        default:
            Serial.print(F("[CMD] OK: "));
            Serial.println(result.message);
            break;
    }
}

// ============================================================================
// TELEMETRY TRANSMISSION
// ============================================================================

void transmitTelemetry() {
    if (!systemStatus.radioNRF) {
        digitalWrite(PIN_LED3, HIGH);
        delay(30);
        digitalWrite(PIN_LED3, LOW);
        return;
    }
    
    // Minimal touch: just send packets (no per-cycle refresh - was likely over-stressing the radio)
    telemetry.sendSystemStatus(
        static_cast<uint8_t>(currentMode),
        systemStatus,
        uptimeSeconds,
        bootCount
    );
    telemetry.sendSensorData(localSensors, gpsData);
    
    if (systemStatus.satbusSCC) {
        telemetry.sendSCCData(sccData);
    }
}

uint32_t getTelemetryInterval() {
    switch (currentMode) {
        case SystemMode::SAFE:
            return sysConfig.telemetryIntervalSlow * 2;  // Very slow in safe mode
        case SystemMode::LOW_POWER:
            return sysConfig.telemetryIntervalSlow;
        case SystemMode::SCIENCE:
            return sysConfig.telemetryIntervalFast / 2;  // Fast in science mode
        case SystemMode::EMERGENCY:
            return sysConfig.telemetryIntervalFast;      // Fast for emergency
        default:
            return sysConfig.telemetryIntervalFast;
    }
}

// ============================================================================
// SYSTEM MANAGEMENT
// ============================================================================

void setSystemMode(SystemMode newMode) {
    if (newMode == currentMode) return;
    
    SystemMode oldMode = currentMode;
    currentMode = newMode;
    
    Serial.print(F("[SYS] Mode change: "));
    Serial.print(getModeString(oldMode));
    Serial.print(F(" -> "));
    Serial.println(getModeString(newMode));
    
    // Log mode change to SD card
    dataLog.logModeChange(static_cast<uint8_t>(oldMode), static_cast<uint8_t>(newMode));
    
    // Mode-specific entry actions
    switch (newMode) {
        case SystemMode::LOW_POWER:
            // Reduce sensor polling, dim LEDs, etc.
            Serial.println(F("[SYS] Entering low power mode"));
            break;
            
        case SystemMode::SCIENCE:
            // Increase sensor sampling rate
            Serial.println(F("[SYS] Entering science mode"));
            break;
            
        case SystemMode::EMERGENCY:
            // Disable non-essential systems
            Serial.println(F("[SYS] EMERGENCY MODE ACTIVE"));
            break;
            
        default:
            break;
    }
}

const char* getModeString(SystemMode mode) {
    switch (mode) {
        case SystemMode::BOOT:      return "BOOT";
        case SystemMode::SAFE:      return "SAFE";
        case SystemMode::NOMINAL:   return "NOMINAL";
        case SystemMode::LOW_POWER: return "LOW_POWER";
        case SystemMode::SCIENCE:   return "SCIENCE";
        case SystemMode::EMERGENCY: return "EMERGENCY";
        default:                    return "UNKNOWN";
    }
}

void updateHeartbeat() {
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(PIN_LED_STATUS, ledState);
}

void checkSystemHealth() {
    uint32_t now = millis();
    
    // Radio: periodic config refresh only (no begin/power-cycle after boot - minimal touch to avoid chip getting stuck)
    if (systemStatus.radioNRF) {
        if (now - lastRadioPeriodicReinit >= RADIO_REINIT_PERIODIC_MS) {
            lastRadioPeriodicReinit = now;
            applyRadioConfig();
        }
    }
    // If radio was marked down at boot, keep trying re-init every 15s
    if (!systemStatus.radioNRF && (now - lastRadioReinitAttempt >= RADIO_REINIT_INTERVAL_MS)) {
        lastRadioReinitAttempt = now;
        initializeRadio(true);
    }
    
    // Check RBF pin state
    bool rbfState = (digitalRead(PIN_RBF_SENSE) == LOW);
    
    // If RBF was just inserted, go to SAFE mode
    if (rbfState && !systemStatus.rbfInserted) {
        systemStatus.rbfInserted = true;
        setSystemMode(SystemMode::SAFE);
    }
    // If RBF was just removed and we're in SAFE mode, go to NOMINAL
    else if (!rbfState && systemStatus.rbfInserted) {
        systemStatus.rbfInserted = false;
        if (currentMode == SystemMode::SAFE) {
            setSystemMode(SystemMode::NOMINAL);
        }
    }
    
    // Check battery status (from BMS)
    if (systemStatus.lowBattery && currentMode == SystemMode::NOMINAL) {
        setSystemMode(SystemMode::LOW_POWER);
    }
}

void handleModeOperations() {
    switch (currentMode) {
        case SystemMode::SAFE:
            // In safe mode, only essential operations run
            // This is already handled by the main loop
            break;
            
        case SystemMode::SCIENCE:
            // In science mode, we might run additional data collection
            // Add custom science operations here
            break;
            
        default:
            break;
    }
}

void printSystemStatus() {
    Serial.println();
    Serial.println(F("╔══════════════════════════════════════╗"));
    Serial.println(F("║         SYSTEM STATUS REPORT         ║"));
    Serial.println(F("╠══════════════════════════════════════╣"));
    
    Serial.print(F("║ Serial:   0x"));
    Serial.print(sysConfig.serialNumber, HEX);
    Serial.println(F("                   ║"));
    
    Serial.print(F("║ Mode:     "));
    Serial.print(getModeString(currentMode));
    Serial.println(F("                    ║"));
    
    Serial.print(F("║ Uptime:   "));
    Serial.print(uptimeSeconds);
    Serial.println(F(" seconds               ║"));
    
    Serial.print(F("║ Boot #:   "));
    Serial.print(bootCount);
    Serial.println(F("                          ║"));
    
    Serial.println(F("╠══════════════════════════════════════╣"));
    Serial.println(F("║ SENSORS:                             ║"));
    Serial.print(F("║   SHT:      "));
    if (systemStatus.sensorSHT) {
        Serial.println(useSHT41 ? "SHT41   ║" : "SHT31   ║");
    } else {
        Serial.println(F("FAIL    ║"));
    }
    Serial.print(F("║   MPU9250:  "));
    Serial.println(systemStatus.sensorIMU ? "OK      ║" : "FAIL    ║");
    Serial.print(F("║   GPS:      "));
    Serial.println(systemStatus.sensorGPS ? "FIX     ║" : "NO FIX  ║");
    
    Serial.println(F("╠══════════════════════════════════════╣"));
    Serial.println(F("║ RADIOS:                              ║"));
    Serial.print(F("║   NRF24L01: "));
    Serial.println(systemStatus.radioNRF ? "OK      ║" : "FAIL    ║");
    Serial.print(F("║   BLE:      "));
    Serial.println(systemStatus.radioBLE ? "OK      ║" : "FAIL    ║");
    
    Serial.println(F("╠══════════════════════════════════════╣"));
    Serial.println(F("║ SATBUS MODULES:                      ║"));
    Serial.print(F("║   ASB:      "));
    Serial.println(systemStatus.satbusASB ? "ONLINE  ║" : "OFFLINE ║");
    Serial.print(F("║   BMS:      "));
    Serial.println(systemStatus.satbusBMS ? "ONLINE  ║" : "OFFLINE ║");
    Serial.print(F("║   SCC:      "));
    Serial.println(systemStatus.satbusSCC ? "ONLINE  ║" : "OFFLINE ║");
    
    Serial.println(F("╠══════════════════════════════════════╣"));
    Serial.println(F("║ FLAGS:                               ║"));
    Serial.print(F("║   RBF:      "));
    Serial.println(systemStatus.rbfInserted ? "IN      ║" : "OUT     ║");
    Serial.print(F("║   Battery:  "));
    Serial.println(systemStatus.lowBattery ? "LOW     ║" : "OK      ║");
    
    Serial.println(F("╚══════════════════════════════════════╝"));
    Serial.println();
}

void printSensorData() {
    Serial.println();
    Serial.println(F("╔═══════════════════════════════════════════╗"));
    Serial.println(F("║            ALL SENSOR DATA                ║"));
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    
    Serial.println(F("║ TEMPERATURE/HUMIDITY:                     ║"));
    Serial.print(F("║   Temp:     ")); Serial.print(localSensors.temperature, 2);
    Serial.println(F(" °C                      ║"));
    Serial.print(F("║   Humidity: ")); Serial.print(localSensors.humidity, 1);
    Serial.println(F(" %                       ║"));
    
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    Serial.println(F("║ ACCELEROMETER (g):                        ║"));
    Serial.print(F("║   X: ")); Serial.print(localSensors.accelX, 3);
    Serial.print(F("  Y: ")); Serial.print(localSensors.accelY, 3);
    Serial.print(F("  Z: ")); Serial.println(localSensors.accelZ, 3);
    
    Serial.println(F("║ GYROSCOPE (°/s):                          ║"));
    Serial.print(F("║   X: ")); Serial.print(localSensors.gyroX, 1);
    Serial.print(F("  Y: ")); Serial.print(localSensors.gyroY, 1);
    Serial.print(F("  Z: ")); Serial.println(localSensors.gyroZ, 1);
    
    Serial.println(F("║ MAGNETOMETER (μT):                        ║"));
    Serial.print(F("║   X: ")); Serial.print(localSensors.magX, 1);
    Serial.print(F("  Y: ")); Serial.print(localSensors.magY, 1);
    Serial.print(F("  Z: ")); Serial.println(localSensors.magZ, 1);
    
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    Serial.println(F("║ GPS:                                      ║"));
    Serial.print(F("║   Fix: ")); Serial.println(gpsData.hasValidFix ? "YES" : "NO");
    Serial.print(F("║   Lat: ")); Serial.println(gpsData.latitude, 6);
    Serial.print(F("║   Lon: ")); Serial.println(gpsData.longitude, 6);
    Serial.print(F("║   Alt: ")); Serial.print(gpsData.altitude, 1); Serial.println(F(" m"));
    Serial.print(F("║   Sats: ")); Serial.println(gpsData.satellites);
    
    Serial.println(F("╚═══════════════════════════════════════════╝"));
    Serial.println();
}

void printGPSData() {
    Serial.println();
    Serial.println(F("╔═══════════════════════════════════════════╗"));
    Serial.println(F("║               GPS DATA                    ║"));
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    
    Serial.print(F("║ Fix Status:  "));
    Serial.println(gpsData.hasValidFix ? "VALID" : "NO FIX");
    Serial.print(F("║ Satellites:  ")); Serial.println(gpsData.satellites);
    Serial.print(F("║ HDOP:        ")); Serial.println(gpsData.hdop, 1);
    
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    Serial.print(F("║ Latitude:    ")); Serial.print(gpsData.latitude, 6); Serial.println(F("°"));
    Serial.print(F("║ Longitude:   ")); Serial.print(gpsData.longitude, 6); Serial.println(F("°"));
    Serial.print(F("║ Altitude:    ")); Serial.print(gpsData.altitude, 1); Serial.println(F(" m"));
    
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    Serial.print(F("║ Date:        "));
    Serial.print(gpsData.year); Serial.print(F("-"));
    if (gpsData.month < 10) Serial.print(F("0"));
    Serial.print(gpsData.month); Serial.print(F("-"));
    if (gpsData.day < 10) Serial.print(F("0"));
    Serial.println(gpsData.day);
    
    Serial.print(F("║ Time (UTC):  "));
    if (gpsData.hour < 10) Serial.print(F("0"));
    Serial.print(gpsData.hour); Serial.print(F(":"));
    if (gpsData.minute < 10) Serial.print(F("0"));
    Serial.print(gpsData.minute); Serial.print(F(":"));
    if (gpsData.second < 10) Serial.print(F("0"));
    Serial.println(gpsData.second);
    
    Serial.println(F("╚═══════════════════════════════════════════╝"));
    Serial.println();
}

void printIMUData() {
    Serial.println();
    Serial.println(F("╔═══════════════════════════════════════════╗"));
    Serial.println(F("║             IMU DATA (MPU9250)            ║"));
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    
    Serial.println(F("║ ACCELEROMETER (g):                        ║"));
    Serial.print(F("║   X: ")); Serial.println(localSensors.accelX, 4);
    Serial.print(F("║   Y: ")); Serial.println(localSensors.accelY, 4);
    Serial.print(F("║   Z: ")); Serial.println(localSensors.accelZ, 4);
    
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    Serial.println(F("║ GYROSCOPE (°/s):                          ║"));
    Serial.print(F("║   X: ")); Serial.println(localSensors.gyroX, 2);
    Serial.print(F("║   Y: ")); Serial.println(localSensors.gyroY, 2);
    Serial.print(F("║   Z: ")); Serial.println(localSensors.gyroZ, 2);
    
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    Serial.println(F("║ MAGNETOMETER (μT):                        ║"));
    Serial.print(F("║   X: ")); Serial.println(localSensors.magX, 2);
    Serial.print(F("║   Y: ")); Serial.println(localSensors.magY, 2);
    Serial.print(F("║   Z: ")); Serial.println(localSensors.magZ, 2);
    
    Serial.println(F("╚═══════════════════════════════════════════╝"));
    Serial.println();
}

void printEnvData() {
    Serial.println();
    Serial.println(F("╔═══════════════════════════════════════════╗"));
    Serial.println(F("║          ENVIRONMENTAL DATA               ║"));
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    
    Serial.println(F("║ MCB LOCAL (SHT):                          ║"));
    Serial.print(F("║   Temperature: ")); Serial.print(localSensors.temperature, 2);
    Serial.println(F(" °C"));
    Serial.print(F("║   Humidity:    ")); Serial.print(localSensors.humidity, 1);
    Serial.println(F(" %"));
    
    if (systemStatus.satbusASB) {
        Serial.println(F("╠═══════════════════════════════════════════╣"));
        Serial.println(F("║ ASB MODULE:                               ║"));
        Serial.print(F("║   Temp 1:      ")); Serial.print(asbData.temperature1, 2);
        Serial.println(F(" °C"));
        Serial.print(F("║   Temp 2:      ")); Serial.print(asbData.temperature2, 2);
        Serial.println(F(" °C"));
        Serial.print(F("║   Humidity:    ")); Serial.print(asbData.humidity, 1);
        Serial.println(F(" %"));
        Serial.print(F("║   Pressure:    ")); Serial.print(asbData.pressure, 0);
        Serial.println(F(" Pa"));
        Serial.print(F("║   Altitude:    ")); Serial.print(asbData.altitude, 1);
        Serial.println(F(" m"));
        Serial.print(F("║   AQI:         ")); Serial.println(asbData.airQualityIndex);
        Serial.print(F("║   TVOC:        ")); Serial.print(asbData.tvoc);
        Serial.println(F(" ppb"));
        Serial.print(F("║   CO2:         ")); Serial.print(asbData.co2);
        Serial.println(F(" ppm"));
    } else {
        Serial.println(F("║ ASB MODULE:    OFFLINE                    ║"));
    }
    
    Serial.println(F("╚═══════════════════════════════════════════╝"));
    Serial.println();
}

void printPowerData() {
    Serial.println();
    Serial.println(F("╔═══════════════════════════════════════════╗"));
    Serial.println(F("║             POWER STATUS                  ║"));
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    
    if (systemStatus.satbusBMS) {
        Serial.println(F("║ BMS MODULE:    ONLINE                     ║"));
        Serial.print(F("║   Error Code:  ")); Serial.println(bmsData.errorCode);
        Serial.print(F("║   Battery:     ")); Serial.print(bmsData.batteryVoltage, 2);
        Serial.print(F(" V (")); Serial.print(bmsData.batteryPercent, 0);
        Serial.println(F("%)"));
        Serial.println(F("╠═══════════════════════════════════════════╣"));
        Serial.println(F("║ LDO CHANNELS:                             ║"));
        Serial.print(F("║   CH1: ")); Serial.print(bmsData.ldo1State ? "ON " : "OFF");
        Serial.print(F("   CH2: ")); Serial.println(bmsData.ldo2State ? "ON" : "OFF");
        Serial.print(F("║   CH3: ")); Serial.print(bmsData.ldo3State ? "ON " : "OFF");
        Serial.print(F("   CH4: ")); Serial.println(bmsData.ldo4State ? "ON" : "OFF");
        Serial.println(F("╠═══════════════════════════════════════════╣"));
        Serial.println(F("║ SOLAR PANELS:                             ║"));
        Serial.print(F("║   Panel 1:     ")); Serial.print(bmsData.solarVoltage1, 2);
        Serial.println(F(" V"));
        Serial.print(F("║   Panel 2:     ")); Serial.print(bmsData.solarVoltage2, 2);
        Serial.println(F(" V"));
    } else {
        Serial.println(F("║ BMS MODULE:    OFFLINE                    ║"));
        Serial.println(F("║   No power data available                 ║"));
    }
    
    if (systemStatus.satbusSCC) {
        Serial.println(F("╠═══════════════════════════════════════════╣"));
        Serial.println(F("║ SCC MODULE:    ONLINE                     ║"));
        Serial.print(F("║   Output:      ")); Serial.print(sccData.outputVoltage, 2);
        Serial.print(F(" V / ")); Serial.print(sccData.outputCurrent, 0);
        Serial.println(F(" mA"));
    }
    
    Serial.println(F("╚═══════════════════════════════════════════╝"));
    Serial.println();
}

void printTelemStats() {
    Serial.println();
    Serial.println(F("╔═══════════════════════════════════════════╗"));
    Serial.println(F("║          TELEMETRY STATISTICS             ║"));
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    
    Serial.print(F("║ Packets Sent:    ")); Serial.println(telemetry.getPacketsSent());
    Serial.print(F("║ Packets Failed:  ")); Serial.println(telemetry.getPacketsFailed());
    
    uint32_t total = telemetry.getPacketsSent() + telemetry.getPacketsFailed();
    if (total > 0) {
        float successRate = (float)telemetry.getPacketsSent() / total * 100.0;
        Serial.print(F("║ Success Rate:    ")); Serial.print(successRate, 1);
        Serial.println(F(" %"));
    }
    
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    Serial.print(F("║ Radio Status:    "));
    Serial.println(systemStatus.radioNRF ? "CONNECTED" : "DISCONNECTED");
    Serial.print(F("║ Current Mode:    ")); Serial.println(getModeString(currentMode));
    Serial.print(F("║ TX Interval:     "));
    Serial.print(getTelemetryInterval()); Serial.println(F(" ms"));
    
    Serial.println(F("╚═══════════════════════════════════════════╝"));
    Serial.println();
}

void printMemoryStats() {
    Serial.println();
    Serial.println(F("╔═══════════════════════════════════════════╗"));
    Serial.println(F("║           MEMORY STATISTICS               ║"));
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    
    Serial.print(F("║ Free Heap:       ")); Serial.print(rp2040.getFreeHeap());
    Serial.println(F(" bytes"));
    Serial.print(F("║ Used Heap:       ")); Serial.print(rp2040.getUsedHeap());
    Serial.println(F(" bytes"));
    Serial.print(F("║ Total Heap:      ")); Serial.print(rp2040.getTotalHeap());
    Serial.println(F(" bytes"));
    
    Serial.println(F("╚═══════════════════════════════════════════╝"));
    Serial.println();
}

// ============================================================================
// UART PASSTHROUGH
// ============================================================================

/**
 * Handle UART passthrough mode
 * 
 * In passthrough mode, data from USB Serial is forwarded to the selected UART
 * and vice versa. This allows direct communication with GPS or SATBUS devices.
 * 
 * Type "exit" on USB Serial to disable passthrough mode.
 */
void handleUARTPassthrough() {
    HardwareSerial* targetSerial = nullptr;
    
    // Select target UART
    if (uartPassthroughTarget == 1) {
        targetSerial = &Serial_GPS;
    } else if (uartPassthroughTarget == 2) {
        targetSerial = &Serial_SATBUS;
    } else {
        // Invalid target - disable passthrough
        uartPassthroughActive = false;
        return;
    }
    
    // Forward data from USB Serial to target UART
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        // Check for exit command ("exit" or "quit")
        static char exitBuffer[8] = {0};
        static uint8_t exitIndex = 0;
        
        if (c == '\n' || c == '\r') {
            exitBuffer[exitIndex] = '\0';
            if (strcasecmp(exitBuffer, "exit") == 0 || 
                strcasecmp(exitBuffer, "quit") == 0) {
                // Disable passthrough
                uartPassthroughActive = false;
                uartPassthroughTarget = 0;
                Serial.println(F("\n[UART] Passthrough disabled"));
                memset(exitBuffer, 0, sizeof(exitBuffer));
                exitIndex = 0;
                return;
            }
            // Reset buffer
            memset(exitBuffer, 0, sizeof(exitBuffer));
            exitIndex = 0;
        } else if (exitIndex < sizeof(exitBuffer) - 1) {
            exitBuffer[exitIndex++] = c;
        }
        
        // Forward character to target UART
        targetSerial->write(c);
    }
    
    // Forward data from target UART to USB Serial
    while (targetSerial->available() > 0) {
        char c = targetSerial->read();
        Serial.write(c);
    }
}

// ============================================================================
// EEPROM & BOOT COUNT
// ============================================================================

void loadBootCount() {
    // Boot count is now at offset 0x0002 in new EEPROM layout
    EEPROM.get(EEPROM_BOOT_COUNT_ADDR, bootCount);
    
    // Reset if uninitialized (0xFFFFFFFF) or corrupted (unreasonably high)
    // A boot count over 1 million is likely corruption
    if (bootCount == 0xFFFFFFFF || bootCount > 1000000) {
        bootCount = 0;
        EEPROM.put(EEPROM_BOOT_COUNT_ADDR, bootCount);
        EEPROM.commit();
        Serial.println(F("[BOOT] Boot counter reset (was uninitialized or corrupted)"));
    }
}

void incrementBootCount() {
    bootCount++;
    EEPROM.put(EEPROM_BOOT_COUNT_ADDR, bootCount);
    EEPROM.commit();
}

// ============================================================================
// BLE MODULE
// ============================================================================

void initializeBLE() {
    Serial.print(F("[BOOT] Initializing BLE module... "));
    digitalWrite(PIN_BLE_EN, HIGH);
    delay(100);
    systemStatus.radioBLE = true;
    Serial.println(F("OK"));
}

// ============================================================================
// BUZZER
// ============================================================================

void setBuzzer(uint16_t frequency, uint16_t duration) {
    if (frequency > 0) {
        tone(PIN_BUZZER, frequency, duration);
    } else {
        noTone(PIN_BUZZER);
    }
}
