/*
 * ============================================================================
 * ParameterHandler.h - System Parameter Get/Set Protocol
 * ============================================================================
 * 
 * This module implements a versioned parameter get/set protocol using "#G"
 * packets over USB serial. It provides comprehensive access to system
 * datapoints, configurations, sensor values, and settings.
 * 
 * PROTOCOL FORMAT:
 *   #G<VV><CC><PPPP>[<VALUE>]
 *   
 *   VV    = Version (2 hex chars, e.g., "01")
 *   CC    = Command (2 hex chars)
 *   PPPP  = Parameter code (4 hex chars, uint16)
 *   VALUE = Hex-encoded value (variable length)
 * 
 * COMMAND CODES:
 *   00 = GET          (Host -> MCB) Request parameter value
 *   7F = GET_RESPONSE (MCB -> Host) Response with current value
 *   FF = SET          (Host -> MCB) Set parameter to new value
 *   FE = SET_RESPONSE (MCB -> Host) Confirmation of set value
 *   01 = ERROR        (MCB -> Host) Error response
 * 
 * PARAMETER RANGES:
 *   0x0000-0x00FF: System Info (read-only)
 *   0x0100-0x01FF: System Settings (read/write, persistent)
 *   0x0200-0x02FF: Radio Configuration (read/write, persistent)
 *   0x0300-0x03FF: Sensor Configuration (read/write)
 *   0x0400-0x04FF: Sensor Values (read-only)
 *   0x0500-0x05FF: GPIO Configuration (read/write)
 *   0x0600-0x06FF: Logging Configuration (read/write)
 * 
 * SECURITY:
 *   This protocol is USB-serial only - not available over radio or BLE.
 * 
 * ============================================================================
 */

#ifndef PARAMETER_HANDLER_H
#define PARAMETER_HANDLER_H

#include <Arduino.h>
#include <EEPROM.h>
#include "DataLogger.h"
#include "TelemetryRadio.h"
#include "SatbusParser.h"
#include "version.h"

// ============================================================================
// PROTOCOL CONSTANTS
// ============================================================================

#define PARAM_PROTOCOL_VERSION      0x01
#define PARAM_HEADER                "#G"
#define PARAM_MIN_PACKET_LENGTH     10      // #G + VV + CC + PPPP

// Command Codes
#define PARAM_CMD_GET               0x00
#define PARAM_CMD_GET_RESPONSE      0x7F
#define PARAM_CMD_SET               0xFF
#define PARAM_CMD_SET_RESPONSE      0xFE
#define PARAM_CMD_ERROR             0x01

// Error Codes
#define PARAM_ERR_UNKNOWN           0xE0    // Unknown parameter
#define PARAM_ERR_READONLY          0xE1    // Parameter is read-only
#define PARAM_ERR_INVALID_SIZE      0xE2    // Invalid value size
#define PARAM_ERR_OUT_OF_RANGE      0xE3    // Value out of range
#define PARAM_ERR_NOT_PERMITTED     0xE4    // Operation not permitted
#define PARAM_ERR_EEPROM_FAIL       0xE5    // EEPROM write failed
#define PARAM_ERR_HW_UNAVAILABLE    0xE6    // Hardware not available
#define PARAM_ERR_INVALID_VERSION   0xE7    // Invalid protocol version
#define PARAM_ERR_INVALID_PACKET    0xE8    // Malformed packet

// ============================================================================
// EEPROM LAYOUT
// ============================================================================

// EEPROM addresses for persistent parameters
// Total EEPROM size: 512 bytes
#define EEPROM_MAGIC_ADDR           0x0000  // 2 bytes - Magic number for validity
#define EEPROM_MAGIC_VALUE          0xCB26  // Magic value to detect initialized EEPROM

#define EEPROM_BOOT_COUNT_ADDR      0x0002  // 4 bytes - Boot counter
#define EEPROM_SERIAL_NUM_ADDR      0x0006  // 4 bytes - Device serial number
#define EEPROM_TELEM_FAST_ADDR      0x000A  // 4 bytes - Telemetry interval fast (ms)
#define EEPROM_TELEM_SLOW_ADDR      0x000E  // 4 bytes - Telemetry interval slow (ms)
#define EEPROM_SENSOR_INT_ADDR      0x0012  // 4 bytes - Sensor read interval (ms)
#define EEPROM_HEARTBEAT_ADDR       0x0016  // 4 bytes - Heartbeat interval (ms)
#define EEPROM_GPS_INT_ADDR         0x001A  // 4 bytes - GPS read interval (ms)

#define EEPROM_DEFAULT_MODE_ADDR    0x001E  // 1 byte  - Default system mode
#define EEPROM_RADIO_CHANNEL_ADDR   0x001F  // 1 byte  - Radio channel
#define EEPROM_RADIO_POWER_ADDR     0x0020  // 1 byte  - Radio power level
#define EEPROM_RADIO_RATE_ADDR      0x0021  // 1 byte  - Radio data rate
#define EEPROM_RADIO_PAYLOAD_ADDR   0x0022  // 1 byte  - Radio payload size

#define EEPROM_RADIO_TX_ADDR        0x0023  // 5 bytes - Radio TX address
#define EEPROM_RADIO_RX_ADDR        0x0028  // 5 bytes - Radio RX address

#define EEPROM_IMU_CAL_ADDR         0x002D  // 24 bytes - IMU calibration data
#define EEPROM_LOG_ENABLED_ADDR     0x0045  // 1 byte  - Logging enabled
#define EEPROM_LOG_INTERVAL_ADDR    0x0046  // 4 bytes - Log interval (ms)
#define EEPROM_MESH_RELAY_ADDR      0x004A  // 1 byte  - Mesh relay enabled
#define EEPROM_MESH_LOG_ADDR        0x004B  // 1 byte  - Mesh SD logging enabled

#define EEPROM_CONFIG_END           0x0050  // End of config area

// ============================================================================
// PARAMETER DATA TYPES
// ============================================================================

enum ParamType : uint8_t {
    PARAM_UINT8,
    PARAM_UINT16,
    PARAM_UINT32,
    PARAM_INT8,
    PARAM_INT16,
    PARAM_INT32,
    PARAM_FLOAT,
    PARAM_DOUBLE,
    PARAM_BOOL,
    PARAM_STRING,
    PARAM_BYTES
};

// ============================================================================
// PARAMETER ACCESS FLAGS
// ============================================================================

enum ParamAccess : uint8_t {
    ACCESS_NONE     = 0x00,
    ACCESS_READ     = 0x01,
    ACCESS_WRITE    = 0x02,
    ACCESS_PERSIST  = 0x04,     // Save to EEPROM on write
    ACCESS_VOLATILE = 0x08,     // Value changes frequently (sensor data)
    ACCESS_RO       = ACCESS_READ,
    ACCESS_RW       = ACCESS_READ | ACCESS_WRITE,
    ACCESS_RWP      = ACCESS_READ | ACCESS_WRITE | ACCESS_PERSIST
};

// ============================================================================
// PARAMETER CODES
// ============================================================================

// System Info (0x00xx) - Read Only
#define PARAM_SYS_SERIAL            0x0001  // Device serial number (uint32)
#define PARAM_SYS_VERSION           0x0002  // Firmware version (string)
#define PARAM_SYS_UPTIME            0x0003  // System uptime (uint32, seconds)
#define PARAM_SYS_BOOTCOUNT         0x0004  // Boot counter (uint32)
#define PARAM_SYS_MODE              0x0005  // Current mode (uint8)
#define PARAM_SYS_STATUS            0x0006  // Status flags (uint16)
#define PARAM_SYS_FREE_HEAP         0x0007  // Free heap memory (uint32)
#define PARAM_SYS_USED_HEAP         0x0008  // Used heap memory (uint32)
#define PARAM_SYS_TOTAL_HEAP        0x0009  // Total heap memory (uint32)

// System Settings (0x01xx) - Read/Write, Persistent
#define PARAM_CFG_SERIAL            0x0100  // Configurable serial number (uint32)
#define PARAM_CFG_TELEM_FAST        0x0101  // Telemetry interval fast (uint32, ms)
#define PARAM_CFG_TELEM_SLOW        0x0102  // Telemetry interval slow (uint32, ms)
#define PARAM_CFG_SENSOR_INT        0x0103  // Sensor read interval (uint32, ms)
#define PARAM_CFG_HEARTBEAT         0x0104  // Heartbeat interval (uint32, ms)
#define PARAM_CFG_GPS_INT           0x0105  // GPS read interval (uint32, ms)
#define PARAM_CFG_DEFAULT_MODE      0x0110  // Default system mode (uint8)
#define PARAM_CFG_SATBUS_POLL       0x0111  // SATBUS poll interval (uint32, ms)

// Radio Configuration (0x02xx)
#define PARAM_RADIO_CHANNEL         0x0200  // Radio channel (uint8, 0-125)
#define PARAM_RADIO_POWER           0x0201  // Radio power level (uint8, 0-3)
#define PARAM_RADIO_RATE            0x0202  // Radio data rate (uint8)
#define PARAM_RADIO_PAYLOAD         0x0203  // Payload size (uint8)
#define PARAM_RADIO_TX_ADDR         0x0210  // TX address (5 bytes)
#define PARAM_RADIO_RX_ADDR         0x0211  // RX address (5 bytes)
#define PARAM_RADIO_PKTS_SENT       0x0220  // Packets sent (uint32, read-only)
#define PARAM_RADIO_PKTS_FAIL       0x0221  // Packets failed (uint32, read-only)
#define PARAM_RADIO_CONNECTED       0x0222  // Radio connected (bool, read-only)

// Sensor Configuration (0x03xx)
#define PARAM_SENS_SHT_TYPE         0x0300  // SHT sensor type (uint8, read-only)
#define PARAM_SENS_IMU_PRESENT      0x0301  // IMU present (bool, read-only)
#define PARAM_SENS_GPS_ENABLED      0x0302  // GPS enabled (bool)
#define PARAM_SENS_IMU_CAL_AX       0x0310  // IMU cal accel X offset (int16)
#define PARAM_SENS_IMU_CAL_AY       0x0311  // IMU cal accel Y offset (int16)
#define PARAM_SENS_IMU_CAL_AZ       0x0312  // IMU cal accel Z offset (int16)
#define PARAM_SENS_IMU_CAL_GX       0x0320  // IMU cal gyro X offset (int16)
#define PARAM_SENS_IMU_CAL_GY       0x0321  // IMU cal gyro Y offset (int16)
#define PARAM_SENS_IMU_CAL_GZ       0x0322  // IMU cal gyro Z offset (int16)
#define PARAM_SENS_IMU_CAL_MX       0x0330  // IMU cal mag X offset (int16)
#define PARAM_SENS_IMU_CAL_MY       0x0331  // IMU cal mag Y offset (int16)
#define PARAM_SENS_IMU_CAL_MZ       0x0332  // IMU cal mag Z offset (int16)

// Sensor Values (0x04xx) - Read Only, Volatile
#define PARAM_VAL_TEMP              0x0400  // Temperature (float, C)
#define PARAM_VAL_HUMIDITY          0x0401  // Humidity (float, %)
#define PARAM_VAL_ACCEL_X           0x0410  // Accel X (float, g)
#define PARAM_VAL_ACCEL_Y           0x0411  // Accel Y (float, g)
#define PARAM_VAL_ACCEL_Z           0x0412  // Accel Z (float, g)
#define PARAM_VAL_GYRO_X            0x0420  // Gyro X (float, deg/s)
#define PARAM_VAL_GYRO_Y            0x0421  // Gyro Y (float, deg/s)
#define PARAM_VAL_GYRO_Z            0x0422  // Gyro Z (float, deg/s)
#define PARAM_VAL_MAG_X             0x0430  // Mag X (float, uT)
#define PARAM_VAL_MAG_Y             0x0431  // Mag Y (float, uT)
#define PARAM_VAL_MAG_Z             0x0432  // Mag Z (float, uT)
#define PARAM_VAL_GPS_LAT           0x0450  // GPS Latitude (double)
#define PARAM_VAL_GPS_LON           0x0451  // GPS Longitude (double)
#define PARAM_VAL_GPS_ALT           0x0452  // GPS Altitude (float)
#define PARAM_VAL_GPS_SATS          0x0453  // GPS Satellites (uint8)
#define PARAM_VAL_GPS_FIX           0x0454  // GPS Fix Status (bool)
#define PARAM_VAL_GPS_HDOP          0x0455  // GPS HDOP (float)

// GPIO Configuration (0x05xx)
#define PARAM_GPIO_LED1             0x0500  // LED1 State (bool)
#define PARAM_GPIO_LED2             0x0501  // LED2 State (bool)
#define PARAM_GPIO_LED3             0x0502  // LED3 State (bool)
#define PARAM_GPIO_BUZZER           0x0503  // Buzzer frequency (uint16, Hz)
#define PARAM_GPIO_BLE_EN           0x0504  // BLE Enable (bool)
#define PARAM_GPIO_RBF              0x0510  // RBF Pin State (bool, read-only)
#define PARAM_GPIO_ADC0             0x0520  // ADC0 Value (uint16, read-only)
#define PARAM_GPIO_ADC1             0x0521  // ADC1 Value (uint16, read-only)
#define PARAM_GPIO_ADC2             0x0522  // ADC2 Value (uint16, read-only)

// Logging Configuration (0x06xx)
#define PARAM_LOG_ENABLED           0x0600  // Logging enabled (bool)
#define PARAM_LOG_INTERVAL          0x0601  // Log interval (uint32, ms)
#define PARAM_LOG_SD_ENABLED        0x0602  // SD logging enabled (bool)
#define PARAM_LOG_FILENAME          0x0610  // Current log filename (string, read-only)

// Mesh Configuration (0x07xx)
#define PARAM_MESH_RELAY            0x0700  // Mesh relay enabled (bool)
#define PARAM_MESH_LOG_SD           0x0701  // Mesh SD logging (bool)
#define PARAM_MESH_ACTIVE_COUNT     0x0710  // Active CubeSats (uint8, read-only)
#define PARAM_MESH_PKTS_RX          0x0720  // Packets received (uint32, read-only)
#define PARAM_MESH_PKTS_RELAY       0x0721  // Packets relayed (uint32, read-only)

// BMS (Battery Management System) Values (0x08xx) - Read Only
#define PARAM_BMS_CONNECTED         0x0800  // BMS connected (bool)
#define PARAM_BMS_ERROR             0x0801  // BMS error code (uint8)
#define PARAM_BMS_BAT_V             0x0810  // Battery voltage (float, V)
#define PARAM_BMS_BAT_PCT           0x0811  // Battery percentage (float, %)
#define PARAM_BMS_LDO1              0x0820  // LDO1 state (bool)
#define PARAM_BMS_LDO2              0x0821  // LDO2 state (bool)
#define PARAM_BMS_LDO3              0x0822  // LDO3 state (bool)
#define PARAM_BMS_LDO4              0x0823  // LDO4 state (bool)
#define PARAM_BMS_LDO_ALL           0x0824  // All LDO states packed (uint8)
#define PARAM_BMS_SOLAR1            0x0830  // Solar panel 1 voltage (float, V)
#define PARAM_BMS_SOLAR2            0x0831  // Solar panel 2 voltage (float, V)

// ASB (Atmospheric Sensing Board) Values (0x09xx) - Read Only
#define PARAM_ASB_CONNECTED         0x0900  // ASB connected (bool)
#define PARAM_ASB_TEMP1             0x0910  // SHT4x temperature (float, °C)
#define PARAM_ASB_TEMP2             0x0911  // BMP180 temperature (float, °C)
#define PARAM_ASB_HUMIDITY          0x0912  // Relative humidity (float, %)
#define PARAM_ASB_PRESSURE          0x0913  // Pressure (float, Pa)
#define PARAM_ASB_ALTITUDE          0x0914  // Pressure altitude (float, m)
#define PARAM_ASB_AQI               0x0920  // Air Quality Index (uint8, 1-5)
#define PARAM_ASB_TVOC              0x0921  // Total VOC (uint16, ppb)
#define PARAM_ASB_ECO2              0x0922  // Estimated CO2 (uint16, ppm)
#define PARAM_ASB_CO2               0x0923  // CO2 concentration (uint16, ppm)
#define PARAM_ASB_PM1_0             0x0930  // PM 1.0 mass conc (float, µg/m³)
#define PARAM_ASB_PM2_5             0x0931  // PM 2.5 mass conc (float, µg/m³)
#define PARAM_ASB_PM4_0             0x0932  // PM 4.0 mass conc (float, µg/m³)
#define PARAM_ASB_PM10              0x0933  // PM 10 mass conc (float, µg/m³)
#define PARAM_ASB_NC0_5             0x0940  // Num conc 0-0.5µm (float, /cm³)
#define PARAM_ASB_NC1_0             0x0941  // Num conc 0.5-1µm (float, /cm³)
#define PARAM_ASB_NC2_5             0x0942  // Num conc 1-2.5µm (float, /cm³)
#define PARAM_ASB_NC4_0             0x0943  // Num conc 2.5-4µm (float, /cm³)
#define PARAM_ASB_NC10              0x0944  // Num conc 4-10µm (float, /cm³)
#define PARAM_ASB_PART_SIZE         0x0945  // Typical particle size (float, µm)

// ASBData byte offsets (reordered struct: 15 floats first at 0-56, then AQI/tvoc/eco2/co2)
// Must match SatbusParser.h ASBData layout exactly.
#define ASB_OFFSET_TEMP1            0
#define ASB_OFFSET_TEMP2            4
#define ASB_OFFSET_HUMIDITY         8
#define ASB_OFFSET_PRESSURE        12
#define ASB_OFFSET_ALTITUDE        16
#define ASB_OFFSET_PM1_0           20
#define ASB_OFFSET_PM2_5           24
#define ASB_OFFSET_PM4_0           28
#define ASB_OFFSET_PM10            32
#define ASB_OFFSET_NC0_5           36
#define ASB_OFFSET_NC1_0           40
#define ASB_OFFSET_NC2_5           44
#define ASB_OFFSET_NC4_0           48
#define ASB_OFFSET_NC10            52
#define ASB_OFFSET_TYPICAL_PARTICLE 56
#define ASB_OFFSET_AQI             60
#define ASB_OFFSET_TVOC            62
#define ASB_OFFSET_ECO2            64
#define ASB_OFFSET_CO2             66

// SCC (Solar Charge Controller) Values (0x0Axx) - Read Only
#define PARAM_SCC_CONNECTED         0x0A00  // SCC connected (bool)
#define PARAM_SCC_V0                0x0A10  // Solar panel 0 voltage (float, V)
#define PARAM_SCC_V1                0x0A11  // Solar panel 1 voltage (float, V)
#define PARAM_SCC_V2                0x0A12  // Solar panel 2 voltage (float, V)
#define PARAM_SCC_V3                0x0A13  // Solar panel 3 voltage (float, V)
#define PARAM_SCC_V4                0x0A14  // Solar panel 4 voltage (float, V)
#define PARAM_SCC_V5                0x0A15  // Solar panel 5 voltage (float, V)
#define PARAM_SCC_T0                0x0A20  // Temperature channel 0 (float, °C)
#define PARAM_SCC_T1                0x0A21  // Temperature channel 1 (float, °C)
#define PARAM_SCC_T2                0x0A22  // Temperature channel 2 (float, °C)
#define PARAM_SCC_T3                0x0A23  // Temperature channel 3 (float, °C)
#define PARAM_SCC_T4                0x0A24  // Temperature channel 4 (float, °C)
#define PARAM_SCC_T5                0x0A25  // Temperature channel 5 (float, °C)
#define PARAM_SCC_T6                0x0A26  // Temperature channel 6 (float, °C)
#define PARAM_SCC_T7                0x0A27  // Temperature channel 7 (float, °C)
#define PARAM_SCC_VOUT              0x0A30  // Output voltage (float, V)
#define PARAM_SCC_IOUT              0x0A31  // Output current (float, mA)
#define PARAM_SCC_MODE              0x0A32  // System mode (uint8)

// ============================================================================
// SYSTEM CONFIGURATION STRUCTURE
// ============================================================================

/**
 * SystemConfig
 * 
 * Holds all configurable system parameters that are persisted to EEPROM.
 * This replaces hardcoded #define values with runtime-configurable settings.
 */
struct SystemConfig {
    // Device identification
    uint32_t serialNumber;
    
    // Timing intervals (milliseconds)
    uint32_t telemetryIntervalFast;
    uint32_t telemetryIntervalSlow;
    uint32_t sensorReadInterval;
    uint32_t heartbeatInterval;
    uint32_t gpsReadInterval;
    uint32_t satbusPollInterval;
    
    // Mode settings
    uint8_t defaultMode;
    
    // Radio configuration
    uint8_t radioChannel;
    uint8_t radioPowerLevel;
    uint8_t radioDataRate;
    uint8_t radioPayloadSize;
    uint8_t radioTxAddress[5];
    uint8_t radioRxAddress[5];
    
    // Logging configuration
    bool loggingEnabled;
    uint32_t logInterval;
    
    // Mesh configuration
    bool meshRelayEnabled;
    bool meshLogToSD;
    
    // Constructor with defaults matching original #define values
    SystemConfig() :
        serialNumber(0x00000044),
        telemetryIntervalFast(1000),
        telemetryIntervalSlow(5000),
        sensorReadInterval(500),
        heartbeatInterval(1000),
        gpsReadInterval(1250),
        satbusPollInterval(100),
        defaultMode(2),  // NOMINAL
        radioChannel(62),
        radioPowerLevel(0),  // RF24_PA_LOW
        radioDataRate(0),    // RF24_1MBPS
        radioPayloadSize(32),
        loggingEnabled(true),
        logInterval(5000),
        meshRelayEnabled(false),
        meshLogToSD(false)
    {
        // Default radio addresses: "EFEF0"
        radioTxAddress[0] = 0x45;
        radioTxAddress[1] = 0x46;
        radioTxAddress[2] = 0x45;
        radioTxAddress[3] = 0x46;
        radioTxAddress[4] = 0x30;
        
        radioRxAddress[0] = 0x45;
        radioRxAddress[1] = 0x46;
        radioRxAddress[2] = 0x45;
        radioRxAddress[3] = 0x46;
        radioRxAddress[4] = 0x30;
    }
};

// ============================================================================
// PARAMETER HANDLER CLASS
// ============================================================================

/**
 * ParameterHandler Class
 * 
 * Handles the #G protocol for getting and setting system parameters.
 * This class manages the parameter registry and EEPROM persistence.
 * 
 * USAGE:
 *   ParameterHandler params;
 *   params.begin(&config, &sensorData, &gpsData, ...);
 *   
 *   // In serial processing:
 *   if (line.startsWith("#G")) {
 *       params.processPacket(line.c_str(), line.length());
 *   }
 */
class ParameterHandler {
public:
    ParameterHandler();
    
    /**
     * Initialize the parameter handler
     * Must be called after EEPROM.begin() but before using other functions
     */
    void begin(SystemConfig* config);
    
    /**
     * Set pointers to external data structures for sensor values
     * These are read-only parameters that reflect current system state
     */
    void setSensorDataPtr(void* sensorData);
    void setGPSDataPtr(void* gpsData);
    void setSystemStatusPtr(void* status);
    void setTelemetryPtr(void* telemetry);
    void setDataLoggerPtr(void* dataLogger);
    void setMeshPtr(void* mesh);
    void setRadioPtr(void* radio);
    
    /**
     * Set pointers to SATBUS module data structures
     */
    void setBMSDataPtr(void* bmsData);
    void setASBDataPtr(void* asbData);
    void setSCCDataPtr(void* sccData);
    
    /**
     * Set pointers to system state variables
     */
    void setUptimePtr(uint32_t* uptime);
    void setBootCountPtr(uint32_t* bootCount);
    void setModePtr(uint8_t* mode);
    
    /**
     * Set pointers to telemetry statistics
     */
    void setPacketsSentPtr(uint32_t* pktsSent);
    void setPacketsFailedPtr(uint32_t* pktsFailed);
    
    /**
     * Set pointer to log filename string
     */
    void setLogFilenamePtr(const char** filename);
    
    /**
     * Process an incoming #G packet
     * @param packet Pointer to packet string (including #G header)
     * @param length Length of packet string
     * @return true if packet was processed successfully
     */
    bool processPacket(const char* packet, size_t length);
    
    /**
     * Load configuration from EEPROM
     * Called automatically by begin(), but can be called manually
     * @return true if valid configuration was loaded
     */
    bool loadConfig();
    
    /**
     * Save configuration to EEPROM
     * Called automatically when persistent parameters are changed
     * @return true if save was successful
     */
    bool saveConfig();
    
    /**
     * Reset configuration to defaults
     * Clears EEPROM and resets all parameters to default values
     */
    void resetToDefaults();
    
    /**
     * Get the system configuration structure
     */
    SystemConfig* getConfig() { return _config; }
    
    /**
     * Check if EEPROM has valid configuration
     */
    bool isConfigValid();
    
    /**
     * Print all parameter values to Serial (for debugging)
     */
    void printAllParameters();
    
private:
    SystemConfig* _config;
    
    // External data pointers (for read-only sensor values)
    void* _sensorData;
    void* _gpsData;
    void* _systemStatus;
    void* _telemetry;
    void* _dataLogger;
    void* _mesh;
    void* _radio;
    
    // SATBUS module data pointers
    void* _bmsData;
    void* _asbData;
    void* _sccData;
    
    // System state pointers
    uint32_t* _uptimePtr;
    uint32_t* _bootCountPtr;
    uint8_t* _modePtr;
    
    // Telemetry statistics pointers
    uint32_t* _packetsSentPtr;
    uint32_t* _packetsFailedPtr;
    
    // Log filename pointer
    const char** _logFilenamePtr;
    
    // Packet parsing
    bool parseHexByte(const char* str, uint8_t* value);
    bool parseHexUint16(const char* str, uint16_t* value);
    bool parseHexValue(const char* str, size_t len, uint8_t* buffer, size_t bufLen);
    
    // Command handlers
    void handleGetCommand(uint16_t paramCode);
    void handleSetCommand(uint16_t paramCode, const uint8_t* value, size_t valueLen);
    
    // Response generators
    void sendResponse(uint8_t cmd, uint16_t paramCode, const uint8_t* value, size_t valueLen);
    void sendError(uint16_t paramCode, uint8_t errorCode);
    
    // Parameter access
    bool getParameterValue(uint16_t paramCode, uint8_t* buffer, size_t* length);
    bool setParameterValue(uint16_t paramCode, const uint8_t* value, size_t valueLen);
    uint8_t getParameterAccess(uint16_t paramCode);
    uint8_t getParameterSize(uint16_t paramCode);
    ParamType getParameterType(uint16_t paramCode);
    
    // EEPROM helpers
    void saveParameterToEEPROM(uint16_t paramCode);
    void loadParameterFromEEPROM(uint16_t paramCode);
    
    // Hex encoding for responses
    void encodeHex(const uint8_t* data, size_t len, char* output);
    char hexChar(uint8_t nibble);
    
    // Big-endian (MSB) byte order conversion helpers
    void toBigEndian16(uint16_t value, uint8_t* buffer);
    void toBigEndian32(uint32_t value, uint8_t* buffer);
    void toBigEndian64(uint64_t value, uint8_t* buffer);
    uint16_t fromBigEndian16(const uint8_t* buffer);
    uint32_t fromBigEndian32(const uint8_t* buffer);
    uint64_t fromBigEndian64(const uint8_t* buffer);
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

ParameterHandler::ParameterHandler() :
    _config(nullptr),
    _sensorData(nullptr),
    _gpsData(nullptr),
    _systemStatus(nullptr),
    _telemetry(nullptr),
    _dataLogger(nullptr),
    _mesh(nullptr),
    _radio(nullptr),
    _bmsData(nullptr),
    _asbData(nullptr),
    _sccData(nullptr),
    _uptimePtr(nullptr),
    _bootCountPtr(nullptr),
    _modePtr(nullptr),
    _packetsSentPtr(nullptr),
    _packetsFailedPtr(nullptr),
    _logFilenamePtr(nullptr)
{
}

void ParameterHandler::begin(SystemConfig* config) {
    _config = config;
    
    // Check if EEPROM has valid configuration
    if (isConfigValid()) {
        loadConfig();
        Serial.println(F("[PARAM] Configuration loaded from EEPROM"));
    } else {
        // First boot or corrupted EEPROM - save defaults
        Serial.println(F("[PARAM] No valid config found, saving defaults"));
        saveConfig();
    }
}

void ParameterHandler::setSensorDataPtr(void* sensorData) { _sensorData = sensorData; }
void ParameterHandler::setGPSDataPtr(void* gpsData) { _gpsData = gpsData; }
void ParameterHandler::setSystemStatusPtr(void* status) { _systemStatus = status; }
void ParameterHandler::setTelemetryPtr(void* telemetry) { _telemetry = telemetry; }
void ParameterHandler::setDataLoggerPtr(void* dataLogger) { _dataLogger = dataLogger; }
void ParameterHandler::setMeshPtr(void* mesh) { _mesh = mesh; }
void ParameterHandler::setRadioPtr(void* radio) { _radio = radio; }
void ParameterHandler::setBMSDataPtr(void* bmsData) { _bmsData = bmsData; }
void ParameterHandler::setASBDataPtr(void* asbData) { _asbData = asbData; }
void ParameterHandler::setSCCDataPtr(void* sccData) { _sccData = sccData; }

void ParameterHandler::setUptimePtr(uint32_t* uptime) { _uptimePtr = uptime; }
void ParameterHandler::setBootCountPtr(uint32_t* bootCount) { _bootCountPtr = bootCount; }
void ParameterHandler::setModePtr(uint8_t* mode) { _modePtr = mode; }
void ParameterHandler::setPacketsSentPtr(uint32_t* pktsSent) { _packetsSentPtr = pktsSent; }
void ParameterHandler::setPacketsFailedPtr(uint32_t* pktsFailed) { _packetsFailedPtr = pktsFailed; }
void ParameterHandler::setLogFilenamePtr(const char** filename) { _logFilenamePtr = filename; }

bool ParameterHandler::isConfigValid() {
    uint16_t magic;
    EEPROM.get(EEPROM_MAGIC_ADDR, magic);
    return (magic == EEPROM_MAGIC_VALUE);
}

bool ParameterHandler::loadConfig() {
    if (!_config) return false;
    
    // Load all persistent parameters from EEPROM
    EEPROM.get(EEPROM_SERIAL_NUM_ADDR, _config->serialNumber);
    EEPROM.get(EEPROM_TELEM_FAST_ADDR, _config->telemetryIntervalFast);
    EEPROM.get(EEPROM_TELEM_SLOW_ADDR, _config->telemetryIntervalSlow);
    EEPROM.get(EEPROM_SENSOR_INT_ADDR, _config->sensorReadInterval);
    EEPROM.get(EEPROM_HEARTBEAT_ADDR, _config->heartbeatInterval);
    EEPROM.get(EEPROM_GPS_INT_ADDR, _config->gpsReadInterval);
    
    EEPROM.get(EEPROM_DEFAULT_MODE_ADDR, _config->defaultMode);
    EEPROM.get(EEPROM_RADIO_CHANNEL_ADDR, _config->radioChannel);
    EEPROM.get(EEPROM_RADIO_POWER_ADDR, _config->radioPowerLevel);
    EEPROM.get(EEPROM_RADIO_RATE_ADDR, _config->radioDataRate);
    EEPROM.get(EEPROM_RADIO_PAYLOAD_ADDR, _config->radioPayloadSize);
    
    for (int i = 0; i < 5; i++) {
        _config->radioTxAddress[i] = EEPROM.read(EEPROM_RADIO_TX_ADDR + i);
        _config->radioRxAddress[i] = EEPROM.read(EEPROM_RADIO_RX_ADDR + i);
    }
    
    uint8_t logEnabled;
    EEPROM.get(EEPROM_LOG_ENABLED_ADDR, logEnabled);
    _config->loggingEnabled = (logEnabled != 0);
    EEPROM.get(EEPROM_LOG_INTERVAL_ADDR, _config->logInterval);
    
    uint8_t meshRelay, meshLog;
    EEPROM.get(EEPROM_MESH_RELAY_ADDR, meshRelay);
    EEPROM.get(EEPROM_MESH_LOG_ADDR, meshLog);
    _config->meshRelayEnabled = (meshRelay != 0);
    _config->meshLogToSD = (meshLog != 0);
    
    return true;
}

bool ParameterHandler::saveConfig() {
    if (!_config) return false;
    
    // Write magic number to indicate valid config
    uint16_t magic = EEPROM_MAGIC_VALUE;
    EEPROM.put(EEPROM_MAGIC_ADDR, magic);
    
    // Save all persistent parameters to EEPROM
    EEPROM.put(EEPROM_SERIAL_NUM_ADDR, _config->serialNumber);
    EEPROM.put(EEPROM_TELEM_FAST_ADDR, _config->telemetryIntervalFast);
    EEPROM.put(EEPROM_TELEM_SLOW_ADDR, _config->telemetryIntervalSlow);
    EEPROM.put(EEPROM_SENSOR_INT_ADDR, _config->sensorReadInterval);
    EEPROM.put(EEPROM_HEARTBEAT_ADDR, _config->heartbeatInterval);
    EEPROM.put(EEPROM_GPS_INT_ADDR, _config->gpsReadInterval);
    
    EEPROM.put(EEPROM_DEFAULT_MODE_ADDR, _config->defaultMode);
    EEPROM.put(EEPROM_RADIO_CHANNEL_ADDR, _config->radioChannel);
    EEPROM.put(EEPROM_RADIO_POWER_ADDR, _config->radioPowerLevel);
    EEPROM.put(EEPROM_RADIO_RATE_ADDR, _config->radioDataRate);
    EEPROM.put(EEPROM_RADIO_PAYLOAD_ADDR, _config->radioPayloadSize);
    
    for (int i = 0; i < 5; i++) {
        EEPROM.write(EEPROM_RADIO_TX_ADDR + i, _config->radioTxAddress[i]);
        EEPROM.write(EEPROM_RADIO_RX_ADDR + i, _config->radioRxAddress[i]);
    }
    
    EEPROM.put(EEPROM_LOG_ENABLED_ADDR, (uint8_t)(_config->loggingEnabled ? 1 : 0));
    EEPROM.put(EEPROM_LOG_INTERVAL_ADDR, _config->logInterval);
    
    EEPROM.put(EEPROM_MESH_RELAY_ADDR, (uint8_t)(_config->meshRelayEnabled ? 1 : 0));
    EEPROM.put(EEPROM_MESH_LOG_ADDR, (uint8_t)(_config->meshLogToSD ? 1 : 0));
    
    EEPROM.commit();
    return true;
}

void ParameterHandler::resetToDefaults() {
    if (_config) {
        // Reset to constructor defaults
        *_config = SystemConfig();
        
        // Also reset boot count to 0
        uint32_t zeroBootCount = 0;
        EEPROM.put(EEPROM_BOOT_COUNT_ADDR, zeroBootCount);
        if (_bootCountPtr) {
            *_bootCountPtr = 0;
        }
        
        saveConfig();
        // Sync logging state to DataLogger so reset enables logging
        if (_dataLogger) {
            ((DataLogger*)_dataLogger)->setEnabled(_config->loggingEnabled);
            ((DataLogger*)_dataLogger)->setSensorInterval(_config->logInterval);
            ((DataLogger*)_dataLogger)->setGPSInterval(_config->logInterval);
        }
        Serial.println(F("[PARAM] Configuration reset to defaults (including boot count)"));
    }
}

bool ParameterHandler::processPacket(const char* packet, size_t length) {
    // Minimum packet: #G + VV + CC + PPPP = 10 chars
    if (length < PARAM_MIN_PACKET_LENGTH) {
        sendError(0x0000, PARAM_ERR_INVALID_PACKET);
        return false;
    }
    
    // Verify header
    if (packet[0] != '#' || packet[1] != 'G') {
        return false;  // Not a #G packet
    }
    
    // Parse version
    uint8_t version;
    if (!parseHexByte(&packet[2], &version)) {
        sendError(0x0000, PARAM_ERR_INVALID_PACKET);
        return false;
    }
    
    // Check version
    if (version != PARAM_PROTOCOL_VERSION) {
        sendError(0x0000, PARAM_ERR_INVALID_VERSION);
        return false;
    }
    
    // Parse command
    uint8_t command;
    if (!parseHexByte(&packet[4], &command)) {
        sendError(0x0000, PARAM_ERR_INVALID_PACKET);
        return false;
    }
    
    // Parse parameter code
    uint16_t paramCode;
    if (!parseHexUint16(&packet[6], &paramCode)) {
        sendError(0x0000, PARAM_ERR_INVALID_PACKET);
        return false;
    }
    
    // Handle command
    switch (command) {
        case PARAM_CMD_GET:
            handleGetCommand(paramCode);
            break;
            
        case PARAM_CMD_SET: {
            // Parse value (remaining bytes after PPPP)
            size_t valueOffset = 10;  // #G + VV + CC + PPPP
            size_t valueHexLen = length - valueOffset;
            
            if (valueHexLen == 0 || (valueHexLen % 2) != 0) {
                sendError(paramCode, PARAM_ERR_INVALID_SIZE);
                return false;
            }
            
            size_t valueLen = valueHexLen / 2;
            uint8_t valueBuffer[32];
            
            if (valueLen > sizeof(valueBuffer)) {
                sendError(paramCode, PARAM_ERR_INVALID_SIZE);
                return false;
            }
            
            if (!parseHexValue(&packet[valueOffset], valueHexLen, valueBuffer, valueLen)) {
                sendError(paramCode, PARAM_ERR_INVALID_PACKET);
                return false;
            }
            
            handleSetCommand(paramCode, valueBuffer, valueLen);
            break;
        }
            
        default:
            sendError(paramCode, PARAM_ERR_INVALID_PACKET);
            return false;
    }
    
    return true;
}

void ParameterHandler::handleGetCommand(uint16_t paramCode) {
    uint8_t buffer[32];
    size_t length = sizeof(buffer);
    
    if (getParameterValue(paramCode, buffer, &length)) {
        sendResponse(PARAM_CMD_GET_RESPONSE, paramCode, buffer, length);
    } else {
        sendError(paramCode, PARAM_ERR_UNKNOWN);
    }
}

void ParameterHandler::handleSetCommand(uint16_t paramCode, const uint8_t* value, size_t valueLen) {
    // Check if parameter exists and is writable
    uint8_t access = getParameterAccess(paramCode);
    
    if (access == ACCESS_NONE) {
        sendError(paramCode, PARAM_ERR_UNKNOWN);
        return;
    }
    
    if (!(access & ACCESS_WRITE)) {
        sendError(paramCode, PARAM_ERR_READONLY);
        return;
    }
    
    // Check value size
    uint8_t expectedSize = getParameterSize(paramCode);
    if (expectedSize > 0 && valueLen != expectedSize) {
        sendError(paramCode, PARAM_ERR_INVALID_SIZE);
        return;
    }
    
    // Set the value
    if (setParameterValue(paramCode, value, valueLen)) {
        // If persistent, save to EEPROM
        if (access & ACCESS_PERSIST) {
            saveConfig();
        }
        
        // Send confirmation with the new value
        uint8_t readBuffer[32];
        size_t readLen = sizeof(readBuffer);
        if (getParameterValue(paramCode, readBuffer, &readLen)) {
            sendResponse(PARAM_CMD_SET_RESPONSE, paramCode, readBuffer, readLen);
        } else {
            sendResponse(PARAM_CMD_SET_RESPONSE, paramCode, value, valueLen);
        }
    } else {
        sendError(paramCode, PARAM_ERR_OUT_OF_RANGE);
    }
}

bool ParameterHandler::getParameterValue(uint16_t paramCode, uint8_t* buffer, size_t* length) {
    if (!_config) return false;
    
    // SensorData, GPSData from SatbusParser.h; SystemStatus, BMSData from TelemetryRadio.h / SatbusParser.h (included above)
    switch (paramCode) {
        // ====== System Info (0x00xx) - Read Only ======
        case PARAM_SYS_SERIAL:
            toBigEndian32(_config->serialNumber, buffer);
            *length = 4;
            return true;
            
        case PARAM_SYS_VERSION: {
            const char* version = FIRMWARE_VERSION;
            size_t vlen = strlen(version);
            memcpy(buffer, version, vlen);
            *length = vlen;
            return true;
        }
            
        case PARAM_SYS_UPTIME:
            if (_uptimePtr) {
                toBigEndian32(*_uptimePtr, buffer);
                *length = 4;
                return true;
            }
            return false;
            
        case PARAM_SYS_BOOTCOUNT:
            if (_bootCountPtr) {
                toBigEndian32(*_bootCountPtr, buffer);
                *length = 4;
                return true;
            }
            return false;
            
        case PARAM_SYS_MODE:
            if (_modePtr) {
                buffer[0] = *_modePtr;
                *length = 1;
                return true;
            }
            return false;
            
        case PARAM_SYS_STATUS:
            if (_systemStatus) {
                // SystemStatus is a bitfield structure, copy as uint16 in big-endian
                uint16_t status;
                memcpy(&status, _systemStatus, 2);
                toBigEndian16(status, buffer);
                *length = 2;
                return true;
            }
            return false;
            
        case PARAM_SYS_FREE_HEAP: {
            uint32_t freeHeap = rp2040.getFreeHeap();
            toBigEndian32(freeHeap, buffer);
            *length = 4;
            return true;
        }
            
        case PARAM_SYS_USED_HEAP: {
            uint32_t usedHeap = rp2040.getUsedHeap();
            toBigEndian32(usedHeap, buffer);
            *length = 4;
            return true;
        }
            
        case PARAM_SYS_TOTAL_HEAP: {
            uint32_t totalHeap = rp2040.getTotalHeap();
            toBigEndian32(totalHeap, buffer);
            *length = 4;
            return true;
        }
            
        // ====== System Settings (0x01xx) - Read/Write ======
        case PARAM_CFG_SERIAL:
            toBigEndian32(_config->serialNumber, buffer);
            *length = 4;
            return true;
            
        case PARAM_CFG_TELEM_FAST:
            toBigEndian32(_config->telemetryIntervalFast, buffer);
            *length = 4;
            return true;
            
        case PARAM_CFG_TELEM_SLOW:
            toBigEndian32(_config->telemetryIntervalSlow, buffer);
            *length = 4;
            return true;
            
        case PARAM_CFG_SENSOR_INT:
            toBigEndian32(_config->sensorReadInterval, buffer);
            *length = 4;
            return true;
            
        case PARAM_CFG_HEARTBEAT:
            toBigEndian32(_config->heartbeatInterval, buffer);
            *length = 4;
            return true;
            
        case PARAM_CFG_GPS_INT:
            toBigEndian32(_config->gpsReadInterval, buffer);
            *length = 4;
            return true;
            
        case PARAM_CFG_DEFAULT_MODE:
            buffer[0] = _config->defaultMode;
            *length = 1;
            return true;
            
        case PARAM_CFG_SATBUS_POLL:
            toBigEndian32(_config->satbusPollInterval, buffer);
            *length = 4;
            return true;
            
        // ====== Radio Configuration (0x02xx) ======
        case PARAM_RADIO_CHANNEL:
            buffer[0] = _config->radioChannel;
            *length = 1;
            return true;
            
        case PARAM_RADIO_POWER:
            buffer[0] = _config->radioPowerLevel;
            *length = 1;
            return true;
            
        case PARAM_RADIO_RATE:
            buffer[0] = _config->radioDataRate;
            *length = 1;
            return true;
            
        case PARAM_RADIO_PAYLOAD:
            buffer[0] = _config->radioPayloadSize;
            *length = 1;
            return true;
            
        case PARAM_RADIO_TX_ADDR:
            memcpy(buffer, _config->radioTxAddress, 5);
            *length = 5;
            return true;
            
        case PARAM_RADIO_RX_ADDR:
            memcpy(buffer, _config->radioRxAddress, 5);
            *length = 5;
            return true;
            
        case PARAM_RADIO_PKTS_SENT:
            if (_packetsSentPtr) {
                toBigEndian32(*_packetsSentPtr, buffer);
                *length = 4;
                return true;
            }
            return false;
            
        case PARAM_RADIO_PKTS_FAIL:
            if (_packetsFailedPtr) {
                toBigEndian32(*_packetsFailedPtr, buffer);
                *length = 4;
                return true;
            }
            return false;
            
        case PARAM_RADIO_CONNECTED:
            if (_systemStatus) {
                // radioNRF is bit 3 of the status flags
                uint16_t status;
                memcpy(&status, _systemStatus, 2);
                buffer[0] = (status & 0x0008) ? 1 : 0;  // Bit 3 = radioNRF
                *length = 1;
                return true;
            }
            return false;
            
        // ====== Sensor Configuration (0x03xx) ======
        case PARAM_SENS_SHT_TYPE:
            if (_systemStatus) {
                // Return 1 if SHT sensor is present (type info not available, assume SHT4x)
                uint16_t status;
                memcpy(&status, _systemStatus, 2);
                buffer[0] = (status & 0x0001) ? 1 : 0;  // bit 0 = sensorSHT
                *length = 1;
                return true;
            }
            return false;
            
        case PARAM_SENS_IMU_PRESENT:
            if (_systemStatus) {
                uint16_t status;
                memcpy(&status, _systemStatus, 2);
                buffer[0] = (status & 0x0002) ? 1 : 0;  // bit 1 = sensorIMU
                *length = 1;
                return true;
            }
            return false;
            
        case PARAM_SENS_GPS_ENABLED:
            if (_systemStatus) {
                uint16_t status;
                memcpy(&status, _systemStatus, 2);
                buffer[0] = (status & 0x0004) ? 1 : 0;  // bit 2 = sensorGPS (has fix = enabled)
                *length = 1;
                return true;
            }
            return false;
            
        case PARAM_SENS_IMU_CAL_AX:
        case PARAM_SENS_IMU_CAL_AY:
        case PARAM_SENS_IMU_CAL_AZ:
        case PARAM_SENS_IMU_CAL_GX:
        case PARAM_SENS_IMU_CAL_GY:
        case PARAM_SENS_IMU_CAL_GZ:
        case PARAM_SENS_IMU_CAL_MX:
        case PARAM_SENS_IMU_CAL_MY:
        case PARAM_SENS_IMU_CAL_MZ:
            // IMU calibration values - return 0 (not calibrated) by default
            // Full implementation would read from EEPROM calibration data
            {
                int16_t calValue = 0;
                toBigEndian16((uint16_t)calValue, buffer);
                *length = 2;
                return true;
            }
            
        // ====== Sensor Values (0x04xx) - Read Only ======
        case PARAM_VAL_TEMP:
        case PARAM_VAL_HUMIDITY:
        case PARAM_VAL_ACCEL_X:
        case PARAM_VAL_ACCEL_Y:
        case PARAM_VAL_ACCEL_Z:
        case PARAM_VAL_GYRO_X:
        case PARAM_VAL_GYRO_Y:
        case PARAM_VAL_GYRO_Z:
        case PARAM_VAL_MAG_X:
        case PARAM_VAL_MAG_Y:
        case PARAM_VAL_MAG_Z:
            if (_sensorData) {
                // SensorData structure from SatbusParser.h
                float* sensors = (float*)_sensorData;
                int index = 0;
                switch (paramCode) {
                    case PARAM_VAL_TEMP:     index = 0; break;  // temperature
                    case PARAM_VAL_HUMIDITY: index = 1; break;  // humidity
                    case PARAM_VAL_ACCEL_X:  index = 2; break;  // accelX
                    case PARAM_VAL_ACCEL_Y:  index = 3; break;  // accelY
                    case PARAM_VAL_ACCEL_Z:  index = 4; break;  // accelZ
                    case PARAM_VAL_GYRO_X:   index = 5; break;  // gyroX
                    case PARAM_VAL_GYRO_Y:   index = 6; break;  // gyroY
                    case PARAM_VAL_GYRO_Z:   index = 7; break;  // gyroZ
                    case PARAM_VAL_MAG_X:    index = 8; break;  // magX
                    case PARAM_VAL_MAG_Y:    index = 9; break;  // magY
                    case PARAM_VAL_MAG_Z:    index = 10; break; // magZ
                }
                // Convert float to big-endian (treat as uint32)
                uint32_t floatBits;
                memcpy(&floatBits, &sensors[index], 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
            
        case PARAM_VAL_GPS_LAT:
        case PARAM_VAL_GPS_LON:
            if (_gpsData) {
                // GPSData structure - latitude and longitude are doubles at offset 0 and 8
                double* gps = (double*)_gpsData;
                int index = (paramCode == PARAM_VAL_GPS_LAT) ? 0 : 1;
                // Convert double to big-endian (treat as uint64)
                uint64_t doubleBits;
                memcpy(&doubleBits, &gps[index], 8);
                toBigEndian64(doubleBits, buffer);
                *length = 8;
                return true;
            }
            return false;
            
        case PARAM_VAL_GPS_ALT:
            if (_gpsData) {
                // altitude is at offset 16 (after two doubles)
                float* altPtr = (float*)((uint8_t*)_gpsData + 16);
                uint32_t floatBits;
                memcpy(&floatBits, altPtr, 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
            
        case PARAM_VAL_GPS_SATS:
            if (_gpsData) {
                // satellites is uint8 at offset 20
                uint8_t* satsPtr = (uint8_t*)_gpsData + 20;
                buffer[0] = *satsPtr;
                *length = 1;
                return true;
            }
            return false;
            
        case PARAM_VAL_GPS_FIX:
            if (_gpsData) {
                // hasValidFix is bool at offset 28
                bool* fixPtr = (bool*)((uint8_t*)_gpsData + 28);
                buffer[0] = *fixPtr ? 1 : 0;
                *length = 1;
                return true;
            }
            return false;
            
        // ====== GPIO Configuration (0x05xx) ======
        case PARAM_GPIO_RBF: {
            buffer[0] = (digitalRead(2) == LOW) ? 1 : 0;  // PIN_RBF_SENSE = 2
            *length = 1;
            return true;
        }
            
        case PARAM_GPIO_ADC0: {
            uint16_t adc = analogRead(26);  // PIN_ADC0
            toBigEndian16(adc, buffer);
            *length = 2;
            return true;
        }
            
        case PARAM_GPIO_ADC1: {
            uint16_t adc = analogRead(27);  // PIN_ADC1
            toBigEndian16(adc, buffer);
            *length = 2;
            return true;
        }
            
        case PARAM_GPIO_ADC2: {
            uint16_t adc = analogRead(28);  // PIN_ADC2
            toBigEndian16(adc, buffer);
            *length = 2;
            return true;
        }
            
        // ====== Logging Configuration (0x06xx) ======
        case PARAM_LOG_ENABLED:
            buffer[0] = _config->loggingEnabled ? 1 : 0;
            *length = 1;
            return true;
            
        case PARAM_LOG_INTERVAL:
            toBigEndian32(_config->logInterval, buffer);
            *length = 4;
            return true;
            
        case PARAM_LOG_SD_ENABLED:
            if (_dataLogger) {
                // DataLogger has isEnabled() which checks enabled && SD ready
                // Use the config value since we can't call methods on void*
                buffer[0] = _config->loggingEnabled ? 1 : 0;
                *length = 1;
                return true;
            }
            buffer[0] = 0;
            *length = 1;
            return true;
            
        case PARAM_LOG_FILENAME:
            if (_logFilenamePtr && *_logFilenamePtr) {
                size_t len = strlen(*_logFilenamePtr);
                size_t maxLen = 32;  // Matches handleGetCommand buffer size
                if (len > maxLen) len = maxLen;
                memcpy(buffer, *_logFilenamePtr, len);
                *length = len;
                return true;
            }
            // Return empty string if not available
            *length = 0;
            return true;
            
        // ====== Mesh Configuration (0x07xx) ======
        case PARAM_MESH_RELAY:
            buffer[0] = _config->meshRelayEnabled ? 1 : 0;
            *length = 1;
            return true;
            
        case PARAM_MESH_LOG_SD:
            buffer[0] = _config->meshLogToSD ? 1 : 0;
            *length = 1;
            return true;
            
        case PARAM_MESH_ACTIVE_COUNT:
            if (_mesh) {
                buffer[0] = ((MeshNetwork*)_mesh)->getActiveCubesatCount();
                *length = 1;
                return true;
            }
            buffer[0] = 0;
            *length = 1;
            return true;
            
        case PARAM_MESH_PKTS_RX:
            if (_mesh) {
                uint32_t val = ((MeshNetwork*)_mesh)->getTotalPacketsReceived();
                toBigEndian32(val, buffer);
                *length = 4;
                return true;
            }
            toBigEndian32(0U, buffer);
            *length = 4;
            return true;
            
        case PARAM_MESH_PKTS_RELAY:
            if (_mesh) {
                uint32_t val = ((MeshNetwork*)_mesh)->getPacketsRelayed();
                toBigEndian32(val, buffer);
                *length = 4;
                return true;
            }
            toBigEndian32(0U, buffer);
            *length = 4;
            return true;
            
        // ====== BMS (Battery Management System) Values (0x08xx) ======
        // Use typed struct access to avoid unaligned/offset bugs on Cortex-M0+
        case PARAM_BMS_CONNECTED:
            if (_systemStatus) {
                buffer[0] = ((SystemStatus*)_systemStatus)->satbusBMS ? 1 : 0;
            } else {
                buffer[0] = 0;
            }
            *length = 1;
            return true;
            
        case PARAM_BMS_ERROR:
            if (_bmsData) {
                buffer[0] = ((BMSData*)_bmsData)->errorCode;
            } else {
                buffer[0] = 0;
            }
            *length = 1;
            return true;
            
        case PARAM_BMS_BAT_V:
            if (_bmsData) {
                uint32_t floatBits;
                memcpy(&floatBits, &((BMSData*)_bmsData)->batteryVoltage, 4);
                toBigEndian32(floatBits, buffer);
            } else {
                toBigEndian32(0U, buffer);
            }
            *length = 4;
            return true;
            
        case PARAM_BMS_BAT_PCT:
            if (_bmsData) {
                uint32_t floatBits;
                memcpy(&floatBits, &((BMSData*)_bmsData)->batteryPercent, 4);
                toBigEndian32(floatBits, buffer);
            } else {
                toBigEndian32(0U, buffer);
            }
            *length = 4;
            return true;
            
        case PARAM_BMS_LDO1:
            buffer[0] = (_bmsData && ((BMSData*)_bmsData)->ldo1State) ? 1 : 0;
            *length = 1;
            return true;
        case PARAM_BMS_LDO2:
            buffer[0] = (_bmsData && ((BMSData*)_bmsData)->ldo2State) ? 1 : 0;
            *length = 1;
            return true;
        case PARAM_BMS_LDO3:
            buffer[0] = (_bmsData && ((BMSData*)_bmsData)->ldo3State) ? 1 : 0;
            *length = 1;
            return true;
        case PARAM_BMS_LDO4:
            buffer[0] = (_bmsData && ((BMSData*)_bmsData)->ldo4State) ? 1 : 0;
            *length = 1;
            return true;
            
        case PARAM_BMS_LDO_ALL:
            if (_bmsData) {
                const BMSData* b = (BMSData*)_bmsData;
                buffer[0] = (b->ldo1State ? 0x01 : 0) | (b->ldo2State ? 0x02 : 0) |
                            (b->ldo3State ? 0x04 : 0) | (b->ldo4State ? 0x08 : 0);
            } else {
                buffer[0] = 0;
            }
            *length = 1;
            return true;
            
        case PARAM_BMS_SOLAR1:
            if (_bmsData) {
                uint32_t floatBits;
                memcpy(&floatBits, &((BMSData*)_bmsData)->solarVoltage1, 4);
                toBigEndian32(floatBits, buffer);
            } else {
                toBigEndian32(0U, buffer);
            }
            *length = 4;
            return true;
            
        case PARAM_BMS_SOLAR2:
            if (_bmsData) {
                uint32_t floatBits;
                memcpy(&floatBits, &((BMSData*)_bmsData)->solarVoltage2, 4);
                toBigEndian32(floatBits, buffer);
            } else {
                toBigEndian32(0U, buffer);
            }
            *length = 4;
            return true;
            
        // ====== ASB (Atmospheric Sensing Board) Values (0x09xx) ======
        case PARAM_ASB_CONNECTED:
            if (_systemStatus) {
                buffer[0] = ((SystemStatus*)_systemStatus)->satbusASB ? 1 : 0;
            } else {
                buffer[0] = 0;
            }
            *length = 1;
            return true;
            
        // ASB: use explicit byte offsets (must match SatbusParser.h ASBData layout)
        case PARAM_ASB_TEMP1:
            if (_asbData) {
                uint32_t floatBits;
                memcpy(&floatBits, (const uint8_t*)_asbData + ASB_OFFSET_TEMP1, 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
        case PARAM_ASB_TEMP2:
            if (_asbData) {
                uint32_t floatBits;
                memcpy(&floatBits, (const uint8_t*)_asbData + ASB_OFFSET_TEMP2, 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
        case PARAM_ASB_HUMIDITY:
            if (_asbData) {
                uint32_t floatBits;
                memcpy(&floatBits, (const uint8_t*)_asbData + ASB_OFFSET_HUMIDITY, 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
        case PARAM_ASB_PRESSURE:
            if (_asbData) {
                uint32_t floatBits;
                memcpy(&floatBits, (const uint8_t*)_asbData + ASB_OFFSET_PRESSURE, 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
        case PARAM_ASB_ALTITUDE:
            if (_asbData) {
                uint32_t floatBits;
                memcpy(&floatBits, (const uint8_t*)_asbData + ASB_OFFSET_ALTITUDE, 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
            
        case PARAM_ASB_AQI:
            if (_asbData) {
                buffer[0] = *((const uint8_t*)_asbData + ASB_OFFSET_AQI);
                *length = 1;
                return true;
            }
            return false;
            
        case PARAM_ASB_TVOC:
            if (_asbData) {
                uint16_t val;
                memcpy(&val, (const uint8_t*)_asbData + ASB_OFFSET_TVOC, 2);
                toBigEndian16(val, buffer);
                *length = 2;
                return true;
            }
            return false;
            
        case PARAM_ASB_ECO2:
            if (_asbData) {
                uint16_t val;
                memcpy(&val, (const uint8_t*)_asbData + ASB_OFFSET_ECO2, 2);
                toBigEndian16(val, buffer);
                *length = 2;
                return true;
            }
            return false;
            
        case PARAM_ASB_CO2:
            if (_asbData) {
                uint16_t val;
                memcpy(&val, (const uint8_t*)_asbData + ASB_OFFSET_CO2, 2);
                toBigEndian16(val, buffer);
                *length = 2;
                return true;
            }
            return false;
            
        case PARAM_ASB_PM1_0:
            if (_asbData) {
                uint32_t floatBits;
                memcpy(&floatBits, (const uint8_t*)_asbData + ASB_OFFSET_PM1_0, 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
        case PARAM_ASB_PM2_5:
            if (_asbData) {
                uint32_t floatBits;
                memcpy(&floatBits, (const uint8_t*)_asbData + ASB_OFFSET_PM2_5, 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
        case PARAM_ASB_PM4_0:
            if (_asbData) {
                uint32_t floatBits;
                memcpy(&floatBits, (const uint8_t*)_asbData + ASB_OFFSET_PM4_0, 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
        case PARAM_ASB_PM10:
            if (_asbData) {
                uint32_t floatBits;
                memcpy(&floatBits, (const uint8_t*)_asbData + ASB_OFFSET_PM10, 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
            
        case PARAM_ASB_NC0_5:
            if (_asbData) {
                uint32_t floatBits;
                memcpy(&floatBits, (const uint8_t*)_asbData + offsetof(ASBData, nc0_5), 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
        case PARAM_ASB_NC1_0:
            if (_asbData) {
                uint32_t floatBits;
                memcpy(&floatBits, (const uint8_t*)_asbData + offsetof(ASBData, nc1_0), 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
        case PARAM_ASB_NC2_5:
            if (_asbData) {
                uint32_t floatBits;
                memcpy(&floatBits, (const uint8_t*)_asbData + offsetof(ASBData, nc2_5), 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
        case PARAM_ASB_NC4_0:
            if (_asbData) {
                uint32_t floatBits;
                memcpy(&floatBits, (const uint8_t*)_asbData + offsetof(ASBData, nc4_0), 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
        case PARAM_ASB_NC10:
            if (_asbData) {
                uint32_t floatBits;
                memcpy(&floatBits, (const uint8_t*)_asbData + offsetof(ASBData, nc10), 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
            
        case PARAM_ASB_PART_SIZE:
            if (_asbData) {
                uint32_t floatBits;
                memcpy(&floatBits, (const uint8_t*)_asbData + offsetof(ASBData, typicalParticleSize), 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
            
        // ====== SCC (Solar Charge Controller) Values (0x0Axx) ======
        case PARAM_SCC_CONNECTED:
            if (_systemStatus) {
                buffer[0] = ((SystemStatus*)_systemStatus)->satbusSCC ? 1 : 0;
            } else {
                buffer[0] = 0;
            }
            *length = 1;
            return true;
            
        case PARAM_SCC_V0:
        case PARAM_SCC_V1:
        case PARAM_SCC_V2:
        case PARAM_SCC_V3:
        case PARAM_SCC_V4:
        case PARAM_SCC_V5:
            if (_sccData) {
                // solarVoltage[6] array starts at offset 0
                int vIndex = paramCode - PARAM_SCC_V0;
                float* val = (float*)((uint8_t*)_sccData + vIndex * 4);
                uint32_t floatBits;
                memcpy(&floatBits, val, 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
            
        case PARAM_SCC_T0:
        case PARAM_SCC_T1:
        case PARAM_SCC_T2:
        case PARAM_SCC_T3:
        case PARAM_SCC_T4:
        case PARAM_SCC_T5:
        case PARAM_SCC_T6:
        case PARAM_SCC_T7:
            if (_sccData) {
                // temperature[8] array starts at offset 24 (after 6 floats)
                int tIndex = paramCode - PARAM_SCC_T0;
                float* val = (float*)((uint8_t*)_sccData + 24 + tIndex * 4);
                uint32_t floatBits;
                memcpy(&floatBits, val, 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
            
        case PARAM_SCC_VOUT:
            if (_sccData) {
                // outputVoltage at offset 56 (after 6 + 8 floats)
                float* vout = (float*)((uint8_t*)_sccData + 56);
                uint32_t floatBits;
                memcpy(&floatBits, vout, 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
            
        case PARAM_SCC_IOUT:
            if (_sccData) {
                // outputCurrent at offset 60
                float* iout = (float*)((uint8_t*)_sccData + 60);
                uint32_t floatBits;
                memcpy(&floatBits, iout, 4);
                toBigEndian32(floatBits, buffer);
                *length = 4;
                return true;
            }
            return false;
            
        case PARAM_SCC_MODE:
            if (_sccData) {
                // systemMode is uint8 at offset 64
                buffer[0] = *((uint8_t*)_sccData + 64);
                *length = 1;
                return true;
            }
            return false;
            
        default:
            return false;
    }
}

bool ParameterHandler::setParameterValue(uint16_t paramCode, const uint8_t* value, size_t valueLen) {
    if (!_config) return false;
    
    switch (paramCode) {
        // ====== System Settings (0x01xx) ======
        case PARAM_CFG_SERIAL:
            if (valueLen == 4) {
                _config->serialNumber = fromBigEndian32(value);
                return true;
            }
            return false;
            
        case PARAM_CFG_TELEM_FAST:
            if (valueLen == 4) {
                _config->telemetryIntervalFast = fromBigEndian32(value);
                // Sanity check
                if (_config->telemetryIntervalFast < 100) _config->telemetryIntervalFast = 100;
                if (_config->telemetryIntervalFast > 60000) _config->telemetryIntervalFast = 60000;
                return true;
            }
            return false;
            
        case PARAM_CFG_TELEM_SLOW:
            if (valueLen == 4) {
                _config->telemetryIntervalSlow = fromBigEndian32(value);
                if (_config->telemetryIntervalSlow < 100) _config->telemetryIntervalSlow = 100;
                if (_config->telemetryIntervalSlow > 300000) _config->telemetryIntervalSlow = 300000;
                return true;
            }
            return false;
            
        case PARAM_CFG_SENSOR_INT:
            if (valueLen == 4) {
                _config->sensorReadInterval = fromBigEndian32(value);
                if (_config->sensorReadInterval < 50) _config->sensorReadInterval = 50;
                if (_config->sensorReadInterval > 10000) _config->sensorReadInterval = 10000;
                return true;
            }
            return false;
            
        case PARAM_CFG_HEARTBEAT:
            if (valueLen == 4) {
                _config->heartbeatInterval = fromBigEndian32(value);
                if (_config->heartbeatInterval < 100) _config->heartbeatInterval = 100;
                if (_config->heartbeatInterval > 10000) _config->heartbeatInterval = 10000;
                return true;
            }
            return false;
            
        case PARAM_CFG_GPS_INT:
            if (valueLen == 4) {
                _config->gpsReadInterval = fromBigEndian32(value);
                if (_config->gpsReadInterval < 100) _config->gpsReadInterval = 100;
                if (_config->gpsReadInterval > 10000) _config->gpsReadInterval = 10000;
                return true;
            }
            return false;
            
        case PARAM_CFG_DEFAULT_MODE:
            if (valueLen == 1 && value[0] <= 5) {
                _config->defaultMode = value[0];
                return true;
            }
            return false;
            
        case PARAM_CFG_SATBUS_POLL:
            if (valueLen == 4) {
                _config->satbusPollInterval = fromBigEndian32(value);
                if (_config->satbusPollInterval < 10) _config->satbusPollInterval = 10;
                if (_config->satbusPollInterval > 5000) _config->satbusPollInterval = 5000;
                return true;
            }
            return false;
            
        // ====== Radio Configuration (0x02xx) ======
        case PARAM_RADIO_CHANNEL:
            if (valueLen == 1 && value[0] <= 125) {
                _config->radioChannel = value[0];
                return true;
            }
            return false;
            
        case PARAM_RADIO_POWER:
            if (valueLen == 1 && value[0] <= 3) {
                _config->radioPowerLevel = value[0];
                return true;
            }
            return false;
            
        case PARAM_RADIO_RATE:
            if (valueLen == 1 && value[0] <= 2) {
                _config->radioDataRate = value[0];
                return true;
            }
            return false;
            
        case PARAM_RADIO_PAYLOAD:
            if (valueLen == 1 && value[0] >= 1 && value[0] <= 32) {
                _config->radioPayloadSize = value[0];
                return true;
            }
            return false;
            
        case PARAM_RADIO_TX_ADDR:
            if (valueLen == 5) {
                memcpy(_config->radioTxAddress, value, 5);
                return true;
            }
            return false;
            
        case PARAM_RADIO_RX_ADDR:
            if (valueLen == 5) {
                memcpy(_config->radioRxAddress, value, 5);
                return true;
            }
            return false;
            
        // ====== GPIO Configuration (0x05xx) ======
        case PARAM_GPIO_LED1:
            if (valueLen == 1) {
                digitalWrite(6, value[0] ? HIGH : LOW);  // PIN_LED1
                return true;
            }
            return false;
            
        case PARAM_GPIO_LED2:
            if (valueLen == 1) {
                digitalWrite(7, value[0] ? HIGH : LOW);  // PIN_LED2
                return true;
            }
            return false;
            
        case PARAM_GPIO_LED3:
            if (valueLen == 1) {
                digitalWrite(8, value[0] ? HIGH : LOW);  // PIN_LED3
                return true;
            }
            return false;
            
        case PARAM_GPIO_BUZZER:
            if (valueLen == 2) {
                uint16_t freq = fromBigEndian16(value);
                if (freq == 0) {
                    noTone(3);  // PIN_BUZZER
                } else {
                    tone(3, freq);
                }
                return true;
            }
            return false;
            
        case PARAM_GPIO_BLE_EN:
            if (valueLen == 1) {
                digitalWrite(20, value[0] ? HIGH : LOW);  // PIN_BLE_EN
                return true;
            }
            return false;
            
        // ====== Logging Configuration (0x06xx) ======
        case PARAM_LOG_ENABLED:
            if (valueLen == 1) {
                _config->loggingEnabled = (value[0] != 0);
                if (_dataLogger) {
                    ((DataLogger*)_dataLogger)->setEnabled(_config->loggingEnabled);
                }
                return true;
            }
            return false;
            
        case PARAM_LOG_INTERVAL:
            if (valueLen == 4) {
                _config->logInterval = fromBigEndian32(value);
                if (_config->logInterval < 100) _config->logInterval = 100;
                if (_config->logInterval > 300000) _config->logInterval = 300000;
                if (_dataLogger) {
                    ((DataLogger*)_dataLogger)->setSensorInterval(_config->logInterval);
                    ((DataLogger*)_dataLogger)->setGPSInterval(_config->logInterval);
                }
                return true;
            }
            return false;
            
        // ====== Mesh Configuration (0x07xx) ======
        case PARAM_MESH_RELAY:
            if (valueLen == 1) {
                _config->meshRelayEnabled = (value[0] != 0);
                return true;
            }
            return false;
            
        case PARAM_MESH_LOG_SD:
            if (valueLen == 1) {
                _config->meshLogToSD = (value[0] != 0);
                return true;
            }
            return false;
            
        default:
            return false;
    }
}

uint8_t ParameterHandler::getParameterAccess(uint16_t paramCode) {
    // System Info - Read Only
    if (paramCode >= 0x0001 && paramCode <= 0x00FF) {
        return ACCESS_RO;
    }
    
    // System Settings - Read/Write/Persist
    if (paramCode >= 0x0100 && paramCode <= 0x01FF) {
        return ACCESS_RWP;
    }
    
    // Radio Configuration - Read/Write/Persist
    if (paramCode >= 0x0200 && paramCode <= 0x020F) {
        return ACCESS_RWP;
    }
    if (paramCode >= 0x0210 && paramCode <= 0x021F) {
        return ACCESS_RWP;  // Addresses
    }
    if (paramCode >= 0x0220 && paramCode <= 0x022F) {
        return ACCESS_RO;   // Statistics
    }
    
    // Sensor Configuration - Read/Write
    if (paramCode >= 0x0300 && paramCode <= 0x030F) {
        if (paramCode == PARAM_SENS_SHT_TYPE || paramCode == PARAM_SENS_IMU_PRESENT) {
            return ACCESS_RO;
        }
        return ACCESS_RW;
    }
    if (paramCode >= 0x0310 && paramCode <= 0x033F) {
        return ACCESS_RWP;  // IMU calibration
    }
    
    // Sensor Values - Read Only
    if (paramCode >= 0x0400 && paramCode <= 0x04FF) {
        return ACCESS_RO | ACCESS_VOLATILE;
    }
    
    // GPIO Configuration - Mixed
    if (paramCode >= 0x0500 && paramCode <= 0x050F) {
        return ACCESS_RW;
    }
    if (paramCode >= 0x0510 && paramCode <= 0x052F) {
        return ACCESS_RO;  // RBF and ADC are read-only
    }
    
    // Logging Configuration - Read/Write/Persist
    if (paramCode >= 0x0600 && paramCode <= 0x060F) {
        return ACCESS_RWP;
    }
    if (paramCode == PARAM_LOG_FILENAME) {
        return ACCESS_RO;
    }
    
    // Mesh Configuration - Read/Write/Persist
    if (paramCode >= 0x0700 && paramCode <= 0x070F) {
        return ACCESS_RWP;
    }
    if (paramCode >= 0x0710 && paramCode <= 0x072F) {
        return ACCESS_RO;  // Statistics
    }
    
    // BMS Values - Read Only
    if (paramCode >= 0x0800 && paramCode <= 0x08FF) {
        return ACCESS_RO | ACCESS_VOLATILE;
    }
    
    // ASB Values - Read Only
    if (paramCode >= 0x0900 && paramCode <= 0x09FF) {
        return ACCESS_RO | ACCESS_VOLATILE;
    }
    
    // SCC Values - Read Only
    if (paramCode >= 0x0A00 && paramCode <= 0x0AFF) {
        return ACCESS_RO | ACCESS_VOLATILE;
    }
    
    return ACCESS_NONE;
}

uint8_t ParameterHandler::getParameterSize(uint16_t paramCode) {
    switch (paramCode) {
        // 1-byte parameters
        case PARAM_SYS_MODE:
        case PARAM_CFG_DEFAULT_MODE:
        case PARAM_RADIO_CHANNEL:
        case PARAM_RADIO_POWER:
        case PARAM_RADIO_RATE:
        case PARAM_RADIO_PAYLOAD:
        case PARAM_SENS_SHT_TYPE:
        case PARAM_SENS_IMU_PRESENT:
        case PARAM_SENS_GPS_ENABLED:
        case PARAM_VAL_GPS_SATS:
        case PARAM_VAL_GPS_FIX:
        case PARAM_GPIO_LED1:
        case PARAM_GPIO_LED2:
        case PARAM_GPIO_LED3:
        case PARAM_GPIO_BLE_EN:
        case PARAM_GPIO_RBF:
        case PARAM_LOG_ENABLED:
        case PARAM_LOG_SD_ENABLED:
        case PARAM_MESH_RELAY:
        case PARAM_MESH_LOG_SD:
        case PARAM_MESH_ACTIVE_COUNT:
        // BMS 1-byte
        case PARAM_BMS_CONNECTED:
        case PARAM_BMS_ERROR:
        case PARAM_BMS_LDO1:
        case PARAM_BMS_LDO2:
        case PARAM_BMS_LDO3:
        case PARAM_BMS_LDO4:
        case PARAM_BMS_LDO_ALL:
        // ASB 1-byte
        case PARAM_ASB_CONNECTED:
        case PARAM_ASB_AQI:
        // SCC 1-byte
        case PARAM_SCC_CONNECTED:
        case PARAM_SCC_MODE:
            return 1;
            
        // 2-byte parameters
        case PARAM_SYS_STATUS:
        case PARAM_GPIO_BUZZER:
        case PARAM_GPIO_ADC0:
        case PARAM_GPIO_ADC1:
        case PARAM_GPIO_ADC2:
        case PARAM_SENS_IMU_CAL_AX:
        case PARAM_SENS_IMU_CAL_AY:
        case PARAM_SENS_IMU_CAL_AZ:
        case PARAM_SENS_IMU_CAL_GX:
        case PARAM_SENS_IMU_CAL_GY:
        case PARAM_SENS_IMU_CAL_GZ:
        case PARAM_SENS_IMU_CAL_MX:
        case PARAM_SENS_IMU_CAL_MY:
        case PARAM_SENS_IMU_CAL_MZ:
        // ASB 2-byte
        case PARAM_ASB_TVOC:
        case PARAM_ASB_ECO2:
        case PARAM_ASB_CO2:
            return 2;
            
        // 4-byte parameters
        case PARAM_SYS_SERIAL:
        case PARAM_SYS_UPTIME:
        case PARAM_SYS_BOOTCOUNT:
        case PARAM_SYS_FREE_HEAP:
        case PARAM_SYS_USED_HEAP:
        case PARAM_SYS_TOTAL_HEAP:
        case PARAM_CFG_SERIAL:
        case PARAM_CFG_TELEM_FAST:
        case PARAM_CFG_TELEM_SLOW:
        case PARAM_CFG_SENSOR_INT:
        case PARAM_CFG_HEARTBEAT:
        case PARAM_CFG_GPS_INT:
        case PARAM_CFG_SATBUS_POLL:
        case PARAM_RADIO_PKTS_SENT:
        case PARAM_RADIO_PKTS_FAIL:
        case PARAM_VAL_TEMP:
        case PARAM_VAL_HUMIDITY:
        case PARAM_VAL_ACCEL_X:
        case PARAM_VAL_ACCEL_Y:
        case PARAM_VAL_ACCEL_Z:
        case PARAM_VAL_GYRO_X:
        case PARAM_VAL_GYRO_Y:
        case PARAM_VAL_GYRO_Z:
        case PARAM_VAL_MAG_X:
        case PARAM_VAL_MAG_Y:
        case PARAM_VAL_MAG_Z:
        case PARAM_VAL_GPS_ALT:
        case PARAM_VAL_GPS_HDOP:
        case PARAM_LOG_INTERVAL:
        case PARAM_MESH_PKTS_RX:
        case PARAM_MESH_PKTS_RELAY:
        // BMS 4-byte
        case PARAM_BMS_BAT_V:
        case PARAM_BMS_BAT_PCT:
        case PARAM_BMS_SOLAR1:
        case PARAM_BMS_SOLAR2:
        // ASB 4-byte (floats)
        case PARAM_ASB_TEMP1:
        case PARAM_ASB_TEMP2:
        case PARAM_ASB_HUMIDITY:
        case PARAM_ASB_PRESSURE:
        case PARAM_ASB_ALTITUDE:
        case PARAM_ASB_PM1_0:
        case PARAM_ASB_PM2_5:
        case PARAM_ASB_PM4_0:
        case PARAM_ASB_PM10:
        case PARAM_ASB_NC0_5:
        case PARAM_ASB_NC1_0:
        case PARAM_ASB_NC2_5:
        case PARAM_ASB_NC4_0:
        case PARAM_ASB_NC10:
        case PARAM_ASB_PART_SIZE:
        // SCC 4-byte (floats)
        case PARAM_SCC_V0:
        case PARAM_SCC_V1:
        case PARAM_SCC_V2:
        case PARAM_SCC_V3:
        case PARAM_SCC_V4:
        case PARAM_SCC_V5:
        case PARAM_SCC_T0:
        case PARAM_SCC_T1:
        case PARAM_SCC_T2:
        case PARAM_SCC_T3:
        case PARAM_SCC_T4:
        case PARAM_SCC_T5:
        case PARAM_SCC_T6:
        case PARAM_SCC_T7:
        case PARAM_SCC_VOUT:
        case PARAM_SCC_IOUT:
            return 4;
            
        // 5-byte parameters (radio addresses)
        case PARAM_RADIO_TX_ADDR:
        case PARAM_RADIO_RX_ADDR:
            return 5;
            
        // 8-byte parameters
        case PARAM_VAL_GPS_LAT:
        case PARAM_VAL_GPS_LON:
            return 8;
            
        // Variable length
        case PARAM_SYS_VERSION:
        case PARAM_LOG_FILENAME:
            return 0;  // Variable
            
        default:
            return 0;
    }
}

ParamType ParameterHandler::getParameterType(uint16_t paramCode) {
    switch (paramCode) {
        case PARAM_VAL_TEMP:
        case PARAM_VAL_HUMIDITY:
        case PARAM_VAL_ACCEL_X:
        case PARAM_VAL_ACCEL_Y:
        case PARAM_VAL_ACCEL_Z:
        case PARAM_VAL_GYRO_X:
        case PARAM_VAL_GYRO_Y:
        case PARAM_VAL_GYRO_Z:
        case PARAM_VAL_MAG_X:
        case PARAM_VAL_MAG_Y:
        case PARAM_VAL_MAG_Z:
        case PARAM_VAL_GPS_ALT:
        case PARAM_VAL_GPS_HDOP:
            return PARAM_FLOAT;
            
        case PARAM_VAL_GPS_LAT:
        case PARAM_VAL_GPS_LON:
            return PARAM_DOUBLE;
            
        case PARAM_SYS_VERSION:
        case PARAM_LOG_FILENAME:
            return PARAM_STRING;
            
        case PARAM_GPIO_LED1:
        case PARAM_GPIO_LED2:
        case PARAM_GPIO_LED3:
        case PARAM_GPIO_BLE_EN:
        case PARAM_GPIO_RBF:
        case PARAM_VAL_GPS_FIX:
        case PARAM_LOG_ENABLED:
        case PARAM_LOG_SD_ENABLED:
        case PARAM_MESH_RELAY:
        case PARAM_MESH_LOG_SD:
        case PARAM_SENS_IMU_PRESENT:
        case PARAM_SENS_GPS_ENABLED:
            return PARAM_BOOL;
            
        case PARAM_SENS_IMU_CAL_AX:
        case PARAM_SENS_IMU_CAL_AY:
        case PARAM_SENS_IMU_CAL_AZ:
        case PARAM_SENS_IMU_CAL_GX:
        case PARAM_SENS_IMU_CAL_GY:
        case PARAM_SENS_IMU_CAL_GZ:
        case PARAM_SENS_IMU_CAL_MX:
        case PARAM_SENS_IMU_CAL_MY:
        case PARAM_SENS_IMU_CAL_MZ:
            return PARAM_INT16;
            
        default:
            // Most parameters are unsigned integers
            uint8_t size = getParameterSize(paramCode);
            if (size == 1) return PARAM_UINT8;
            if (size == 2) return PARAM_UINT16;
            if (size == 4) return PARAM_UINT32;
            return PARAM_BYTES;
    }
}

void ParameterHandler::sendResponse(uint8_t cmd, uint16_t paramCode, const uint8_t* value, size_t valueLen) {
    // Build response: #G<VV><CC><PPPP><VALUE>
    char response[80];
    int pos = 0;
    
    // Header
    response[pos++] = '#';
    response[pos++] = 'G';
    
    // Version
    response[pos++] = hexChar(PARAM_PROTOCOL_VERSION >> 4);
    response[pos++] = hexChar(PARAM_PROTOCOL_VERSION & 0x0F);
    
    // Command
    response[pos++] = hexChar(cmd >> 4);
    response[pos++] = hexChar(cmd & 0x0F);
    
    // Parameter code
    response[pos++] = hexChar((paramCode >> 12) & 0x0F);
    response[pos++] = hexChar((paramCode >> 8) & 0x0F);
    response[pos++] = hexChar((paramCode >> 4) & 0x0F);
    response[pos++] = hexChar(paramCode & 0x0F);
    
    // Value
    for (size_t i = 0; i < valueLen && pos < sizeof(response) - 2; i++) {
        response[pos++] = hexChar(value[i] >> 4);
        response[pos++] = hexChar(value[i] & 0x0F);
    }
    
    response[pos] = '\0';
    Serial.println(response);
}

void ParameterHandler::sendError(uint16_t paramCode, uint8_t errorCode) {
    uint8_t errBuf[1] = { errorCode };
    sendResponse(PARAM_CMD_ERROR, paramCode, errBuf, 1);
}

bool ParameterHandler::parseHexByte(const char* str, uint8_t* value) {
    uint8_t hi = 0, lo = 0;
    
    if (str[0] >= '0' && str[0] <= '9') hi = str[0] - '0';
    else if (str[0] >= 'A' && str[0] <= 'F') hi = str[0] - 'A' + 10;
    else if (str[0] >= 'a' && str[0] <= 'f') hi = str[0] - 'a' + 10;
    else return false;
    
    if (str[1] >= '0' && str[1] <= '9') lo = str[1] - '0';
    else if (str[1] >= 'A' && str[1] <= 'F') lo = str[1] - 'A' + 10;
    else if (str[1] >= 'a' && str[1] <= 'f') lo = str[1] - 'a' + 10;
    else return false;
    
    *value = (hi << 4) | lo;
    return true;
}

bool ParameterHandler::parseHexUint16(const char* str, uint16_t* value) {
    uint8_t hi, lo;
    if (!parseHexByte(str, &hi)) return false;
    if (!parseHexByte(str + 2, &lo)) return false;
    *value = ((uint16_t)hi << 8) | lo;
    return true;
}

bool ParameterHandler::parseHexValue(const char* str, size_t len, uint8_t* buffer, size_t bufLen) {
    if (len / 2 > bufLen) return false;
    
    for (size_t i = 0; i < len / 2; i++) {
        if (!parseHexByte(&str[i * 2], &buffer[i])) {
            return false;
        }
    }
    return true;
}

char ParameterHandler::hexChar(uint8_t nibble) {
    nibble &= 0x0F;
    if (nibble < 10) return '0' + nibble;
    return 'A' + nibble - 10;
}

void ParameterHandler::printAllParameters() {
    Serial.println(F("\n=== Parameter Handler - All Parameters ==="));
    Serial.println(F("\nSystem Info (Read-Only):"));
    
    uint8_t buffer[32];
    size_t len;
    
    // Print system info
    len = sizeof(buffer);
    if (getParameterValue(PARAM_SYS_SERIAL, buffer, &len)) {
        uint32_t val;
        memcpy(&val, buffer, 4);
        Serial.print(F("  0x0001 Serial:     0x")); Serial.println(val, HEX);
    }
    
    len = sizeof(buffer);
    if (getParameterValue(PARAM_SYS_VERSION, buffer, &len)) {
        Serial.print(F("  0x0002 Version:    "));
        buffer[len] = 0;
        Serial.println((char*)buffer);
    }
    
    len = sizeof(buffer);
    if (getParameterValue(PARAM_SYS_UPTIME, buffer, &len)) {
        uint32_t val;
        memcpy(&val, buffer, 4);
        Serial.print(F("  0x0003 Uptime:     ")); Serial.print(val); Serial.println(F(" s"));
    }
    
    len = sizeof(buffer);
    if (getParameterValue(PARAM_SYS_BOOTCOUNT, buffer, &len)) {
        uint32_t val;
        memcpy(&val, buffer, 4);
        Serial.print(F("  0x0004 Boot Count: ")); Serial.println(val);
    }
    
    Serial.println(F("\nConfiguration (Read/Write):"));
    Serial.print(F("  0x0101 Telem Fast: ")); Serial.print(_config->telemetryIntervalFast); Serial.println(F(" ms"));
    Serial.print(F("  0x0102 Telem Slow: ")); Serial.print(_config->telemetryIntervalSlow); Serial.println(F(" ms"));
    Serial.print(F("  0x0103 Sensor Int: ")); Serial.print(_config->sensorReadInterval); Serial.println(F(" ms"));
    Serial.print(F("  0x0104 Heartbeat:  ")); Serial.print(_config->heartbeatInterval); Serial.println(F(" ms"));
    
    Serial.println(F("\nRadio Configuration:"));
    Serial.print(F("  0x0200 Channel:    ")); Serial.println(_config->radioChannel);
    Serial.print(F("  0x0201 Power:      ")); Serial.println(_config->radioPowerLevel);
    Serial.print(F("  0x0202 Data Rate:  ")); Serial.println(_config->radioDataRate);
    
    Serial.println(F("\n==========================================\n"));
}

// ============================================================================
// BIG-ENDIAN (MSB) BYTE ORDER CONVERSION
// ============================================================================

void ParameterHandler::toBigEndian16(uint16_t value, uint8_t* buffer) {
    buffer[0] = (value >> 8) & 0xFF;  // MSB first
    buffer[1] = value & 0xFF;         // LSB last
}

void ParameterHandler::toBigEndian32(uint32_t value, uint8_t* buffer) {
    buffer[0] = (value >> 24) & 0xFF;  // MSB first
    buffer[1] = (value >> 16) & 0xFF;
    buffer[2] = (value >> 8) & 0xFF;
    buffer[3] = value & 0xFF;          // LSB last
}

void ParameterHandler::toBigEndian64(uint64_t value, uint8_t* buffer) {
    buffer[0] = (value >> 56) & 0xFF;  // MSB first
    buffer[1] = (value >> 48) & 0xFF;
    buffer[2] = (value >> 40) & 0xFF;
    buffer[3] = (value >> 32) & 0xFF;
    buffer[4] = (value >> 24) & 0xFF;
    buffer[5] = (value >> 16) & 0xFF;
    buffer[6] = (value >> 8) & 0xFF;
    buffer[7] = value & 0xFF;          // LSB last
}

uint16_t ParameterHandler::fromBigEndian16(const uint8_t* buffer) {
    return ((uint16_t)buffer[0] << 8) | buffer[1];
}

uint32_t ParameterHandler::fromBigEndian32(const uint8_t* buffer) {
    return ((uint32_t)buffer[0] << 24) | 
           ((uint32_t)buffer[1] << 16) | 
           ((uint32_t)buffer[2] << 8) | 
           buffer[3];
}

uint64_t ParameterHandler::fromBigEndian64(const uint8_t* buffer) {
    return ((uint64_t)buffer[0] << 56) | 
           ((uint64_t)buffer[1] << 48) | 
           ((uint64_t)buffer[2] << 40) | 
           ((uint64_t)buffer[3] << 32) |
           ((uint64_t)buffer[4] << 24) | 
           ((uint64_t)buffer[5] << 16) | 
           ((uint64_t)buffer[6] << 8) | 
           buffer[7];
}

#endif // PARAMETER_HANDLER_H
