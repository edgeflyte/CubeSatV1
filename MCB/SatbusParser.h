/*
 * ============================================================================
 * SatbusParser.h - SATBUS v1 Protocol Parser
 * ============================================================================
 * 
 * SATBUS v1 is the inter-satellite communication protocol for connecting
 * modules and subsystems to the Main Control Board (MCB).
 * 
 * This file defines the data structures and parser interface for handling
 * SATBUS packets from expansion modules:
 *   - ASB: Atmospheric Sensing Board
 *   - BMS: Battery Management System
 *   - SCC: Solar Charge Controller
 * 
 * Protocol Overview:
 *   - Physical: UART 9600 8N1 3.3V
 *   - Packet Format: $MODULE,ID,DATA1,DATA2,...,*
 *   - Command Format: &MODULE,CMD,VALUE
 * 
 * EDUCATIONAL NOTES:
 * This demonstrates a common pattern in embedded systems - parsing 
 * comma-delimited ASCII protocols (similar to NMEA for GPS).
 * 
 * ============================================================================
 */

#ifndef SATBUS_PARSER_H
#define SATBUS_PARSER_H

#include <Arduino.h>

// ============================================================================
// SATBUS CONFIGURATION
// ============================================================================

#define SATBUS_BAUD_RATE        9600
#define SATBUS_MAX_PACKET_LEN   128
#define SATBUS_MAX_FIELDS       16

// Packet delimiters
#define SATBUS_START_DATA       '$'   // Start of data packet
#define SATBUS_START_CMD        '&'   // Start of command packet
#define SATBUS_FIELD_DELIM      ','   // Field separator
#define SATBUS_END_CHAR         '*'   // End of packet

// ============================================================================
// MODULE TYPE IDENTIFIERS
// ============================================================================

/**
 * SATBUS Module Types
 * 
 * Each expansion module on the SATBUS has a unique identifier.
 * This allows the parser to route packets to the correct handler.
 */
enum SatbusModuleType : uint8_t {
    SATBUS_MODULE_UNKNOWN = 0,
    SATBUS_MODULE_ASB = 1,    // Atmospheric Sensing Board
    SATBUS_MODULE_BMS = 2,    // Battery Management System
    SATBUS_MODULE_SCC = 3,    // Solar Charge Controller
    SATBUS_MODULE_CUSTOM = 99 // User-defined modules
};

// ============================================================================
// SATBUS PACKET STRUCTURE
// ============================================================================

/**
 * Parsed SATBUS Packet
 * 
 * This structure holds a parsed SATBUS packet with its fields
 * separated for easy access. The raw data is also preserved
 * for debugging or custom parsing needs.
 */
struct SatbusPacket {
    bool isValid;                           // True if packet parsed successfully
    bool isCommand;                         // True if command packet (&), false if data ($)
    SatbusModuleType moduleType;            // Which module sent this packet
    uint8_t packetId;                       // Packet type identifier
    uint8_t fieldCount;                     // Number of data fields
    char fields[SATBUS_MAX_FIELDS][16];     // Parsed field values
    char rawData[SATBUS_MAX_PACKET_LEN];    // Original packet string
    uint32_t timestamp;                     // When packet was received (millis)
    
    // Constructor - initialize with defaults
    SatbusPacket() : isValid(false), isCommand(false), 
                     moduleType(SATBUS_MODULE_UNKNOWN), 
                     packetId(0), fieldCount(0), timestamp(0) {
        memset(fields, 0, sizeof(fields));
        memset(rawData, 0, sizeof(rawData));
    }
};

// ============================================================================
// ASB (Atmospheric Sensing Board) DATA STRUCTURES
// ============================================================================

/**
 * ASB Environmental Sensor Data
 * 
 * Contains readings from the environmental sensors on the ASB:
 *   - SHT4x: Temperature, Humidity
 *   - BMP180: Temperature, Pressure, Altitude
 *   - ENS160: Air Quality Index, TVOC, eCO2
 *   - SCD4x: CO2 concentration
 *   - SPS30: Particulate matter
 * Packed to 4-byte alignment so layout is identical in all translation units.
 */
#pragma pack(push, 4)
struct ASBData {
    // All floats first (contiguous, no padding)
    // Environmental Sensors (Packet ID 3)
    float temperature1;      // SHT4x temperature (°C)
    float temperature2;      // BMP180 temperature (°C)
    float humidity;          // Relative humidity (%)
    float pressure;          // Atmospheric pressure (Pa)
    float altitude;          // Pressure altitude (m)
    // Particulate Matter (Packet ID 4)
    float pm1_0;             // PM 1.0 mass concentration (μg/m³)
    float pm2_5;             // PM 2.5 mass concentration (μg/m³)
    float pm4_0;             // PM 4.0 mass concentration (μg/m³)
    float pm10;              // PM 10.0 mass concentration (μg/m³)
    float nc0_5;             // Number concentration 0-0.5μm (/cm³)
    float nc1_0;             // Number concentration 0.5-1.0μm (/cm³)
    float nc2_5;             // Number concentration 1.0-2.5μm (/cm³)
    float nc4_0;             // Number concentration 2.5-4.0μm (/cm³)
    float nc10;              // Number concentration 4.0-10μm (/cm³)
    float typicalParticleSize; // Typical particle size (μm)
    // Non-float fields after all floats
    uint8_t airQualityIndex; // AQI (1-5 scale)
    uint16_t tvoc;           // Total VOC (ppb)
    uint16_t eco2;           // Estimated CO2 (ppm)
    uint16_t co2;            // CO2 concentration (ppm)
    // Status
    bool hasEnvironmental;   // True if environmental data valid
    bool hasParticulate;     // True if particulate data valid
    uint32_t lastUpdate;     // Timestamp of last update
    
    ASBData() : hasEnvironmental(false), hasParticulate(false), lastUpdate(0) {}
};
#pragma pack(pop)

// Compile-time check: 15 floats at 0-56, then AQI/tvoc/eco2/co2
// So nc1_0 is at 40, nc2_5 at 44. Ensures parser and #G handler use same layout.
static_assert(offsetof(ASBData, nc0_5) == 36, "ASBData: nc0_5 offset");
static_assert(offsetof(ASBData, nc1_0) == 40, "ASBData: nc1_0 offset");
static_assert(offsetof(ASBData, nc2_5) == 44, "ASBData: nc2_5 offset");
static_assert(offsetof(ASBData, pressure) == 12, "ASBData: pressure offset");
static_assert(offsetof(ASBData, altitude) == 16, "ASBData: altitude offset");

// ============================================================================
// BMS (Battery Management System) DATA STRUCTURES
// ============================================================================

/**
 * BMS Status Data
 * 
 * Contains battery and power channel information from the BMS:
 *   - Battery voltage and percentage
 *   - LDO channel states (4 independent channels)
 *   - Solar panel voltages
 *   - Error status
 */
struct BMSData {
    // Status
    uint8_t errorCode;       // 0=OK, 1=Low (≤20%), 2=Critical (≤5%)
    
    // Power Channels
    bool ldo1State;          // Channel 1 state (MCB Power)
    bool ldo2State;          // Channel 2 state
    bool ldo3State;          // Channel 3 state
    bool ldo4State;          // Channel 4 state
    
    // Battery
    float batteryVoltage;    // Battery voltage (V)
    float batteryPercent;    // Battery percentage (%)
    
    // Solar Panels
    float solarVoltage1;     // Solar panel 1 voltage (V)
    float solarVoltage2;     // Solar panel 2 voltage (V)
    
    // Metadata
    uint32_t lastUpdate;     // Timestamp of last update
    
    BMSData() : errorCode(0), ldo1State(false), ldo2State(false),
                ldo3State(false), ldo4State(false), batteryVoltage(0),
                batteryPercent(0), solarVoltage1(0), solarVoltage2(0),
                lastUpdate(0) {}
};

// ============================================================================
// SCC (Solar Charge Controller) DATA STRUCTURES
// ============================================================================

/**
 * SCC Status Data
 * 
 * Contains solar panel and thermal monitoring data:
 *   - 6 solar panel voltage inputs
 *   - 8 temperature channels
 *   - Output voltage and current
 *   - System mode
 */
struct SCCData {
    // Solar Panel Voltages (Packet ID 1)
    float solarVoltage[6];   // Solar panel voltages V0-V5 (V)
    
    // Temperature Channels (Packet ID 2)
    float temperature[8];    // Temperature channels T0-T7 (°C)
    
    // Output Monitoring (Packet ID 3)
    float outputVoltage;     // Output voltage (V)
    float outputCurrent;     // Output current (mA)
    uint8_t systemMode;      // System mode
    
    // Status flags
    bool hasVoltages;        // True if voltage data valid
    bool hasTemperatures;    // True if temperature data valid
    bool hasOutput;          // True if output data valid
    uint32_t lastUpdate;     // Timestamp of last update
    
    SCCData() : outputVoltage(0), outputCurrent(0), systemMode(0),
                hasVoltages(false), hasTemperatures(false), hasOutput(false),
                lastUpdate(0) {
        memset(solarVoltage, 0, sizeof(solarVoltage));
        memset(temperature, 0, sizeof(temperature));
    }
};

// ============================================================================
// MCB LOCAL SENSOR DATA
// ============================================================================

/**
 * Local Sensor Data
 * 
 * Data from sensors directly connected to the MCB (not via SATBUS)
 */
struct SensorData {
    // SHT4x
    float temperature;       // Temperature (°C)
    float humidity;          // Relative humidity (%)
    
    // MPU9250 Accelerometer
    float accelX;            // X-axis acceleration (g)
    float accelY;            // Y-axis acceleration (g)
    float accelZ;            // Z-axis acceleration (g)
    
    // MPU9250 Gyroscope
    float gyroX;             // X-axis rotation (°/s)
    float gyroY;             // Y-axis rotation (°/s)
    float gyroZ;             // Z-axis rotation (°/s)
    
    // MPU9250 Magnetometer
    float magX;              // X-axis magnetic field (μT)
    float magY;              // Y-axis magnetic field (μT)
    float magZ;              // Z-axis magnetic field (μT)
    
    SensorData() : temperature(0), humidity(0),
                   accelX(0), accelY(0), accelZ(0),
                   gyroX(0), gyroY(0), gyroZ(0),
                   magX(0), magY(0), magZ(0) {}
};

/**
 * GPS Data
 */
struct GPSData {
    double latitude;         // Latitude (degrees)
    double longitude;        // Longitude (degrees)
    float altitude;          // Altitude (m)
    uint8_t satellites;      // Number of satellites
    float hdop;              // Horizontal dilution of precision
    bool hasValidFix;        // True if GPS has valid fix
    
    // Time/Date
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t day;
    uint8_t month;
    uint16_t year;
    
    GPSData() : latitude(0), longitude(0), altitude(0), satellites(0),
                hdop(99.9), hasValidFix(false), hour(0), minute(0),
                second(0), day(0), month(0), year(0) {}
};

// ============================================================================
// SATBUS PARSER CLASS
// ============================================================================

/**
 * SatbusParser Class
 * 
 * Handles parsing of incoming SATBUS packets and provides methods
 * for sending commands to SATBUS modules.
 * 
 * USAGE:
 *   SatbusParser satbus;
 *   satbus.begin(&Serial1);
 *   
 *   // In loop:
 *   while (Serial1.available()) {
 *       SatbusPacket pkt = satbus.parseChar(Serial1.read());
 *       if (pkt.isValid) {
 *           // Process packet
 *       }
 *   }
 */
class SatbusParser {
public:
    /**
     * Initialize the SATBUS parser
     * @param serial Pointer to the Serial port connected to SATBUS
     */
    void begin(HardwareSerial* serial);
    
    /**
     * Parse a single character from the SATBUS stream
     * @param c The character to parse
     * @return SatbusPacket - valid if a complete packet was parsed
     * 
     * This implements a state machine that builds up packets
     * character by character. When a complete packet is received,
     * it returns a valid SatbusPacket structure.
     */
    SatbusPacket parseChar(char c);
    
    /**
     * Parse a complete packet string
     * @param packetStr The complete packet string to parse
     * @return SatbusPacket with parsed data
     */
    SatbusPacket parsePacket(const char* packetStr);
    
    // ========================================================================
    // ASB Packet Parsers
    // ========================================================================
    
    /**
     * Parse ASB environmental sensor packet ($ASB,3,...)
     */
    void parseASBEnvironmental(SatbusPacket& packet, ASBData& data);
    
    /**
     * Parse ASB particulate matter packet ($ASB,4,...)
     */
    void parseASBParticulate(SatbusPacket& packet, ASBData& data);
    
    // ========================================================================
    // BMS Packet Parsers
    // ========================================================================
    
    /**
     * Parse BMS status packet ($BMS,...)
     */
    void parseBMSStatus(SatbusPacket& packet, BMSData& data);
    
    /**
     * Send command to BMS
     * @param channel Channel number (1-4) or 0 for system commands
     * @param value Value to set (0=OFF, 1=ON, or command-specific)
     */
    void sendBMSCommand(uint8_t channel, uint8_t value);

    /**
     * Write the LDO boot mask to the BMS.
     * Bit 0 = channel 1 ... bit 3 = channel 4. 1 = on after reboot.
     */
    void sendBMSStore(uint8_t mask);
    
    // ========================================================================
    // SCC Packet Parsers
    // ========================================================================
    
    /**
     * Parse SCC solar voltages packet ($SCC,1,...)
     */
    void parseSCCVoltages(SatbusPacket& packet, SCCData& data);
    
    /**
     * Parse SCC temperature packet ($SCC,2,...)
     */
    void parseSCCTemperatures(SatbusPacket& packet, SCCData& data);
    
    /**
     * Parse SCC output monitoring packet ($SCC,3,...)
     */
    void parseSCCOutput(SatbusPacket& packet, SCCData& data);
    
    /**
     * Send text command to SCC
     * @param command Command string (e.g., "status", "help")
     */
    void sendSCCCommand(const char* command);
    
    // ========================================================================
    // Custom Module Support
    // ========================================================================
    
    /**
     * Register a custom module parser callback
     * 
     * This allows users to extend the SATBUS parser with their own
     * custom modules without modifying this library.
     * 
     * @param modulePrefix The 3-character module prefix (e.g., "USR")
     * @param callback Function to call when packet received
     * 
     * EXAMPLE:
     *   void myModuleHandler(SatbusPacket& pkt) {
     *       // Parse custom packet
     *   }
     *   satbus.registerCustomModule("USR", myModuleHandler);
     */
    typedef void (*CustomModuleCallback)(SatbusPacket& packet);
    void registerCustomModule(const char* modulePrefix, CustomModuleCallback callback);
    
    /**
     * Get last parser error message (for debugging)
     */
    const char* getLastError() { return _lastError; }
    
private:
    HardwareSerial* _serial;
    
    // Parser state machine
    enum ParserState {
        WAITING_START,      // Waiting for $ or &
        READING_MODULE,     // Reading module identifier
        READING_DATA,       // Reading packet data
        PACKET_COMPLETE     // Packet ready for processing
    };
    
    ParserState _state;
    char _buffer[SATBUS_MAX_PACKET_LEN];
    uint8_t _bufferIndex;
    bool _isCommand;
    char _lastError[64];
    
    // Custom module support
    static const uint8_t MAX_CUSTOM_MODULES = 4;
    struct CustomModule {
        char prefix[4];
        CustomModuleCallback callback;
    };
    CustomModule _customModules[MAX_CUSTOM_MODULES];
    uint8_t _customModuleCount;
    
    // Helper functions
    SatbusModuleType identifyModule(const char* prefix);
    void resetParser();
    bool validatePacket(const char* packet);
    int splitFields(const char* data, char fields[][16], int maxFields);
};

// ============================================================================
// CUSTOM MODULE INTEGRATION EXAMPLE
// ============================================================================

/*
 * HOW TO ADD CUSTOM SATBUS MODULES
 * ================================
 * 
 * 1. Define your packet format:
 *    $USR,1,DATA1,DATA2,DATA3,*
 *    
 * 2. Create a data structure for your module:
 *    struct MyModuleData {
 *        float value1;
 *        int value2;
 *    };
 *    
 * 3. Create a callback function:
 *    void handleMyModule(SatbusPacket& pkt) {
 *        MyModuleData data;
 *        data.value1 = atof(pkt.fields[0]);
 *        data.value2 = atoi(pkt.fields[1]);
 *        // Use data...
 *    }
 *    
 * 4. Register in setup():
 *    satbus.registerCustomModule("USR", handleMyModule);
 */

#endif // SATBUS_PARSER_H
