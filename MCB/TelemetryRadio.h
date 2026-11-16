/*
 * ============================================================================
 * TelemetryRadio.h - Radio Telemetry Dictionary & Transmission
 * ============================================================================
 * 
 * This module handles the transmission of telemetry data to the ground station.
 * It implements the Telemetry Packet Format specification for encoding and
 * transmitting CubeSat data.
 * 
 * PACKET FORMAT:
 *   #1<TxID><DestID><Type><Data>
 *   - Preamble: #1 (identifies packet start and version)
 *   - TxID: 8-char hex transmitting device serial number
 *   - DestID: 8-char hex destination (00000000 = broadcast)
 *   - Type: 4-char hex packet type from dictionary
 *   - Data: Variable length hex-encoded payload
 * 
 * TELEMETRY DICTIONARY:
 *   This file defines all telemetry data points that can be transmitted.
 *   Each data point has:
 *   - Unique identifier (hex code)
 *   - ID code (mnemonic)
 *   - Data format
 *   - Source subsystem
 * 
 * EDUCATIONAL NOTES:
 * This demonstrates how satellite telemetry systems encode data for
 * transmission. Key concepts include:
 *   - Data serialization (converting structures to bytes)
 *   - Packet framing (how to identify packet boundaries)
 *   - Data dictionaries (standardized data definitions)
 * 
 * ============================================================================
 */

#ifndef TELEMETRY_RADIO_H
#define TELEMETRY_RADIO_H

#include <Arduino.h>
#include <RF24.h>
#include "SatbusParser.h"

// ============================================================================
// SYSTEM STATUS STRUCTURE
// ============================================================================

struct SystemStatus {
    bool sensorSHT       : 1;
    bool sensorIMU       : 1;
    bool sensorGPS       : 1;
    bool radioNRF        : 1;
    bool radioBLE        : 1;
    bool satbusASB       : 1;
    bool satbusBMS       : 1;
    bool satbusSCC       : 1;
    bool rbfInserted     : 1;
    bool lowBattery      : 1;
    bool errorState      : 1;
};

// ============================================================================
// TELEMETRY PACKET STRUCTURE
// ============================================================================

/**
 * Maximum sizes for telemetry packets
 */
#define TELEM_MAX_PACKET_SIZE   32      // NRF24L01 max payload
#define TELEM_MAX_DATA_SIZE     24      // Struct storage reserved for payload
#define TELEM_HEADER_SIZE       12      // Serialized header: 2 preamble + 4 txId + 4 destId + 2 type
#define TELEM_MAX_WIRE_DATA_SIZE 20     // Usable data bytes on-wire (32 - 12 header)

/**
 * Packet Preamble
 * The preamble identifies the start of a packet and its version
 */
#define TELEM_PREAMBLE          "#1"
#define TELEM_BROADCAST_ADDR    0x00000000

/**
 * TelemetryPacket Structure
 * 
 * This is the raw packet structure that gets transmitted over radio.
 * All multi-byte values are stored in big-endian (network byte order).
 */
struct TelemetryPacket {
    uint8_t preamble[2];         // "#1" = 0x23, 0x31
    uint32_t transmitterId;      // Transmitting device serial number
    uint32_t destinationId;      // Destination device (0 = broadcast)
    uint16_t packetType;         // Packet type from dictionary
    uint8_t data[TELEM_MAX_DATA_SIZE];  // Payload data
    uint8_t dataLength;          // Actual data length used
    
    TelemetryPacket() : transmitterId(0), destinationId(0), 
                        packetType(0), dataLength(0) {
        preamble[0] = '#';
        preamble[1] = '1';
        memset(data, 0, sizeof(data));
    }
};

// ============================================================================
// TELEMETRY DICTIONARY - PACKET TYPE DEFINITIONS
// ============================================================================

/**
 * Telemetry Packet Types
 * 
 * These define the different types of telemetry packets the CubeSat
 * can transmit. Each type has a specific data format.
 * 
 * NAMING CONVENTION:
 *   0x00xx - System packets
 *   0x01xx - Power system packets
 *   0x02xx - Communication packets
 *   0x03xx - Environmental/sensor packets
 *   0x04xx - GPS/navigation packets
 *   0x05xx - IMU/attitude packets
 *   0x06xx - Payload packets
 *   0x0Fxx - Debug/diagnostic packets
 */
enum TelemetryType : uint16_t {
    // ========================================================================
    // System Packets (0x00xx) - MCB System Status
    // ========================================================================
    TELEM_SYS_STATUS        = 0x0001,   // System status summary
    TELEM_SYS_SERIAL        = 0x0002,   // Device serial number
    TELEM_SYS_UPTIME        = 0x0003,   // System uptime
    TELEM_SYS_BOOTCOUNT     = 0x0004,   // Boot counter
    TELEM_SYS_MODE          = 0x0005,   // Operating mode
    TELEM_SYS_ERROR         = 0x0006,   // Error codes
    TELEM_SYS_VERSION       = 0x0007,   // Firmware version
    TELEM_CMD_ACK           = 0x0008,   // Uplink command acknowledgement
    
    // ========================================================================
    // BMS - Battery Management System Packets (0x01xx)
    // ========================================================================
    TELEM_BMS_ERROR         = 0x0100,   // BMS error code
    TELEM_BMS_BATV          = 0x0101,   // Battery voltage
    TELEM_BMS_BATPCT        = 0x0102,   // Battery percentage
    TELEM_BMS_LDO1          = 0x0103,   // LDO channel 1 state
    TELEM_BMS_LDO2          = 0x0104,   // LDO channel 2 state
    TELEM_BMS_LDO3          = 0x0105,   // LDO channel 3 state
    TELEM_BMS_LDO4          = 0x0106,   // LDO channel 4 state
    TELEM_BMS_LDOALL        = 0x0107,   // All LDO states packed
    TELEM_BMS_SOL1          = 0x0108,   // BMS solar panel 1
    TELEM_BMS_SOL2          = 0x0109,   // BMS solar panel 2
    TELEM_BMS_ALL           = 0x010F,   // BMS combined status
    
    // ========================================================================
    // SCC - Solar Charge Controller Packets (0x011x-0x013x)
    // ========================================================================
    // SCC Solar Panel Voltages
    TELEM_SCC_V0            = 0x0110,   // SCC solar panel 0
    TELEM_SCC_V1            = 0x0111,   // SCC solar panel 1
    TELEM_SCC_V2            = 0x0112,   // SCC solar panel 2
    TELEM_SCC_V3            = 0x0113,   // SCC solar panel 3
    TELEM_SCC_V4            = 0x0114,   // SCC solar panel 4
    TELEM_SCC_V5            = 0x0115,   // SCC solar panel 5
    TELEM_SCC_VALL          = 0x011F,   // SCC voltages combined
    
    // SCC Output Monitoring
    TELEM_SCC_VOUT          = 0x0120,   // SCC output voltage
    TELEM_SCC_IOUT          = 0x0121,   // SCC output current
    TELEM_SCC_MODE          = 0x0122,   // SCC system mode
    TELEM_SCC_OUT           = 0x012F,   // SCC output combined
    
    // SCC Temperature Channels
    TELEM_SCC_T0            = 0x0130,   // SCC temperature ch 0
    TELEM_SCC_T1            = 0x0131,   // SCC temperature ch 1
    TELEM_SCC_T2            = 0x0132,   // SCC temperature ch 2
    TELEM_SCC_T3            = 0x0133,   // SCC temperature ch 3
    TELEM_SCC_T4            = 0x0134,   // SCC temperature ch 4
    TELEM_SCC_T5            = 0x0135,   // SCC temperature ch 5
    TELEM_SCC_T6            = 0x0136,   // SCC temperature ch 6
    TELEM_SCC_T7            = 0x0137,   // SCC temperature ch 7
    TELEM_SCC_TALL          = 0x013F,   // SCC temps combined
    
    // ========================================================================
    // Communication Packets (0x02xx)
    // ========================================================================
    TELEM_COMM_RSSI         = 0x0200,   // Signal strength
    TELEM_COMM_PKTSSENT     = 0x0201,   // Packets sent
    TELEM_COMM_PKTSRECV     = 0x0202,   // Packets received
    TELEM_COMM_PKTSFAIL     = 0x0203,   // Failed packets
    
    // ========================================================================
    // ASB - Atmospheric Sensing Board Packets (0x03xx)
    // ========================================================================
    // ASB Environmental Sensors
    TELEM_ASB_T1            = 0x0300,   // ASB SHT4x temperature
    TELEM_ASB_T2            = 0x0301,   // ASB BMP180 temperature
    TELEM_ASB_RH            = 0x0302,   // ASB relative humidity
    TELEM_ASB_P1            = 0x0303,   // ASB pressure
    TELEM_ASB_PA1           = 0x0304,   // ASB pressure altitude
    TELEM_ASB_AQI           = 0x0305,   // ASB air quality index
    TELEM_ASB_TVOC          = 0x0306,   // ASB total VOC
    TELEM_ASB_ECO2          = 0x0307,   // ASB estimated CO2
    TELEM_ASB_CO2           = 0x0308,   // ASB CO2 concentration
    TELEM_ASB_ENV           = 0x030F,   // ASB environmental all
    
    // ASB Particulate Matter
    TELEM_ASB_MC1           = 0x0310,   // ASB PM 1.0 mass conc
    TELEM_ASB_MC25          = 0x0311,   // ASB PM 2.5 mass conc
    TELEM_ASB_MC4           = 0x0312,   // ASB PM 4.0 mass conc
    TELEM_ASB_MC10          = 0x0313,   // ASB PM 10 mass conc
    TELEM_ASB_NC05          = 0x0314,   // ASB num conc 0-0.5μm
    TELEM_ASB_NC1           = 0x0315,   // ASB num conc 0.5-1μm
    TELEM_ASB_NC25          = 0x0316,   // ASB num conc 1-2.5μm
    TELEM_ASB_NC4           = 0x0317,   // ASB num conc 2.5-4μm
    TELEM_ASB_NC10          = 0x0318,   // ASB num conc 4-10μm
    TELEM_ASB_TPS           = 0x0319,   // ASB typical particle size
    TELEM_ASB_PM            = 0x031F,   // ASB particulate all
    
    // MCB Local Environmental
    TELEM_MCB_TEMP          = 0x0320,   // MCB temperature
    TELEM_MCB_HUM           = 0x0321,   // MCB humidity
    
    // ========================================================================
    // GPS/Navigation Packets (0x04xx)
    // ========================================================================
    TELEM_GPS_LAT           = 0x0400,   // GPS latitude
    TELEM_GPS_LON           = 0x0401,   // GPS longitude
    TELEM_GPS_ALT           = 0x0402,   // GPS altitude
    TELEM_GPS_SAT           = 0x0403,   // GPS satellites
    TELEM_GPS_HDOP          = 0x0404,   // GPS HDOP
    TELEM_GPS_FIX           = 0x0405,   // GPS fix status
    TELEM_GPS_TIME          = 0x0410,   // GPS time
    TELEM_GPS_DATE          = 0x0411,   // GPS date
    TELEM_GPS_ALL           = 0x041F,   // GPS combined
    
    // ========================================================================
    // IMU/Attitude Packets (0x05xx) - MCB MPU9250
    // ========================================================================
    TELEM_IMU_ACCELX        = 0x0500,   // Accelerometer X
    TELEM_IMU_ACCELY        = 0x0501,   // Accelerometer Y
    TELEM_IMU_ACCELZ        = 0x0502,   // Accelerometer Z
    TELEM_IMU_ACCEL         = 0x050F,   // Accelerometer combined
    TELEM_IMU_GYROX         = 0x0510,   // Gyroscope X
    TELEM_IMU_GYROY         = 0x0511,   // Gyroscope Y
    TELEM_IMU_GYROZ         = 0x0512,   // Gyroscope Z
    TELEM_IMU_GYRO          = 0x051F,   // Gyroscope combined
    TELEM_IMU_MAGX          = 0x0520,   // Magnetometer X
    TELEM_IMU_MAGY          = 0x0521,   // Magnetometer Y
    TELEM_IMU_MAGZ          = 0x0522,   // Magnetometer Z
    TELEM_IMU_MAG           = 0x052F,   // Magnetometer combined
    TELEM_ATT_ROLL          = 0x0530,   // Roll angle
    TELEM_ATT_PITCH         = 0x0531,   // Pitch angle
    TELEM_ATT_YAW           = 0x0532,   // Yaw angle
    TELEM_ATT_ALL           = 0x053F,   // Attitude combined
    TELEM_IMU_ALL           = 0x05FF,   // IMU all data
    
    // ========================================================================
    // Payload Packets (0x06xx) - Reserved for User Extensions
    // ========================================================================
    TELEM_PAYLOAD_1         = 0x0600,   // Payload data 1
    TELEM_PAYLOAD_2         = 0x0601,   // Payload data 2
    TELEM_PAYLOAD_3         = 0x0602,   // Payload data 3
    TELEM_PAYLOAD_4         = 0x0603,   // Payload data 4
    TELEM_PAYLOAD_STATUS    = 0x060F,   // Payload status

    // ========================================================================
    // Mesh Messaging (0x09xx) — short text across the shared radio pipe
    // ========================================================================
    // Payload (20 bytes max):
    //   [0]     msgId       uint8  message identifier (reassembly key with TxID)
    //   [1]     fragIdx     uint8  fragment index (0..fragCount-1)
    //   [2]     fragCount   uint8  total fragments (1..4)
    //   [3]     flags       uint8  bit0 = ground-station sender
    //   [4..19] text        up to 16 UTF-8 / ASCII bytes
    TELEM_MSG_TEXT          = 0x0900,   // Short mesh / ground-station message
    
    // ========================================================================
    // Debug Packets (0x0Fxx)
    // ========================================================================
    TELEM_DEBUG_RAW         = 0x0F00,   // Raw debug data
    TELEM_DEBUG_LOG         = 0x0F01,   // Log message
    TELEM_DEBUG_MEMORY      = 0x0F02,   // Memory statistics
};

#define TELEM_MSG_FLAG_GROUND   0x01
#define TELEM_MSG_MAX_FRAGS     4
#define TELEM_MSG_FRAG_BYTES    16
#define TELEM_MSG_MAX_CHARS     (TELEM_MSG_MAX_FRAGS * TELEM_MSG_FRAG_BYTES)
#define CMD_CONFIRM_BYTE        0xA5

// ============================================================================
// DATA FORMAT DEFINITIONS
// ============================================================================

/**
 * Data Format Types
 * 
 * Defines how data fields are encoded in packets
 */
enum DataFormat : uint8_t {
    FMT_UINT8,
    FMT_UINT16,
    FMT_UINT32,
    FMT_INT8,
    FMT_INT16,
    FMT_INT32,
    FMT_FLOAT,
    FMT_DOUBLE,
    FMT_BOOL,
    FMT_TEXT,
    FMT_HEX
};

/**
 * Metric Source Categories
 */
enum MetricSource : uint8_t {
    SRC_SYS,        // System
    SRC_MCB,        // Main Control Board
    SRC_BMS,        // Battery Management System
    SRC_ENV,        // Environmental Sensors
    SRC_GPS,        // GPS Module
    SRC_IMU,        // Inertial Measurement Unit
    SRC_COMM,       // Communications
    SRC_PWR,        // Power System
    SRC_PAYLOAD,    // Payload
    SRC_THERMAL,    // Thermal Control
    SRC_CUSTOM      // User-defined
};

// ============================================================================
// TELEMETRY DATA POINT DEFINITION
// ============================================================================

/**
 * TelemetryDataPoint
 * 
 * Defines a single data point in the telemetry dictionary.
 * This structure is used to document and validate telemetry data.
 */
struct TelemetryDataPoint {
    uint16_t identifier;        // Hex identifier (e.g., 0x0100)
    const char* idCode;         // Mnemonic code (e.g., "BATV")
    const char* description;    // Human-readable description
    DataFormat format;          // Data format
    MetricSource source;        // Source subsystem
    const char* unit;           // Unit of measurement
    float minValue;             // Minimum valid value
    float maxValue;             // Maximum valid value
};

// ============================================================================
// TELEMETRY RADIO CLASS
// ============================================================================

/**
 * TelemetryRadio Class
 * 
 * Handles building and transmitting telemetry packets.
 * 
 * USAGE:
 *   TelemetryRadio telemetry;
 *   telemetry.begin(&radio, DEVICE_SERIAL);
 *   
 *   // Send status packet
 *   telemetry.sendSystemStatus(mode, status, uptime, bootCount);
 */
class TelemetryRadio {
public:
    /**
     * Initialize telemetry system
     * @param radio Pointer to RF24 radio object
     * @param deviceSerial This device's serial number
     */
    void begin(RF24* radio, uint32_t deviceSerial);
    
    // ========================================================================
    // System Telemetry
    // ========================================================================
    
    /**
     * Send system status packet
     * Includes mode, status flags, uptime, and boot count
     */
    void sendSystemStatus(uint8_t mode, SystemStatus& status, 
                          uint32_t uptime, uint32_t bootCount);
    
    /**
     * Send firmware version
     */
    void sendVersion(const char* version);
    
    /**
     * Send error code
     */
    void sendError(uint8_t errorCode, const char* message);
    
    // ========================================================================
    // Sensor Telemetry
    // ========================================================================
    
    /**
     * Send local sensor data (SHT4x, MPU9250)
     */
    void sendSensorData(SensorData& sensors, GPSData& gps);
    
    /**
     * Send GPS position data
     */
    void sendGPSPosition(GPSData& gps);
    
    /**
     * Send IMU data
     */
    void sendIMUData(SensorData& sensors);
    
    // ========================================================================
    // SATBUS Module Telemetry
    // ========================================================================
    
    /**
     * Send ASB (Atmospheric Sensing Board) data
     */
    void sendASBData(ASBData& data);
    
    /**
     * Send BMS (Battery Management System) data
     */
    void sendBMSData(BMSData& data);
    
    /**
     * Send SCC (Solar Charge Controller) data
     */
    void sendSCCData(SCCData& data);
    
    // ========================================================================
    // Generic Packet Building
    // ========================================================================
    
    /**
     * Build a raw telemetry packet
     * @param type Packet type from dictionary
     * @param data Pointer to data buffer
     * @param length Data length in bytes
     * @return TelemetryPacket ready for transmission
     */
    TelemetryPacket buildPacket(uint16_t type, uint8_t* data, uint8_t length);
    
    /**
     * Transmit a packet
     * @param packet The packet to transmit
     * @return true if transmission successful
     */
    bool transmit(TelemetryPacket& packet);
    
    /**
     * Deserialize a received radio buffer (wire format) into a packet struct.
     * Inverse of serializePacket(): 12-byte big-endian header + data bytes.
     * A raw struct read must NOT be used - struct padding/endianness do not
     * match the on-wire layout.
     * @param buffer Received bytes (network/wire order)
     * @param length Number of valid bytes in buffer
     * @param packet Output packet
     * @return true if the buffer was long enough to contain a full header
     */
    bool deserializePacket(const uint8_t* buffer, uint8_t length, TelemetryPacket& packet);
    
    /**
     * Start a transmit batch: radio stays in TX until endTransmitBatch().
     * Use to send multiple packets back-to-back with no delay.
     */
    void beginTransmitBatch();
    
    /**
     * End transmit batch: return radio to RX with minimal settling.
     */
    void endTransmitBatch();
    
    /**
     * Set destination address for packets
     * @param destination Destination device serial (0 = broadcast)
     */
    void setDestination(uint32_t destination);
    
    /**
     * Set GPIO pins for per-packet LED feedback (0 = disabled).
     * @param successPin Flash on successful transmit (e.g. LED1)
     * @param failurePin Flash on failed transmit (e.g. LED3)
     */
    void setLedPins(uint8_t successPin, uint8_t failurePin);
    
    // ========================================================================
    // Packet Statistics
    // ========================================================================
    
    /**
     * Get transmission statistics
     */
    uint32_t getPacketsSent() { return _packetsSent; }
    uint32_t getPacketsFailed() { return _packetsFailed; }
    uint32_t* getPacketsSentPtr() { return &_packetsSent; }
    uint32_t* getPacketsFailedPtr() { return &_packetsFailed; }
    void resetStatistics();
    
    // ========================================================================
    // Dictionary Access (for debugging/display)
    // ========================================================================
    
    /**
     * Get data point definition by type
     */
    static const TelemetryDataPoint* getDataPoint(uint16_t type);
    
    /**
     * Get number of defined data points
     */
    static uint16_t getDataPointCount();
    
private:
    RF24* _radio;
    uint32_t _deviceSerial;
    uint32_t _destinationSerial;
    uint32_t _packetsSent;
    uint32_t _packetsFailed;
    bool _inBatch;  // true = stay in TX between transmit() calls until endTransmitBatch()
    uint8_t _ledSuccessPin;  // 0 = off
    uint8_t _ledFailPin;    // 0 = off
    
    // Helper functions for data encoding
    void encodeFloat(uint8_t* buffer, float value);
    void encodeUint32(uint8_t* buffer, uint32_t value);
    void encodeUint16(uint8_t* buffer, uint16_t value);
    void encodeInt16(uint8_t* buffer, int16_t value);
    
    // Convert packet to transmit buffer
    void serializePacket(TelemetryPacket& packet, uint8_t* buffer, uint8_t& length);
};

// ============================================================================
// CUSTOM TELEMETRY EXTENSION
// ============================================================================

/**
 * CustomTelemetryPoint
 * 
 * Allows users to register custom telemetry data points for their
 * payloads or experiments without modifying the core library.
 * 
 * USAGE:
 *   // Define your custom data point
 *   CustomTelemetryPoint myPoint = {
 *       0x0610,                 // Identifier (in payload range)
 *       "MYSENS",              // ID Code
 *       "My Custom Sensor",    // Description
 *       FMT_FLOAT,             // Data format
 *       SRC_PAYLOAD,           // Source
 *       "mV",                  // Unit
 *       0.0,                   // Min value
 *       3300.0                 // Max value
 *   };
 */
typedef TelemetryDataPoint CustomTelemetryPoint;

// ============================================================================
// TELEMETRY DICTIONARY LOOKUP TABLE
// ============================================================================

/**
 * Default Telemetry Dictionary
 * 
 * This table defines all standard data points. It can be extended
 * by registering custom data points at runtime.
 */
extern const TelemetryDataPoint TELEMETRY_DICTIONARY[];
extern const uint16_t TELEMETRY_DICTIONARY_SIZE;

// ============================================================================
// MESH NETWORKING - CUBESAT DETECTION & RELAY
// ============================================================================

/**
 * Configuration for mesh networking
 */
#define MESH_MAX_CUBESATS           8       // Max number of remote CubeSats to track
#define MESH_MAX_DATAPOINTS         16      // Max data points per CubeSat
#define MESH_CUBESAT_TIMEOUT_MS     60000   // CubeSat expires after 1 minute
#define MESH_DATAPOINT_TIMEOUT_MS   10000   // Data point expires after 10 seconds

#define TELEM_PREAMBLE_RELAY        "#R"    // Preamble for relayed packets

/**
 * Stored data point from a remote CubeSat
 */
struct RemoteDataPoint {
    uint16_t type;                          // Telemetry type
    uint8_t data[TELEM_MAX_DATA_SIZE];      // Raw data
    uint8_t dataLength;                     // Data length
    uint32_t timestamp;                     // When received (millis)
    bool valid;                             // Is this entry valid
    
    RemoteDataPoint() : type(0), dataLength(0), timestamp(0), valid(false) {
        memset(data, 0, sizeof(data));
    }
};

/**
 * Tracked remote CubeSat
 */
struct RemoteCubeSat {
    uint32_t serialNumber;                  // CubeSat serial number
    uint32_t lastSeen;                      // Last packet received (millis)
    uint32_t packetsReceived;               // Total packets from this CubeSat
    int8_t rssi;                            // Last RSSI (if available)
    bool active;                            // Is this slot in use
    RemoteDataPoint dataPoints[MESH_MAX_DATAPOINTS];  // Stored data points
    
    RemoteCubeSat() : serialNumber(0), lastSeen(0), packetsReceived(0),
                      rssi(0), active(false) {}
};

/**
 * Mesh network configuration
 */
struct MeshConfig {
    bool enabled;                           // Mesh networking enabled
    bool logToSD;                           // Log received packets to SD card
    bool relayEnabled;                      // Relay packets from other CubeSats
    uint32_t cubesatTimeout;                // CubeSat timeout (ms)
    uint32_t datapointTimeout;              // Data point timeout (ms)
    
    MeshConfig() : enabled(true), logToSD(false), relayEnabled(false),
                   cubesatTimeout(MESH_CUBESAT_TIMEOUT_MS),
                   datapointTimeout(MESH_DATAPOINT_TIMEOUT_MS) {}
};

/**
 * MeshNetwork Class
 * 
 * Handles detection, storage, and relay of telemetry from other CubeSats.
 * 
 * USAGE:
 *   MeshNetwork mesh;
 *   mesh.begin(&radio, &telemetry, DEVICE_SERIAL);
 *   
 *   // When packet received:
 *   if (mesh.processPacket(rxPacket)) {
 *       // Packet was from another CubeSat
 *   }
 *   
 *   // Periodic update (call in loop):
 *   mesh.update();  // Handles expiry and relay
 */
class MeshNetwork {
public:
    /**
     * Initialize mesh network
     */
    void begin(RF24* radio, TelemetryRadio* telemetry, uint32_t ownSerial);
    
    /**
     * Process a received packet
     * @param packet Received telemetry packet
     * @param isRelay True if packet has relay preamble
     * @return True if packet was from another CubeSat (not for us)
     */
    bool processPacket(TelemetryPacket& packet, bool isRelay = false);
    
    /**
     * Periodic update - call in main loop
     * Handles expiry checking and packet relay
     */
    void update();
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    void setEnabled(bool enabled) { _config.enabled = enabled; }
    bool isEnabled() { return _config.enabled; }
    
    void setLogToSD(bool enabled) { _config.logToSD = enabled; }
    bool isLogToSD() { return _config.logToSD; }
    
    void setRelayEnabled(bool enabled) { _config.relayEnabled = enabled; }
    bool isRelayEnabled() { return _config.relayEnabled; }
    
    void setCubesatTimeout(uint32_t ms) { _config.cubesatTimeout = ms; }
    void setDatapointTimeout(uint32_t ms) { _config.datapointTimeout = ms; }
    
    MeshConfig& getConfig() { return _config; }
    
    // ========================================================================
    // Query Functions
    // ========================================================================
    
    /**
     * Get number of active (non-expired) CubeSats
     */
    uint8_t getActiveCubesatCount();
    
    /**
     * Get list of active CubeSat serial numbers
     * @param serials Array to fill with serial numbers
     * @param maxCount Maximum entries to return
     * @return Number of entries filled
     */
    uint8_t getActiveCubesats(uint32_t* serials, uint8_t maxCount);
    
    /**
     * Get a specific CubeSat by serial number
     * @param serial Serial number to find
     * @return Pointer to RemoteCubeSat or nullptr if not found
     */
    RemoteCubeSat* getCubesat(uint32_t serial);
    
    /**
     * Get a specific data point from a CubeSat
     * @param serial CubeSat serial number
     * @param type Telemetry type to find
     * @return Pointer to RemoteDataPoint or nullptr
     */
    RemoteDataPoint* getDataPoint(uint32_t serial, uint16_t type);
    
    /**
     * Check if a CubeSat is still active (not timed out)
     */
    bool isCubesatActive(uint32_t serial);
    
    /**
     * Check if a data point is still valid (not timed out)
     */
    bool isDataPointValid(RemoteDataPoint* dp);
    
    // ========================================================================
    // Statistics
    // ========================================================================
    
    uint32_t getTotalPacketsReceived() { return _totalPacketsReceived; }
    uint32_t getPacketsRelayed() { return _packetsRelayed; }
    uint32_t getPacketsLogged() { return _packetsLogged; }
    void resetStatistics();
    
    // ========================================================================
    // SD Card Logging
    // ========================================================================
    
    /**
     * Set SD card chip select pin
     */
    void setSDPin(uint8_t csPin) { _sdCSPin = csPin; }
    
    /**
     * Initialize SD card for logging
     * @return True if SD card initialized successfully
     */
    bool initSD();
    
    /**
     * Log a packet to SD card
     */
    void logPacketToSD(TelemetryPacket& packet, uint32_t senderSerial);
    
    // ========================================================================
    // Debug
    // ========================================================================
    
    /**
     * Print mesh network status to Serial
     */
    void printStatus();
    
    /**
     * Print data from a specific CubeSat
     */
    void printCubesatData(uint32_t serial);
    
private:
    RF24* _radio;
    TelemetryRadio* _telemetry;
    uint32_t _ownSerial;
    MeshConfig _config;
    
    // Remote CubeSat storage
    RemoteCubeSat _cubesats[MESH_MAX_CUBESATS];
    
    // Statistics
    uint32_t _totalPacketsReceived;
    uint32_t _packetsRelayed;
    uint32_t _packetsLogged;
    uint32_t _lastRelayTime;
    
    // SD Card
    uint8_t _sdCSPin;
    bool _sdInitialized;
    
    // Internal functions
    RemoteCubeSat* findOrCreateCubesat(uint32_t serial);
    void storeDataPoint(RemoteCubeSat* sat, TelemetryPacket& packet);
    void expireCubesats();
    void expireDataPoints(RemoteCubeSat* sat);
    void relayPacket(TelemetryPacket& packet);
};

#endif // TELEMETRY_RADIO_H
