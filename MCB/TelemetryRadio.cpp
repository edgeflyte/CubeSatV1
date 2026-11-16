/*
 * ============================================================================
 * TelemetryRadio.cpp - Radio Telemetry Dictionary & Transmission
 * ============================================================================
 * 
 * Implementation of the telemetry transmission system and data dictionary.
 * 
 * EDUCATIONAL NOTES:
 * -----------------
 * This file demonstrates several important concepts:
 * 
 * 1. DATA SERIALIZATION
 *    Converting structured data (floats, integers, structures) into a 
 *    byte stream for transmission. This is essential for any communication
 *    protocol. We use big-endian (network byte order) for consistency.
 * 
 * 2. TELEMETRY DICTIONARIES
 *    A standardized way to define what data is being transmitted. This
 *    allows ground stations to automatically decode and display data
 *    without hardcoded knowledge of the satellite's firmware.
 * 
 * 3. PACKET FRAMING
 *    How to structure data so the receiver knows where packets begin
 *    and end, and what type of data is contained.
 * 
 * ============================================================================
 */

#include "TelemetryRadio.h"

// ============================================================================
// TELEMETRY DICTIONARY DEFINITION
// ============================================================================

/**
 * The Telemetry Dictionary
 * 
 * This table defines all telemetry data points that the CubeSat can transmit.
 * Each entry specifies:
 *   - Unique identifier (hex code for packet type)
 *   - Mnemonic ID code for easy reference
 *   - Human-readable description
 *   - Data format for parsing
 *   - Source subsystem
 *   - Unit of measurement
 *   - Valid range (min/max values)
 * 
 * When adding custom telemetry, add entries to this table or use the
 * CustomTelemetryPoint system.
 */
const TelemetryDataPoint TELEMETRY_DICTIONARY[] = {
    // ========================================================================
    // System Packets (0x00xx) - MCB System Status
    // ========================================================================
    {0x0001, "STATUS",   "System Status Summary",    FMT_UINT16, SRC_SYS, "",     0,    65535},
    {0x0002, "SERIAL",   "Device Serial Number",     FMT_UINT32, SRC_SYS, "",     0,    0},
    {0x0003, "UPTIME",   "System Uptime",            FMT_UINT32, SRC_SYS, "s",    0,    0},
    {0x0004, "BOOTCNT",  "Boot Counter",             FMT_UINT32, SRC_SYS, "",     0,    0},
    {0x0005, "MODE",     "Operating Mode",           FMT_UINT8,  SRC_SYS, "",     0,    255},
    {0x0006, "ERROR",    "Error Code",               FMT_UINT8,  SRC_SYS, "",     0,    255},
    {0x0007, "VERSION",  "Firmware Version",         FMT_TEXT,   SRC_SYS, "",     0,    0},
    {0x0008, "CMDACK",   "Command Acknowledgement",  FMT_HEX,    SRC_SYS, "",     0,    0},
    
    
    // ========================================================================
    // BMS - Battery Management System Packets (0x01xx)
    // Source: $BMS,ERR,LDO1,LDO2,LDO3,LDO4,BATV,BATPCT,SOL1,SOL2*
    // ========================================================================
    {0x0100, "BMSERR",   "BMS Error Code",           FMT_UINT8,  SRC_BMS, "",     0,    2},
    {0x0101, "BATV",     "Battery Voltage",          FMT_FLOAT,  SRC_BMS, "V",    3.3,  5.6},
    {0x0102, "BATPCT",   "Battery Percentage",       FMT_FLOAT,  SRC_BMS, "%",    0,    100},
    {0x0103, "LDO1",     "LDO Channel 1 State",      FMT_BOOL,   SRC_BMS, "",     0,    1},
    {0x0104, "LDO2",     "LDO Channel 2 State",      FMT_BOOL,   SRC_BMS, "",     0,    1},
    {0x0105, "LDO3",     "LDO Channel 3 State",      FMT_BOOL,   SRC_BMS, "",     0,    1},
    {0x0106, "LDO4",     "LDO Channel 4 State",      FMT_BOOL,   SRC_BMS, "",     0,    1},
    {0x0107, "LDOALL",   "LDO States Packed",        FMT_UINT8,  SRC_BMS, "",     0,    15},
    {0x0108, "BSOL1",    "BMS Solar Panel 1",        FMT_FLOAT,  SRC_BMS, "V",    0,    3.3},
    {0x0109, "BSOL2",    "BMS Solar Panel 2",        FMT_FLOAT,  SRC_BMS, "V",    0,    3.3},
    {0x010F, "BMSALL",   "BMS Combined Status",      FMT_HEX,    SRC_BMS, "",     0,    0},
    
    // ========================================================================
    // SCC - Solar Charge Controller Packets (0x011x-0x013x)
    // Source: $SCC,1 (Voltages), $SCC,2 (Temps), $SCC,3 (Output)
    // ========================================================================
    // SCC Solar Panel Voltages ($SCC,1,V0,V1,V2,V3,V4,V5,*)
    {0x0110, "SCCV0",    "SCC Solar Panel 0",        FMT_FLOAT,  SRC_PWR, "V",    0,    5.0},
    {0x0111, "SCCV1",    "SCC Solar Panel 1",        FMT_FLOAT,  SRC_PWR, "V",    0,    5.0},
    {0x0112, "SCCV2",    "SCC Solar Panel 2",        FMT_FLOAT,  SRC_PWR, "V",    0,    5.0},
    {0x0113, "SCCV3",    "SCC Solar Panel 3",        FMT_FLOAT,  SRC_PWR, "V",    0,    5.0},
    {0x0114, "SCCV4",    "SCC Solar Panel 4",        FMT_FLOAT,  SRC_PWR, "V",    0,    5.0},
    {0x0115, "SCCV5",    "SCC Solar Panel 5",        FMT_FLOAT,  SRC_PWR, "V",    0,    5.0},
    {0x011F, "SCCVALL",  "SCC Voltages Combined",    FMT_HEX,    SRC_PWR, "",     0,    0},
    
    // SCC Output Monitoring ($SCC,3,VOUT,IOUT,MODE*)
    {0x0120, "SCCVOUT",  "SCC Output Voltage",       FMT_FLOAT,  SRC_PWR, "V",    0,    5.5},
    {0x0121, "SCCIOUT",  "SCC Output Current",       FMT_FLOAT,  SRC_PWR, "mA",   0,    2000},
    {0x0122, "SCCMODE",  "SCC System Mode",          FMT_UINT8,  SRC_PWR, "",     0,    255},
    {0x012F, "SCCOUT",   "SCC Output Combined",      FMT_HEX,    SRC_PWR, "",     0,    0},
    
    // SCC Temperature Channels ($SCC,2,T0,T1,T2,T3,T4,T5,T6,T7*)
    {0x0130, "SCCT0",    "SCC Temperature Ch 0",     FMT_FLOAT,  SRC_THERMAL, "°C", -20, 40},
    {0x0131, "SCCT1",    "SCC Temperature Ch 1",     FMT_FLOAT,  SRC_THERMAL, "°C", -20, 40},
    {0x0132, "SCCT2",    "SCC Temperature Ch 2",     FMT_FLOAT,  SRC_THERMAL, "°C", -20, 40},
    {0x0133, "SCCT3",    "SCC Temperature Ch 3",     FMT_FLOAT,  SRC_THERMAL, "°C", -20, 40},
    {0x0134, "SCCT4",    "SCC Temperature Ch 4",     FMT_FLOAT,  SRC_THERMAL, "°C", -20, 40},
    {0x0135, "SCCT5",    "SCC Temperature Ch 5",     FMT_FLOAT,  SRC_THERMAL, "°C", -20, 40},
    {0x0136, "SCCT6",    "SCC Temperature Ch 6",     FMT_FLOAT,  SRC_THERMAL, "°C", -20, 40},
    {0x0137, "SCCT7",    "SCC Temperature Ch 7",     FMT_FLOAT,  SRC_THERMAL, "°C", -20, 40},
    {0x013F, "SCCTALL",  "SCC Temps Combined",       FMT_HEX,    SRC_THERMAL, "",   0,    0},
    
    // ========================================================================
    // Communication Packets (0x02xx)
    // ========================================================================
    {0x0200, "RSSI",     "Signal Strength",          FMT_INT16,  SRC_COMM, "dBm", -100, 0},
    {0x0201, "PKTSSENT", "Packets Sent",             FMT_UINT32, SRC_COMM, "",    0,    0},
    {0x0202, "PKTSRECV", "Packets Received",         FMT_UINT32, SRC_COMM, "",    0,    0},
    {0x0203, "PKTSFAIL", "Failed Packets",           FMT_UINT32, SRC_COMM, "",    0,    0},
    
    // ========================================================================
    // ASB - Atmospheric Sensing Board Packets (0x03xx)
    // Source: $ASB,3 (Environmental), $ASB,4 (Particulate)
    // ========================================================================
    // ASB Environmental Sensors ($ASB,3,T1,T2,RH,P1,PA1,AQI,TVOC,ECO2,CO2,*)
    {0x0300, "ASBT1",    "ASB SHT4x Temperature",    FMT_FLOAT,  SRC_ENV, "°C",   -40,  125},
    {0x0301, "ASBT2",    "ASB BMP180 Temperature",   FMT_FLOAT,  SRC_ENV, "°C",   -40,  85},
    {0x0302, "ASBRH",    "ASB Relative Humidity",    FMT_FLOAT,  SRC_ENV, "%",    0,    100},
    {0x0303, "ASBP1",    "ASB Pressure",             FMT_FLOAT,  SRC_ENV, "Pa",   30000, 110000},
    {0x0304, "ASBPA1",   "ASB Pressure Altitude",    FMT_FLOAT,  SRC_ENV, "m",    -500, 10000},
    {0x0305, "ASBAQI",   "ASB Air Quality Index",    FMT_UINT8,  SRC_ENV, "",     1,    5},
    {0x0306, "ASBTVOC",  "ASB Total VOC",            FMT_UINT16, SRC_ENV, "ppb",  0,    65000},
    {0x0307, "ASBECO2",  "ASB Estimated CO2",        FMT_UINT16, SRC_ENV, "ppm",  400,  65000},
    {0x0308, "ASBCO2",   "ASB CO2 Concentration",    FMT_UINT16, SRC_ENV, "ppm",  400,  10000},
    {0x030F, "ASBENV",   "ASB Environmental All",    FMT_HEX,    SRC_ENV, "",     0,    0},
    
    // ASB Particulate Matter ($ASB,4,MC1,MC25,MC4,MC10,NC05,NC1,NC25,NC4,NC10,TPS,*)
    {0x0310, "ASBMC1",   "ASB PM 1.0 Mass Conc",     FMT_FLOAT,  SRC_ENV, "μg/m³", 0,   1000},
    {0x0311, "ASBMC25",  "ASB PM 2.5 Mass Conc",     FMT_FLOAT,  SRC_ENV, "μg/m³", 0,   1000},
    {0x0312, "ASBMC4",   "ASB PM 4.0 Mass Conc",     FMT_FLOAT,  SRC_ENV, "μg/m³", 0,   1000},
    {0x0313, "ASBMC10",  "ASB PM 10 Mass Conc",      FMT_FLOAT,  SRC_ENV, "μg/m³", 0,   1000},
    {0x0314, "ASBNC05",  "ASB Num Conc 0-0.5μm",     FMT_FLOAT,  SRC_ENV, "/cm³", 0,    10000},
    {0x0315, "ASBNC1",   "ASB Num Conc 0.5-1μm",     FMT_FLOAT,  SRC_ENV, "/cm³", 0,    10000},
    {0x0316, "ASBNC25",  "ASB Num Conc 1-2.5μm",     FMT_FLOAT,  SRC_ENV, "/cm³", 0,    10000},
    {0x0317, "ASBNC4",   "ASB Num Conc 2.5-4μm",     FMT_FLOAT,  SRC_ENV, "/cm³", 0,    10000},
    {0x0318, "ASBNC10",  "ASB Num Conc 4-10μm",      FMT_FLOAT,  SRC_ENV, "/cm³", 0,    10000},
    {0x0319, "ASBTPS",   "ASB Typical Part Size",    FMT_FLOAT,  SRC_ENV, "μm",   0,    10},
    {0x031F, "ASBPM",    "ASB Particulate All",      FMT_HEX,    SRC_ENV, "",     0,    0},
    
    // MCB Local Environmental (for backwards compatibility)
    {0x0320, "MCBTEMP",  "MCB Temperature",          FMT_FLOAT,  SRC_MCB, "°C",   -40,  85},
    {0x0321, "MCBHUM",   "MCB Humidity",             FMT_FLOAT,  SRC_MCB, "%",    0,    100},
    
    // ========================================================================
    // GPS/Navigation Packets (0x04xx)
    // ========================================================================
    {0x0400, "GPSLAT",   "GPS Latitude",             FMT_DOUBLE, SRC_GPS, "°",    -90,  90},
    {0x0401, "GPSLON",   "GPS Longitude",            FMT_DOUBLE, SRC_GPS, "°",    -180, 180},
    {0x0402, "GPSALT",   "GPS Altitude",             FMT_FLOAT,  SRC_GPS, "m",    -500, 50000},
    {0x0403, "GPSSAT",   "GPS Satellites",           FMT_UINT8,  SRC_GPS, "",     0,    24},
    {0x0404, "GPSHDOP",  "GPS HDOP",                 FMT_FLOAT,  SRC_GPS, "",     0,    99.9},
    {0x0405, "GPSFIX",   "GPS Fix Status",           FMT_BOOL,   SRC_GPS, "",     0,    1},
    {0x0410, "GPSTIME",  "GPS Time",                 FMT_UINT32, SRC_GPS, "",     0,    0},
    {0x0411, "GPSDATE",  "GPS Date",                 FMT_UINT32, SRC_GPS, "",     0,    0},
    {0x041F, "GPSALL",   "GPS Combined",             FMT_HEX,    SRC_GPS, "",     0,    0},
    
    // ========================================================================
    // IMU/Attitude Packets (0x05xx) - MCB MPU9250
    // ========================================================================
    {0x0500, "ACCELX",   "Accelerometer X",          FMT_FLOAT,  SRC_IMU, "g",    -16,  16},
    {0x0501, "ACCELY",   "Accelerometer Y",          FMT_FLOAT,  SRC_IMU, "g",    -16,  16},
    {0x0502, "ACCELZ",   "Accelerometer Z",          FMT_FLOAT,  SRC_IMU, "g",    -16,  16},
    {0x050F, "ACCELALL", "Accelerometer Combined",   FMT_HEX,    SRC_IMU, "",     0,    0},
    {0x0510, "GYROX",    "Gyroscope X",              FMT_FLOAT,  SRC_IMU, "°/s",  -2000, 2000},
    {0x0511, "GYROY",    "Gyroscope Y",              FMT_FLOAT,  SRC_IMU, "°/s",  -2000, 2000},
    {0x0512, "GYROZ",    "Gyroscope Z",              FMT_FLOAT,  SRC_IMU, "°/s",  -2000, 2000},
    {0x051F, "GYROALL",  "Gyroscope Combined",       FMT_HEX,    SRC_IMU, "",     0,    0},
    {0x0520, "MAGX",     "Magnetometer X",           FMT_FLOAT,  SRC_IMU, "μT",   -4900, 4900},
    {0x0521, "MAGY",     "Magnetometer Y",           FMT_FLOAT,  SRC_IMU, "μT",   -4900, 4900},
    {0x0522, "MAGZ",     "Magnetometer Z",           FMT_FLOAT,  SRC_IMU, "μT",   -4900, 4900},
    {0x052F, "MAGALL",   "Magnetometer Combined",    FMT_HEX,    SRC_IMU, "",     0,    0},
    {0x0530, "ROLL",     "Roll Angle",               FMT_FLOAT,  SRC_IMU, "°",    -180, 180},
    {0x0531, "PITCH",    "Pitch Angle",              FMT_FLOAT,  SRC_IMU, "°",    -90,  90},
    {0x0532, "YAW",      "Yaw Angle",                FMT_FLOAT,  SRC_IMU, "°",    -180, 180},
    {0x053F, "ATTALL",   "Attitude Combined",        FMT_HEX,    SRC_IMU, "",     0,    0},
    {0x05FF, "IMUALL",   "IMU All Data",             FMT_HEX,    SRC_IMU, "",     0,    0},
    
    // ========================================================================
    // Payload Packets (0x06xx) - Reserved for User Extensions
    // ========================================================================
    {0x0600, "PAYLOAD1", "Payload Data 1",           FMT_FLOAT,  SRC_PAYLOAD, "",  0,    0},
    {0x0601, "PAYLOAD2", "Payload Data 2",           FMT_FLOAT,  SRC_PAYLOAD, "",  0,    0},
    {0x0602, "PAYLOAD3", "Payload Data 3",           FMT_FLOAT,  SRC_PAYLOAD, "",  0,    0},
    {0x0603, "PAYLOAD4", "Payload Data 4",           FMT_FLOAT,  SRC_PAYLOAD, "",  0,    0},
    {0x060F, "PLDSTAT",  "Payload Status",           FMT_UINT8,  SRC_PAYLOAD, "",  0,    255},

    // ========================================================================
    // Mesh Messaging (0x09xx)
    // ========================================================================
    {0x0900, "MSGTEXT",  "Short Mesh Message",       FMT_TEXT,   SRC_SYS, "",     0,    0},
};

// Calculate dictionary size at compile time
const uint16_t TELEMETRY_DICTIONARY_SIZE = sizeof(TELEMETRY_DICTIONARY) / sizeof(TelemetryDataPoint);

// ============================================================================
// TELEMETRY RADIO IMPLEMENTATION
// ============================================================================

/**
 * Initialize the telemetry radio system
 */
void TelemetryRadio::begin(RF24* radio, uint32_t deviceSerial) {
    _radio = radio;
    _deviceSerial = deviceSerial;
    _destinationSerial = TELEM_BROADCAST_ADDR;  // Default to broadcast
    _packetsSent = 0;
    _packetsFailed = 0;
    _inBatch = false;
    _ledSuccessPin = 0;
    _ledFailPin = 0;
}

void TelemetryRadio::setLedPins(uint8_t successPin, uint8_t failurePin) {
    _ledSuccessPin = successPin;
    _ledFailPin = failurePin;
}

/**
 * Set destination address for packets
 */
void TelemetryRadio::setDestination(uint32_t destination) {
    _destinationSerial = destination;
}

/**
 * Reset transmission statistics
 */
void TelemetryRadio::resetStatistics() {
    _packetsSent = 0;
    _packetsFailed = 0;
}

// ============================================================================
// SYSTEM TELEMETRY
// ============================================================================

/**
 * Send comprehensive system status packet
 * 
 * Packet Data Format:
 *   [0]     Mode (1 byte)
 *   [1-2]   Status flags (2 bytes)
 *   [3-6]   Uptime (4 bytes)
 *   [7-10]  Boot count (4 bytes)
 */
void TelemetryRadio::sendSystemStatus(uint8_t mode, SystemStatus& status, 
                                       uint32_t uptime, uint32_t bootCount) {
    uint8_t data[11];
    
    // Mode
    data[0] = mode;
    
    // Status flags - pack into 2 bytes
    uint16_t flags = 0;
    if (status.sensorSHT)    flags |= (1 << 0);
    if (status.sensorIMU)    flags |= (1 << 1);
    if (status.sensorGPS)    flags |= (1 << 2);
    if (status.radioNRF)     flags |= (1 << 3);
    if (status.radioBLE)     flags |= (1 << 4);
    if (status.satbusASB)    flags |= (1 << 5);
    if (status.satbusBMS)    flags |= (1 << 6);
    if (status.satbusSCC)    flags |= (1 << 7);
    if (status.rbfInserted)  flags |= (1 << 8);
    if (status.lowBattery)   flags |= (1 << 9);
    if (status.errorState)   flags |= (1 << 10);
    encodeUint16(&data[1], flags);
    
    // Uptime
    encodeUint32(&data[3], uptime);
    
    // Boot count
    encodeUint32(&data[7], bootCount);
    
    TelemetryPacket packet = buildPacket(TELEM_SYS_STATUS, data, 11);
    transmit(packet);
}

/**
 * Send firmware version
 */
void TelemetryRadio::sendVersion(const char* version) {
    uint8_t data[16];
    memset(data, 0, sizeof(data));
    strncpy((char*)data, version, 15);
    
    TelemetryPacket packet = buildPacket(TELEM_SYS_VERSION, data, strlen((char*)data));
    transmit(packet);
}

/**
 * Send error code with message
 */
void TelemetryRadio::sendError(uint8_t errorCode, const char* message) {
    uint8_t data[17];
    data[0] = errorCode;
    memset(&data[1], 0, 16);
    strncpy((char*)&data[1], message, 15);
    
    TelemetryPacket packet = buildPacket(TELEM_SYS_ERROR, data, strlen((char*)&data[1]) + 1);
    transmit(packet);
}

// ============================================================================
// SENSOR TELEMETRY
// ============================================================================

/**
 * Send local sensor data
 * 
 * Transmits MCB onboard sensor readings (SHT4x temp/humidity, MPU9250 IMU)
 */
void TelemetryRadio::sendSensorData(SensorData& sensors, GPSData& gps) {
    // Send MCB temperature/humidity packet
    {
        uint8_t data[8];
        encodeFloat(&data[0], sensors.temperature);
        encodeFloat(&data[4], sensors.humidity);
        
        TelemetryPacket packet = buildPacket(TELEM_MCB_TEMP, data, 8);
        transmit(packet);
    }
    
    // Send IMU data
    sendIMUData(sensors);
    
    // Send GPS if available
    if (gps.hasValidFix) {
        sendGPSPosition(gps);
    }
}

/**
 * Send GPS position data
 * 
 * Transmits GPS coordinates, altitude, satellite count, and fix quality.
 * Lat/Lon encoded as scaled integers (x1e7) to preserve precision.
 */
void TelemetryRadio::sendGPSPosition(GPSData& gps) {
    // GPS packets need special handling for double precision lat/lon
    // We send as scaled integers to fit in the packet
    
    uint8_t data[16];
    
    // Latitude: multiply by 1e7 and store as int32 (-90 to +90 degrees)
    int32_t latInt = (int32_t)(gps.latitude * 10000000.0);
    encodeUint32(&data[0], (uint32_t)latInt);
    
    // Longitude: multiply by 1e7 and store as int32 (-180 to +180 degrees)
    int32_t lonInt = (int32_t)(gps.longitude * 10000000.0);
    encodeUint32(&data[4], (uint32_t)lonInt);
    
    // Altitude in meters
    encodeFloat(&data[8], gps.altitude);
    
    // Number of satellites tracked
    data[12] = gps.satellites;
    
    // HDOP scaled by 10 (e.g., 1.5 -> 15)
    data[13] = (uint8_t)(gps.hdop * 10);
    
    // Fix status (0=no fix, 1=valid fix)
    data[14] = gps.hasValidFix ? 1 : 0;
    
    TelemetryPacket packet = buildPacket(TELEM_GPS_ALL, data, 15);
    transmit(packet);
}

/**
 * Send IMU data (accelerometer, gyroscope, magnetometer)
 * 
 * Transmits all 9 axes of MPU9250 data in a single combined packet.
 * Values are scaled to preserve precision within int16 range.
 */
void TelemetryRadio::sendIMUData(SensorData& sensors) {
    uint8_t data[18];
    
    // Accelerometer X/Y/Z (scaled by 1000: ±16g -> ±16000)
    encodeInt16(&data[0], (int16_t)(sensors.accelX * 1000));
    encodeInt16(&data[2], (int16_t)(sensors.accelY * 1000));
    encodeInt16(&data[4], (int16_t)(sensors.accelZ * 1000));
    
    // Gyroscope X/Y/Z (scaled by 10: ±2000°/s -> ±20000)
    encodeInt16(&data[6], (int16_t)(sensors.gyroX * 10));
    encodeInt16(&data[8], (int16_t)(sensors.gyroY * 10));
    encodeInt16(&data[10], (int16_t)(sensors.gyroZ * 10));
    
    // Magnetometer X/Y/Z (scaled by 10: ±4900μT -> ±49000)
    encodeInt16(&data[12], (int16_t)(sensors.magX * 10));
    encodeInt16(&data[14], (int16_t)(sensors.magY * 10));
    encodeInt16(&data[16], (int16_t)(sensors.magZ * 10));
    
    TelemetryPacket packet = buildPacket(TELEM_IMU_ALL, data, 18);
    transmit(packet);
}

// ============================================================================
// SATBUS MODULE TELEMETRY
// ============================================================================

/**
 * Send ASB (Atmospheric Sensing Board) data
 * 
 * Transmits environmental and particulate matter data from the ASB module.
 * Data is packed efficiently to fit within NRF24L01 payload limits.
 */
void TelemetryRadio::sendASBData(ASBData& data) {
    if (data.hasEnvironmental) {
        // Pack ASB environmental data
        // Format: T1(4) + T2(4) + RH(4) + P1(4) + AQI(1) + TVOC(2) + CO2(2) = 21 bytes
        uint8_t envData[22];
        
        encodeFloat(&envData[0], data.temperature1);   // SHT4x temp
        encodeFloat(&envData[4], data.temperature2);   // BMP180 temp
        encodeFloat(&envData[8], data.humidity);       // Relative humidity
        encodeFloat(&envData[12], data.pressure);      // Pressure (Pa)
        envData[16] = data.airQualityIndex;            // AQI (1-5)
        encodeUint16(&envData[17], data.tvoc);         // TVOC (ppb)
        encodeUint16(&envData[19], data.eco2);         // eCO2 (ppm)
        encodeUint16(&envData[21], data.co2);          // CO2 (ppm)
        
        TelemetryPacket packet = buildPacket(TELEM_ASB_ENV, envData, 23);
        transmit(packet);
    }
    
    if (data.hasParticulate) {
        // Pack ASB particulate matter data
        // Format: MC1-MC10(16) + TPS(4) = 20 bytes
        uint8_t pmData[20];
        
        encodeFloat(&pmData[0], data.pm1_0);           // PM 1.0 mass conc
        encodeFloat(&pmData[4], data.pm2_5);           // PM 2.5 mass conc
        encodeFloat(&pmData[8], data.pm4_0);           // PM 4.0 mass conc
        encodeFloat(&pmData[12], data.pm10);           // PM 10 mass conc
        encodeFloat(&pmData[16], data.typicalParticleSize);  // Typical size
        
        TelemetryPacket packet = buildPacket(TELEM_ASB_PM, pmData, 20);
        transmit(packet);
    }
}

/**
 * Send BMS (Battery Management System) data
 * 
 * Transmits battery status, LDO channel states, and solar panel voltages.
 * Maps to SATBUS packet: $BMS,ERR,LDO1,LDO2,LDO3,LDO4,BATV,BATPCT,SOL1,SOL2*
 */
void TelemetryRadio::sendBMSData(BMSData& data) {
    // Pack all BMS data into single packet
    // Format: ERR(1) + LDO_packed(1) + BATV(4) + BATPCT(4) + SOL1(2) + SOL2(2) = 14 bytes
    uint8_t bmsData[14];
    
    // Error code: 0=OK, 1=Low (≤20%), 2=Critical (≤5%)
    bmsData[0] = data.errorCode;
    
    // LDO states packed into one byte (bit flags)
    bmsData[1] = (data.ldo1State ? 0x01 : 0) |
                 (data.ldo2State ? 0x02 : 0) |
                 (data.ldo3State ? 0x04 : 0) |
                 (data.ldo4State ? 0x08 : 0);
    
    // Battery voltage (V) and percentage (%)
    encodeFloat(&bmsData[2], data.batteryVoltage);
    encodeFloat(&bmsData[6], data.batteryPercent);
    
    // Solar panel voltages (scaled by 100 for 2-byte encoding)
    encodeUint16(&bmsData[10], (uint16_t)(data.solarVoltage1 * 100));
    encodeUint16(&bmsData[12], (uint16_t)(data.solarVoltage2 * 100));
    
    TelemetryPacket packet = buildPacket(TELEM_BMS_ALL, bmsData, 14);
    transmit(packet);
}

/**
 * Send SCC (Solar Charge Controller) data
 * 
 * Transmits solar panel voltages, temperatures, and output monitoring.
 * Maps to SATBUS packets: $SCC,1 (voltages), $SCC,2 (temps), $SCC,3 (output)
 */
void TelemetryRadio::sendSCCData(SCCData& data) {
    // Send solar panel voltages ($SCC,1,V0,V1,V2,V3,V4,V5,*)
    if (data.hasVoltages) {
        uint8_t volData[12];
        
        // 6 solar panel voltages, 2 bytes each (scaled by 100)
        for (int i = 0; i < 6; i++) {
            encodeUint16(&volData[i * 2], (uint16_t)(data.solarVoltage[i] * 100));
        }
        
        TelemetryPacket packet = buildPacket(TELEM_SCC_VALL, volData, 12);
        transmit(packet);
    }
    
    // Send temperature channels ($SCC,2,T0,T1,T2,T3,T4,T5,T6,T7*)
    if (data.hasTemperatures) {
        uint8_t tempData[16];
        
        // 8 temperature channels, 2 bytes each (scaled by 10)
        for (int i = 0; i < 8; i++) {
            int16_t tempScaled = (int16_t)(data.temperature[i] * 10);
            encodeInt16(&tempData[i * 2], tempScaled);
        }
        
        TelemetryPacket packet = buildPacket(TELEM_SCC_TALL, tempData, 16);
        transmit(packet);
    }
    
    // Send output monitoring ($SCC,3,VOUT,IOUT,MODE*)
    if (data.hasOutput) {
        uint8_t outData[9];
        
        encodeFloat(&outData[0], data.outputVoltage);   // Output voltage (V)
        encodeFloat(&outData[4], data.outputCurrent);   // Output current (mA)
        outData[8] = data.systemMode;                   // System mode
        
        TelemetryPacket packet = buildPacket(TELEM_SCC_OUT, outData, 9);
        transmit(packet);
    }
}

// ============================================================================
// PACKET BUILDING AND TRANSMISSION
// ============================================================================

/**
 * Build a telemetry packet
 */
TelemetryPacket TelemetryRadio::buildPacket(uint16_t type, uint8_t* data, uint8_t length) {
    TelemetryPacket packet;
    
    packet.preamble[0] = '#';
    packet.preamble[1] = '1';
    packet.transmitterId = _deviceSerial;
    packet.destinationId = _destinationSerial;
    packet.packetType = type;
    
    // Clamp to the usable on-wire data size (20 bytes) so the stored
    // dataLength never exceeds what serializePacket() actually transmits.
    if (length > TELEM_MAX_WIRE_DATA_SIZE) {
        length = TELEM_MAX_WIRE_DATA_SIZE;
    }
    
    memcpy(packet.data, data, length);
    packet.dataLength = length;
    
    return packet;
}

/**
 * Start a transmit batch: radio stays in TX until endTransmitBatch().
 * (Kept for API compatibility; main loop uses one-packet-at-a-time for ruggedness.)
 */
void TelemetryRadio::beginTransmitBatch() {
    if (_radio == nullptr) return;
    _inBatch = true;
    _radio->stopListening();
}

/**
 * End transmit batch: return radio to RX.
 */
void TelemetryRadio::endTransmitBatch() {
    if (_radio == nullptr) return;
    _inBatch = false;
    _radio->flush_tx();
    _radio->flush_rx();
    _radio->startListening();
    delay(2);
}

/**
 * Transmit one packet: full stop→write→flush→start→settle every time (rugged).
 * Broadcast = multicast/no-ACK. LED1 flashes on success, LED3 on failure.
 */
bool TelemetryRadio::transmit(TelemetryPacket& packet) {
    if (_radio == nullptr) {
        if (_ledFailPin) { digitalWrite(_ledFailPin, HIGH); delay(30); digitalWrite(_ledFailPin, LOW); }
        return false;
    }
    
    uint8_t txBuffer[32];
    uint8_t txLength;
    serializePacket(packet, txBuffer, txLength);
    const bool multicast = (packet.destinationId == TELEM_BROADCAST_ADDR);
    
    // Minimal sequence: no flush in path (can leave some NRF24 in bad state)
    _radio->stopListening();
    delay(1);
    bool success = _radio->write(txBuffer, txLength, multicast);
    _radio->startListening();
    delay(2);
    
    if (success) {
        _packetsSent++;
        if (_ledSuccessPin) { digitalWrite(_ledSuccessPin, HIGH); delay(30); digitalWrite(_ledSuccessPin, LOW); }
    } else {
        _packetsFailed++;
        if (_ledFailPin) { digitalWrite(_ledFailPin, HIGH); delay(30); digitalWrite(_ledFailPin, LOW); }
    }
    
    return success;
}

/**
 * Serialize packet structure to byte buffer
 * 
 * Output format:
 *   [0-1]   Preamble "#1"
 *   [2-5]   Transmitter ID (big-endian)
 *   [6-9]   Destination ID (big-endian)
 *   [10-11] Packet Type (big-endian)
 *   [12+]   Data bytes
 */
void TelemetryRadio::serializePacket(TelemetryPacket& packet, uint8_t* buffer, uint8_t& length) {
    uint8_t idx = 0;
    
    // Preamble
    buffer[idx++] = packet.preamble[0];
    buffer[idx++] = packet.preamble[1];
    
    // Transmitter ID (big-endian)
    buffer[idx++] = (packet.transmitterId >> 24) & 0xFF;
    buffer[idx++] = (packet.transmitterId >> 16) & 0xFF;
    buffer[idx++] = (packet.transmitterId >> 8) & 0xFF;
    buffer[idx++] = packet.transmitterId & 0xFF;
    
    // Destination ID (big-endian)
    buffer[idx++] = (packet.destinationId >> 24) & 0xFF;
    buffer[idx++] = (packet.destinationId >> 16) & 0xFF;
    buffer[idx++] = (packet.destinationId >> 8) & 0xFF;
    buffer[idx++] = packet.destinationId & 0xFF;
    
    // Packet type (big-endian)
    buffer[idx++] = (packet.packetType >> 8) & 0xFF;
    buffer[idx++] = packet.packetType & 0xFF;
    
    // Data
    uint8_t dataLen = min(packet.dataLength, (uint8_t)(32 - idx));
    memcpy(&buffer[idx], packet.data, dataLen);
    idx += dataLen;
    
    length = idx;
}

/**
 * Deserialize a received radio buffer into a packet structure.
 *
 * Inverse of serializePacket(): reads a 12-byte big-endian header
 * (preamble, transmitter ID, destination ID, packet type) followed by the
 * data payload. Returns false if the buffer is too short for a full header.
 */
bool TelemetryRadio::deserializePacket(const uint8_t* buffer, uint8_t length, TelemetryPacket& packet) {
    if (buffer == nullptr || length < TELEM_HEADER_SIZE) {
        return false;
    }
    
    uint8_t idx = 0;
    
    // Preamble
    packet.preamble[0] = buffer[idx++];
    packet.preamble[1] = buffer[idx++];
    
    // Transmitter ID (big-endian)
    packet.transmitterId = ((uint32_t)buffer[idx]     << 24) |
                           ((uint32_t)buffer[idx + 1] << 16) |
                           ((uint32_t)buffer[idx + 2] << 8)  |
                            (uint32_t)buffer[idx + 3];
    idx += 4;
    
    // Destination ID (big-endian)
    packet.destinationId = ((uint32_t)buffer[idx]     << 24) |
                           ((uint32_t)buffer[idx + 1] << 16) |
                           ((uint32_t)buffer[idx + 2] << 8)  |
                            (uint32_t)buffer[idx + 3];
    idx += 4;
    
    // Packet type (big-endian)
    packet.packetType = ((uint16_t)buffer[idx] << 8) | (uint16_t)buffer[idx + 1];
    idx += 2;
    
    // Remaining bytes are payload data
    uint8_t dataLen = length - idx;
    if (dataLen > TELEM_MAX_DATA_SIZE) {
        dataLen = TELEM_MAX_DATA_SIZE;
    }
    memcpy(packet.data, &buffer[idx], dataLen);
    packet.dataLength = dataLen;
    
    return true;
}

// ============================================================================
// DATA ENCODING HELPERS
// ============================================================================

/**
 * Encode a float as 4 bytes (IEEE 754, big-endian)
 */
void TelemetryRadio::encodeFloat(uint8_t* buffer, float value) {
    union {
        float f;
        uint32_t u;
    } conv;
    
    conv.f = value;
    encodeUint32(buffer, conv.u);
}

/**
 * Encode a 32-bit unsigned integer (big-endian)
 */
void TelemetryRadio::encodeUint32(uint8_t* buffer, uint32_t value) {
    buffer[0] = (value >> 24) & 0xFF;
    buffer[1] = (value >> 16) & 0xFF;
    buffer[2] = (value >> 8) & 0xFF;
    buffer[3] = value & 0xFF;
}

/**
 * Encode a 16-bit unsigned integer (big-endian)
 */
void TelemetryRadio::encodeUint16(uint8_t* buffer, uint16_t value) {
    buffer[0] = (value >> 8) & 0xFF;
    buffer[1] = value & 0xFF;
}

/**
 * Encode a 16-bit signed integer (big-endian)
 */
void TelemetryRadio::encodeInt16(uint8_t* buffer, int16_t value) {
    encodeUint16(buffer, (uint16_t)value);
}

// ============================================================================
// DICTIONARY ACCESS
// ============================================================================

/**
 * Get data point definition by type identifier
 */
const TelemetryDataPoint* TelemetryRadio::getDataPoint(uint16_t type) {
    for (uint16_t i = 0; i < TELEMETRY_DICTIONARY_SIZE; i++) {
        if (TELEMETRY_DICTIONARY[i].identifier == type) {
            return &TELEMETRY_DICTIONARY[i];
        }
    }
    return nullptr;
}

/**
 * Get total number of defined data points
 */
uint16_t TelemetryRadio::getDataPointCount() {
    return TELEMETRY_DICTIONARY_SIZE;
}

// ============================================================================
// END OF FILE
// ============================================================================

/*
 * ADDING CUSTOM TELEMETRY DATA POINTS
 * ===================================
 * 
 * To add custom telemetry for your payload:
 * 
 * 1. Choose an identifier in the payload range (0x06xx)
 * 
 * 2. Add entry to TELEMETRY_DICTIONARY[] above:
 *    {0x0610, "MYDATA", "My Custom Data", FMT_FLOAT, SRC_PAYLOAD, "units", 0, 100},
 * 
 * 3. Create a send function in your code:
 * 
 *    void sendMyPayloadData(float value) {
 *        uint8_t data[4];
 *        // Encode your data...
 *        TelemetryPacket pkt = telemetry.buildPacket(0x0610, data, 4);
 *        telemetry.transmit(pkt);
 *    }
 * 
 * 4. Update your ground station's telemetry dictionary to match
 */

// ============================================================================
// MESH NETWORK IMPLEMENTATION
// ============================================================================

/**
 * Initialize mesh network
 */
void MeshNetwork::begin(RF24* radio, TelemetryRadio* telemetry, uint32_t ownSerial) {
    _radio = radio;
    _telemetry = telemetry;
    _ownSerial = ownSerial;
    
    _totalPacketsReceived = 0;
    _packetsRelayed = 0;
    _packetsLogged = 0;
    _lastRelayTime = 0;
    
    _sdCSPin = 17;  // Default SD CS pin
    _sdInitialized = false;
    
    // Initialize config with defaults
    _config.enabled = true;
    _config.logToSD = false;
    _config.relayEnabled = false;
    _config.cubesatTimeout = MESH_CUBESAT_TIMEOUT_MS;
    _config.datapointTimeout = MESH_DATAPOINT_TIMEOUT_MS;
    
    // Clear CubeSat storage
    for (uint8_t i = 0; i < MESH_MAX_CUBESATS; i++) {
        _cubesats[i].active = false;
        _cubesats[i].serialNumber = 0;
    }
}

/**
 * Process a received packet
 */
bool MeshNetwork::processPacket(TelemetryPacket& packet, bool isRelay) {
    if (!_config.enabled) return false;
    
    // Check if packet is from another CubeSat (not us)
    if (packet.transmitterId == _ownSerial) {
        return false;  // It's our own packet
    }
    
    // Check if it's addressed to us specifically
    if (packet.destinationId != 0x00000000 && 
        packet.destinationId != _ownSerial) {
        // Not for us and not broadcast - still track it
    }
    
    _totalPacketsReceived++;
    
    // Find or create entry for this CubeSat
    RemoteCubeSat* sat = findOrCreateCubesat(packet.transmitterId);
    if (sat == nullptr) {
        return true;  // Storage full, but packet was from another CubeSat
    }
    
    // Update CubeSat info
    sat->lastSeen = millis();
    sat->packetsReceived++;
    
    // Store the data point
    storeDataPoint(sat, packet);
    
    // Log to SD if enabled
    if (_config.logToSD && _sdInitialized) {
        logPacketToSD(packet, packet.transmitterId);
    }
    
    // Relay telemetry and messages; never relay command packets (0x8xxx)
    if (_config.relayEnabled && !isRelay && (packet.packetType & 0x8000) == 0) {
        relayPacket(packet);
    }
    
    return true;
}

/**
 * Periodic update
 */
void MeshNetwork::update() {
    if (!_config.enabled) return;
    
    // Expire old CubeSats and data points
    expireCubesats();
}

/**
 * Find an existing CubeSat or create a new entry
 */
RemoteCubeSat* MeshNetwork::findOrCreateCubesat(uint32_t serial) {
    RemoteCubeSat* emptySlot = nullptr;
    
    for (uint8_t i = 0; i < MESH_MAX_CUBESATS; i++) {
        if (_cubesats[i].active && _cubesats[i].serialNumber == serial) {
            return &_cubesats[i];
        }
        if (!_cubesats[i].active && emptySlot == nullptr) {
            emptySlot = &_cubesats[i];
        }
    }
    
    // Create new entry if we have space
    if (emptySlot != nullptr) {
        emptySlot->active = true;
        emptySlot->serialNumber = serial;
        emptySlot->lastSeen = millis();
        emptySlot->packetsReceived = 0;
        emptySlot->rssi = 0;
        
        // Clear data points
        for (uint8_t j = 0; j < MESH_MAX_DATAPOINTS; j++) {
            emptySlot->dataPoints[j].valid = false;
        }
        
        return emptySlot;
    }
    
    return nullptr;  // No space
}

/**
 * Store a data point for a CubeSat
 */
void MeshNetwork::storeDataPoint(RemoteCubeSat* sat, TelemetryPacket& packet) {
    // Find existing data point of same type or empty slot
    RemoteDataPoint* emptySlot = nullptr;
    
    for (uint8_t i = 0; i < MESH_MAX_DATAPOINTS; i++) {
        if (sat->dataPoints[i].valid && sat->dataPoints[i].type == packet.packetType) {
            // Update existing
            memcpy(sat->dataPoints[i].data, packet.data, packet.dataLength);
            sat->dataPoints[i].dataLength = packet.dataLength;
            sat->dataPoints[i].timestamp = millis();
            return;
        }
        if (!sat->dataPoints[i].valid && emptySlot == nullptr) {
            emptySlot = &sat->dataPoints[i];
        }
    }
    
    // Store in new slot
    if (emptySlot != nullptr) {
        emptySlot->valid = true;
        emptySlot->type = packet.packetType;
        memcpy(emptySlot->data, packet.data, packet.dataLength);
        emptySlot->dataLength = packet.dataLength;
        emptySlot->timestamp = millis();
    }
}

/**
 * Expire old CubeSats
 */
void MeshNetwork::expireCubesats() {
    uint32_t now = millis();
    
    for (uint8_t i = 0; i < MESH_MAX_CUBESATS; i++) {
        if (_cubesats[i].active) {
            // Check CubeSat timeout
            if (now - _cubesats[i].lastSeen > _config.cubesatTimeout) {
                _cubesats[i].active = false;
            } else {
                // Expire old data points
                expireDataPoints(&_cubesats[i]);
            }
        }
    }
}

/**
 * Expire old data points for a CubeSat
 */
void MeshNetwork::expireDataPoints(RemoteCubeSat* sat) {
    uint32_t now = millis();
    
    for (uint8_t i = 0; i < MESH_MAX_DATAPOINTS; i++) {
        if (sat->dataPoints[i].valid) {
            if (now - sat->dataPoints[i].timestamp > _config.datapointTimeout) {
                sat->dataPoints[i].valid = false;
            }
        }
    }
}

/**
 * Relay a packet from another CubeSat
 */
void MeshNetwork::relayPacket(TelemetryPacket& packet) {
    if (_radio == nullptr) return;
    
    // Don't relay too frequently
    uint32_t now = millis();
    if (now - _lastRelayTime < 100) return;  // Min 100ms between relays
    _lastRelayTime = now;
    
    // Build relay packet with #R preamble
    TelemetryPacket relayPkt = packet;
    relayPkt.preamble[0] = '#';
    relayPkt.preamble[1] = 'R';
    
    // Transmit
    _radio->stopListening();
    
    uint8_t buffer[32];
    uint8_t bufLen = 0;
    
    // Serialize packet
    buffer[bufLen++] = relayPkt.preamble[0];
    buffer[bufLen++] = relayPkt.preamble[1];
    buffer[bufLen++] = (relayPkt.transmitterId >> 24) & 0xFF;
    buffer[bufLen++] = (relayPkt.transmitterId >> 16) & 0xFF;
    buffer[bufLen++] = (relayPkt.transmitterId >> 8) & 0xFF;
    buffer[bufLen++] = relayPkt.transmitterId & 0xFF;
    buffer[bufLen++] = (relayPkt.destinationId >> 24) & 0xFF;
    buffer[bufLen++] = (relayPkt.destinationId >> 16) & 0xFF;
    buffer[bufLen++] = (relayPkt.destinationId >> 8) & 0xFF;
    buffer[bufLen++] = relayPkt.destinationId & 0xFF;
    buffer[bufLen++] = (relayPkt.packetType >> 8) & 0xFF;
    buffer[bufLen++] = relayPkt.packetType & 0xFF;
    
    uint8_t dataLen = min(relayPkt.dataLength, (uint8_t)(32 - bufLen));
    memcpy(&buffer[bufLen], relayPkt.data, dataLen);
    bufLen += dataLen;
    
    _radio->write(buffer, bufLen, true);
    _radio->startListening();
    delay(2);
    _packetsRelayed++;
}

// ========================================================================
// QUERY FUNCTIONS
// ========================================================================

uint8_t MeshNetwork::getActiveCubesatCount() {
    uint8_t count = 0;
    uint32_t now = millis();
    
    for (uint8_t i = 0; i < MESH_MAX_CUBESATS; i++) {
        if (_cubesats[i].active && 
            (now - _cubesats[i].lastSeen) < _config.cubesatTimeout) {
            count++;
        }
    }
    return count;
}

uint8_t MeshNetwork::getActiveCubesats(uint32_t* serials, uint8_t maxCount) {
    uint8_t count = 0;
    uint32_t now = millis();
    
    for (uint8_t i = 0; i < MESH_MAX_CUBESATS && count < maxCount; i++) {
        if (_cubesats[i].active && 
            (now - _cubesats[i].lastSeen) < _config.cubesatTimeout) {
            serials[count++] = _cubesats[i].serialNumber;
        }
    }
    return count;
}

RemoteCubeSat* MeshNetwork::getCubesat(uint32_t serial) {
    for (uint8_t i = 0; i < MESH_MAX_CUBESATS; i++) {
        if (_cubesats[i].active && _cubesats[i].serialNumber == serial) {
            return &_cubesats[i];
        }
    }
    return nullptr;
}

RemoteDataPoint* MeshNetwork::getDataPoint(uint32_t serial, uint16_t type) {
    RemoteCubeSat* sat = getCubesat(serial);
    if (sat == nullptr) return nullptr;
    
    for (uint8_t i = 0; i < MESH_MAX_DATAPOINTS; i++) {
        if (sat->dataPoints[i].valid && sat->dataPoints[i].type == type) {
            return &sat->dataPoints[i];
        }
    }
    return nullptr;
}

bool MeshNetwork::isCubesatActive(uint32_t serial) {
    RemoteCubeSat* sat = getCubesat(serial);
    if (sat == nullptr) return false;
    return (millis() - sat->lastSeen) < _config.cubesatTimeout;
}

bool MeshNetwork::isDataPointValid(RemoteDataPoint* dp) {
    if (dp == nullptr || !dp->valid) return false;
    return (millis() - dp->timestamp) < _config.datapointTimeout;
}

void MeshNetwork::resetStatistics() {
    _totalPacketsReceived = 0;
    _packetsRelayed = 0;
    _packetsLogged = 0;
}

// ========================================================================
// SD CARD LOGGING
// ========================================================================

bool MeshNetwork::initSD() {
    // Note: Requires SD library to be included
    // This is a placeholder - actual SD init depends on your SD library
    _sdInitialized = false;
    
    // Uncomment when SD library is added:
    // if (SD.begin(_sdCSPin)) {
    //     _sdInitialized = true;
    // }
    
    return _sdInitialized;
}

void MeshNetwork::logPacketToSD(TelemetryPacket& packet, uint32_t senderSerial) {
    if (!_sdInitialized) return;
    
    _packetsLogged++;
    
    // Note: Requires SD library
    // Placeholder implementation:
    // File logFile = SD.open("mesh_log.csv", FILE_WRITE);
    // if (logFile) {
    //     logFile.print(millis());
    //     logFile.print(",");
    //     logFile.print(senderSerial, HEX);
    //     logFile.print(",");
    //     logFile.print(packet.packetType, HEX);
    //     logFile.print(",");
    //     for (int i = 0; i < packet.dataLength; i++) {
    //         logFile.print(packet.data[i], HEX);
    //     }
    //     logFile.println();
    //     logFile.close();
    // }
}

// ========================================================================
// DEBUG OUTPUT
// ========================================================================

void MeshNetwork::printStatus() {
    Serial.println();
    Serial.println(F("╔═══════════════════════════════════════════╗"));
    Serial.println(F("║           MESH NETWORK STATUS             ║"));
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    
    Serial.print(F("║ Mesh Enabled:    "));
    Serial.println(_config.enabled ? "YES" : "NO");
    Serial.print(F("║ Relay Enabled:   "));
    Serial.println(_config.relayEnabled ? "YES" : "NO");
    Serial.print(F("║ SD Logging:      "));
    Serial.println(_config.logToSD ? "YES" : "NO");
    
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    Serial.print(F("║ Active CubeSats: "));
    Serial.println(getActiveCubesatCount());
    Serial.print(F("║ Packets Received:")); Serial.println(_totalPacketsReceived);
    Serial.print(F("║ Packets Relayed: ")); Serial.println(_packetsRelayed);
    Serial.print(F("║ Packets Logged:  ")); Serial.println(_packetsLogged);
    
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    Serial.println(F("║ DETECTED CUBESATS:                        ║"));
    
    uint32_t now = millis();
    bool foundAny = false;
    
    for (uint8_t i = 0; i < MESH_MAX_CUBESATS; i++) {
        if (_cubesats[i].active) {
            uint32_t age = (now - _cubesats[i].lastSeen) / 1000;
            
            Serial.print(F("║   0x"));
            Serial.print(_cubesats[i].serialNumber, HEX);
            Serial.print(F(" - "));
            Serial.print(_cubesats[i].packetsReceived);
            Serial.print(F(" pkts, "));
            Serial.print(age);
            Serial.println(F("s ago"));
            
            foundAny = true;
        }
    }
    
    if (!foundAny) {
        Serial.println(F("║   (none detected)                         ║"));
    }
    
    Serial.println(F("╚═══════════════════════════════════════════╝"));
    Serial.println();
}

void MeshNetwork::printCubesatData(uint32_t serial) {
    RemoteCubeSat* sat = getCubesat(serial);
    
    Serial.println();
    Serial.println(F("╔═══════════════════════════════════════════╗"));
    Serial.print(F("║        CUBESAT 0x"));
    Serial.print(serial, HEX);
    Serial.println(F("                  ║"));
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    
    if (sat == nullptr) {
        Serial.println(F("║ CubeSat not found                         ║"));
        Serial.println(F("╚═══════════════════════════════════════════╝"));
        return;
    }
    
    uint32_t now = millis();
    uint32_t age = (now - sat->lastSeen) / 1000;
    
    Serial.print(F("║ Last Seen:       ")); Serial.print(age);
    Serial.println(F(" seconds ago"));
    Serial.print(F("║ Packets:         ")); Serial.println(sat->packetsReceived);
    Serial.print(F("║ Status:          "));
    Serial.println(isCubesatActive(serial) ? "ACTIVE" : "EXPIRED");
    
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    Serial.println(F("║ DATA POINTS:                              ║"));
    
    bool foundData = false;
    for (uint8_t i = 0; i < MESH_MAX_DATAPOINTS; i++) {
        if (sat->dataPoints[i].valid) {
            uint32_t dpAge = (now - sat->dataPoints[i].timestamp) / 1000;
            bool valid = isDataPointValid(&sat->dataPoints[i]);
            
            Serial.print(F("║   Type 0x"));
            Serial.print(sat->dataPoints[i].type, HEX);
            Serial.print(F(": "));
            Serial.print(sat->dataPoints[i].dataLength);
            Serial.print(F(" bytes, "));
            Serial.print(dpAge);
            Serial.print(F("s ago "));
            Serial.println(valid ? "" : "(expired)");
            
            foundData = true;
        }
    }
    
    if (!foundData) {
        Serial.println(F("║   (no data stored)                        ║"));
    }
    
    Serial.println(F("╚═══════════════════════════════════════════╝"));
    Serial.println();
}
