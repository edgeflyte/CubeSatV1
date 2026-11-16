/*
 * ============================================================================
 * SatbusParser.cpp - SATBUS v1 Protocol Parser Implementation
 * ============================================================================
 * 
 * Implementation of the SATBUS packet parser and module handlers.
 * 
 * SATBUS PACKET STRUCTURE:
 *   Data Packet:    $MODULE,ID,FIELD1,FIELD2,...,*
 *   Command Packet: &MODULE,CMD,VALUE
 * 
 * EDUCATIONAL NOTES:
 * -----------------
 * This file demonstrates several important embedded programming concepts:
 * 
 * 1. STATE MACHINE PARSING
 *    The parser uses a state machine to process incoming bytes one at a time.
 *    This is essential for handling asynchronous serial data where we can't
 *    predict when complete packets will arrive.
 * 
 * 2. BUFFER MANAGEMENT
 *    Fixed-size buffers are used instead of dynamic allocation. In embedded
 *    systems, we avoid malloc/new to prevent memory fragmentation and ensure
 *    predictable memory usage.
 * 
 * 3. DATA CONVERSION
 *    The protocol transmits data as scaled integers to preserve precision
 *    without using floating-point in the packet. We convert back to floats
 *    for easier use in the application.
 * 
 * ============================================================================
 */

#include "SatbusParser.h"

// ============================================================================
// INITIALIZATION
// ============================================================================

/**
 * Initialize the SATBUS parser
 * 
 * Sets up the serial port reference and resets the parser state machine.
 */
void SatbusParser::begin(HardwareSerial* serial) {
    _serial = serial;
    _state = WAITING_START;
    _bufferIndex = 0;
    _isCommand = false;
    _customModuleCount = 0;
    memset(_buffer, 0, sizeof(_buffer));
    memset(_lastError, 0, sizeof(_lastError));
    memset(_customModules, 0, sizeof(_customModules));
}

// ============================================================================
// MAIN PARSING FUNCTIONS
// ============================================================================

/**
 * Parse a single character from the SATBUS stream
 * 
 * This function implements a simple state machine:
 * 
 *   WAITING_START -> ($ or &) -> READING_MODULE -> (,) -> READING_DATA -> (*) -> PACKET_COMPLETE
 *                                                   ^                      |
 *                                                   +----------------------+
 * 
 * The state machine allows us to process data byte-by-byte as it arrives,
 * which is necessary because serial data arrives asynchronously.
 */
SatbusPacket SatbusParser::parseChar(char c) {
    SatbusPacket result;  // Default: invalid packet
    
    switch (_state) {
        case WAITING_START:
            // Look for start character
            if (c == SATBUS_START_DATA) {
                _isCommand = false;
                _buffer[0] = c;
                _bufferIndex = 1;
                _state = READING_MODULE;
            } else if (c == SATBUS_START_CMD) {
                _isCommand = true;
                _buffer[0] = c;
                _bufferIndex = 1;
                _state = READING_MODULE;
            }
            // Ignore all other characters while waiting
            break;
            
        case READING_MODULE:
        case READING_DATA:
            // Check for buffer overflow
            if (_bufferIndex >= SATBUS_MAX_PACKET_LEN - 1) {
                strcpy(_lastError, "Packet too long");
                resetParser();
                break;
            }
            
            // Store character
            _buffer[_bufferIndex++] = c;
            
            // Check for end of packet
            if (c == SATBUS_END_CHAR || c == '\n' || c == '\r') {
                _buffer[_bufferIndex] = '\0';  // Null terminate
                _state = PACKET_COMPLETE;
                
                // Parse the complete packet
                result = parsePacket(_buffer);
            } else if (_state == READING_MODULE && c == SATBUS_FIELD_DELIM) {
                // First comma means we're now reading data fields
                _state = READING_DATA;
            }
            break;
            
        case PACKET_COMPLETE:
            // This state is transient - reset and check for new packet start
            resetParser();
            if (c == SATBUS_START_DATA || c == SATBUS_START_CMD) {
                // New packet starting immediately - recurse
                return parseChar(c);
            }
            break;
    }
    
    return result;
}

/**
 * Parse a complete packet string
 * 
 * Takes a full packet string and extracts:
 *   - Module type (ASB, BMS, SCC, etc.)
 *   - Packet ID
 *   - Data fields
 */
SatbusPacket SatbusParser::parsePacket(const char* packetStr) {
    SatbusPacket packet;
    packet.timestamp = millis();
    
    // Validate basic structure
    if (!validatePacket(packetStr)) {
        strcpy(_lastError, "Invalid packet structure");
        return packet;
    }
    
    // Copy raw data
    strncpy(packet.rawData, packetStr, SATBUS_MAX_PACKET_LEN - 1);
    packet.rawData[SATBUS_MAX_PACKET_LEN - 1] = '\0';
    
    // Determine if command or data
    packet.isCommand = (packetStr[0] == SATBUS_START_CMD);
    
    // Extract module prefix (3 characters after start)
    // Ensure we only copy alphanumeric characters (filter out potential garbage)
    char modulePrefix[4] = {0};
    int prefixLen = 0;
    for (int i = 1; i < 4 && packetStr[i] != '\0' && packetStr[i] != SATBUS_FIELD_DELIM; i++) {
        char c = packetStr[i];
        // Only accept letters for module prefix
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) {
            modulePrefix[prefixLen++] = c;
        }
    }
    modulePrefix[prefixLen] = '\0';
    
    packet.moduleType = identifyModule(modulePrefix);
    
    // Find first comma (end of module prefix)
    const char* dataStart = strchr(packetStr, SATBUS_FIELD_DELIM);
    if (!dataStart) {
        strcpy(_lastError, "No data fields found");
        return packet;
    }
    dataStart++;  // Skip the comma
    
    // Split remaining data into fields
    // Create a working copy since we need to strip the trailing *
    char workBuffer[SATBUS_MAX_PACKET_LEN];
    strncpy(workBuffer, dataStart, sizeof(workBuffer) - 1);
    workBuffer[sizeof(workBuffer) - 1] = '\0';
    
    // Remove trailing * and anything after
    char* endMarker = strchr(workBuffer, SATBUS_END_CHAR);
    if (endMarker) *endMarker = '\0';
    
    // Split fields by comma
    packet.fieldCount = splitFields(workBuffer, packet.fields, SATBUS_MAX_FIELDS);
    
    // Extract packet ID from first field (for data packets)
    if (!packet.isCommand && packet.fieldCount > 0) {
        packet.packetId = atoi(packet.fields[0]);
        
        // Shift fields down (remove packet ID from fields array)
        for (int i = 0; i < packet.fieldCount - 1; i++) {
            strcpy(packet.fields[i], packet.fields[i + 1]);
        }
        packet.fieldCount--;
    }
    
    // Check for custom module handler
    for (uint8_t i = 0; i < _customModuleCount; i++) {
        if (strcmp(modulePrefix, _customModules[i].prefix) == 0) {
            if (_customModules[i].callback) {
                _customModules[i].callback(packet);
            }
            break;
        }
    }
    
    packet.isValid = true;
    return packet;
}

// ============================================================================
// ASB (Atmospheric Sensing Board) PARSERS
// ============================================================================

/**
 * Parse ASB Environmental Sensor Packet ($ASB,3,...)
 * 
 * Packet Format: $ASB,3,T1,T2,RH,P1,PA1,AQI,TVOC,ECO2,CO2,*
 * 
 * ENCODING:
 *   T1, T2: Temperature = (value / 100) - 40 °C
 *   RH: Humidity = value / 10 %
 *   P1: Pressure = value * 10 Pa
 *   PA1: Altitude = (value) - 100 m  (ASB adds a +100 m bias on the wire)
 *   AQI: Air Quality Index 1-5 (no scaling)
 *   TVOC: ppb (no scaling)
 *   ECO2: ppm (no scaling)
 *   CO2: ppm (no scaling)
 */
void SatbusParser::parseASBEnvironmental(SatbusPacket& packet, ASBData& data) {
    if (packet.fieldCount < 9) {
        strcpy(_lastError, "ASB env packet: insufficient fields");
        return;
    }
    
    // Temperature 1 (SHT4x): ((°C + 40) × 100) -> °C
    int rawT1 = atoi(packet.fields[0]);                      // e.g. 6234 (centi-degrees, biased)
    data.temperature1 = (rawT1 / 100.0f) - 40.0f;
    
    // Temperature 2 (BMP180): ((°C + 40) × 100) -> °C
    int rawT2 = atoi(packet.fields[1]);
    data.temperature2 = (rawT2 / 100.0f) - 40.0f;
    
    // Relative Humidity: (%RH × 10) -> %
    int rawRH = atoi(packet.fields[2]);
    data.humidity = rawRH / 10.0f;
    
    // Pressure: (Pa / 10) -> Pa
    int rawP = atoi(packet.fields[3]);
    data.pressure = rawP * 10.0f;
    
    // Altitude: ASB encodes (int)(altitude_m + 100) to keep small negative
    // values positive on the wire; reverse the +100 m bias here.
    // See docs/protocols/satbus-v1.md.
    data.altitude = atof(packet.fields[4]) - 100.0f;
    
    // Air Quality Index: 1-5 scale
    data.airQualityIndex = atoi(packet.fields[5]);
    
    // TVOC: ppb
    data.tvoc = atoi(packet.fields[6]);
    
    // eCO2: ppm
    data.eco2 = atoi(packet.fields[7]);
    
    // CO2: ppm
    data.co2 = atoi(packet.fields[8]);
    
    data.hasEnvironmental = true;
    data.lastUpdate = millis();
}

static float nonNegative(float value) {
    return (value < 0.0f) ? 0.0f : value;
}

static float clampSolarVolts(float volts) {
    if (volts < 0.0f) return 0.0f;
    if (volts > 3.3f) return 3.3f;
    return volts;
}

/**
 * Parse ASB Particulate Matter Packet ($ASB,1,... or legacy $ASB,4,...)
 * 
 * Packet Format: $ASB,1,MC1,MC25,MC4,MC10,NC05,NC1,NC25,NC4,NC10,TPS,*
 * 
 * ENCODING (matches ASB.ino txPM2Pkt):
 *   MCx: Mass Concentration = value × 10  -> decode / 10 μg/m³
 *   NCx: Number Concentration = value × 10 -> decode / 10 /cm³
 *   TPS: Typical Particle Size = value × 100 -> decode / 100 μm
 */
void SatbusParser::parseASBParticulate(SatbusPacket& packet, ASBData& data) {
    if (packet.fieldCount < 10) {
        strcpy(_lastError, "ASB PM packet: insufficient fields");
        return;
    }
    
    // Mass concentrations: (μg/m³ × 10) -> μg/m³. Negative wire values are
    // SPS30 startup noise and are stored as zero.
    data.pm1_0 = nonNegative(atoi(packet.fields[0]) / 10.0f);
    data.pm2_5 = nonNegative(atoi(packet.fields[1]) / 10.0f);
    data.pm4_0 = nonNegative(atoi(packet.fields[2]) / 10.0f);
    data.pm10 = nonNegative(atoi(packet.fields[3]) / 10.0f);
    
    // Number concentrations: (/cm³ × 10) -> /cm³
    data.nc0_5 = nonNegative(atoi(packet.fields[4]) / 10.0f);
    data.nc1_0 = nonNegative(atoi(packet.fields[5]) / 10.0f);
    data.nc2_5 = nonNegative(atoi(packet.fields[6]) / 10.0f);
    data.nc4_0 = nonNegative(atoi(packet.fields[7]) / 10.0f);
    data.nc10 = nonNegative(atoi(packet.fields[8]) / 10.0f);
    
    // Typical particle size: (μm × 100) -> μm
    data.typicalParticleSize = nonNegative(atoi(packet.fields[9]) / 100.0f);
    
    data.hasParticulate = true;
    data.lastUpdate = millis();
}

// ============================================================================
// BMS (Battery Management System) PARSERS
// ============================================================================

/**
 * Parse BMS Status Packet ($BMS,ERR,LDO1,LDO2,LDO3,LDO4,BATV,BATPCT,SOL1,SOL2*)
 * 
 * Packet Format: $BMS,ERR,LDO1,LDO2,LDO3,LDO4,BATV,BATPCT,SOL1,SOL2*
 * 
 * Note: BMS packets don't have a packet ID like ASB/SCC, so we handle
 * the slightly different format here.
 */
void SatbusParser::parseBMSStatus(SatbusPacket& packet, BMSData& data) {
    // BMS packet has module prefix but no numeric packet ID
    // First "field" after module is actually ERR
    
    // Re-parse from raw data since BMS format differs
    const char* rawData = packet.rawData;
    
    // Find first comma to skip module prefix (more robust than fixed offset)
    const char* dataStart = strchr(rawData, SATBUS_FIELD_DELIM);
    if (!dataStart) {
        strcpy(_lastError, "BMS packet: no data fields");
        return;
    }
    dataStart++;  // Skip the comma
    
    char fields[10][16];
    char workBuffer[SATBUS_MAX_PACKET_LEN];
    strncpy(workBuffer, dataStart, sizeof(workBuffer) - 1);
    workBuffer[sizeof(workBuffer) - 1] = '\0';
    
    // Remove trailing * and anything after
    char* endMarker = strchr(workBuffer, '*');
    if (endMarker) *endMarker = '\0';
    
    // Also remove newlines/carriage returns
    char* nlMarker = strchr(workBuffer, '\r');
    if (nlMarker) *nlMarker = '\0';
    nlMarker = strchr(workBuffer, '\n');
    if (nlMarker) *nlMarker = '\0';
    
    int fieldCount = splitFields(workBuffer, fields, 10);
    
    if (fieldCount < 9) {
        strcpy(_lastError, "BMS packet: insufficient fields");
        return;
    }
    
    // Parse fields: ERR,LDO1,LDO2,LDO3,LDO4,BATV,BATPCT,SOL1,SOL2
    data.errorCode = atoi(fields[0]);
    data.ldo1State = (atoi(fields[1]) == 1);
    data.ldo2State = (atoi(fields[2]) == 1);
    data.ldo3State = (atoi(fields[3]) == 1);
    data.ldo4State = (atoi(fields[4]) == 1);
    data.batteryVoltage = atof(fields[5]);
    data.batteryPercent = atof(fields[6]);
    // Solar sense is a direct ADC pin on the ATmega328PB. Full scale is the
    // 3.3 V reference; an open panel floats to that rail and no higher.
    data.solarVoltage1 = clampSolarVolts(atof(fields[7]));
    data.solarVoltage2 = clampSolarVolts(atof(fields[8]));
    
    data.lastUpdate = millis();
}

/**
 * Send command to BMS
 * 
 * Commands:
 *   Channel 1-4: &BMS,CHx,0/1 (OFF/ON)
 *   System reset: channel=0, value=1 sends &BMS,RST,1
 */
void SatbusParser::sendBMSCommand(uint8_t channel, uint8_t value) {
    if (_serial == nullptr) return;
    
    if (channel == 0) {
        // System command
        if (value == 1) {
            _serial->println("&BMS,RST,1");
        }
    } else if (channel >= 1 && channel <= 4) {
        // Channel control
        _serial->print("&BMS,CH");
        _serial->print(channel);
        _serial->print(",");
        _serial->println(value);
    }
}

void SatbusParser::sendBMSStore(uint8_t mask) {
    if (_serial == nullptr) return;
    _serial->print(F("&BMS,CFG,"));
    _serial->println(mask & 0x0F);
}

// ============================================================================
// SCC (Solar Charge Controller) PARSERS
// ============================================================================

/**
 * Parse SCC Solar Voltages Packet ($SCC,1,V0,V1,V2,V3,V4,V5,*)
 */
void SatbusParser::parseSCCVoltages(SatbusPacket& packet, SCCData& data) {
    if (packet.fieldCount < 6) {
        strcpy(_lastError, "SCC voltage packet: insufficient fields");
        return;
    }
    
    for (int i = 0; i < 6 && i < packet.fieldCount; i++) {
        data.solarVoltage[i] = atof(packet.fields[i]);
    }
    
    data.hasVoltages = true;
    data.lastUpdate = millis();
}

/**
 * Parse SCC Temperature Packet ($SCC,2,T0,T1,T2,T3,T4,T5,T6,T7*)
 */
void SatbusParser::parseSCCTemperatures(SatbusPacket& packet, SCCData& data) {
    if (packet.fieldCount < 8) {
        strcpy(_lastError, "SCC temperature packet: insufficient fields");
        return;
    }
    
    for (int i = 0; i < 8 && i < packet.fieldCount; i++) {
        data.temperature[i] = atof(packet.fields[i]);
    }
    
    data.hasTemperatures = true;
    data.lastUpdate = millis();
}

/**
 * Parse SCC Output Monitoring Packet ($SCC,3,VOUT,IOUT,MODE*)
 */
void SatbusParser::parseSCCOutput(SatbusPacket& packet, SCCData& data) {
    if (packet.fieldCount < 3) {
        strcpy(_lastError, "SCC output packet: insufficient fields");
        return;
    }
    
    data.outputVoltage = atof(packet.fields[0]);
    data.outputCurrent = atof(packet.fields[1]);
    data.systemMode = atoi(packet.fields[2]);
    
    data.hasOutput = true;
    data.lastUpdate = millis();
}

/**
 * Send text command to SCC
 */
void SatbusParser::sendSCCCommand(const char* command) {
    if (_serial == nullptr) return;
    
    _serial->println(command);
}

// ============================================================================
// CUSTOM MODULE SUPPORT
// ============================================================================

/**
 * Register a custom module callback
 * 
 * This allows extending the SATBUS parser for user-defined modules
 * without modifying the core library code.
 */
void SatbusParser::registerCustomModule(const char* modulePrefix, CustomModuleCallback callback) {
    if (_customModuleCount >= MAX_CUSTOM_MODULES) {
        strcpy(_lastError, "Max custom modules reached");
        return;
    }
    
    strncpy(_customModules[_customModuleCount].prefix, modulePrefix, 3);
    _customModules[_customModuleCount].prefix[3] = '\0';
    _customModules[_customModuleCount].callback = callback;
    _customModuleCount++;
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/**
 * Identify module type from 3-character prefix
 * Uses case-insensitive comparison for robustness
 */
SatbusModuleType SatbusParser::identifyModule(const char* prefix) {
    // Use case-insensitive comparison for robustness
    if (strcasecmp(prefix, "ASB") == 0) return SATBUS_MODULE_ASB;
    if (strcasecmp(prefix, "BMS") == 0) return SATBUS_MODULE_BMS;
    if (strcasecmp(prefix, "SCC") == 0) return SATBUS_MODULE_SCC;
    
    // Check custom modules (case-insensitive)
    for (uint8_t i = 0; i < _customModuleCount; i++) {
        if (strcasecmp(prefix, _customModules[i].prefix) == 0) {
            return SATBUS_MODULE_CUSTOM;
        }
    }
    
    return SATBUS_MODULE_UNKNOWN;
}

/**
 * Reset parser state machine
 */
void SatbusParser::resetParser() {
    _state = WAITING_START;
    _bufferIndex = 0;
    memset(_buffer, 0, sizeof(_buffer));
}

/**
 * Validate basic packet structure
 */
bool SatbusParser::validatePacket(const char* packet) {
    // Check minimum length ($XXX, = at least 5 chars)
    if (strlen(packet) < 5) return false;
    
    // Check start character
    if (packet[0] != SATBUS_START_DATA && packet[0] != SATBUS_START_CMD) {
        return false;
    }
    
    // Check that position 1-3 contains letters (module prefix)
    for (int i = 1; i <= 3; i++) {
        char c = packet[i];
        if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))) {
            // Not a letter - might be garbage or wrong format
            return false;
        }
    }
    
    // Check that position 4 is a comma (module prefix delimiter)
    if (packet[4] != SATBUS_FIELD_DELIM) {
        return false;
    }
    
    return true;
}

/**
 * Split a comma-delimited string into fields
 * 
 * @param data The string to split
 * @param fields Output array for fields
 * @param maxFields Maximum number of fields to extract
 * @return Number of fields extracted
 */
int SatbusParser::splitFields(const char* data, char fields[][16], int maxFields) {
    int fieldIndex = 0;
    int charIndex = 0;
    
    memset(fields, 0, maxFields * 16);
    
    while (*data && fieldIndex < maxFields) {
        if (*data == SATBUS_FIELD_DELIM) {
            // End of current field
            fields[fieldIndex][charIndex] = '\0';
            fieldIndex++;
            charIndex = 0;
        } else if (charIndex < 15) {
            // Add character to current field
            fields[fieldIndex][charIndex++] = *data;
        }
        data++;
    }
    
    // Don't forget the last field
    if (charIndex > 0 && fieldIndex < maxFields) {
        fields[fieldIndex][charIndex] = '\0';
        fieldIndex++;
    }
    
    return fieldIndex;
}

// ============================================================================
// END OF FILE
// ============================================================================

/*
 * ADDING CUSTOM MODULES - STEP BY STEP GUIDE
 * ==========================================
 * 
 * Let's say you want to add a "Payload Module" (PLM) that sends:
 *   $PLM,1,SENSOR1,SENSOR2,STATUS*
 * 
 * Step 1: Define your data structure (in your main code or a header):
 * 
 *   struct PayloadData {
 *       float sensor1;
 *       float sensor2;
 *       uint8_t status;
 *   };
 *   PayloadData payloadData;
 * 
 * Step 2: Create a callback function:
 * 
 *   void handlePayloadPacket(SatbusPacket& pkt) {
 *       if (pkt.packetId == 1 && pkt.fieldCount >= 3) {
 *           payloadData.sensor1 = atof(pkt.fields[0]);
 *           payloadData.sensor2 = atof(pkt.fields[1]);
 *           payloadData.status = atoi(pkt.fields[2]);
 *       }
 *   }
 * 
 * Step 3: Register the module in setup():
 * 
 *   satbus.registerCustomModule("PLM", handlePayloadPacket);
 * 
 * That's it! The parser will now automatically call your function
 * whenever a $PLM packet is received.
 */
