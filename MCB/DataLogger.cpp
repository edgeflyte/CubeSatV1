/*
 * ============================================================================
 * DataLogger.cpp - SD Card Data Logging Implementation
 * ============================================================================
 */

#include "DataLogger.h"
#include "version.h"

// Replace CSV-breaking characters so a single field cannot corrupt a row.
static void sanitizeCsvField(const char* in, char* out, size_t outSize) {
    if (outSize == 0) return;
    size_t j = 0;
    for (size_t i = 0; in != nullptr && in[i] != '\0' && j < outSize - 1; i++) {
        char c = in[i];
        if (c == ',' || c == '\n' || c == '\r') c = ' ';
        out[j++] = c;
    }
    out[j] = '\0';
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool DataLogger::begin(uint8_t csPin, uint32_t bootCount) {
    _csPin = csPin;
    _sdReady = false;
    _fileOpen = false;
    _lastSensorLog = 0;
    _lastGPSLog = 0;
    _lastFlush = 0;
    
    // Reset stats
    _stats = LogStats();
    _consecutiveWriteErrors = 0;
    
    // Initialize SD card
    if (!SD.begin(_csPin)) {
        Serial.println(F("[LOG] SD card initialization failed"));
        return false;
    }
    
    _sdReady = true;
#if LOG_VERBOSE
    Serial.println(F("[LOG] SD card initialized"));
#endif
    
    // Create log file
    if (!newFile(bootCount)) {
        return false;
    }
    
    return true;
}

void DataLogger::end() {
    if (_fileOpen) {
        _logFile.flush();
        _logFile.close();
        _fileOpen = false;
#if LOG_VERBOSE
        Serial.println(F("[LOG] Log file closed"));
#endif
    }
}

// ============================================================================
// FILE MANAGEMENT
// ============================================================================

void DataLogger::buildFilename(uint32_t bootCount) {
    // Format: log_XXXX.csv where XXXX is zero-padded boot count
    snprintf(_filename, LOG_MAX_FILENAME_LEN, "%s%04lu%s", 
             LOG_FILENAME_PREFIX, bootCount, LOG_FILENAME_EXT);
}

bool DataLogger::newFile(uint32_t bootCount) {
    // Close existing file
    if (_fileOpen) {
        _logFile.flush();
        _logFile.close();
        _fileOpen = false;
    }
    
    // Build new filename
    buildFilename(bootCount);
    
    // Check if file exists (append vs new)
    bool exists = SD.exists(_filename);
    
    // Open file for writing (append mode)
    _logFile = SD.open(_filename, FILE_WRITE);
    
    if (!_logFile) {
        Serial.print(F("[LOG] Failed to open: "));
        Serial.println(_filename);
        return false;
    }
    
    _fileOpen = true;
    _stats.startTime = millis();
    
#if LOG_VERBOSE
    Serial.print(F("[LOG] Opened: "));
    Serial.println(_filename);
#endif
    
    // Write header if new file
    if (!exists) {
        writeHeader();
    }
    
    // Log boot event
    logBoot(bootCount, FIRMWARE_VERSION);
    
    return true;
}

void DataLogger::writeHeader() {
    if (!_fileOpen) return;
    
    _logFile.println(F("timestamp_ms,type,data"));
    _logFile.flush();
}

void DataLogger::writeLine(const char* line) {
    if (!_fileOpen) return;
    
    size_t written = _logFile.println(line);
    
    if (written > 0) {
        _stats.entriesWritten++;
        _stats.bytesWritten += written;
        _stats.fileSize += written;
        _consecutiveWriteErrors = 0;
    } else {
        _stats.writeErrors++;
        // Stop logging after repeated failures (e.g. card removed) so we do
        // not spin on a dead card or silently drop every subsequent write.
        if (++_consecutiveWriteErrors >= LOG_MAX_CONSECUTIVE_ERRORS) {
            _logFile.close();
            _fileOpen = false;
            _sdReady = false;
        }
    }
}

void DataLogger::flush() {
    if (_fileOpen) {
        _logFile.flush();
        _lastFlush = millis();
    }
}

void DataLogger::formatTimestamp(char* buffer, size_t size) {
    snprintf(buffer, size, "%lu", millis());
}

// ============================================================================
// CONFIGURATION
// ============================================================================

void DataLogger::setEnabled(bool enabled) {
    _config.enabled = enabled;
    
    if (enabled && _fileOpen) {
        logEvent("Logging enabled", 0);
    }
}

void DataLogger::setSensorInterval(uint32_t ms) {
    if (ms < LOG_MIN_INTERVAL_MS) ms = LOG_MIN_INTERVAL_MS;
    if (ms > LOG_MAX_INTERVAL_MS) ms = LOG_MAX_INTERVAL_MS;
    _config.sensorInterval = ms;
}

void DataLogger::setGPSInterval(uint32_t ms) {
    if (ms < LOG_MIN_INTERVAL_MS) ms = LOG_MIN_INTERVAL_MS;
    if (ms > LOG_MAX_INTERVAL_MS) ms = LOG_MAX_INTERVAL_MS;
    _config.gpsInterval = ms;
}

// ============================================================================
// PERIODIC UPDATE
// ============================================================================

void DataLogger::update(SensorData& sensors, GPSData& gps) {
    if (!isEnabled()) return;
    
    uint32_t now = millis();
    
    // Log sensor data
    if (_config.logSensors && (now - _lastSensorLog >= _config.sensorInterval)) {
        _lastSensorLog = now;
        logSensorData(sensors);
    }
    
    // Log GPS data
    if (_config.logGPS && gps.hasValidFix && (now - _lastGPSLog >= _config.gpsInterval)) {
        _lastGPSLog = now;
        logGPSData(gps);
    }
    
    // Periodic flush
    if (now - _lastFlush >= LOG_FLUSH_INTERVAL_MS) {
        flush();
    }
}

// ============================================================================
// LOGGING FUNCTIONS
// ============================================================================

void DataLogger::logSensorData(SensorData& sensors) {
    if (!_fileOpen) return;
    
    char buffer[LOG_BUFFER_SIZE];
    char timestamp[16];
    formatTimestamp(timestamp, sizeof(timestamp));
    
    // Format: timestamp,SENSOR,temp,humidity,ax,ay,az,gx,gy,gz,mx,my,mz
    snprintf(buffer, sizeof(buffer),
             "%s,SENSOR,%.2f,%.1f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f",
             timestamp,
             sensors.temperature,
             sensors.humidity,
             sensors.accelX, sensors.accelY, sensors.accelZ,
             sensors.gyroX, sensors.gyroY, sensors.gyroZ,
             sensors.magX, sensors.magY, sensors.magZ);
    
    writeLine(buffer);
}

void DataLogger::logGPSData(GPSData& gps) {
    if (!_fileOpen) return;
    
    char buffer[LOG_BUFFER_SIZE];
    char timestamp[16];
    formatTimestamp(timestamp, sizeof(timestamp));
    
    // Format: timestamp,GPS,lat,lon,alt,sats,hdop,fix,date,time
    snprintf(buffer, sizeof(buffer),
             "%s,GPS,%.6f,%.6f,%.1f,%d,%.1f,%d,%04d-%02d-%02d,%02d:%02d:%02d",
             timestamp,
             gps.latitude, gps.longitude, gps.altitude,
             gps.satellites, gps.hdop, gps.hasValidFix ? 1 : 0,
             gps.year, gps.month, gps.day,
             gps.hour, gps.minute, gps.second);


    writeLine(buffer);
}

void DataLogger::logIMUData(SensorData& sensors) {
    if (!_fileOpen) return;
    
    char buffer[LOG_BUFFER_SIZE];
    char timestamp[16];
    formatTimestamp(timestamp, sizeof(timestamp));
    
    // Format: timestamp,IMU,ax,ay,az,gx,gy,gz,mx,my,mz
    snprintf(buffer, sizeof(buffer),
             "%s,IMU,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f",
             timestamp,
             sensors.accelX, sensors.accelY, sensors.accelZ,
             sensors.gyroX, sensors.gyroY, sensors.gyroZ,
             sensors.magX, sensors.magY, sensors.magZ);
    
    writeLine(buffer);
}

void DataLogger::logEvent(const char* event, uint8_t severity) {
    if (!_fileOpen) return;
    
    char buffer[LOG_BUFFER_SIZE];
    char timestamp[16];
    char safeEvent[64];
    formatTimestamp(timestamp, sizeof(timestamp));
    sanitizeCsvField(event, safeEvent, sizeof(safeEvent));
    
    snprintf(buffer, sizeof(buffer), "%s,EVENT,%d,%s",
             timestamp, severity, safeEvent);
    
    writeLine(buffer);
    flush();  // Flush events immediately
}

void DataLogger::logError(const char* error, uint16_t errorCode) {
    if (!_fileOpen) return;
    
    char buffer[LOG_BUFFER_SIZE];
    char timestamp[16];
    char safeError[64];
    formatTimestamp(timestamp, sizeof(timestamp));
    sanitizeCsvField(error, safeError, sizeof(safeError));
    
    snprintf(buffer, sizeof(buffer), "%s,ERROR,%d,%s",
             timestamp, errorCode, safeError);
    
    writeLine(buffer);
    flush();  // Flush errors immediately
}

void DataLogger::logModeChange(uint8_t oldMode, uint8_t newMode) {
    if (!_fileOpen) return;
    
    char buffer[LOG_BUFFER_SIZE];
    char timestamp[16];
    formatTimestamp(timestamp, sizeof(timestamp));
    
    snprintf(buffer, sizeof(buffer), "%s,MODE,%d,%d",
             timestamp, oldMode, newMode);
    
    writeLine(buffer);
    flush();
}

void DataLogger::logBoot(uint32_t bootCount, const char* version) {
    if (!_fileOpen) return;
    
    char buffer[LOG_BUFFER_SIZE];
    char timestamp[16];
    formatTimestamp(timestamp, sizeof(timestamp));
    
    snprintf(buffer, sizeof(buffer), "%s,BOOT,%lu,%s",
             timestamp, bootCount, version);
    
    writeLine(buffer);
    flush();
}

void DataLogger::logASBData(ASBData& data) {
    if (!_fileOpen || !_config.logSatbus) return;
    
    char buffer[LOG_BUFFER_SIZE];
    char timestamp[16];
    formatTimestamp(timestamp, sizeof(timestamp));
    
    // Log environmental data
    snprintf(buffer, sizeof(buffer),
             "%s,ASB_ENV,%.2f,%.2f,%.1f,%.0f,%.1f,%d,%d,%d,%d",
             timestamp,
             data.temperature1, data.temperature2, data.humidity,
             data.pressure, data.altitude,
             data.airQualityIndex, data.tvoc, data.eco2, data.co2);
    
    writeLine(buffer);
    
    // Log particulate data if available
    if (data.hasParticulate) {
        snprintf(buffer, sizeof(buffer),
                 "%s,ASB_PM,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f",
                 timestamp,
                 data.pm1_0, data.pm2_5, data.pm4_0, data.pm10,
                 data.nc0_5, data.nc1_0, data.nc2_5, data.nc4_0, data.nc10,
                 data.typicalParticleSize);
        
        writeLine(buffer);
    }
}

void DataLogger::logBMSData(BMSData& data) {
    if (!_fileOpen || !_config.logSatbus) return;
    
    char buffer[LOG_BUFFER_SIZE];
    char timestamp[16];
    formatTimestamp(timestamp, sizeof(timestamp));
    
    snprintf(buffer, sizeof(buffer),
             "%s,BMS,%d,%d%d%d%d,%.2f,%.1f,%.2f,%.2f",
             timestamp,
             data.errorCode,
             data.ldo1State ? 1 : 0, data.ldo2State ? 1 : 0,
             data.ldo3State ? 1 : 0, data.ldo4State ? 1 : 0,
             data.batteryVoltage, data.batteryPercent,
             data.solarVoltage1, data.solarVoltage2);
    
    writeLine(buffer);
}

void DataLogger::logSCCData(SCCData& data) {
    if (!_fileOpen || !_config.logSatbus) return;
    
    char buffer[LOG_BUFFER_SIZE];
    char timestamp[16];
    formatTimestamp(timestamp, sizeof(timestamp));
    
    // Log solar voltages
    snprintf(buffer, sizeof(buffer),
             "%s,SCC_V,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
             timestamp,
             data.solarVoltage[0], data.solarVoltage[1], data.solarVoltage[2],
             data.solarVoltage[3], data.solarVoltage[4], data.solarVoltage[5]);
    
    writeLine(buffer);
    
    // Log temperatures
    snprintf(buffer, sizeof(buffer),
             "%s,SCC_T,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
             timestamp,
             data.temperature[0], data.temperature[1], data.temperature[2],
             data.temperature[3], data.temperature[4], data.temperature[5],
             data.temperature[6], data.temperature[7]);
    
    writeLine(buffer);
    
    // Log output
    snprintf(buffer, sizeof(buffer),
             "%s,SCC_OUT,%.2f,%.0f,%d",
             timestamp,
             data.outputVoltage, data.outputCurrent, data.systemMode);
    
    writeLine(buffer);
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================

void DataLogger::printStatus() {
    Serial.println();
    Serial.println(F("╔═══════════════════════════════════════════╗"));
    Serial.println(F("║            DATA LOGGER STATUS             ║"));
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    
    Serial.print(F("║ SD Card:       "));
    Serial.println(_sdReady ? "READY" : "NOT READY");
    Serial.print(F("║ File Open:     "));
    Serial.println(_fileOpen ? "YES" : "NO");
    Serial.print(F("║ Logging:       "));
    Serial.println(_config.enabled ? "ENABLED" : "DISABLED");
    
    if (_fileOpen) {
        Serial.println(F("╠═══════════════════════════════════════════╣"));
        Serial.print(F("║ Filename:      ")); Serial.println(_filename);
        Serial.print(F("║ Entries:       ")); Serial.println(_stats.entriesWritten);
        Serial.print(F("║ Bytes Written: ")); Serial.println(_stats.bytesWritten);
        Serial.print(F("║ Write Errors:  ")); Serial.println(_stats.writeErrors);
        
        uint32_t runtime = (millis() - _stats.startTime) / 1000;
        Serial.print(F("║ Runtime:       ")); Serial.print(runtime);
        Serial.println(F(" seconds"));
    }
    
    Serial.println(F("╠═══════════════════════════════════════════╣"));
    Serial.println(F("║ CONFIGURATION:                            ║"));
    Serial.print(F("║   Sensor Log:    "));
    Serial.println(_config.logSensors ? "ON" : "OFF");
    Serial.print(F("║   GPS Log:       "));
    Serial.println(_config.logGPS ? "ON" : "OFF");
    Serial.print(F("║   IMU Log:       "));
    Serial.println(_config.logIMU ? "ON" : "OFF");
    Serial.print(F("║   Events Log:    "));
    Serial.println(_config.logEvents ? "ON" : "OFF");
    Serial.print(F("║   SATBUS Log:    "));
    Serial.println(_config.logSatbus ? "ON" : "OFF");
    Serial.print(F("║   Sensor Interval: ")); Serial.print(_config.sensorInterval);
    Serial.println(F(" ms"));
    Serial.print(F("║   GPS Interval:    ")); Serial.print(_config.gpsInterval);
    Serial.println(F(" ms"));
    
    Serial.println(F("╚═══════════════════════════════════════════╝"));
    Serial.println();
}
