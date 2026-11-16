/*
 * ============================================================================
 * DataLogger.h - SD Card Data Logging System
 * ============================================================================
 * 
 * Provides continuous logging of CubeSat sensor data to SD card.
 * Log files are named based on boot count for easy identification.
 * 
 * FEATURES:
 *   - Configurable logging interval
 *   - Boot count based file naming (log_0001.csv, log_0002.csv, etc.)
 *   - CSV format for easy analysis
 *   - Automatic file rotation
 *   - Event logging support
 * 
 * ============================================================================
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "SatbusParser.h"

// ============================================================================
// LOGGING CONFIGURATION
// ============================================================================

#define LOG_DEFAULT_INTERVAL_MS     5000    // Default: log every 5 seconds
#define LOG_MIN_INTERVAL_MS         100     // Minimum: 100ms
#define LOG_MAX_INTERVAL_MS         60000   // Maximum: 1 minute
#define LOG_FILENAME_PREFIX         "log_"
#define LOG_FILENAME_EXT            ".csv"
#define LOG_MAX_FILENAME_LEN        32
#define LOG_BUFFER_SIZE             256     // Write buffer size
#define LOG_FLUSH_INTERVAL_MS       30000   // Flush to SD every 30 seconds
#define LOG_MAX_CONSECUTIVE_ERRORS  10      // Disable logging after this many consecutive write failures

// Set to 1 to emit verbose informational logging messages on Serial.
#ifndef LOG_VERBOSE
#define LOG_VERBOSE                 0
#endif

// ============================================================================
// LOG ENTRY TYPES
// ============================================================================

enum LogEntryType : uint8_t {
    LOG_TYPE_SENSOR,        // Periodic sensor data
    LOG_TYPE_GPS,           // GPS position update
    LOG_TYPE_EVENT,         // System event
    LOG_TYPE_ERROR,         // Error condition
    LOG_TYPE_MODE_CHANGE,   // Operating mode change
    LOG_TYPE_BOOT,          // Boot event
    LOG_TYPE_SATBUS,        // SATBUS module data
    LOG_TYPE_MESH           // Mesh network event
};

// ============================================================================
// LOGGING CONFIGURATION STRUCTURE
// ============================================================================

struct LogConfig {
    bool enabled;               // Logging enabled
    bool logSensors;            // Log local sensor data
    bool logGPS;                // Log GPS data
    bool logIMU;                // Log IMU data
    bool logEvents;             // Log system events
    bool logSatbus;             // Log SATBUS module data
    uint32_t sensorInterval;    // Sensor logging interval (ms)
    uint32_t gpsInterval;       // GPS logging interval (ms)
    
    LogConfig() : enabled(false), logSensors(true), logGPS(true),
                  logIMU(true), logEvents(true), logSatbus(true),
                  sensorInterval(LOG_DEFAULT_INTERVAL_MS),
                  gpsInterval(LOG_DEFAULT_INTERVAL_MS) {}
};

// ============================================================================
// LOGGING STATISTICS
// ============================================================================

struct LogStats {
    uint32_t entriesWritten;    // Total log entries
    uint32_t bytesWritten;      // Total bytes written
    uint32_t writeErrors;       // Write error count
    uint32_t fileSize;          // Current file size
    uint32_t startTime;         // Logging start time
    
    LogStats() : entriesWritten(0), bytesWritten(0), writeErrors(0),
                 fileSize(0), startTime(0) {}
};

// ============================================================================
// DATA LOGGER CLASS
// ============================================================================

class DataLogger {
public:
    /**
     * Initialize the data logger
     * @param csPin SD card chip select pin
     * @param bootCount Current boot count for file naming
     * @return true if SD card initialized successfully
     */
    bool begin(uint8_t csPin, uint32_t bootCount);
    
    /**
     * Close the log file (call before shutdown)
     */
    void end();
    
    /**
     * Check if logger is ready
     */
    bool isReady() { return _sdReady && _fileOpen; }
    
    /**
     * Check if logging is enabled
     */
    bool isEnabled() { return _config.enabled && isReady(); }
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    void setEnabled(bool enabled);
    void setSensorInterval(uint32_t ms);
    void setGPSInterval(uint32_t ms);
    
    void setLogSensors(bool enabled) { _config.logSensors = enabled; }
    void setLogGPS(bool enabled) { _config.logGPS = enabled; }
    void setLogIMU(bool enabled) { _config.logIMU = enabled; }
    void setLogEvents(bool enabled) { _config.logEvents = enabled; }
    void setLogSatbus(bool enabled) { _config.logSatbus = enabled; }
    
    LogConfig& getConfig() { return _config; }
    LogStats& getStats() { return _stats; }
    
    // ========================================================================
    // Periodic Update (call from main loop)
    // ========================================================================
    
    /**
     * Update the logger - handles periodic logging and file flushing
     * @param sensors Current sensor data
     * @param gps Current GPS data
     */
    void update(SensorData& sensors, GPSData& gps);
    
    // ========================================================================
    // Manual Logging Functions
    // ========================================================================
    
    /**
     * Log sensor data (temperature, humidity, IMU)
     */
    void logSensorData(SensorData& sensors);
    
    /**
     * Log GPS position
     */
    void logGPSData(GPSData& gps);
    
    /**
     * Log IMU data separately (higher rate if needed)
     */
    void logIMUData(SensorData& sensors);
    
    /**
     * Log a system event
     */
    void logEvent(const char* event, uint8_t severity = 0);
    
    /**
     * Log an error
     */
    void logError(const char* error, uint16_t errorCode = 0);
    
    /**
     * Log mode change
     */
    void logModeChange(uint8_t oldMode, uint8_t newMode);
    
    /**
     * Log boot event
     */
    void logBoot(uint32_t bootCount, const char* version);
    
    /**
     * Log SATBUS module data
     */
    void logASBData(ASBData& data);
    void logBMSData(BMSData& data);
    void logSCCData(SCCData& data);
    
    // ========================================================================
    // File Management
    // ========================================================================
    
    /**
     * Get current log filename
     */
    const char* getFilename() { return _filename; }
    
    /**
     * Force flush buffer to SD card
     */
    void flush();
    
    /**
     * Start a new log file
     */
    bool newFile(uint32_t bootCount);
    
    // ========================================================================
    // Debug Output
    // ========================================================================
    
    /**
     * Print logger status to Serial
     */
    void printStatus();
    
private:
    // SD Card
    uint8_t _csPin;
    bool _sdReady;
    bool _fileOpen;
    File _logFile;
    char _filename[LOG_MAX_FILENAME_LEN];
    
    // Configuration
    LogConfig _config;
    LogStats _stats;
    
    // Timing
    uint32_t _lastSensorLog;
    uint32_t _lastGPSLog;
    uint32_t _lastFlush;
    
    // Error tracking
    uint32_t _consecutiveWriteErrors;
    
    // Internal functions
    void writeHeader();
    void writeLine(const char* line);
    void formatTimestamp(char* buffer, size_t size);
    void buildFilename(uint32_t bootCount);
};

#endif // DATA_LOGGER_H
