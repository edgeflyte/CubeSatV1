/*
 * ============================================================================
 * CommandHandler.h - CubeSat Command Processing System
 * ============================================================================
 * 
 * This module handles incoming commands from both the ground station (via radio)
 * and local interfaces (via Bluetooth/Serial). Commands follow the same packet
 * structure as telemetry for consistency.
 * 
 * COMMAND SOURCES:
 *   - Ground Station: Commands received via NRF24L01 radio
 *   - Local Interface: Text commands via Serial/BLE for testing
 * 
 * COMMAND TYPES:
 *   - System Commands: Reboot, mode changes, status queries
 *   - Power Commands: BMS channel control
 *   - Configuration: Setting changes, calibration
 *   - Debug Commands: Test and diagnostic functions
 * 
 * SECURITY:
 *   Commands can be restricted based on:
 *   - Source (local vs ground station)
 *   - Current system mode (some commands blocked in safe mode)
 *   - Authentication (optional command signing)
 * 
 * EDUCATIONAL NOTES:
 * This demonstrates command and control systems for spacecraft, including:
 *   - Command parsing and validation
 *   - Access control and security
 *   - Command acknowledgment and feedback
 * 
 * ============================================================================
 */

#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <Arduino.h>
#include "TelemetryRadio.h"

// ============================================================================
// COMMAND CONFIGURATION
// ============================================================================

#define CMD_MAX_ARGS            8       // Maximum command arguments
#define CMD_MAX_ARG_LENGTH      16      // Maximum argument string length
#define CMD_MAX_MESSAGE_LENGTH  64      // Maximum response message length

// ============================================================================
// COMMAND TYPE DEFINITIONS
// ============================================================================

/**
 * Command Types (Packet Types for Commands)
 * 
 * Commands use the same packet structure as telemetry but with
 * command-specific type codes in the 0x80xx range.
 * 
 * NAMING CONVENTION:
 *   0x80xx - System commands
 *   0x81xx - Power/BMS commands
 *   0x82xx - Communication commands
 *   0x83xx - Sensor commands
 *   0x84xx - GPS commands
 *   0x85xx - IMU commands
 *   0x86xx - Payload commands
 *   0x8Fxx - Debug commands
 */
enum CommandType : uint16_t {
    // System Commands (0x80xx)
    CMD_SYS_NOP             = 0x8000,   // No operation (ping)
    CMD_SYS_REBOOT          = 0x8001,   // System reboot
    CMD_SYS_RESTART         = 0x8002,   // Restart application (no hardware reset)
    CMD_SYS_HIBERNATE       = 0x8003,   // Enter low-power hibernate
    CMD_SYS_WAKE            = 0x8004,   // Wake from hibernate
    CMD_SYS_SET_MODE        = 0x8005,   // Set operating mode
    CMD_SYS_GET_STATUS      = 0x8006,   // Request status telemetry
    CMD_SYS_GET_VERSION     = 0x8007,   // Request version info
    CMD_SYS_RESET_BOOTCNT   = 0x8008,   // Reset boot counter
    CMD_SYS_SET_TIME        = 0x8009,   // Set system time
    CMD_SYS_GET_TIME        = 0x800A,   // Get system time
    CMD_SYS_BUZZER          = 0x800B,   // Play buzzer tone (freq u16 BE, duration u16 BE ms)
    
    // Power/BMS Commands (0x81xx)
    CMD_PWR_SET_CHANNEL     = 0x8100,   // Set LDO channel state
    CMD_PWR_GET_BATTERY     = 0x8101,   // Request battery status
    CMD_PWR_SET_LOW_THRESH  = 0x8102,   // Set low battery threshold
    CMD_PWR_EMERGENCY_OFF   = 0x8103,   // Emergency power off
    CMD_PWR_RESET_BMS       = 0x8104,   // Reset BMS module
    CMD_PWR_STORE_CHANNELS  = 0x8105,   // Write LDO boot mask to BMS EEPROM
    
    // Communication Commands (0x82xx)
    CMD_COMM_SET_CHANNEL    = 0x8200,   // Set radio channel
    CMD_COMM_SET_POWER      = 0x8201,   // Set radio power level
    CMD_COMM_GET_STATS      = 0x8202,   // Get communication statistics
    CMD_COMM_RESET_STATS    = 0x8203,   // Reset statistics
    CMD_COMM_SET_INTERVAL   = 0x8204,   // Set telemetry interval
    CMD_COMM_SAVE           = 0x8205,   // Save channel, PA power, and telem interval to EEPROM
    
    // Sensor Commands (0x83xx)
    CMD_SENS_CALIBRATE      = 0x8300,   // Calibrate sensor
    CMD_SENS_ENABLE         = 0x8301,   // Enable sensor
    CMD_SENS_DISABLE        = 0x8302,   // Disable sensor
    CMD_SENS_GET_DATA       = 0x8303,   // Request sensor data
    
    // Payload Commands (0x86xx)
    CMD_PAYLOAD_START       = 0x8600,   // Start payload operation
    CMD_PAYLOAD_STOP        = 0x8601,   // Stop payload operation
    CMD_PAYLOAD_CONFIG      = 0x8602,   // Configure payload
    CMD_PAYLOAD_DATA        = 0x8603,   // Request payload data
    
    // Debug Commands (0x8Fxx)
    CMD_DEBUG_ECHO          = 0x8F00,   // Echo back data
    CMD_DEBUG_MEMORY        = 0x8F01,   // Get memory statistics
    CMD_DEBUG_SENSORS       = 0x8F02,   // Dump all sensor data
    CMD_DEBUG_RAW_TX        = 0x8F03,   // Transmit raw data
    CMD_DEBUG_LOG_LEVEL     = 0x8F04,   // Set debug log level
    CMD_DEBUG_UART_PASSTHROUGH = 0x8F05, // UART serial passthrough
};

// ============================================================================
// COMMAND ACTION TYPES
// ============================================================================

/**
 * Command Action Results
 * 
 * After parsing a command, the handler returns an action code
 * that tells the main firmware what to do.
 */
enum CommandAction : uint8_t {
    CMD_ACTION_NONE,            // No action needed
    CMD_ACTION_REBOOT,          // Reboot the system
    CMD_ACTION_RESTART,         // Restart application
    CMD_ACTION_SET_MODE,        // Change operating mode
    CMD_ACTION_HIBERNATE,       // Enter hibernate
    CMD_ACTION_WAKE,            // Wake from hibernate
    CMD_ACTION_BMS_CHANNEL,     // Control BMS channel
    CMD_ACTION_BMS_STORE,       // Write BMS channel boot mask to EEPROM
    CMD_ACTION_SEND_TELEMETRY,  // Send telemetry now
    CMD_ACTION_PRINT_STATUS,    // Print system status
    CMD_ACTION_PRINT_SENSORS,   // Print sensor readings
    CMD_ACTION_PRINT_GPS,       // Print GPS data
    CMD_ACTION_PRINT_IMU,       // Print IMU data
    CMD_ACTION_PRINT_ENV,       // Print environmental data
    CMD_ACTION_PRINT_POWER,     // Print power/battery status
    CMD_ACTION_PRINT_TELEM_STATS, // Print telemetry statistics
    CMD_ACTION_PRINT_MEMORY,    // Print memory statistics
    CMD_ACTION_PRINT_MESH,      // Print mesh network status
    CMD_ACTION_PRINT_CUBESAT,   // Print specific CubeSat data
    CMD_ACTION_MESH_RELAY,      // Set mesh relay on/off
    CMD_ACTION_MESH_LOG,        // Set mesh SD logging on/off
    CMD_ACTION_PRINT_LOG_STATUS,// Print data logger status
    CMD_ACTION_LOG_ENABLE,      // Enable/disable data logging
    CMD_ACTION_LOG_INTERVAL,    // Set logging interval
    CMD_ACTION_LOG_FLUSH,       // Force flush log to SD
    CMD_ACTION_CALIBRATE,       // Run calibration
    CMD_ACTION_CONFIG_CHANGE,   // Configuration changed
    CMD_ACTION_UART_PASSTHROUGH, // Enable/disable UART passthrough
    CMD_ACTION_RADIO_CHANNEL,   // Apply radio channel (parameter = channel)
    CMD_ACTION_RADIO_POWER,     // Apply radio PA level (parameter = 0..3)
    CMD_ACTION_TELEM_INTERVAL,  // Set fast telem interval (parameter<<8|value ms)
    CMD_ACTION_SAVE_RADIO,      // Write radio channel, PA, and telem interval to EEPROM
    CMD_ACTION_BUZZER,          // Play buzzer (aux0 = Hz, aux1 = duration ms; 0 Hz stops)
    CMD_ACTION_CUSTOM,          // Custom action (check parameter)
};

// ============================================================================
// COMMAND RESULT STRUCTURE
// ============================================================================

/**
 * CommandResult
 * 
 * Returned by the command processor to indicate success/failure
 * and any actions that need to be taken.
 */
struct CommandResult {
    bool success;                           // Command executed successfully
    CommandAction action;                   // Action for main loop to take
    uint8_t parameter;                      // Action parameter
    uint8_t value;                          // Additional value
    uint16_t aux0;                          // Wider parameter (buzzer frequency, Hz)
    uint16_t aux1;                          // Wider value (buzzer duration, ms)
    char message[CMD_MAX_MESSAGE_LENGTH];   // Response/error message
    
    CommandResult() : success(false), action(CMD_ACTION_NONE), 
                      parameter(0), value(0), aux0(0), aux1(0) {
        message[0] = '\0';
    }
};

// ============================================================================
// TEXT COMMAND STRUCTURE (for Serial/BLE)
// ============================================================================

/**
 * ParsedCommand
 * 
 * Result of parsing a text command string
 */
struct ParsedCommand {
    bool valid;
    char command[16];
    uint8_t argCount;
    char args[CMD_MAX_ARGS][CMD_MAX_ARG_LENGTH];
    
    ParsedCommand() : valid(false), argCount(0) {
        command[0] = '\0';
    }
};

// ============================================================================
// COMMAND HANDLER CLASS
// ============================================================================

/**
 * CommandHandler Class
 * 
 * Processes commands from both packet-based (radio) and text-based
 * (Serial/BLE) sources.
 * 
 * USAGE:
 *   CommandHandler commands;
 *   commands.begin(DEVICE_SERIAL);
 *   
 *   // Process radio packet:
 *   CommandResult result = commands.processPacket(rxPacket);
 *   
 *   // Process text command:
 *   CommandResult result = commands.processTextCommand("status");
 */
class CommandHandler {
public:
    /**
     * Initialize command handler
     * @param deviceSerial This device's serial number (for addressing)
     */
    void begin(uint32_t deviceSerial);
    
    // ========================================================================
    // Packet Command Processing
    // ========================================================================
    
    /**
     * Process a received telemetry packet as a command
     * @param packet The received packet
     * @return CommandResult with action and status
     */
    CommandResult processPacket(TelemetryPacket& packet);
    
    // ========================================================================
    // Text Command Processing (Serial/BLE Interface)
    // ========================================================================
    
    /**
     * Process a text command string
     * @param commandLine The command string (e.g., "status" or "mode nominal")
     * @return CommandResult with action and status
     */
    CommandResult processTextCommand(const String& commandLine);
    
    /**
     * Parse a text command into components
     * @param commandLine The raw command string
     * @return ParsedCommand structure
     */
    ParsedCommand parseTextCommand(const String& commandLine);
    
    // ========================================================================
    // Command Registration (for custom commands)
    // ========================================================================
    
    /**
     * Custom command callback type
     * @param cmd The parsed command
     * @return CommandResult
     */
    typedef CommandResult (*CustomCommandCallback)(ParsedCommand& cmd);
    
    /**
     * Register a custom text command
     * @param commandName The command name (e.g., "mycommand")
     * @param callback Function to handle the command
     */
    void registerCustomCommand(const char* commandName, CustomCommandCallback callback);
    
    // ========================================================================
    // Configuration
    // ========================================================================
    
    /**
     * Enable/disable command echo (for debugging)
     */
    void setEchoEnabled(bool enabled) { _echoEnabled = enabled; }
    
    /**
     * Set command access level required
     * 0 = All commands allowed
     * 1 = Local commands only
     * 2 = Admin commands require authentication
     */
    void setAccessLevel(uint8_t level) { _accessLevel = level; }
    
    /**
     * Print available commands to Serial
     */
    void printHelp();
    
private:
    uint32_t _deviceSerial;
    bool _echoEnabled;
    uint8_t _accessLevel;
    
    // Custom command support
    static const uint8_t MAX_CUSTOM_COMMANDS = 8;
    struct CustomCommand {
        char name[16];
        CustomCommandCallback callback;
    };
    CustomCommand _customCommands[MAX_CUSTOM_COMMANDS];
    uint8_t _customCommandCount;
    
    // Internal command handlers
    CommandResult handleSystemCommand(uint16_t cmdType, uint8_t* data, uint8_t length);
    CommandResult handlePowerCommand(uint16_t cmdType, uint8_t* data, uint8_t length);
    CommandResult handleCommCommand(uint16_t cmdType, uint8_t* data, uint8_t length);
    CommandResult handleSensorCommand(uint16_t cmdType, uint8_t* data, uint8_t length);
    CommandResult handleDebugCommand(uint16_t cmdType, uint8_t* data, uint8_t length);
    
    // Text command handlers
    CommandResult handleStatusCommand(ParsedCommand& cmd);
    CommandResult handleModeCommand(ParsedCommand& cmd);
    CommandResult handleRebootCommand(ParsedCommand& cmd);
    CommandResult handlePowerCommand(ParsedCommand& cmd);
    CommandResult handleTelemetryCommand(ParsedCommand& cmd);
    CommandResult handleConfigCommand(ParsedCommand& cmd);
    CommandResult handleHelpCommand(ParsedCommand& cmd);
};

// ============================================================================
// COMMAND DICTIONARY
// ============================================================================

/**
 * CommandDefinition
 * 
 * Defines a command in the command dictionary for documentation
 * and help text generation.
 */
struct CommandDefinition {
    uint16_t type;              // Command type code
    const char* name;           // Command name (for text interface)
    const char* description;    // Human-readable description
    const char* usage;          // Usage example
    uint8_t minArgs;            // Minimum arguments required
    uint8_t maxArgs;            // Maximum arguments allowed
};

// Declare command dictionary (defined in .cpp)
extern const CommandDefinition COMMAND_DICTIONARY[];
extern const uint16_t COMMAND_DICTIONARY_SIZE;

// ============================================================================
// TEXT COMMAND REFERENCE
// ============================================================================

/*
 * AVAILABLE TEXT COMMANDS
 * =======================
 * 
 * System Commands:
 *   help                    - Show available commands
 *   status                  - Show system status
 *   version                 - Show firmware version
 *   uptime                  - Show system uptime
 *   reboot                  - Reboot the system
 *   restart                 - Restart application
 *   
 * Mode Commands:
 *   mode                    - Show current mode
 *   mode <name>             - Set mode (safe/nominal/lowpower/science/emergency)
 *   
 * Power Commands:
 *   power                   - Show power status
 *   power ch<n> on          - Turn channel n ON
 *   power ch<n> off         - Turn channel n OFF
 *   power reset             - Reset BMS
 *   
 * Telemetry Commands:
 *   telem                   - Send telemetry now
 *   telem interval <ms>     - Set telemetry interval
 *   telem stats             - Show transmission statistics
 *   
 * Sensor Commands:
 *   sensors                 - Show all sensor readings
 *   gps                     - Show GPS data
 *   imu                     - Show IMU data
 *   env                     - Show environmental data
 *   
 * Debug Commands:
 *   debug on                - Enable debug output
 *   debug off               - Disable debug output
 *   memory                  - Show memory statistics
 *   echo <text>             - Echo text back
 */

#endif // COMMAND_HANDLER_H
