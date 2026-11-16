/*
 * ============================================================================
 * CommandHandler.cpp - CubeSat Command Processing Implementation
 * ============================================================================
 * 
 * Implementation of the command processing system for both radio and
 * text-based commands.
 * 
 * EDUCATIONAL NOTES:
 * -----------------
 * This file demonstrates important spacecraft command handling concepts:
 * 
 * 1. COMMAND PARSING
 *    Converting raw bytes or text into actionable commands. This includes
 *    validation, bounds checking, and error handling.
 * 
 * 2. COMMAND ROUTING
 *    Directing commands to the appropriate handler based on type. This
 *    uses a dispatch pattern common in embedded systems.
 * 
 * 3. ACCESS CONTROL
 *    Not all commands should be available in all modes. Some commands
 *    are dangerous (like reboot) and should be restricted.
 * 
 * 4. COMMAND ACKNOWLEDGMENT
 *    Every command should produce a result that can be reported back
 *    to the command source.
 * 
 * ============================================================================
 */

#include "CommandHandler.h"
#include "version.h"

static void applyBuzzerResult(CommandResult& result, unsigned long freq, unsigned long dur);

// ============================================================================
// COMMAND DICTIONARY
// ============================================================================

/**
 * Command Dictionary
 * 
 * This table defines all available commands for documentation and help text.
 */
const CommandDefinition COMMAND_DICTIONARY[] = {
    // System Commands
    {CMD_SYS_NOP,           "nop",      "No operation (ping)",          "nop",              0, 0},
    {CMD_SYS_REBOOT,        "reboot",   "Reboot system",                "reboot confirm",   1, 1},
    {CMD_SYS_RESTART,       "restart",  "Restart application",          "restart confirm",  1, 1},
    {CMD_SYS_HIBERNATE,     "hibernate","Enter low-power mode",         "hibernate",        0, 0},
    {CMD_SYS_WAKE,          "wake",     "Wake from hibernate",          "wake",             0, 0},
    {CMD_SYS_SET_MODE,      "mode",     "Set operating mode",           "mode <name>",      0, 1},
    {CMD_SYS_GET_STATUS,    "status",   "Show system status",           "status",           0, 0},
    {CMD_SYS_GET_VERSION,   "version",  "Show firmware version",        "version",          0, 0},
    
    // Power Commands
    {CMD_PWR_SET_CHANNEL,   "power",    "Control power channels",       "power ch<n> on/off", 0, 2},
    {CMD_PWR_GET_BATTERY,   "battery",  "Show battery status",          "battery",          0, 0},
    {CMD_PWR_RESET_BMS,     "bmsreset", "Reset BMS module",             "bmsreset",         0, 0},
    {CMD_PWR_STORE_CHANNELS,"power",    "Save LDO boot mask to BMS",    "power boot <mask>", 1, 1},
    
    // Communication Commands
    {CMD_COMM_GET_STATS,    "telem",    "Telemetry control",            "telem [interval <ms>]", 0, 2},
    {CMD_COMM_SET_CHANNEL,  "channel",  "Set radio channel",            "channel <n>",      1, 1},
    
    // Sensor Commands
    {CMD_SENS_GET_DATA,     "sensors",  "Show sensor readings",         "sensors",          0, 0},
    
    // Debug Commands
    {CMD_DEBUG_ECHO,        "echo",     "Echo text back",               "echo <text>",      1, 8},
    {CMD_DEBUG_MEMORY,      "memory",   "Show memory statistics",       "memory",           0, 0},
    {CMD_DEBUG_LOG_LEVEL,   "debug",    "Set debug level",              "debug on/off",     0, 1},
    {CMD_DEBUG_UART_PASSTHROUGH, "uart", "UART passthrough mode",       "uart <gps|satbus> [on|off]", 1, 2},
};

const uint16_t COMMAND_DICTIONARY_SIZE = sizeof(COMMAND_DICTIONARY) / sizeof(CommandDefinition);

// ============================================================================
// INITIALIZATION
// ============================================================================

/**
 * Initialize the command handler
 */
void CommandHandler::begin(uint32_t deviceSerial) {
    _deviceSerial = deviceSerial;
    _echoEnabled = false;
    _accessLevel = 0;  // All commands allowed by default
    _customCommandCount = 0;
    memset(_customCommands, 0, sizeof(_customCommands));
}

// ============================================================================
// PACKET COMMAND PROCESSING
// ============================================================================

/**
 * Process a command received via radio packet
 * 
 * The packet type field indicates the command, and the data field
 * contains any parameters.
 */
CommandResult CommandHandler::processPacket(TelemetryPacket& packet) {
    CommandResult result;
    uint16_t cmdType = packet.packetType;
    
    // Verify this is a command packet (0x8xxx range)
    if ((cmdType & 0x8000) == 0) {
        result.success = false;
        strcpy(result.message, "Not a command packet");
        return result;
    }
    
    // Defensive: never let handlers read past the packet data buffer
    if (packet.dataLength > TELEM_MAX_DATA_SIZE) {
        packet.dataLength = TELEM_MAX_DATA_SIZE;
    }
    
    // Echo if enabled
    if (_echoEnabled) {
        Serial.print(F("[CMD] Received packet command: 0x"));
        Serial.println(cmdType, HEX);
    }
    
    // Route to appropriate handler based on command category
    uint8_t category = (cmdType >> 8) & 0x0F;
    
    switch (category) {
        case 0x00:  // System commands (0x80xx)
            result = handleSystemCommand(cmdType, packet.data, packet.dataLength);
            break;
            
        case 0x01:  // Power commands (0x81xx)
            result = handlePowerCommand(cmdType, packet.data, packet.dataLength);
            break;
            
        case 0x02:  // Communication commands (0x82xx)
            result = handleCommCommand(cmdType, packet.data, packet.dataLength);
            break;
            
        case 0x03:  // Sensor commands (0x83xx)
            result = handleSensorCommand(cmdType, packet.data, packet.dataLength);
            break;
            
        case 0x0F:  // Debug commands (0x8Fxx)
            result = handleDebugCommand(cmdType, packet.data, packet.dataLength);
            break;
            
        default:
            result.success = false;
            strcpy(result.message, "Unknown command category");
            break;
    }
    
    return result;
}

// ============================================================================
// TEXT COMMAND PROCESSING
// ============================================================================

/**
 * Process a text command from Serial/BLE
 */
CommandResult CommandHandler::processTextCommand(const String& commandLine) {
    CommandResult result;
    
    // Parse the command string
    ParsedCommand cmd = parseTextCommand(commandLine);
    
    if (!cmd.valid) {
        result.success = false;
        strcpy(result.message, "Invalid command format");
        return result;
    }
    
    // Echo if enabled
    if (_echoEnabled) {
        Serial.print(F("[CMD] Text command: "));
        Serial.println(cmd.command);
    }
    
    // Check for custom commands first
    for (uint8_t i = 0; i < _customCommandCount; i++) {
        if (strcasecmp(cmd.command, _customCommands[i].name) == 0) {
            if (_customCommands[i].callback) {
                return _customCommands[i].callback(cmd);
            }
        }
    }
    
    // Route to built-in command handlers
    if (strcasecmp(cmd.command, "status") == 0) {
        return handleStatusCommand(cmd);
    }
    else if (strcasecmp(cmd.command, "mode") == 0) {
        return handleModeCommand(cmd);
    }
    else if (strcasecmp(cmd.command, "reboot") == 0 || 
             strcasecmp(cmd.command, "restart") == 0) {
        return handleRebootCommand(cmd);
    }
    else if (strcasecmp(cmd.command, "power") == 0 ||
             strcasecmp(cmd.command, "battery") == 0 ||
             strcasecmp(cmd.command, "bmsreset") == 0) {
        return handlePowerCommand(cmd);
    }
    else if (strcasecmp(cmd.command, "telem") == 0 ||
             strcasecmp(cmd.command, "telemetry") == 0) {
        return handleTelemetryCommand(cmd);
    }
    else if (strcasecmp(cmd.command, "help") == 0 ||
             strcasecmp(cmd.command, "?") == 0) {
        return handleHelpCommand(cmd);
    }
    else if (strcasecmp(cmd.command, "version") == 0) {
        result.success = true;
        snprintf(result.message, CMD_MAX_MESSAGE_LENGTH, "%s", FIRMWARE_VERSION);
        return result;
    }
    else if (strcasecmp(cmd.command, "uptime") == 0) {
        result.success = true;
        result.action = CMD_ACTION_PRINT_STATUS;
        strcpy(result.message, "Uptime displayed");
        return result;
    }
    else if (strcasecmp(cmd.command, "sensors") == 0) {
        result.success = true;
        result.action = CMD_ACTION_PRINT_SENSORS;
        strcpy(result.message, "Sensor data displayed");
        return result;
    }
    else if (strcasecmp(cmd.command, "gps") == 0) {
        result.success = true;
        result.action = CMD_ACTION_PRINT_GPS;
        strcpy(result.message, "GPS data displayed");
        return result;
    }
    else if (strcasecmp(cmd.command, "imu") == 0) {
        result.success = true;
        result.action = CMD_ACTION_PRINT_IMU;
        strcpy(result.message, "IMU data displayed");
        return result;
    }
    else if (strcasecmp(cmd.command, "env") == 0) {
        result.success = true;
        result.action = CMD_ACTION_PRINT_ENV;
        strcpy(result.message, "Environmental data displayed");
        return result;
    }
    else if (strcasecmp(cmd.command, "echo") == 0) {
        result.success = true;
        // Concatenate all args into message
        result.message[0] = '\0';
        for (int i = 0; i < cmd.argCount; i++) {
            if (i > 0) strcat(result.message, " ");
            strncat(result.message, cmd.args[i], CMD_MAX_MESSAGE_LENGTH - strlen(result.message) - 1);
        }
        Serial.println(result.message);
        return result;
    }
    else if (strcasecmp(cmd.command, "debug") == 0) {
        if (cmd.argCount > 0 && strcasecmp(cmd.args[0], "on") == 0) {
            _echoEnabled = true;
            result.success = true;
            strcpy(result.message, "Debug output enabled");
        } else if (cmd.argCount > 0 && strcasecmp(cmd.args[0], "off") == 0) {
            _echoEnabled = false;
            result.success = true;
            strcpy(result.message, "Debug output disabled");
        } else {
            result.success = true;
            snprintf(result.message, CMD_MAX_MESSAGE_LENGTH, 
                     "Debug: %s", _echoEnabled ? "on" : "off");
        }
        return result;
    }
    else if (strcasecmp(cmd.command, "memory") == 0) {
        result.success = true;
        result.action = CMD_ACTION_PRINT_MEMORY;
        strcpy(result.message, "Memory info displayed");
        return result;
    }
    else if (strcasecmp(cmd.command, "uart") == 0 ||
             strcasecmp(cmd.command, "passthrough") == 0) {
        // uart <gps|satbus> [on|off]
        if (cmd.argCount == 0) {
            result.success = false;
            strcpy(result.message, "Usage: uart <gps|satbus> [on|off]");
            return result;
        }
        
        result.success = true;
        result.action = CMD_ACTION_UART_PASSTHROUGH;
        
        // Determine which UART (parameter: 1=GPS, 2=SATBUS)
        if (strcasecmp(cmd.args[0], "gps") == 0) {
            result.parameter = 1;  // GPS UART
        } else if (strcasecmp(cmd.args[0], "satbus") == 0) {
            result.parameter = 2;  // SATBUS UART
        } else {
            result.success = false;
            strcpy(result.message, "Invalid UART. Use: gps or satbus");
            return result;
        }
        
        // Determine on/off state (value: 0=off, 1=on, 0xFF=query)
        if (cmd.argCount >= 2) {
            if (strcasecmp(cmd.args[1], "on") == 0) {
                result.value = 1;
            } else if (strcasecmp(cmd.args[1], "off") == 0) {
                result.value = 0;
            } else {
                result.success = false;
                strcpy(result.message, "Invalid state. Use: on or off");
                return result;
            }
        } else {
            result.value = 0xFF;  // Query mode
        }
        
        snprintf(result.message, CMD_MAX_MESSAGE_LENGTH,
                 "UART passthrough %s %s", 
                 cmd.args[0],
                 result.value == 0xFF ? "status queried" : (result.value ? "enabled" : "disabled"));
        return result;
    }
    else if (strcasecmp(cmd.command, "nop") == 0 ||
             strcasecmp(cmd.command, "ping") == 0) {
        result.success = true;
        strcpy(result.message, "PONG");
        return result;
    }
    else if (strcasecmp(cmd.command, "buzzer") == 0 ||
             strcasecmp(cmd.command, "beep") == 0) {
        if (cmd.argCount >= 1 && strcasecmp(cmd.args[0], "off") == 0) {
            applyBuzzerResult(result, 0, 0);
            return result;
        }
        if (cmd.argCount >= 2) {
            unsigned long freq = strtoul(cmd.args[0], nullptr, 10);
            unsigned long dur = strtoul(cmd.args[1], nullptr, 10);
            applyBuzzerResult(result, freq, dur);
            return result;
        }
        result.success = false;
        strcpy(result.message, "Usage: buzzer <hz> <ms> | off");
        return result;
    }
    else if (strcasecmp(cmd.command, "radio") == 0 &&
             cmd.argCount >= 1 && strcasecmp(cmd.args[0], "save") == 0) {
        result.success = true;
        result.action = CMD_ACTION_SAVE_RADIO;
        strcpy(result.message, "Radio saved");
        return result;
    }
    else if (strcasecmp(cmd.command, "hibernate") == 0) {
        result.success = true;
        result.action = CMD_ACTION_SET_MODE;
        result.parameter = 3;  // LOW_POWER mode
        strcpy(result.message, "Entering low power mode");
        return result;
    }
    // ========================================================================
    // Mesh Network Commands
    // ========================================================================
    else if (strcasecmp(cmd.command, "mesh") == 0 ||
             strcasecmp(cmd.command, "cubesats") == 0) {
        // mesh - show mesh status
        // mesh relay on/off - enable/disable relay
        // mesh log on/off - enable/disable SD logging
        if (cmd.argCount == 0) {
            result.success = true;
            result.action = CMD_ACTION_PRINT_MESH;
            strcpy(result.message, "Mesh status displayed");
            return result;
        }
        
        if (strcasecmp(cmd.args[0], "relay") == 0) {
            result.success = true;
            result.action = CMD_ACTION_MESH_RELAY;
            if (cmd.argCount >= 2) {
                result.parameter = (strcasecmp(cmd.args[1], "on") == 0) ? 1 : 0;
                snprintf(result.message, CMD_MAX_MESSAGE_LENGTH,
                         "Mesh relay %s", result.parameter ? "enabled" : "disabled");
            } else {
                result.parameter = 0xFF;  // Query mode
                strcpy(result.message, "Relay status queried");
            }
            return result;
        }
        
        if (strcasecmp(cmd.args[0], "log") == 0) {
            result.success = true;
            result.action = CMD_ACTION_MESH_LOG;
            if (cmd.argCount >= 2) {
                result.parameter = (strcasecmp(cmd.args[1], "on") == 0) ? 1 : 0;
                snprintf(result.message, CMD_MAX_MESSAGE_LENGTH,
                         "Mesh SD logging %s", result.parameter ? "enabled" : "disabled");
            } else {
                result.parameter = 0xFF;  // Query mode
                strcpy(result.message, "SD logging status queried");
            }
            return result;
        }
        
        // mesh list - same as no args
        if (strcasecmp(cmd.args[0], "list") == 0) {
            result.success = true;
            result.action = CMD_ACTION_PRINT_MESH;
            strcpy(result.message, "Mesh status displayed");
            return result;
        }
        
        result.success = false;
        strcpy(result.message, "Usage: mesh [relay|log] [on|off]");
        return result;
    }
    else if (strcasecmp(cmd.command, "cubesat") == 0) {
        // cubesat <serial> - show data from specific CubeSat
        if (cmd.argCount == 0) {
            result.success = false;
            strcpy(result.message, "Usage: cubesat <serial_hex>");
            return result;
        }
        
        result.success = true;
        result.action = CMD_ACTION_PRINT_CUBESAT;
        // Parse hex serial number
        result.parameter = strtoul(cmd.args[0], NULL, 16) & 0xFF;  // Store low byte
        result.value = (strtoul(cmd.args[0], NULL, 16) >> 8) & 0xFF;  // Store next byte
        strcpy(result.message, "CubeSat data displayed");
        return result;
    }
    // ========================================================================
    // Data Logging Commands
    // ========================================================================
    else if (strcasecmp(cmd.command, "log") == 0) {
        // log - show status
        // log on/off - enable/disable
        // log interval <ms> - set interval
        // log flush - force flush
        if (cmd.argCount == 0) {
            result.success = true;
            result.action = CMD_ACTION_PRINT_LOG_STATUS;
            strcpy(result.message, "Log status displayed");
            return result;
        }
        
        if (strcasecmp(cmd.args[0], "on") == 0) {
            result.success = true;
            result.action = CMD_ACTION_LOG_ENABLE;
            result.parameter = 1;
            strcpy(result.message, "Logging enabled");
            return result;
        }
        
        if (strcasecmp(cmd.args[0], "off") == 0) {
            result.success = true;
            result.action = CMD_ACTION_LOG_ENABLE;
            result.parameter = 0;
            strcpy(result.message, "Logging disabled");
            return result;
        }
        
        if (strcasecmp(cmd.args[0], "interval") == 0) {
            if (cmd.argCount < 2) {
                result.success = false;
                strcpy(result.message, "Usage: log interval <ms>");
                return result;
            }
            result.success = true;
            result.action = CMD_ACTION_LOG_INTERVAL;
            uint32_t interval = atol(cmd.args[1]);
            // Interval is transported as two bytes; clamp to a valid range.
            if (interval < 100) interval = 100;
            if (interval > 60000) interval = 60000;
            result.parameter = (interval >> 8) & 0xFF;  // High byte
            result.value = interval & 0xFF;              // Low byte
            snprintf(result.message, CMD_MAX_MESSAGE_LENGTH,
                     "Log interval set to %lu ms", interval);
            return result;
        }
        
        if (strcasecmp(cmd.args[0], "flush") == 0) {
            result.success = true;
            result.action = CMD_ACTION_LOG_FLUSH;
            strcpy(result.message, "Log flushed");
            return result;
        }
        
        if (strcasecmp(cmd.args[0], "status") == 0) {
            result.success = true;
            result.action = CMD_ACTION_PRINT_LOG_STATUS;
            strcpy(result.message, "Log status displayed");
            return result;
        }
        
        result.success = false;
        strcpy(result.message, "Usage: log [on|off|interval <ms>|flush|status]");
        return result;
    }
    
    // Unknown command
    result.success = false;
    snprintf(result.message, CMD_MAX_MESSAGE_LENGTH, 
             "Unknown command: %s (try 'help')", cmd.command);
    return result;
}

/**
 * Parse a text command string into components
 */
ParsedCommand CommandHandler::parseTextCommand(const String& commandLine) {
    ParsedCommand cmd;
    
    // Trim whitespace
    String line = commandLine;
    line.trim();
    
    if (line.length() == 0) {
        return cmd;  // Invalid (empty)
    }
    
    // Find first space (separates command from args)
    int spaceIndex = line.indexOf(' ');
    
    if (spaceIndex < 0) {
        // No arguments - just command
        strncpy(cmd.command, line.c_str(), sizeof(cmd.command) - 1);
        cmd.command[sizeof(cmd.command) - 1] = '\0';
        cmd.argCount = 0;
        cmd.valid = true;
        return cmd;
    }
    
    // Extract command
    String cmdPart = line.substring(0, spaceIndex);
    strncpy(cmd.command, cmdPart.c_str(), sizeof(cmd.command) - 1);
    cmd.command[sizeof(cmd.command) - 1] = '\0';
    
    // Parse arguments
    String argPart = line.substring(spaceIndex + 1);
    argPart.trim();
    
    cmd.argCount = 0;
    while (argPart.length() > 0 && cmd.argCount < CMD_MAX_ARGS) {
        spaceIndex = argPart.indexOf(' ');
        
        String arg;
        if (spaceIndex < 0) {
            arg = argPart;
            argPart = "";
        } else {
            arg = argPart.substring(0, spaceIndex);
            argPart = argPart.substring(spaceIndex + 1);
            argPart.trim();
        }
        
        strncpy(cmd.args[cmd.argCount], arg.c_str(), CMD_MAX_ARG_LENGTH - 1);
        cmd.args[cmd.argCount][CMD_MAX_ARG_LENGTH - 1] = '\0';
        cmd.argCount++;
    }
    
    cmd.valid = true;
    return cmd;
}

// ============================================================================
// CUSTOM COMMAND REGISTRATION
// ============================================================================

/**
 * Register a custom command handler
 */
void CommandHandler::registerCustomCommand(const char* commandName, CustomCommandCallback callback) {
    if (_customCommandCount >= MAX_CUSTOM_COMMANDS) {
        Serial.println(F("[CMD] Error: Max custom commands reached"));
        return;
    }
    
    strncpy(_customCommands[_customCommandCount].name, commandName, 15);
    _customCommands[_customCommandCount].name[15] = '\0';
    _customCommands[_customCommandCount].callback = callback;
    _customCommandCount++;
}

// ============================================================================
// INTERNAL COMMAND HANDLERS - PACKET BASED
// ============================================================================

static void applyBuzzerResult(CommandResult& result, unsigned long freq, unsigned long dur) {
    if (freq == 0) {
        result.success = true;
        result.action = CMD_ACTION_BUZZER;
        result.aux0 = 0;
        result.aux1 = 0;
        strcpy(result.message, "Buzzer off");
        return;
    }
    if (freq < 20 || freq > 20000 || dur < 10 || dur > 30000) {
        result.success = false;
        strcpy(result.message, "Hz 20-20000, ms 10-30000");
        return;
    }
    result.success = true;
    result.action = CMD_ACTION_BUZZER;
    result.aux0 = (uint16_t)freq;
    result.aux1 = (uint16_t)dur;
    snprintf(result.message, CMD_MAX_MESSAGE_LENGTH, "%luHz %lums", freq, dur);
}

/**
 * Handle system commands (0x80xx)
 */
CommandResult CommandHandler::handleSystemCommand(uint16_t cmdType, uint8_t* data, uint8_t length) {
    CommandResult result;
    
    switch (cmdType) {
        case CMD_SYS_NOP:
            result.success = true;
            strcpy(result.message, "NOP acknowledged");
            break;
            
        case CMD_SYS_REBOOT:
            if (length >= 1 && data[0] == CMD_CONFIRM_BYTE) {
                result.success = true;
                result.action = CMD_ACTION_REBOOT;
                strcpy(result.message, "Rebooting...");
            } else {
                result.success = false;
                strcpy(result.message, "Confirm required (payload 0xA5)");
            }
            break;
            
        case CMD_SYS_RESTART:
            if (length >= 1 && data[0] == CMD_CONFIRM_BYTE) {
                result.success = true;
                result.action = CMD_ACTION_RESTART;
                strcpy(result.message, "Restarting...");
            } else {
                result.success = false;
                strcpy(result.message, "Confirm required (payload 0xA5)");
            }
            break;
            
        case CMD_SYS_HIBERNATE:
            result.success = true;
            result.action = CMD_ACTION_SET_MODE;
            result.parameter = 3;  // LOW_POWER
            strcpy(result.message, "Entering hibernate");
            break;
            
        case CMD_SYS_WAKE:
            result.success = true;
            result.action = CMD_ACTION_SET_MODE;
            result.parameter = 2;  // NOMINAL
            strcpy(result.message, "Waking up");
            break;
            
        case CMD_SYS_SET_MODE:
            // Valid settable modes: 1=SAFE .. 5=EMERGENCY (see SystemMode)
            if (length >= 1 && data[0] >= 1 && data[0] <= 5) {
                result.success = true;
                result.action = CMD_ACTION_SET_MODE;
                result.parameter = data[0];
                strcpy(result.message, "Mode changed");
            } else {
                result.success = false;
                strcpy(result.message, "Invalid or missing mode parameter");
            }
            break;
            
        case CMD_SYS_GET_STATUS:
            result.success = true;
            result.action = CMD_ACTION_SEND_TELEMETRY;
            strcpy(result.message, "Status telemetry sent");
            break;
            
        case CMD_SYS_GET_VERSION:
            result.success = true;
            snprintf(result.message, CMD_MAX_MESSAGE_LENGTH, "%s", FIRMWARE_VERSION);
            break;

        case CMD_SYS_BUZZER:
            if (length >= 4) {
                unsigned long freq = ((unsigned long)data[0] << 8) | data[1];
                unsigned long dur = ((unsigned long)data[2] << 8) | data[3];
                applyBuzzerResult(result, freq, dur);
            } else {
                result.success = false;
                strcpy(result.message, "Need freq+duration (4 bytes)");
            }
            break;
            
        default:
            result.success = false;
            strcpy(result.message, "Unknown system command");
            break;
    }
    
    return result;
}

/**
 * Handle power/BMS commands (0x81xx)
 */
CommandResult CommandHandler::handlePowerCommand(uint16_t cmdType, uint8_t* data, uint8_t length) {
    CommandResult result;
    
    switch (cmdType) {
        case CMD_PWR_SET_CHANNEL:
            if (length >= 2 && data[0] >= 1 && data[0] <= 4) {
                result.success = true;
                result.action = CMD_ACTION_BMS_CHANNEL;
                result.parameter = data[0];  // Channel number (1-4)
                result.value = data[1];      // State (0=off, 1=on)
                snprintf(result.message, CMD_MAX_MESSAGE_LENGTH,
                         "Channel %d set to %s", data[0], data[1] ? "ON" : "OFF");
            } else {
                result.success = false;
                strcpy(result.message, "Invalid channel/state (channel 1-4)");
            }
            break;
            
        case CMD_PWR_GET_BATTERY:
            result.success = true;
            result.action = CMD_ACTION_SEND_TELEMETRY;
            strcpy(result.message, "Battery status sent");
            break;
            
        case CMD_PWR_EMERGENCY_OFF:
            if (length >= 1 && data[0] == CMD_CONFIRM_BYTE) {
                result.success = true;
                result.action = CMD_ACTION_BMS_CHANNEL;
                result.parameter = 0xFF;  // All channels
                result.value = 0;         // OFF
                strcpy(result.message, "EMERGENCY: All power OFF");
            } else {
                result.success = false;
                strcpy(result.message, "Confirm required (payload 0xA5)");
            }
            break;
            
        case CMD_PWR_RESET_BMS:
            result.success = true;
            result.action = CMD_ACTION_BMS_CHANNEL;
            result.parameter = 0;     // System command
            result.value = 1;         // Reset
            strcpy(result.message, "BMS reset");
            break;

        case CMD_PWR_STORE_CHANNELS:
            if (length >= 1 && data[0] <= 0x0F) {
                result.success = true;
                result.action = CMD_ACTION_BMS_STORE;
                result.parameter = data[0];
                snprintf(result.message, CMD_MAX_MESSAGE_LENGTH,
                         "Store BMS boot mask 0x%X", data[0]);
            } else {
                result.success = false;
                strcpy(result.message, "Boot mask must be 0x00–0x0F (bits 0–3)");
            }
            break;
            
        default:
            result.success = false;
            strcpy(result.message, "Unknown power command");
            break;
    }
    
    return result;
}

/**
 * Handle communication commands (0x82xx)
 */
CommandResult CommandHandler::handleCommCommand(uint16_t cmdType, uint8_t* data, uint8_t length) {
    CommandResult result;
    
    switch (cmdType) {
        case CMD_COMM_SET_CHANNEL:
            if (length >= 1 && data[0] <= 125) {
                result.success = true;
                result.action = CMD_ACTION_RADIO_CHANNEL;
                result.parameter = data[0];
                snprintf(result.message, CMD_MAX_MESSAGE_LENGTH,
                         "Radio channel set to %u", data[0]);
            } else {
                result.success = false;
                strcpy(result.message, "Invalid channel (0-125)");
            }
            break;

        case CMD_COMM_SET_POWER:
            if (length >= 1 && data[0] <= 3) {
                result.success = true;
                result.action = CMD_ACTION_RADIO_POWER;
                result.parameter = data[0];
                snprintf(result.message, CMD_MAX_MESSAGE_LENGTH,
                         "Radio power level set to %u", data[0]);
            } else {
                result.success = false;
                strcpy(result.message, "Invalid power level (0-3)");
            }
            break;

        case CMD_COMM_GET_STATS:
            result.success = true;
            result.action = CMD_ACTION_PRINT_STATUS;
            strcpy(result.message, "Comm stats displayed");
            break;
            
        case CMD_COMM_RESET_STATS:
            result.success = true;
            strcpy(result.message, "Stats reset");
            break;

        case CMD_COMM_SET_INTERVAL:
            if (length >= 2) {
                uint32_t interval = ((uint32_t)data[0] << 8) | data[1];
                if (interval < 100) interval = 100;
                if (interval > 60000) interval = 60000;
                result.success = true;
                result.action = CMD_ACTION_TELEM_INTERVAL;
                result.parameter = (interval >> 8) & 0xFF;
                result.value = interval & 0xFF;
                snprintf(result.message, CMD_MAX_MESSAGE_LENGTH,
                         "Telem interval set to %lu ms", (unsigned long)interval);
            } else {
                result.success = false;
                strcpy(result.message, "Missing interval (2 bytes BE ms)");
            }
            break;

        case CMD_COMM_SAVE:
            result.success = true;
            result.action = CMD_ACTION_SAVE_RADIO;
            strcpy(result.message, "Radio saved");
            break;
            
        default:
            result.success = false;
            strcpy(result.message, "Unknown comm command");
            break;
    }
    
    return result;
}

/**
 * Handle sensor commands (0x83xx)
 */
CommandResult CommandHandler::handleSensorCommand(uint16_t cmdType, uint8_t* data, uint8_t length) {
    CommandResult result;
    
    switch (cmdType) {
        case CMD_SENS_GET_DATA:
            result.success = true;
            result.action = CMD_ACTION_SEND_TELEMETRY;
            strcpy(result.message, "Sensor data sent");
            break;
            
        case CMD_SENS_CALIBRATE:
            result.success = true;
            result.action = CMD_ACTION_CALIBRATE;
            result.parameter = (length > 0) ? data[0] : 0;
            strcpy(result.message, "Calibration started");
            break;
            
        default:
            result.success = false;
            strcpy(result.message, "Unknown sensor command");
            break;
    }
    
    return result;
}

/**
 * Handle debug commands (0x8Fxx)
 */
CommandResult CommandHandler::handleDebugCommand(uint16_t cmdType, uint8_t* data, uint8_t length) {
    CommandResult result;
    
    switch (cmdType) {
        case CMD_DEBUG_ECHO:
            result.success = true;
            // Echo back the data
            memcpy(result.message, data, min(length, (uint8_t)(CMD_MAX_MESSAGE_LENGTH - 1)));
            result.message[min(length, (uint8_t)(CMD_MAX_MESSAGE_LENGTH - 1))] = '\0';
            break;
            
        case CMD_DEBUG_MEMORY:
            result.success = true;
            result.action = CMD_ACTION_PRINT_STATUS;
            strcpy(result.message, "Memory info");
            break;
            
        case CMD_DEBUG_SENSORS:
            result.success = true;
            result.action = CMD_ACTION_PRINT_STATUS;
            strcpy(result.message, "Sensor dump");
            break;
            
        case CMD_DEBUG_LOG_LEVEL:
            if (length >= 1) {
                _echoEnabled = (data[0] > 0);
                result.success = true;
                strcpy(result.message, _echoEnabled ? "Debug ON" : "Debug OFF");
            } else {
                result.success = false;
                strcpy(result.message, "Missing level");
            }
            break;
            
        case CMD_DEBUG_UART_PASSTHROUGH:
            if (length >= 1) {
                result.success = true;
                result.action = CMD_ACTION_UART_PASSTHROUGH;
                result.parameter = data[0];  // 1=GPS, 2=SATBUS
                result.value = (length >= 2) ? data[1] : 0xFF;  // 0=off, 1=on, 0xFF=query
                strcpy(result.message, "UART passthrough command");
            } else {
                result.success = false;
                strcpy(result.message, "Missing UART parameter");
            }
            break;
            
        default:
            result.success = false;
            strcpy(result.message, "Unknown debug command");
            break;
    }
    
    return result;
}

// ============================================================================
// INTERNAL COMMAND HANDLERS - TEXT BASED
// ============================================================================

/**
 * Handle 'status' command
 */
CommandResult CommandHandler::handleStatusCommand(ParsedCommand& cmd) {
    CommandResult result;
    result.success = true;
    result.action = CMD_ACTION_PRINT_STATUS;
    strcpy(result.message, "Status displayed");
    return result;
}

/**
 * Handle 'mode' command
 */
CommandResult CommandHandler::handleModeCommand(ParsedCommand& cmd) {
    CommandResult result;
    
    if (cmd.argCount == 0) {
        // Just show current mode
        result.success = true;
        strcpy(result.message, "Current mode displayed");
        return result;
    }
    
    // Set mode
    result.success = true;
    result.action = CMD_ACTION_SET_MODE;
    
    if (strcasecmp(cmd.args[0], "safe") == 0) {
        result.parameter = 1;
    } else if (strcasecmp(cmd.args[0], "nominal") == 0) {
        result.parameter = 2;
    } else if (strcasecmp(cmd.args[0], "lowpower") == 0 || 
               strcasecmp(cmd.args[0], "low_power") == 0) {
        result.parameter = 3;
    } else if (strcasecmp(cmd.args[0], "science") == 0) {
        result.parameter = 4;
    } else if (strcasecmp(cmd.args[0], "emergency") == 0) {
        result.parameter = 5;
    } else {
        result.success = false;
        strcpy(result.message, "Unknown mode. Use: safe/nominal/lowpower/science/emergency");
        return result;
    }
    
    snprintf(result.message, CMD_MAX_MESSAGE_LENGTH, "Mode set to %s", cmd.args[0]);
    return result;
}

/**
 * Handle 'reboot' and 'restart' commands (require "confirm" argument)
 */
CommandResult CommandHandler::handleRebootCommand(ParsedCommand& cmd) {
    CommandResult result;

    if (cmd.argCount < 1 || strcasecmp(cmd.args[0], "confirm") != 0) {
        result.success = false;
        snprintf(result.message, CMD_MAX_MESSAGE_LENGTH,
                 "Requires: %s confirm", cmd.command);
        return result;
    }

    result.success = true;
    if (strcasecmp(cmd.command, "reboot") == 0) {
        result.action = CMD_ACTION_REBOOT;
        strcpy(result.message, "Rebooting system...");
    } else {
        result.action = CMD_ACTION_RESTART;
        strcpy(result.message, "Restarting application...");
    }
    
    return result;
}

/**
 * Handle power-related commands
 */
CommandResult CommandHandler::handlePowerCommand(ParsedCommand& cmd) {
    CommandResult result;
    
    if (strcasecmp(cmd.command, "battery") == 0) {
        result.success = true;
        result.action = CMD_ACTION_PRINT_POWER;
        strcpy(result.message, "Battery status displayed");
        return result;
    }
    
    if (strcasecmp(cmd.command, "bmsreset") == 0) {
        result.success = true;
        result.action = CMD_ACTION_BMS_CHANNEL;
        result.parameter = 0;
        result.value = 1;
        strcpy(result.message, "BMS reset command sent");
        return result;
    }

    // power emergency confirm — turn all LDO rails off
    if (cmd.argCount >= 2 &&
        strcasecmp(cmd.args[0], "emergency") == 0 &&
        strcasecmp(cmd.args[1], "confirm") == 0) {
        result.success = true;
        result.action = CMD_ACTION_BMS_CHANNEL;
        result.parameter = 0xFF;
        result.value = 0;
        strcpy(result.message, "EMERGENCY: All power OFF");
        return result;
    }
    
    // power boot <mask> — write channels 1–4 boot state to the BMS
    if (cmd.argCount >= 2 && strcasecmp(cmd.args[0], "boot") == 0) {
        int mask = atoi(cmd.args[1]);
        if (mask < 0 || mask > 15) {
            result.success = false;
            strcpy(result.message, "Boot mask must be 0–15 (bit0=CH1 .. bit3=CH4)");
            return result;
        }
        result.success = true;
        result.action = CMD_ACTION_BMS_STORE;
        result.parameter = (uint8_t)mask;
        snprintf(result.message, CMD_MAX_MESSAGE_LENGTH,
                 "Store BMS boot mask %d", mask);
        return result;
    }

    // Power channel commands: power ch1 on, power ch2 off, etc.
    if (cmd.argCount >= 2) {
        // Parse channel number
        uint8_t channel = 0;
        if (strncasecmp(cmd.args[0], "ch", 2) == 0) {
            channel = atoi(&cmd.args[0][2]);
        } else {
            channel = atoi(cmd.args[0]);
        }
        
        if (channel < 1 || channel > 4) {
            result.success = false;
            strcpy(result.message, "Invalid channel (use 1-4)");
            return result;
        }
        
        // Parse state
        uint8_t state = 0;
        if (strcasecmp(cmd.args[1], "on") == 0 || strcmp(cmd.args[1], "1") == 0) {
            state = 1;
        } else if (strcasecmp(cmd.args[1], "off") == 0 || strcmp(cmd.args[1], "0") == 0) {
            state = 0;
        } else {
            result.success = false;
            strcpy(result.message, "Invalid state (use on/off)");
            return result;
        }
        
        result.success = true;
        result.action = CMD_ACTION_BMS_CHANNEL;
        result.parameter = channel;
        result.value = state;
        snprintf(result.message, CMD_MAX_MESSAGE_LENGTH,
                 "Channel %d set to %s", channel, state ? "ON" : "OFF");
        return result;
    }
    
    // Just 'power' - show status
    result.success = true;
    result.action = CMD_ACTION_PRINT_POWER;
    strcpy(result.message, "Power status displayed");
    return result;
}

/**
 * Handle telemetry commands
 */
CommandResult CommandHandler::handleTelemetryCommand(ParsedCommand& cmd) {
    CommandResult result;
    
    if (cmd.argCount == 0) {
        // Send telemetry now
        result.success = true;
        result.action = CMD_ACTION_SEND_TELEMETRY;
        strcpy(result.message, "Telemetry sent");
        return result;
    }
    
    if (strcasecmp(cmd.args[0], "stats") == 0) {
        result.success = true;
        result.action = CMD_ACTION_PRINT_TELEM_STATS;
        strcpy(result.message, "Telemetry stats displayed");
        return result;
    }
    
    if (strcasecmp(cmd.args[0], "interval") == 0 && cmd.argCount >= 2) {
        // Stored as ms/100 in a single byte, so clamp to [100, 25500] ms.
        int telemMs = atoi(cmd.args[1]);
        if (telemMs < 100) telemMs = 100;
        if (telemMs > 25500) telemMs = 25500;
        result.success = true;
        result.action = CMD_ACTION_CONFIG_CHANGE;
        result.parameter = 1;  // Telemetry interval config
        result.value = telemMs / 100;
        snprintf(result.message, CMD_MAX_MESSAGE_LENGTH,
                 "Telemetry interval set to %d ms", telemMs);
        return result;
    }
    
    result.success = false;
    strcpy(result.message, "Unknown telemetry command");
    return result;
}

/**
 * Handle config commands
 */
CommandResult CommandHandler::handleConfigCommand(ParsedCommand& cmd) {
    CommandResult result;
    result.success = true;
    result.action = CMD_ACTION_CONFIG_CHANGE;
    strcpy(result.message, "Configuration updated");
    return result;
}

/**
 * Handle help command
 */
CommandResult CommandHandler::handleHelpCommand(ParsedCommand& cmd) {
    CommandResult result;
    result.success = true;
    strcpy(result.message, "Help displayed");
    
    printHelp();
    
    return result;
}

// ============================================================================
// HELP TEXT
// ============================================================================

/**
 * Print help text to Serial
 */
void CommandHandler::printHelp() {
    Serial.println();
    Serial.println(F("╔═══════════════════════════════════════════════════════════════╗"));
    Serial.println(F("║              MCB SP26 COMMAND REFERENCE                       ║"));
    Serial.println(F("╠═══════════════════════════════════════════════════════════════╣"));
    Serial.println(F("║ SYSTEM COMMANDS                                               ║"));
    Serial.println(F("║   help, ?          - Show this help message                   ║"));
    Serial.println(F("║   status           - Show system status                       ║"));
    Serial.println(F("║   version          - Show firmware version                    ║"));
    Serial.println(F("║   uptime           - Show system uptime                       ║"));
    Serial.println(F("║   reboot           - Reboot the system                        ║"));
    Serial.println(F("║   restart          - Restart application                      ║"));
    Serial.println(F("║   nop, ping        - No-op (connection test)                  ║"));
    Serial.println(F("║   buzzer <hz> <ms> - Play tone (20-20000 Hz, 10-30000 ms)    ║"));
    Serial.println(F("║   buzzer off       - Stop the buzzer                          ║"));
    Serial.println(F("╠═══════════════════════════════════════════════════════════════╣"));
    Serial.println(F("║ MODE COMMANDS                                                 ║"));
    Serial.println(F("║   mode             - Show current operating mode              ║"));
    Serial.println(F("║   mode <name>      - Set mode (safe/nominal/lowpower/         ║"));
    Serial.println(F("║                      science/emergency)                       ║"));
    Serial.println(F("║   hibernate        - Enter low power mode                     ║"));
    Serial.println(F("╠═══════════════════════════════════════════════════════════════╣"));
    Serial.println(F("║ POWER COMMANDS                                                ║"));
    Serial.println(F("║   power            - Show power status                        ║"));
    Serial.println(F("║   power ch<n> on   - Turn power channel n ON (1-4)            ║"));
    Serial.println(F("║   power ch<n> off  - Turn power channel n OFF (1-4)           ║"));
    Serial.println(F("║   power boot <m>   - Save boot mask m to the BMS (0-15)      ║"));
    Serial.println(F("║   battery          - Show battery status                      ║"));
    Serial.println(F("║   bmsreset         - Reset BMS module                         ║"));
    Serial.println(F("╠═══════════════════════════════════════════════════════════════╣"));
    Serial.println(F("║ TELEMETRY COMMANDS                                            ║"));
    Serial.println(F("║   telem            - Transmit telemetry now                   ║"));
    Serial.println(F("║   telem interval N - Set telemetry interval (ms)              ║"));
    Serial.println(F("║   radio save       - Save channel, power, interval           ║"));
    Serial.println(F("║   telem stats      - Show transmission statistics             ║"));
    Serial.println(F("╠═══════════════════════════════════════════════════════════════╣"));
    Serial.println(F("║ SENSOR COMMANDS                                               ║"));
    Serial.println(F("║   sensors          - Show all sensor readings                 ║"));
    Serial.println(F("║   gps              - Show GPS data                            ║"));
    Serial.println(F("║   imu              - Show IMU data                            ║"));
    Serial.println(F("║   env              - Show environmental data                  ║"));
    Serial.println(F("╠═══════════════════════════════════════════════════════════════╣"));
    Serial.println(F("║ MESH NETWORK COMMANDS                                         ║"));
    Serial.println(F("║   mesh             - Show mesh network status                 ║"));
    Serial.println(F("║   mesh relay on    - Enable packet relay                      ║"));
    Serial.println(F("║   mesh relay off   - Disable packet relay                     ║"));
    Serial.println(F("║   mesh log on      - Enable mesh SD logging                   ║"));
    Serial.println(F("║   mesh log off     - Disable mesh SD logging                  ║"));
    Serial.println(F("║   cubesats         - List detected CubeSats                   ║"));
    Serial.println(F("║   cubesat <serial> - Show data from CubeSat (hex)             ║"));
    Serial.println(F("╠═══════════════════════════════════════════════════════════════╣"));
    Serial.println(F("║ DATA LOGGING COMMANDS                                         ║"));
    Serial.println(F("║   log              - Show data logger status                  ║"));
    Serial.println(F("║   log on           - Enable data logging to SD                ║"));
    Serial.println(F("║   log off          - Disable data logging                     ║"));
    Serial.println(F("║   log interval N   - Set logging interval (ms)                ║"));
    Serial.println(F("║   log flush        - Force flush log to SD card               ║"));
    Serial.println(F("╠═══════════════════════════════════════════════════════════════╣"));
    Serial.println(F("║ DEBUG COMMANDS                                                ║"));
    Serial.println(F("║   debug on/off     - Enable/disable debug output              ║"));
    Serial.println(F("║   memory           - Show memory statistics                   ║"));
    Serial.println(F("║   echo <text>      - Echo text back                           ║"));
    Serial.println(F("║   uart <uart> [on|off] - UART passthrough (gps|satbus)       ║"));
    Serial.println(F("╚═══════════════════════════════════════════════════════════════╝"));
    Serial.println();
}

// ============================================================================
// END OF FILE
// ============================================================================

/*
 * ADDING CUSTOM COMMANDS
 * ======================
 * 
 * You can add custom commands for your payload or experiments:
 * 
 * 1. Create a callback function:
 * 
 *    CommandResult myPayloadCommand(ParsedCommand& cmd) {
 *        CommandResult result;
 *        result.success = true;
 *        
 *        if (cmd.argCount > 0 && strcasecmp(cmd.args[0], "start") == 0) {
 *            // Start payload
 *            startPayload();
 *            strcpy(result.message, "Payload started");
 *        } else if (cmd.argCount > 0 && strcasecmp(cmd.args[0], "stop") == 0) {
 *            // Stop payload
 *            stopPayload();
 *            strcpy(result.message, "Payload stopped");
 *        } else {
 *            // Show payload status
 *            strcpy(result.message, "Payload status: running");
 *        }
 *        
 *        return result;
 *    }
 * 
 * 2. Register the command in setup():
 * 
 *    commands.registerCustomCommand("payload", myPayloadCommand);
 * 
 * 3. Now you can use "payload", "payload start", "payload stop"
 */
