/**
 * @file GS.ino
 * @brief 8601 CubeSat Ground Station receiver
 * @author EdgeFlyte, LLC
 * @copyright Copyright (c) 2025-2026 EdgeFlyte, LLC
 *
 * Version 2.1.0 ("Orbit") comes from version.h.
 *
 * Receives 32-byte #1 / #R radio frames and prints them as [HEX] lines on
 * USB serial (115200 8N1). A host sends one frame with `tx` and 64 hex
 * characters. Frame layout: docs/protocols/telemetry-radio.md.
 *
 * Hardware (8601 / Adafruit Feather M0, SAMD21):
 *   D2 LED1 green, D3 LED2 red, D4 LED3 blue, D5 LED4 yellow, D13 LED5 white
 *   D9 nRF24 CSN, D10 nRF24 CE, D8 piezo, D0/D1 external UART
 *
 * Radio defaults: channel 62, 32-byte payload, RF24_PA_LOW, RF24_1MBPS,
 * address { 0x45, 0x46, 0x45, 0x46, 0x30 }.
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <stdlib.h>
#include <string.h>
#include "version.h"

// SAMD21 watchdog registers (datasheet). Left disabled unless WDT_AVAILABLE is set.
#if defined(__SAMD21__) || defined(ARDUINO_SAMD_ZERO) || defined(ARDUINO_SAMD_MKR1000) || defined(__SAMD21G18A__)
  #define WDT_BASE_ADDR     (0x40001000UL)
  #define WDT_CTRL_REG      (*(volatile uint8_t *)(WDT_BASE_ADDR + 0x00))
  #define WDT_CONFIG_REG    (*(volatile uint8_t *)(WDT_BASE_ADDR + 0x01))
  #define WDT_CLEAR_REG     (*(volatile uint8_t *)(WDT_BASE_ADDR + 0x08))
  #define WDT_CONFIG_PER_11 (0x0Bu)   // 16384 cycles, about 0.5 s at 32 kHz
  #define WDT_CLEAR_KEY     (0xA5u)
#else
  #define WDT_AVAILABLE 0
#endif

#define LED1_PIN          2
#define LED2_PIN          3
#define LED3_PIN          4
#define LED4_PIN          5
#define LED5_PIN          13

#define NRF_CE_PIN        10
#define NRF_CSN_PIN       9
#define PIEZO_PIN         8
#define EXT_UART_RX       0
#define EXT_UART_TX       1

#define RADIO_CHANNEL           62
#define RADIO_PAYLOAD_SIZE      32
#define RADIO_POWER_LEVEL       RF24_PA_LOW
#define RADIO_DATA_RATE         RF24_1MBPS
#define RADIO_DRAIN_MAX_PER_LOOP 8
#define RADIO_REINIT_INTERVAL_MS 2000
#define RADIO_REINIT_LOG_INTERVAL_MS 10000
#define RADIO_STUCK_WINDOW_MS   80
#define RADIO_STUCK_COPIES      3
#define RADIO_FAULTS_BEFORE_RESET 3
#define RADIO_FAULT_WINDOW_MS   5000
#define SERIAL_CMD_MAX_LEN      96

// 12-byte on-wire header: "#1" or "#R", transmitter id, destination id, type.
#define TELEM_HEADER_SIZE       12

const uint8_t RADIO_ADDRESS[6] = { 0x45, 0x46, 0x45, 0x46, 0x30, 0x00 };
uint8_t radioChannel = RADIO_CHANNEL;

#define DEVICE_MODEL "8601"
#define DEVICE_SERIAL_NUMBER "8601-0042"
#define DEVICE_SERIAL_HEX 0x0016

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

uint32_t boot_count = 0;
unsigned long system_start_time = 0;

struct Statistics {
    uint32_t total_packets_received;
    uint32_t valid_packets;
    uint32_t invalid_packets;
    uint32_t packet_counts[16];
    unsigned long last_packet_time;
    unsigned long session_start_time;
} stats;

struct Config {
    bool debug_output;
    bool raw_hex_output;
    uint32_t target_device_id;
    bool piezo_enabled;
    bool led_indicators_enabled;
} config;

unsigned long last_radio_reinit_attempt = 0;
unsigned long last_radio_lost_log = 0;

static uint8_t led_off_pin = 0;
static unsigned long led_off_at = 0;

static uint8_t radioRxFaults = 0;
static unsigned long radioRxFaultWindow = 0;

static char serial_cmd_buf[SERIAL_CMD_MAX_LEN + 1];
static size_t serial_cmd_len = 0;

void initializeRadio();
bool reinitializeRadio();
bool ensureRadio();
uint8_t serviceRadio();
void initializePeripherals();
void processReceivedPacket(uint8_t* packet);
bool validatePacket(uint8_t* packet);
void printPacketHex(uint8_t* packet);
void printStatistics();
void printHelp();
void processSerialCommands();

const char* getFirmwareVersion();
const char* getDeviceSerialNumber();

void handleEdgeFlyteIdentification();
void getChipID(char* buffer);
float getDieTemperature();
float getInternalVoltage();
uint32_t getFreeRAM();
bool getSystemHealthStatus();

void setLEDStatus(bool led1, bool led2, bool led3, bool led4, bool led5);
void indicateError();
void indicateSuccess();
void indicateActivity();
void startupLEDSequence();

void beepPiezo(uint16_t duration, uint16_t frequency = 1000);
void beepSequence(uint8_t count, uint16_t duration = 100);

void watchdogSetup();
void watchdogReset();

static void applyRadioConfig() {
    radio.setChannel(radioChannel);
    radio.setPALevel(RADIO_POWER_LEVEL);
    radio.setDataRate(RADIO_DATA_RATE);
    radio.setPayloadSize(RADIO_PAYLOAD_SIZE);
    radio.setAutoAck(true);
    radio.enableAckPayload();
    radio.setRetries(3, 15);
    uint8_t radioTxAddress[6];
    memcpy(radioTxAddress, RADIO_ADDRESS, sizeof(RADIO_ADDRESS));
    radio.openWritingPipe(radioTxAddress);
    radio.openReadingPipe(1, RADIO_ADDRESS);
    radio.flush_rx();
    radio.startListening();
}

static void flashLEDDeferred(uint8_t led_pin, uint16_t duration_ms) {
    if (led_off_at) digitalWrite(led_off_pin, LOW);
    digitalWrite(led_pin, HIGH);
    led_off_pin = led_pin;
    led_off_at = millis() + duration_ms;
}

static void noteRadioRxFault(const __FlashStringHelper* reason) {
    unsigned long now = millis();
    if (radioRxFaultWindow == 0 || (now - radioRxFaultWindow) > RADIO_FAULT_WINDOW_MS) {
        radioRxFaults = 0;
        radioRxFaultWindow = now;
    }
    radioRxFaults++;
    if (radioRxFaults == 1) {
        Serial.print(F("[RADIO] RX recovered: "));
        Serial.println(reason);
    }
    if (radioRxFaults < RADIO_FAULTS_BEFORE_RESET) return;

    radioRxFaults = 0;
    radioRxFaultWindow = now;
    Serial.println(F("[RADIO] RX still stuck, resetting radio"));
    if (reinitializeRadio()) {
        Serial.println(F("[RADIO] Reinitialized successfully"));
        if (config.led_indicators_enabled) indicateSuccess();
    } else if (config.led_indicators_enabled) {
        indicateError();
    }
}

static const __FlashStringHelper* packetCategoryName(uint8_t category) {
    switch (category) {
        case 0:  return F("System");
        case 1:  return F("Power");
        case 2:  return F("Comm");
        case 3:  return F("Environment");
        case 4:  return F("GPS");
        case 5:  return F("IMU");
        case 6:  return F("Payload");
        case 8:  return F("Command");
        case 9:  return F("Message");
        case 15: return F("Debug");
        default: return F("Other");
    }
}

static uint8_t packetCategory(uint16_t packetType) {
    // 0x8xxx commands share the high nibble with 0x08xx; count both as Command.
    if ((packetType & 0xF000) == 0x8000) return 8;
    return (packetType >> 8) & 0x0F;
}

void setup() {
    system_start_time = millis();
    boot_count++;

    Serial.begin(115200);

    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(LED3_PIN, OUTPUT);
    pinMode(LED4_PIN, OUTPUT);
    pinMode(LED5_PIN, OUTPUT);
    pinMode(PIEZO_PIN, OUTPUT);
    setLEDStatus(false, false, false, false, false);

    config.debug_output = false;
    config.raw_hex_output = true;
    config.target_device_id = 0;
    config.piezo_enabled = true;
    config.led_indicators_enabled = true;

    memset(&stats, 0, sizeof(stats));
    stats.session_start_time = millis();

    initializePeripherals();
    initializeRadio();

    startupLEDSequence();
    if (config.piezo_enabled) {
        beepSequence(3, 150);
    }

    Serial.println(F("=============================================================="));
    Serial.print(F("  8601 GROUND STATION v"));
    Serial.print(getFirmwareVersion());
    Serial.println(F("  (c) EdgeFlyte, LLC"));
    Serial.println(F("=============================================================="));
    Serial.println(F("Educational CubeSat Ground Station Receiver"));
    Serial.print(F("Device Serial: "));
    Serial.println(getDeviceSerialNumber());
    Serial.print(F("Firmware Version: "));
    Serial.println(getFirmwareVersion());
    Serial.print(F("Target transmitter ID: 0x"));
    if (config.target_device_id == 0) {
        Serial.println(F("00000000 (all)"));
    } else {
        Serial.println(config.target_device_id, HEX);
    }
    Serial.println(F("Radio initialized and listening..."));
    Serial.println(F("Type 'help' for available commands"));
    Serial.println();

#if WDT_AVAILABLE
    watchdogSetup();
    Serial.println(F("[WDT] Watchdog enabled"));
#endif
}

void loop() {
#if WDT_AVAILABLE
    watchdogReset();
#endif
    if (led_off_at && (long)(millis() - led_off_at) >= 0) {
        digitalWrite(led_off_pin, LOW);
        led_off_at = 0;
    }

    uint8_t packets_this_loop = serviceRadio();
    processSerialCommands();
    delay(packets_this_loop > 0 ? 0 : 1);
}

void initializeRadio() {
    Serial.println(F("[RADIO] Initializing nRF24L01+ radio..."));

    const uint8_t max_retries = 5;
    uint8_t retries = 0;
    while (!radio.begin()) {
        retries++;
        Serial.println(F("[RADIO] begin() failed, retrying..."));
        if (retries >= max_retries) {
            Serial.println(F("[ERROR] Radio initialization failed!"));
            Serial.println(F("Check wiring and connections:"));
            Serial.println(F("  CE Pin: D10"));
            Serial.println(F("  CSN Pin: D9"));
            Serial.println(F("  MOSI: D11"));
            Serial.println(F("  MISO: D12"));
            Serial.println(F("  SCK: D13"));
            while (1) {
                indicateError();
                delay(500);
            }
        }
        delay(200);
    }

    applyRadioConfig();

    Serial.println(F("[RADIO] Configuration:"));
    Serial.print(F("  Channel: "));
    Serial.println(radioChannel);
    Serial.print(F("  Payload Size: "));
    Serial.println(RADIO_PAYLOAD_SIZE);
    Serial.print(F("  Power Level: "));
    Serial.println(RADIO_POWER_LEVEL);
    Serial.print(F("  Data Rate: "));
    Serial.println(RADIO_DATA_RATE);
    Serial.print(F("  Address: 0x"));
    for (int i = 0; i < 5; i++) {
        if (RADIO_ADDRESS[i] < 0x10) Serial.print(F("0"));
        Serial.print(RADIO_ADDRESS[i], HEX);
    }
    Serial.println();
    Serial.println(F("[RADIO] Radio initialized successfully!"));
}

bool reinitializeRadio() {
    unsigned long now = millis();
    if (now - last_radio_reinit_attempt < RADIO_REINIT_INTERVAL_MS) {
        return false;
    }
    last_radio_reinit_attempt = now;

    radio.stopListening();
    digitalWrite(NRF_CE_PIN, LOW);
    delay(2);
    radio.powerDown();
    delay(5);
    radio.powerUp();
    delay(5);

    if (!radio.begin()) {
        return false;
    }
    applyRadioConfig();
    return true;
}

bool ensureRadio() {
    if (radio.isChipConnected()) {
        return true;
    }
    unsigned long now = millis();
    if (now - last_radio_lost_log >= RADIO_REINIT_LOG_INTERVAL_MS) {
        Serial.println(F("[RADIO] Chip disconnected, reinitializing..."));
        last_radio_lost_log = now;
    }
    if (reinitializeRadio()) {
        Serial.println(F("[RADIO] Reinitialized successfully"));
        if (config.led_indicators_enabled) {
            indicateSuccess();
        }
        return true;
    }
    if (config.led_indicators_enabled) {
        indicateError();
    }
    return false;
}

uint8_t serviceRadio() {
    if (!ensureRadio()) return 0;

    static uint8_t lastPacket[RADIO_PAYLOAD_SIZE];
    static bool haveLast = false;
    static uint8_t repeatCount = 0;
    static unsigned long lastRepeatMs = 0;

    uint8_t delivered = 0;
    while (radio.available() && delivered < RADIO_DRAIN_MAX_PER_LOOP) {
        uint8_t width = radio.getDynamicPayloadSize();
        if (width == 0 || width > RADIO_PAYLOAD_SIZE) {
            if (width > RADIO_PAYLOAD_SIZE) radio.flush_rx();
            noteRadioRxFault(F("bad payload width"));
            break;
        }

        uint8_t packet[RADIO_PAYLOAD_SIZE];
        memset(packet, 0, sizeof(packet));
        radio.read(packet, width);

        unsigned long now = millis();
        bool same = haveLast && memcmp(packet, lastPacket, RADIO_PAYLOAD_SIZE) == 0;
        if (same && (now - lastRepeatMs) < RADIO_STUCK_WINDOW_MS) {
            repeatCount++;
            lastRepeatMs = now;
            if (repeatCount >= RADIO_STUCK_COPIES) {
                radio.flush_rx();
                repeatCount = 0;
                noteRadioRxFault(F("stuck packet"));
                break;
            }
            continue;
        }

        memcpy(lastPacket, packet, RADIO_PAYLOAD_SIZE);
        haveLast = true;
        repeatCount = 0;
        lastRepeatMs = now;
        stats.total_packets_received++;
        stats.last_packet_time = now;
        delivered++;
        if (config.led_indicators_enabled) indicateActivity();
        processReceivedPacket(packet);
    }
    return delivered;
}

void initializePeripherals() {
    Serial.println(F("[I2C] Initializing..."));
    Wire.begin();
    Serial.println(F("[I2C] Ready"));
}

void processReceivedPacket(uint8_t* packet) {
    if (packet == nullptr) return;

    if (config.raw_hex_output) {
        printPacketHex(packet);
    }

    if (validatePacket(packet)) {
        stats.valid_packets++;
        if (config.led_indicators_enabled) {
            indicateSuccess();
        }
        uint16_t packetType = ((uint16_t)packet[10] << 8) | packet[11];
        uint8_t category = packetCategory(packetType);
        if (category < 16) {
            stats.packet_counts[category]++;
        }
    } else {
        stats.invalid_packets++;
        if (config.led_indicators_enabled) {
            indicateError();
        }
        if (config.debug_output) {
            Serial.println(F("[ERROR] Invalid packet received"));
            printPacketHex(packet);
        }
    }
}

bool validatePacket(uint8_t* packet) {
    if (packet == nullptr) return false;

    const bool origin = (packet[0] == '#' && packet[1] == '1');
    const bool relay = (packet[0] == '#' && packet[1] == 'R');
    if (!origin && !relay) return false;

    uint32_t transmitterId =
        ((uint32_t)packet[2] << 24) |
        ((uint32_t)packet[3] << 16) |
        ((uint32_t)packet[4] << 8) |
        (uint32_t)packet[5];
    if (transmitterId == 0) return false;

    // Bytes 6..9 (destination) and 10..11 (type) complete the 12-byte header.
    // Destination 0 is broadcast. Any packet type is accepted and counted.
    if (config.target_device_id != 0 && transmitterId != config.target_device_id) {
        if (config.debug_output) {
            Serial.print(F("[DEBUG] Transmitter ID mismatch: 0x"));
            Serial.println(transmitterId, HEX);
        }
        return false;
    }
    return true;
}

#if WDT_AVAILABLE
void watchdogSetup() {
    WDT_CTRL_REG = 0;
    WDT_CONFIG_REG = WDT_CONFIG_PER_11;
    WDT_CTRL_REG = WDT_CTRL_ENABLE;
}

void watchdogReset() {
    WDT_CLEAR_REG = WDT_CLEAR_KEY;
}
#else
void watchdogSetup() { (void)0; }
void watchdogReset() { (void)0; }
#endif

void printPacketHex(uint8_t* packet) {
    Serial.print(F("[HEX] "));
    for (int i = 0; i < RADIO_PAYLOAD_SIZE; i++) {
        if (packet[i] < 0x10) Serial.print(F("0"));
        Serial.print(packet[i], HEX);
    }
    Serial.println();
}

void printStatistics() {
    unsigned long session_time = (millis() - stats.session_start_time) / 1000;
    unsigned long time_since_last = stats.last_packet_time ?
                                   (millis() - stats.last_packet_time) / 1000 : 0;

    Serial.println(F("=============================================================="));
    Serial.println(F("  RECEPTION STATISTICS"));
    Serial.println(F("=============================================================="));
    Serial.print(F("  Session Time: "));
    Serial.print(session_time);
    Serial.println(F(" seconds"));
    Serial.print(F("  Total Packets: "));
    Serial.println(stats.total_packets_received);
    Serial.print(F("  Valid Packets: "));
    Serial.println(stats.valid_packets);
    Serial.print(F("  Invalid Packets: "));
    Serial.println(stats.invalid_packets);

    if (stats.total_packets_received > 0) {
        float success_rate = (float)stats.valid_packets / stats.total_packets_received * 100.0;
        Serial.print(F("  Success Rate: "));
        Serial.print(success_rate, 1);
        Serial.println(F("%"));
    }

    if (stats.last_packet_time) {
        Serial.print(F("  Last Packet: "));
        Serial.print(time_since_last);
        Serial.println(F(" seconds ago"));
    }

    Serial.println(F("  Packet categories:"));
    for (int i = 0; i < 16; i++) {
        if (stats.packet_counts[i] == 0) continue;
        Serial.print(F("    "));
        Serial.print(packetCategoryName((uint8_t)i));
        Serial.print(F(" ("));
        if (i == 8) {
            Serial.print(F("0x8xxx"));
        } else {
            Serial.print(F("0x0"));
            Serial.print(i, HEX);
            Serial.print(F("xx"));
        }
        Serial.print(F("): "));
        Serial.println(stats.packet_counts[i]);
    }
    Serial.println(F("=============================================================="));
}

void printHelp() {
    Serial.println(F("=============================================================="));
    Serial.println(F("  GROUND STATION COMMANDS"));
    Serial.println(F("=============================================================="));
    Serial.println(F("  help, h         Show this help menu"));
    Serial.println(F("  stats, s        Show reception statistics"));
    Serial.println(F("  debug, d        Toggle debug output"));
    Serial.println(F("  hex             Toggle raw hex packet display"));
    Serial.println(F("  device <ID>     Target transmitter ID, hex (0 = all)"));
    Serial.println(F("  reset, r        Reset statistics"));
    Serial.println(F("  radio           Show radio configuration"));
    Serial.println(F("  radio reset     Reinitialize the radio"));
    Serial.println(F("  piezo           Toggle the piezo buzzer"));
    Serial.println(F("  leds            Toggle LED indicators"));
    Serial.println(F("  version, v      Show firmware version"));
    Serial.println(F("  serial          Show device serial number"));
    Serial.println(F("  tx <64 hex>     Send one 32-byte radio payload"));
    Serial.println(F("  channel <0-125> Set this ground station's channel"));
    Serial.println(F("  #?EF?#          EdgeFlyte identification response"));
    Serial.println(F("=============================================================="));
}

void processSerialCommands() {
    if (!Serial.available()) return;
    int c = Serial.read();
    if (c == '\n' || c == '\r' || c < 0) {
        if (serial_cmd_len == 0) return;
        serial_cmd_buf[serial_cmd_len] = '\0';
        String command = String(serial_cmd_buf);
        serial_cmd_len = 0;
        command.trim();

        if (command.length() == 0) return;
        command.replace("\r", "");

        if (command == "#?EF?#") {
            handleEdgeFlyteIdentification();
            return;
        }

        if (command.startsWith("tx ") || command.startsWith("TX ")) {
            String hex = command.substring(3);
            hex.trim();
            if (hex.length() != 64) {
                Serial.println(F("TX ERR"));
                return;
            }
            uint8_t buf[32];
            for (uint8_t i = 0; i < 32; i++) {
                char pair[3] = { hex.charAt(i * 2), hex.charAt(i * 2 + 1), 0 };
                char* end = nullptr;
                long v = strtol(pair, &end, 16);
                if (end == pair || v < 0 || v > 255) {
                    Serial.println(F("TX ERR"));
                    return;
                }
                buf[i] = (uint8_t)v;
            }
            const bool broadcast = (buf[6] == 0 && buf[7] == 0 && buf[8] == 0 && buf[9] == 0);
            radio.stopListening();
            delay(1);
            // Unicast waits for the nRF24 hardware ACK. Broadcast is multicast.
            // TX OK is that result only. The MCB sends 0x0008 separately.
            const bool ok = radio.write(buf, RADIO_PAYLOAD_SIZE, broadcast);
            radio.startListening();
            delay(2);
            Serial.println(ok ? F("TX OK") : F("TX ERR"));
            return;
        }

        command.toLowerCase();

        if (command == "help" || command == "h") {
            printHelp();
        }
        else if (command == "stats" || command == "s") {
            printStatistics();
        }
        else if (command == "debug" || command == "d") {
            config.debug_output = !config.debug_output;
            Serial.print(F("[CONFIG] Debug output: "));
            Serial.println(config.debug_output ? F("ON") : F("OFF"));
        }
        else if (command == "hex") {
            config.raw_hex_output = !config.raw_hex_output;
            Serial.print(F("[CONFIG] Raw hex output: "));
            Serial.println(config.raw_hex_output ? F("ON") : F("OFF"));
        }
        else if (command.startsWith("device ")) {
            String deviceStr = command.substring(7);
            deviceStr.trim();
            char* end = nullptr;
            unsigned long id = strtoul(deviceStr.c_str(), &end, 16);
            if (deviceStr.length() == 0 || end == deviceStr.c_str()) {
                Serial.println(F("[ERROR] Usage: device <hex id> (0 = all)"));
                return;
            }
            config.target_device_id = (uint32_t)id;
            Serial.print(F("[CONFIG] Target transmitter ID: 0x"));
            Serial.println(config.target_device_id, HEX);
        }
        else if (command == "reset" || command == "r") {
            memset(&stats, 0, sizeof(stats));
            stats.session_start_time = millis();
            Serial.println(F("[CONFIG] Statistics reset"));
        }
        else if (command.startsWith("channel ")) {
            String numeric = command.substring(8);
            numeric.trim();
            long ch = numeric.toInt();
            if (numeric.length() == 0 || ch < 0 || ch > 125) {
                Serial.println(F("[ERROR] Channel must be 0-125"));
                return;
            }
            radioChannel = (uint8_t)ch;
            radio.stopListening();
            radio.setChannel(radioChannel);
            radio.startListening();
            Serial.print(F("[RADIO] Channel set to "));
            Serial.println(radioChannel);
        }
        else if (command == "radio") {
            bool connected = radio.isChipConnected();
            Serial.println(F("[RADIO] Configuration:"));
            Serial.print(F("  Channel: "));
            Serial.println(radioChannel);
            Serial.print(F("  Payload Size: "));
            Serial.println(RADIO_PAYLOAD_SIZE);
            Serial.print(F("  Power Level: "));
            Serial.println(RADIO_POWER_LEVEL);
            Serial.print(F("  Data Rate: "));
            Serial.println(RADIO_DATA_RATE);
            Serial.print(F("  Chip Connected: "));
            Serial.println(connected ? F("Yes") : F("No"));
            if (connected) {
                Serial.print(F("  Available: "));
                Serial.println(radio.available() ? F("Yes") : F("No"));
            }
            Serial.println(F("  (Use 'radio reset' to reinitialize)"));
        }
        else if (command == "radio reset") {
            if (reinitializeRadio()) {
                Serial.println(F("[RADIO] Reinitialized successfully"));
            } else {
                Serial.println(F("[RADIO] Reinit failed or rate-limited; try again in a few seconds"));
            }
        }
        else if (command == "piezo") {
            config.piezo_enabled = !config.piezo_enabled;
            Serial.print(F("[CONFIG] Piezo buzzer: "));
            Serial.println(config.piezo_enabled ? F("ON") : F("OFF"));
            if (config.piezo_enabled) {
                beepSequence(2, 100);
            }
        }
        else if (command == "leds") {
            config.led_indicators_enabled = !config.led_indicators_enabled;
            Serial.print(F("[CONFIG] LED indicators: "));
            Serial.println(config.led_indicators_enabled ? F("ON") : F("OFF"));
            if (config.led_indicators_enabled) {
                startupLEDSequence();
            } else {
                setLEDStatus(false, false, false, false, false);
            }
        }
        else if (command == "version" || command == "v") {
            Serial.print(F("[INFO] Firmware Version: "));
            Serial.println(getFirmwareVersion());
        }
        else if (command == "serial") {
            Serial.print(F("[INFO] Device Serial: "));
            Serial.println(getDeviceSerialNumber());
        }
        else if (command.length() > 0) {
            Serial.print(F("[ERROR] Unknown command: '"));
            Serial.print(command);
            Serial.println(F("'"));
            Serial.println(F("Type 'help' for available commands"));
        }
    } else if (serial_cmd_len < SERIAL_CMD_MAX_LEN && c > 0) {
        serial_cmd_buf[serial_cmd_len++] = (char)c;
    }
}

const char* getFirmwareVersion() {
    return FIRMWARE_VERSION;
}

const char* getDeviceSerialNumber() {
    return DEVICE_SERIAL_NUMBER;
}

// Response: #!EF!:MODEL,SERIAL,FW_VERSION,STATUS,UPTIME,BOOT_COUNT,TEMP,VOLT,FREE_RAM,CHIP_ID#
void handleEdgeFlyteIdentification() {
    char chip_id[33];
    getChipID(chip_id);

    float temperature = getDieTemperature();
    float voltage = getInternalVoltage();
    uint32_t free_ram = getFreeRAM();
    uint32_t uptime = (millis() - system_start_time) / 1000;
    bool health_ok = getSystemHealthStatus();

    Serial.print(F("#!EF!:"));
    Serial.print(DEVICE_MODEL);
    Serial.print(F(","));

    if (DEVICE_SERIAL_HEX < 0x1000) Serial.print(F("0"));
    if (DEVICE_SERIAL_HEX < 0x100) Serial.print(F("0"));
    if (DEVICE_SERIAL_HEX < 0x10) Serial.print(F("0"));
    Serial.print(DEVICE_SERIAL_HEX, HEX);
    Serial.print(F(","));

    Serial.print(FIRMWARE_VERSION_INT);
    Serial.print(F(","));
    Serial.print(health_ok ? F("OK") : F("ERR"));
    Serial.print(F(","));
    Serial.print(uptime);
    Serial.print(F(","));
    Serial.print(boot_count);
    Serial.print(F(","));
    Serial.print(temperature, 1);
    Serial.print(F(","));
    Serial.print(voltage, 2);
    Serial.print(F(","));
    Serial.print(free_ram);
    Serial.print(F(","));
    Serial.print(chip_id);
    Serial.println(F("#"));
}

// SAMD21 unique ID words at 0x0080A00C, 0x0080A040, 0x0080A044, 0x0080A048.
void getChipID(char* buffer) {
    volatile uint32_t* id0 = (volatile uint32_t*)0x0080A00C;
    volatile uint32_t* id1 = (volatile uint32_t*)0x0080A040;
    volatile uint32_t* id2 = (volatile uint32_t*)0x0080A044;
    volatile uint32_t* id3 = (volatile uint32_t*)0x0080A048;
    sprintf(buffer, "%08lX%08lX%08lX%08lX", *id0, *id1, *id2, *id3);
}

// Factory temperature calibration lives in the NVM software calibration area.
float getDieTemperature() {
    uint32_t saved_inputctrl = ADC->INPUTCTRL.reg;
    uint32_t saved_ctrlb = ADC->CTRLB.reg;
    uint32_t saved_sampctrl = ADC->SAMPCTRL.reg;

    ADC->CTRLA.bit.ENABLE = 0;
    while (ADC->STATUS.bit.SYNCBUSY);

    ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_TEMP;
    ADC->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT | ADC_CTRLB_PRESCALER_DIV512;
    ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(63);

    ADC->CTRLA.bit.ENABLE = 1;
    while (ADC->STATUS.bit.SYNCBUSY);

    ADC->SWTRIG.bit.START = 1;
    while (ADC->INTFLAG.bit.RESRDY == 0);

    uint16_t adc_value = ADC->RESULT.reg;
    ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

    ADC->CTRLA.bit.ENABLE = 0;
    while (ADC->STATUS.bit.SYNCBUSY);

    ADC->INPUTCTRL.reg = saved_inputctrl;
    ADC->CTRLB.reg = saved_ctrlb;
    ADC->SAMPCTRL.reg = saved_sampctrl;

    ADC->CTRLA.bit.ENABLE = 1;
    while (ADC->STATUS.bit.SYNCBUSY);

    uint32_t* temp_log_ptr = (uint32_t*)0x00806030;
    uint8_t room_temp_val = (*temp_log_ptr & 0xFF);
    uint8_t hot_temp_val = ((*temp_log_ptr >> 20) & 0xFF);
    uint16_t room_adc_val = ((*(temp_log_ptr + 1) >> 1) & 0xFFF);
    uint16_t hot_adc_val = ((*(temp_log_ptr + 1) >> 13) & 0xFFF);

    if (room_temp_val == 0xFF || hot_temp_val == 0xFF || hot_temp_val == 0) {
        return (adc_value - 400) * 0.1;
    }
    int32_t denom = (int32_t)hot_adc_val - (int32_t)room_adc_val;
    if (denom == 0) {
        return (adc_value - 400) * 0.1;
    }

    float room_temp = room_temp_val;
    float hot_temp = 125.0 - ((125.0 - room_temp) / (float)hot_temp_val) * (float)(hot_adc_val - adc_value) / (float)denom;

    if (hot_temp < -40.0) hot_temp = -40.0;
    if (hot_temp > 125.0) hot_temp = 125.0;
    return hot_temp;
}

// ADC reads VCC/4 against the 1 V reference, then scales back to VCC.
float getInternalVoltage() {
    uint32_t saved_inputctrl = ADC->INPUTCTRL.reg;
    uint32_t saved_ctrlb = ADC->CTRLB.reg;

    ADC->CTRLA.bit.ENABLE = 0;
    while (ADC->STATUS.bit.SYNCBUSY);

    ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC;
    ADC->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT | ADC_CTRLB_PRESCALER_DIV512;

    ADC->CTRLA.bit.ENABLE = 1;
    while (ADC->STATUS.bit.SYNCBUSY);

    ADC->SWTRIG.bit.START = 1;
    while (ADC->INTFLAG.bit.RESRDY == 0);

    uint16_t adc_value = ADC->RESULT.reg;
    ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

    ADC->CTRLA.bit.ENABLE = 0;
    while (ADC->STATUS.bit.SYNCBUSY);

    ADC->INPUTCTRL.reg = saved_inputctrl;
    ADC->CTRLB.reg = saved_ctrlb;

    ADC->CTRLA.bit.ENABLE = 1;
    while (ADC->STATUS.bit.SYNCBUSY);

    return (adc_value / 4095.0) * 4.0;
}

// Largest malloc that succeeds, to 64-byte precision. SAMD21 SRAM is 32 KB.
uint32_t getFreeRAM() {
    uint32_t low = 0;
    uint32_t high = 32768;

    while (high - low > 64) {
        uint32_t mid = (low + high) / 2;
        void* ptr = malloc(mid);
        if (ptr != NULL) {
            free(ptr);
            low = mid;
        } else {
            high = mid;
        }
    }
    return low;
}

bool getSystemHealthStatus() {
    if (!radio.isChipConnected()) return false;

    float temp = getDieTemperature();
    if (temp < -20.0 || temp > 85.0) return false;

    float voltage = getInternalVoltage();
    if (voltage < 3.0 || voltage > 3.6) return false;

    return true;
}

void setLEDStatus(bool led1, bool led2, bool led3, bool led4, bool led5) {
    digitalWrite(LED1_PIN, led1 ? HIGH : LOW);
    digitalWrite(LED2_PIN, led2 ? HIGH : LOW);
    digitalWrite(LED3_PIN, led3 ? HIGH : LOW);
    digitalWrite(LED4_PIN, led4 ? HIGH : LOW);
    digitalWrite(LED5_PIN, led5 ? HIGH : LOW);
}

void indicateError() {
    flashLEDDeferred(LED2_PIN, 50);
}

void indicateSuccess() {
    flashLEDDeferred(LED1_PIN, 30);
}

void indicateActivity() {
    flashLEDDeferred(LED5_PIN, 20);
}

void startupLEDSequence() {
    for (int i = 0; i < 2; i++) {
        digitalWrite(LED1_PIN, HIGH);
        delay(150);
        digitalWrite(LED1_PIN, LOW);
        digitalWrite(LED2_PIN, HIGH);
        delay(150);
        digitalWrite(LED2_PIN, LOW);
        digitalWrite(LED3_PIN, HIGH);
        delay(150);
        digitalWrite(LED3_PIN, LOW);
        digitalWrite(LED4_PIN, HIGH);
        delay(150);
        digitalWrite(LED4_PIN, LOW);
        digitalWrite(LED5_PIN, HIGH);
        delay(150);
        digitalWrite(LED5_PIN, LOW);
        delay(100);
    }
}

void beepPiezo(uint16_t duration, uint16_t frequency) {
    if (!config.piezo_enabled) return;

    uint16_t half_period = 1000000 / (2 * frequency);
    uint16_t cycles = duration * 1000 / (2 * half_period);

    for (uint16_t i = 0; i < cycles; i++) {
        digitalWrite(PIEZO_PIN, HIGH);
        delayMicroseconds(half_period);
        digitalWrite(PIEZO_PIN, LOW);
        delayMicroseconds(half_period);
    }
}

void beepSequence(uint8_t count, uint16_t duration) {
    for (uint8_t i = 0; i < count; i++) {
        beepPiezo(duration, 1000);
        delay(duration);
    }
}
