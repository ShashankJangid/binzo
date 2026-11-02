/**
 * ============================================================================
 * Project: M5Core2 Mecanum Trash Can Robot (4x4 Omnidirectional Swerve Drive)
 * Hardware:
 *   - M5Stack Core2 (ESP32)
 *   - M5Stack 4-Channel Encoder Motor Module (STM32 + BL5617 H-Bridge, I2C 0x24)
 *   - RadioMaster Pocket (USB HID to Laptop) -> UDP over WiFi
 *
 * WiFi Network:
 *   - SSID: 0xMakers_Dev
 *   - Pass: Welc0me@123
 *
 * Control Protocol:
 *   - Listens on UDP Port 4210 for drive packets:
 *     "TRASH,<seq>,<arm>,<vx>,<vy>,<yaw>,<thr>,<m0>,<m1>,<m2>,<m3>\n"
 *   - Broadcasts discovery beacon on UDP Port 4211 every 1 second
 *   - Streams debug & telemetry logs to Laptop on UDP Port 4212:
 *     "DBG,<millis>,<vbat>,<arm>,[TAG] message\n"
 * ============================================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <M5Unified.h>
#include <esp_system.h>
#include "M5Module4EncoderMotor.h"

// ----------------------------------------------------------------------------
// WIFI CONFIGURATION
// ----------------------------------------------------------------------------
const char* WIFI_SSID     = "0xMakers_Dev";
const char* WIFI_PASSWORD = "Welc0me@123";

// ----------------------------------------------------------------------------
// NETWORK PORTS & SETTINGS
// ----------------------------------------------------------------------------
const uint16_t UDP_CONTROL_PORT   = 4210;   // Incoming drive packets
const uint16_t UDP_DISCOVERY_PORT = 4211;   // Outgoing discovery beacon
const uint16_t UDP_LOG_PORT       = 4212;   // Outgoing serial / error logs to laptop
const unsigned long FAILSAFE_TIMEOUT_MS = 1500; // 1.5s timeout (prevents stutter from WiFi latency)
const unsigned long BEACON_INTERVAL_MS  = 1000; // Broadcast beacon every 1s
const unsigned long LOG_INTERVAL_MS     = 400;  // Send UDP error/telemetry log to laptop at 2.5Hz
const unsigned long UI_REFRESH_MS       = 200;  // 5 Hz UI update (prevents SPI display from starving motors)

IPAddress laptopHostIp(0, 0, 0, 0);
unsigned long lastLogTxTime = 0;

// RTC memory to track boot count across resets & brownouts
RTC_NOINIT_ATTR uint32_t bootCount;

// ----------------------------------------------------------------------------
// MOTOR MODULE & DRIVETRAIN SETTINGS
// ----------------------------------------------------------------------------
// Motor channel assignments (0 to 3)
const uint8_t CH_FRONT_LEFT  = 0;
const uint8_t CH_FRONT_RIGHT = 1;
const uint8_t CH_REAR_LEFT   = 2;
const uint8_t CH_REAR_RIGHT  = 3;

// Direction multipliers: change to -1 if any wheel turns the wrong direction
const int8_t INV_FL = 1;
const int8_t INV_FR = 1;
const int8_t INV_RL = 1;
const int8_t INV_RR = 1;

// ----------------------------------------------------------------------------
// GLOBAL OBJECTS & STATE
// ----------------------------------------------------------------------------
M5Module4EncoderMotor motorDriver;
WiFiUDP udpControl;
WiFiUDP udpDiscovery;

unsigned long lastPacketRxTime = 0;
unsigned long lastBeaconTxTime = 0;
unsigned long lastUiRefreshTime = 0;
uint32_t packetCount = 0;
uint32_t lastPacketCount = 0;
float currentPacketRate = 0.0f;

// Robot telemetry state
bool isEmergencyStop = false;
bool isArmed = false;
int16_t inVx = 0;
int16_t inVy = 0;
int16_t inYaw = 0;
int16_t inThr = 0;
int8_t targetMotorPWM[4] = {0, 0, 0, 0};
int8_t actualMotorPWM[4] = {0, 0, 0, 0};

bool motorModuleFound = false;

// UI Colors (RGB565)
#define COLOR_BG          0x0842  // Very dark slate grey
#define COLOR_PANEL       0x18E3  // Dark card background
#define COLOR_ACCENT      0x04FF  // Cyan
#define COLOR_GREEN_ARMED 0x07E0  // Bright green
#define COLOR_RED_ESTOP   0xF800  // Bright red
#define COLOR_ORANGE_FAIL 0xFD20  // Orange
#define COLOR_TEXT_DIM    0x9CD3  // Light grey

// ----------------------------------------------------------------------------
// FORWARD DECLARATIONS
// ----------------------------------------------------------------------------
void setupWiFi();
void initMotorDriver();
void processIncomingUDP();
void sendDiscoveryBeacon();
void sendTelemetryUDP(const char* statusMsg);
void logDebugUDP(const char* tag, const char* fmt, ...);
void applyMotorSpeeds(int8_t m0, int8_t m1, int8_t m2, int8_t m3);
void stopAllMotors(const char* reason = "Stop");
void drawStaticUI();
void updateDisplay();
void handleTouch();
const char* getResetReasonStr(esp_reset_reason_t reason);

// ----------------------------------------------------------------------------
// RESET REASON HELPER
// ----------------------------------------------------------------------------
const char* getResetReasonStr(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:   return "POWERON (Cold boot)";
        case ESP_RST_EXT:       return "EXTERNAL_PIN";
        case ESP_RST_SW:        return "SW_RESET";
        case ESP_RST_PANIC:     return "PANIC_CRASH";
        case ESP_RST_INT_WDT:   return "INT_WATCHDOG";
        case ESP_RST_TASK_WDT:  return "TASK_WATCHDOG";
        case ESP_RST_WDT:       return "OTHER_WATCHDOG";
        case ESP_RST_DEEPSLEEP: return "DEEP_SLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT (Voltage dropped below threshold!)";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

// ----------------------------------------------------------------------------
// SEND UDP DEBUG LOGS TO LAPTOP
// ----------------------------------------------------------------------------
void logDebugUDP(const char* tag, const char* fmt, ...) {
    char textBuf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(textBuf, sizeof(textBuf), fmt, args);
    va_end(args);

    Serial.printf("[%s] %s\n", tag, textBuf);

    if (WiFi.status() != WL_CONNECTED) return;

    char udpPacket[240];
    int vbat = M5.Power.getBatteryVoltage();
    snprintf(udpPacket, sizeof(udpPacket), "DBG,%lu,%d,%d,[%s] %s\n",
             millis(), vbat, isArmed ? 1 : 0, tag, textBuf);

    IPAddress target = (laptopHostIp[0] != 0) ? laptopHostIp : IPAddress(255, 255, 255, 255);
    udpControl.beginPacket(target, UDP_LOG_PORT);
    udpControl.write((const uint8_t*)udpPacket, strlen(udpPacket));
    udpControl.endPacket();
}

// ----------------------------------------------------------------------------
// SETUP
// ----------------------------------------------------------------------------
void setup() {
    auto cfg = M5.config();
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);
    M5.Display.begin();
    M5.Display.setRotation(1); // Landscape
    M5.Display.fillScreen(BLACK);

    M5.Display.setTextDatum(top_center);
    M5.Display.setFont(&fonts::FreeSansBold12pt7b);
    M5.Display.setTextColor(COLOR_ACCENT);
    M5.Display.drawString("M5 MECANUM TRASH CAN", 160, 20);

    M5.Display.setFont(&fonts::FreeSans9pt7b);
    M5.Display.setTextColor(WHITE);
    M5.Display.drawString("Initializing hardware...", 160, 60);

    Serial.println("\n=============================================");
    Serial.println("  M5Core2 4x4 Mecanum Trash Can Robot");
    Serial.println("=============================================");

    bootCount++;
    esp_reset_reason_t rstReason = esp_reset_reason();
    Serial.printf("[SYSTEM] Boot #%u | Reset Reason: %s\n", bootCount, getResetReasonStr(rstReason));

    // Initialize Motor Driver
    initMotorDriver();

    // Connect to WiFi
    setupWiFi();

    // Start UDP Listeners
    udpControl.begin(UDP_CONTROL_PORT);
    udpDiscovery.begin(UDP_DISCOVERY_PORT);
    Serial.printf("[UDP] Listening for control packets on port %d\n", UDP_CONTROL_PORT);

    // Initial boot debug log
    logDebugUDP("BOOT", "Boot #%u, Reason=%s, Vbat=%dmV, BatPct=%d%%", 
                bootCount, getResetReasonStr(rstReason), 
                M5.Power.getBatteryVoltage(), M5.Power.getBatteryLevel());

    drawStaticUI();
    lastPacketRxTime = millis();
}

// ----------------------------------------------------------------------------
// INITIALIZE 4-ENCODER MOTOR MODULE
// ----------------------------------------------------------------------------
void initMotorDriver() {
    Serial.println("[MOTOR] Initializing 4EncoderMotor module on I2C (0x24)...");
    
    // Core2 internal I2C bus on pins 21 (SDA), 22 (SCL)
    uint8_t attempts = 0;
    while (!motorDriver.begin(&Wire1, MODULE_4ENCODERMOTOR_ADDR, 21, 22)) {
        attempts++;
        Serial.printf("[MOTOR] Driver init failed (attempt %d). Retrying...\n", attempts);
        M5.Display.fillRect(0, 80, 320, 30, BLACK);
        M5.Display.setTextColor(COLOR_RED_ESTOP);
        M5.Display.drawString("Motor Module NOT Detected!", 160, 90);
        delay(800);
        if (attempts > 5) {
            Serial.println("[MOTOR] Warning: Motor driver module not responding.");
            break;
        }
    }

    if (attempts <= 5) {
        motorModuleFound = true;
        Serial.println("[MOTOR] Driver initialized successfully!");
        
        // Fast I2C mode: 400kHz + 25ms timeout so I2C stalls never freeze main loop
        Wire1.setClock(400000);
        Wire1.setTimeOut(25);

        // Check firmware & disable soft-start/stop for crisp, instant PWM responsiveness
        uint8_t fw = 0;
        if (motorDriver.getFirewareVersion(&fw)) {
            Serial.printf("[MOTOR] Firmware Version: %d\n", fw);
            if (fw >= 3) {
                for (uint8_t i = 0; i < 4; i++) {
                    motorDriver.setSoftStartAndStop(i, false);
                }
                Serial.println("[MOTOR] Disabled Soft Start/Stop for 1:1 control.");
            }
        }

        // Configure all 4 channels to NORMAL_MODE (direct PWM/duty speed control)
        for (uint8_t i = 0; i < 4; i++) {
            motorDriver.setMode(i, NORMAL_MODE);
            motorDriver.setMotorSpeed(i, 0);
        }
    }
}

// ----------------------------------------------------------------------------
// SETUP WIFI
// ----------------------------------------------------------------------------
void setupWiFi() {
    M5.Display.fillRect(0, 120, 320, 100, BLACK);
    M5.Display.setTextColor(WHITE);
    M5.Display.drawString(String("Connecting to: ") + WIFI_SSID, 160, 130);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int dots = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(400);
        Serial.print(".");
        dots++;
        String dotStr = "";
        for (int i = 0; i < (dots % 5); i++) dotStr += ".";
        M5.Display.fillRect(0, 160, 320, 30, BLACK);
        M5.Display.drawString(String("Connecting") + dotStr, 160, 160);

        if (dots > 30) {
            Serial.println("\n[WiFi] Connection taking long. Retrying begin...");
            WiFi.disconnect();
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            dots = 0;
        }
    }

    // CRITICAL: Disable WiFi modem sleep to eliminate latency jitter (<2ms)
    WiFi.setSleep(false);

    Serial.println("\n[WiFi] Connected successfully!");
    Serial.printf("[WiFi] IP Address: %s | RSSI: %d dBm\n", 
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());

    M5.Display.fillRect(0, 120, 320, 100, BLACK);
    M5.Display.setTextColor(COLOR_GREEN_ARMED);
    M5.Display.drawString("WiFi Connected!", 160, 130);
    M5.Display.setTextColor(WHITE);
    M5.Display.drawString(String("IP: ") + WiFi.localIP().toString(), 160, 160);
    delay(1000);
}

// ----------------------------------------------------------------------------
// MAIN LOOP
// ----------------------------------------------------------------------------
void loop() {
    M5.update();
    unsigned long now = millis();
    unsigned long t0_loop = millis();

    // 1. Process touch input on M5Core2 screen (Emergency Stop button)
    unsigned long t_touch_start = micros();
    handleTouch();
    unsigned long dt_touch = micros() - t_touch_start;

    // 2. Receive and process UDP control packets (drains queue for latest packet)
    unsigned long t_udp_start = micros();
    processIncomingUDP();
    unsigned long dt_udp = micros() - t_udp_start;

    // 3. Failsafe Check: If armed but no packet received within timeout, STOP!
    if (!isEmergencyStop) {
        if (now - lastPacketRxTime > FAILSAFE_TIMEOUT_MS) {
            if (isArmed) {
                isArmed = false;
                logDebugUDP("FAILSAFE", "TIMEOUT! No packet received for %lu ms (>%lu ms limit). Disarming & killing motors.",
                            now - lastPacketRxTime, FAILSAFE_TIMEOUT_MS);
                stopAllMotors("Failsafe Packet Timeout");
                sendTelemetryUDP("[FAILSAFE] Radio packet timeout > 1.5s!");
            }
        }
    } else {
        stopAllMotors("Emergency Stop Active");
    }

    // 4. Send Discovery Beacon every 1 second
    if (now - lastBeaconTxTime >= BEACON_INTERVAL_MS) {
        lastBeaconTxTime = now;
        sendDiscoveryBeacon();

        // Calculate packet rate (Hz)
        currentPacketRate = (float)(packetCount - lastPacketCount) / (BEACON_INTERVAL_MS / 1000.0f);
        lastPacketCount = packetCount;
    }

    // 5. Send UDP Telemetry & Error Log to Laptop (2.5 Hz)
    if (now - lastLogTxTime >= LOG_INTERVAL_MS) {
        lastLogTxTime = now;
        const char* statusMsg = isEmergencyStop ? "[ESTOP] Screen Pressed" : 
                               (isArmed ? (inThr > 0 ? "[OK] Driving" : "[OK] Idle (Thr 0%)") : "[FAILSAFE] Waiting for Signal");
        sendTelemetryUDP(statusMsg);
    }

    // 6. Refresh UI at a calm 5 Hz (200ms) - Never blocks real-time motor control
    unsigned long dt_ui = 0;
    if (now - lastUiRefreshTime >= UI_REFRESH_MS) {
        lastUiRefreshTime = now;
        unsigned long t_ui_start = micros();
        updateDisplay();
        dt_ui = micros() - t_ui_start;
    }

    // Diagnostics: Detect any slow loop iteration (>25ms) that could cause jitter
    unsigned long dt_loop = millis() - t0_loop;
    if (dt_loop > 25) {
        logDebugUDP("SLOW_LOOP", "Loop took %lu ms! (touch=%lu us, udp=%lu us, ui=%lu us)",
                    dt_loop, dt_touch, dt_udp, dt_ui);
    }

    delay(1);
}

// ----------------------------------------------------------------------------
// SEND TELEMETRY & ERROR LOGS OVER UDP TO LAPTOP
// ----------------------------------------------------------------------------
void sendTelemetryUDP(const char* logMsg) {
    if (WiFi.status() != WL_CONNECTED || laptopHostIp[0] == 0) return;

    char logBuf[180];
    int vbat = M5.Power.getBatteryVoltage();
    int batPct = M5.Power.getBatteryLevel();
    unsigned long age = millis() - lastPacketRxTime;

    // Format: M5LOG,<millis>,<vbat>,<batPct>,<armed>,<estop>,<age>,<packetCount>,<m0>,<m1>,<m2>,<m3>,<msg>\n
    snprintf(logBuf, sizeof(logBuf), 
             "M5LOG,%lu,%d,%d,%d,%d,%lu,%u,%d,%d,%d,%d,%s\n",
             millis(), vbat, batPct, isArmed ? 1 : 0, isEmergencyStop ? 1 : 0,
             age, packetCount, 
             actualMotorPWM[0], actualMotorPWM[1], actualMotorPWM[2], actualMotorPWM[3],
             logMsg);

    udpControl.beginPacket(laptopHostIp, UDP_LOG_PORT);
    udpControl.write((const uint8_t*)logBuf, strlen(logBuf));
    udpControl.endPacket();
}

// ----------------------------------------------------------------------------
// HANDLE TOUCH INPUT (E-STOP TOGGLE - HARDENED AGAINST VIBRATION/ESD)
// ----------------------------------------------------------------------------
void handleTouch() {
    static unsigned long lastTouchTime = 0;
    unsigned long now = millis();

    if (M5.Touch.getCount() > 0) {
        auto detail = M5.Touch.getDetail(0);
        
        // Log raw touch event to detect phantom touches from motor EMI / chassis vibrations
        static unsigned long lastTouchLog = 0;
        if (now - lastTouchLog > 250) {
            lastTouchLog = now;
            logDebugUDP("TOUCH_RAW", "Screen touch detected: X=%d, Y=%d (pressed=%d)", 
                        detail.x, detail.y, detail.wasPressed() ? 1 : 0);
        }

        // Only respond if user deliberately presses within the bottom banner (y >= 195)
        if (now - lastTouchTime > 500) {
            if (detail.wasPressed() && detail.y >= 195) {
                lastTouchTime = now;
                isEmergencyStop = !isEmergencyStop;
                if (isEmergencyStop) {
                    stopAllMotors("E-Stop Pressed by User");
                    isArmed = false;
                    M5.Speaker.tone(1000, 200);
                    logDebugUDP("ESTOP", "ACTIVATED by touch on bottom banner (Y=%d)!", detail.y);
                } else {
                    M5.Speaker.tone(2000, 100);
                    logDebugUDP("ESTOP", "RELEASED by touch on bottom banner (Y=%d).", detail.y);
                }
            }
        }
    }
}

// ----------------------------------------------------------------------------
// PROCESS INCOMING UDP CONTROL PACKETS (BURST / QUEUE DRAINING)
// ----------------------------------------------------------------------------
void processIncomingUDP() {
    char rxBuffer[128];
    bool gotPacket = false;

    // Drain all queued packets in buffer and use the latest one
    while (udpControl.parsePacket() > 0) {
        laptopHostIp = udpControl.remoteIP(); // Capture laptop IP while packet is active
        int len = udpControl.read(rxBuffer, sizeof(rxBuffer) - 1);
        if (len > 0) {
            rxBuffer[len] = '\0';
            gotPacket = true;
        }
    }

    if (!gotPacket) return;

    // Expected format: "TRASH,<seq>,<arm>,<vx>,<vy>,<yaw>,<thr>,<m0>,<m1>,<m2>,<m3>"
    if (strncmp(rxBuffer, "TRASH,", 6) == 0) {
        uint32_t seq = 0;
        int arm = 0;
        int vx = 0, vy = 0, yaw = 0, thr = 0;
        int m0 = 0, m1 = 0, m2 = 0, m3 = 0;

        int parsed = sscanf(rxBuffer, "TRASH,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                            &seq, &arm, &vx, &vy, &yaw, &thr, &m0, &m1, &m2, &m3);

        if (parsed == 10) {
            unsigned long now = millis();
            unsigned long gap = now - lastPacketRxTime;
            lastPacketRxTime = now;
            packetCount++;

            // Detect packet interval gaps (>200ms) that could cause jerky control
            if (gap > 200 && gap < FAILSAFE_TIMEOUT_MS) {
                logDebugUDP("PKT_GAP", "Packet interval spike: %lu ms (seq=%u)", gap, seq);
            }

            inVx  = vx;
            inVy  = vy;
            inYaw = yaw;
            inThr = thr;

            if (isEmergencyStop) {
                stopAllMotors("Emergency Stop Active");
                return;
            }

            bool prevArmed = isArmed;
            isArmed = (arm == 1);

            if (isArmed != prevArmed) {
                logDebugUDP("ARM_TRANSITION", "isArmed changed %d -> %d by bridge packet (seq=%u, thr=%d)", 
                            prevArmed ? 1 : 0, isArmed ? 1 : 0, seq, thr);
            }

            if (isArmed) {
                // If throttle is 0 or all sticks are centered, freeze all motors!
                if (thr <= 0 || (thr < 5 && abs(vx) < 5 && abs(vy) < 5 && abs(yaw) < 5)) {
                    stopAllMotors("Throttle Zero / Neutral Sticks");
                } else {
                    static unsigned long lastDriveLog = 0;
                    if (now - lastDriveLog > 1000 || !prevArmed) {
                        lastDriveLog = now;
                        logDebugUDP("DRIVING", "Thr=%d%%, Vx=%d, Vy=%d, Yaw=%d | M=[%d,%d,%d,%d]",
                                    thr, vx, vy, yaw, m0, m1, m2, m3);
                    }
                    targetMotorPWM[0] = constrain(m0 * INV_FL, -127, 127);
                    targetMotorPWM[1] = constrain(m1 * INV_FR, -127, 127);
                    targetMotorPWM[2] = constrain(m2 * INV_RL, -127, 127);
                    targetMotorPWM[3] = constrain(m3 * INV_RR, -127, 127);
                    applyMotorSpeeds(targetMotorPWM[0], targetMotorPWM[1], 
                                     targetMotorPWM[2], targetMotorPWM[3]);
                }
            } else {
                stopAllMotors("Bridge Sent Disarm");
            }
        }
    }
}

// ----------------------------------------------------------------------------
// MOTOR DRIVE HELPERS (CACHED / ZERO-REDUNDANCY I2C)
// ----------------------------------------------------------------------------
void applyMotorSpeeds(int8_t m0, int8_t m1, int8_t m2, int8_t m3) {
    actualMotorPWM[CH_FRONT_LEFT]  = m0;
    actualMotorPWM[CH_FRONT_RIGHT] = m1;
    actualMotorPWM[CH_REAR_LEFT]   = m2;
    actualMotorPWM[CH_REAR_RIGHT]  = m3;

    if (!motorModuleFound) return;

    static int8_t lastSentPWM[4] = {-128, -128, -128, -128};
    static unsigned long lastSentTime = 0;
    bool forceKeepalive = (millis() - lastSentTime > 120);

    bool changed = (m0 != lastSentPWM[0] || m1 != lastSentPWM[1] || 
                    m2 != lastSentPWM[2] || m3 != lastSentPWM[3]);

    if (forceKeepalive || changed) {
        unsigned long t0 = micros();
        motorDriver.setMotorSpeed(CH_FRONT_LEFT, m0);
        motorDriver.setMotorSpeed(CH_FRONT_RIGHT, m1);
        motorDriver.setMotorSpeed(CH_REAR_LEFT, m2);
        motorDriver.setMotorSpeed(CH_REAR_RIGHT, m3);
        unsigned long dt_us = micros() - t0;

        lastSentPWM[0] = m0;
        lastSentPWM[1] = m1;
        lastSentPWM[2] = m2;
        lastSentPWM[3] = m3;
        lastSentTime = millis();

        if (dt_us > 15000) {
            logDebugUDP("SLOW_I2C", "I2C motor writes took %lu us!", dt_us);
        }
    }
}

void stopAllMotors(const char* reason) {
    for (uint8_t i = 0; i < 4; i++) {
        targetMotorPWM[i] = 0;
        actualMotorPWM[i] = 0;
    }
    if (motorModuleFound) {
        for (uint8_t i = 0; i < 4; i++) {
            motorDriver.setMotorSpeed(i, 0);
        }
    }
    static String lastLoggedReason = "";
    if (String(reason) != lastLoggedReason) {
        lastLoggedReason = String(reason);
        logDebugUDP("MOTORS_STOP", "All 4 motors stopped. Reason: %s", reason);
    }
}

// ----------------------------------------------------------------------------
// BROADCAST DISCOVERY BEACON (UDP PORT 4211)
// ----------------------------------------------------------------------------
void sendDiscoveryBeacon() {
    if (WiFi.status() != WL_CONNECTED) return;

    char beaconBuf[128];
    snprintf(beaconBuf, sizeof(beaconBuf), 
             "{\"device\":\"M5_TRASHCAN\",\"ip\":\"%s\",\"port\":%u,\"armed\":%d}",
             WiFi.localIP().toString().c_str(), UDP_CONTROL_PORT, isArmed ? 1 : 0);

    udpDiscovery.beginPacket(IPAddress(255, 255, 255, 255), UDP_DISCOVERY_PORT);
    udpDiscovery.write((const uint8_t*)beaconBuf, strlen(beaconBuf));
    udpDiscovery.endPacket();
}

// ----------------------------------------------------------------------------
// STATIC UI (DRAWN ONCE AT BOOT)
// ----------------------------------------------------------------------------
void drawStaticUI() {
    M5.Display.fillScreen(COLOR_BG);

    // Top Header Background
    M5.Display.fillRect(0, 0, 320, 24, 0x10A2);

    // Drivetrain Card Box
    M5.Display.fillRoundRect(8, 70, 148, 122, 6, COLOR_PANEL);
    M5.Display.drawRoundRect(8, 70, 148, 122, 6, 0x3186);
    M5.Display.setFont(&fonts::Font0);
    M5.Display.setTextDatum(top_center);
    M5.Display.setTextColor(COLOR_TEXT_DIM);
    M5.Display.drawString("SWERVE 2D VECTOR", 82, 75);

    // Motor Channels Card Box
    M5.Display.fillRoundRect(164, 70, 148, 122, 6, COLOR_PANEL);
    M5.Display.drawRoundRect(164, 70, 148, 122, 6, 0x3186);
    M5.Display.drawString("4X ENCODER MOTORS", 238, 75);

    const char* labels[4] = {"FL (0)", "FR (1)", "RL (2)", "RR (3)"};
    M5.Display.setTextDatum(top_left);
    for (int i = 0; i < 4; i++) {
        int y = 92 + (i * 22);
        M5.Display.drawString(labels[i], 174, y);
    }
}

// ----------------------------------------------------------------------------
// DYNAMIC UI UPDATE (CALLED AT 5 HZ)
// ----------------------------------------------------------------------------
void updateDisplay() {
    bool signalAlive = (millis() - lastPacketRxTime <= FAILSAFE_TIMEOUT_MS);

    // 1. TOP STATUS BAR (Only update text numbers)
    M5.Display.fillRect(0, 0, 320, 24, 0x10A2);
    M5.Display.setFont(&fonts::Font0);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextDatum(top_left);

    String netStr = String("IP: ") + WiFi.localIP().toString() + 
                    " (" + String(WiFi.RSSI()) + "dBm)";
    M5.Display.drawString(netStr, 8, 6);

    int bat = M5.Power.getBatteryLevel();
    String rightStr = "Rate: " + String((int)currentPacketRate) + "Hz  BAT:" + String(bat) + "%";
    M5.Display.setTextDatum(top_right);
    M5.Display.drawString(rightStr, 312, 6);

    // 2. ROBOT STATUS BADGE (Only redraw if state changed)
    static uint16_t lastBadgeColor = 0;
    static String lastStatusText = "";
    uint16_t badgeColor = COLOR_PANEL;
    String statusText = "INITIALIZING";

    if (isEmergencyStop) {
        badgeColor = COLOR_RED_ESTOP;
        statusText = "EMERGENCY STOP (TOUCH TO UNLOCK)";
    } else if (!signalAlive) {
        badgeColor = COLOR_ORANGE_FAIL;
        statusText = "WAITING FOR RADIO SIGNAL (FAILSAFE)";
    } else if (isArmed) {
        badgeColor = COLOR_GREEN_ARMED;
        statusText = "ARMED - SWERVE MECANUM ACTIVE";
    } else {
        badgeColor = COLOR_PANEL;
        statusText = "DISARMED / THROTTLE IDLE";
    }

    if (badgeColor != lastBadgeColor || statusText != lastStatusText) {
        lastBadgeColor = badgeColor;
        lastStatusText = statusText;
        M5.Display.fillRoundRect(8, 30, 304, 34, 6, badgeColor);
        M5.Display.setTextDatum(middle_center);
        M5.Display.setFont(&fonts::FreeSansBold9pt7b);
        M5.Display.setTextColor(isArmed ? BLACK : WHITE);
        M5.Display.drawString(statusText, 160, 47);
    }

    // 3. SWERVE DIRECTION VECTOR (Fast Circle Clear)
    int cx = 82;
    int cy = 136;
    int radius = 36;
    M5.Display.fillCircle(cx, cy, radius - 1, COLOR_PANEL);
    M5.Display.drawCircle(cx, cy, radius, COLOR_TEXT_DIM);
    M5.Display.drawFastHLine(cx - radius, cy, radius * 2, 0x3186);
    M5.Display.drawFastVLine(cx, cy - radius, radius * 2, 0x3186);

    int vecX = (inVx * (radius - 3)) / 100;
    int vecY = (-inVy * (radius - 3)) / 100;

    if (abs(inVx) > 5 || abs(inVy) > 5) {
        M5.Display.drawLine(cx, cy, cx + vecX, cy + vecY, COLOR_GREEN_ARMED);
        M5.Display.fillCircle(cx + vecX, cy + vecY, 3, COLOR_GREEN_ARMED);
    } else {
        M5.Display.fillCircle(cx, cy, 2, COLOR_TEXT_DIM);
    }

    // Clear and redraw yaw
    M5.Display.fillRect(45, 178, 75, 12, COLOR_PANEL);
    M5.Display.setFont(&fonts::Font0);
    M5.Display.setTextDatum(top_center);
    M5.Display.setTextColor(COLOR_TEXT_DIM);
    if (abs(inYaw) > 10) {
        M5.Display.drawString(inYaw > 0 ? "ROT >>" : "<< ROT", 82, 178);
    } else {
        M5.Display.drawString("YAW: 0", 82, 178);
    }

    // 4. MOTOR PWM GAUGES (Only clear the 80x8 mini bar area)
    for (int i = 0; i < 4; i++) {
        int barY = 94 + (i * 22);
        int barCenterX = 265;
        int barW = (actualMotorPWM[i] * 38) / 127;

        M5.Display.fillRect(225, barY + 4, 80, 8, BLACK);
        M5.Display.drawFastVLine(barCenterX, barY + 1, 14, COLOR_TEXT_DIM);

        if (barW > 0) {
            M5.Display.fillRect(barCenterX, barY + 4, barW, 8, COLOR_GREEN_ARMED);
        } else if (barW < 0) {
            M5.Display.fillRect(barCenterX + barW, barY + 4, -barW, 8, COLOR_RED_ESTOP);
        }
    }

    // 5. BOTTOM BAR / TOUCH ACTION BUTTON
    static bool lastEStopState = false;
    if (isEmergencyStop != lastEStopState || lastBadgeColor == 0) {
        lastEStopState = isEmergencyStop;
        uint16_t btnColor = isEmergencyStop ? COLOR_GREEN_ARMED : 0x8800;
        M5.Display.fillRoundRect(8, 198, 304, 36, 6, btnColor);
        M5.Display.setTextDatum(middle_center);
        M5.Display.setFont(&fonts::FreeSansBold9pt7b);
        M5.Display.setTextColor(WHITE);
        M5.Display.drawString(isEmergencyStop ? "TAP TO RESUME" : "[ TAP BOTTOM BANNER FOR EMERGENCY STOP ]", 160, 216);
    }
}
