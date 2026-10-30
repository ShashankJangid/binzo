/*
 * BINZO - Smart Dustbin Robot (Mecanum 4-Motor Variant)
 * Hardware: ESP32-WROOM-32 (30-pin DevKit)
 *
 * Capabilities:
 * - 4-Wheel Mecanum drive with 2x L298N dual H-bridges
 * - 2x TCRT5000 IR line follower with boot auto-calibration & PD control
 * - MG90S servo automated lid mechanism
 * - HC-SR04 ultrasonic obstacle avoidance & proximity fallback
 * - Wi-Fi SoftAP ("Binzo-Bin") & ESP-NOW receiver for ESP32-CAM detections
 * - Non-blocking FSM with person detection latch & UART telemetry
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "config.h"

// =============================================================================
// HARDWARE / CORE COMPATIBILITY MACROS
// =============================================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define USE_ESP32_CORE_V3 1
#else
  #define USE_ESP32_CORE_V3 0
#endif

// PWM Channel assignments (for ESP32 Core v2)
enum PwmChannels {
  CH_FL = 0,
  CH_FR = 1,
  CH_RL = 2,
  CH_RR = 3,
  CH_SERVO = 4
};

// =============================================================================
// FINITE STATE MACHINE DEFINITIONS
// =============================================================================
enum RobotState {
  STATE_BOOT = 0,
  STATE_IDLE,
  STATE_FOLLOW,
  STATE_OPEN,
  STATE_HOLD,
  STATE_CLOSE,
  STATE_SEARCH,
  STATE_LOST,
  STATE_OBSTACLE,
  STATE_LOW_BATTERY
};

const char* stateNames[] = {
  "BOOT", "IDLE", "FOLLOW", "OPEN", "HOLD", "CLOSE", 
  "SEARCH", "LOST", "OBSTACLE", "LOW_BAT"
};

RobotState currentState = STATE_BOOT;

// =============================================================================
// TELEMETRY & RUNTIME VARIABLES
// =============================================================================
int lastCmdFL = 0, lastCmdFR = 0, lastCmdRL = 0, lastCmdRR = 0;
bool isRunSwitchActive = false; // GPIO36 pulled low = RUN, high = STOP

// IR Line Sensor calibration
int irLeftMin = 4095, irLeftMax = 0;
int irRightMin = 4095, irRightMax = 0;
int irThresholdLeft = 2000;
int irThresholdRight = 2000;
int rawIRLeft = 0, rawIRRight = 0;
bool onLineLeft = false, onLineRight = false;
float lineError = 0.0f;
float lastLineError = 0.0f;
unsigned long lineLostTimestamp = 0;
int lastKnownLineSide = 0; // -1: Left, +1: Right

// Ultrasonic Sensor
float currentDistanceCm = 999.0f;
unsigned long lastPingTime = 0;
const unsigned long PING_INTERVAL_MS = 60; // 16 Hz ping cycle

// ESP-NOW & Camera Heartbeat
volatile bool humanDetectedCam = false;
volatile unsigned long lastCamHeartbeat = 0;
bool isCamOnline = false;

// Human detection latching
bool personLidLatched = false;
unsigned long stateHoldStartTime = 0;

// Debug Serial Print Timer
unsigned long lastDebugPrintTime = 0;
const unsigned long DEBUG_INTERVAL_MS = 200;

// =============================================================================
// LOW-LEVEL PWM WRITING HELPER
// =============================================================================
void initPwmPin(uint8_t pin, uint8_t channel, uint32_t freq, uint8_t resolution) {
#if USE_ESP32_CORE_V3
  ledcAttach(pin, freq, resolution);
#else
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(pin, channel);
#endif
}

void writePwm(uint8_t pin, uint8_t channel, uint32_t duty) {
#if USE_ESP32_CORE_V3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

// =============================================================================
// MOTOR ACTUATION (L298N & Mecanum Kinematics)
// =============================================================================
void setSingleMotor(int speed, uint8_t in1Pin, uint8_t in2Pin, uint8_t enPin, uint8_t channel) {
  speed = constrain(speed, -255, 255);
  uint8_t absDuty = abs(speed);

  // Apply deadband compensation
  if (absDuty > 0 && absDuty < MIN_PWM) {
    absDuty = MIN_PWM;
  }

  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    writePwm(enPin, channel, absDuty);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    writePwm(enPin, channel, absDuty);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    writePwm(enPin, channel, 0);
  }
}

void setMotors(int fl, int fr, int rl, int rr) {
  // Constrain speeds to allowable max
  fl = constrain(fl, -MAX_PWM, MAX_PWM);
  fr = constrain(fr, -MAX_PWM, MAX_PWM);
  rl = constrain(rl, -MAX_PWM, MAX_PWM);
  rr = constrain(rr, -MAX_PWM, MAX_PWM);

  // Check if commanded speeds changed to stream UART telemetry
  if (fl != lastCmdFL || fr != lastCmdFR || rl != lastCmdRL || rr != lastCmdRR) {
    lastCmdFL = fl;
    lastCmdFR = fr;
    lastCmdRL = rl;
    lastCmdRR = rr;
    Serial.printf("MOTORS FL=%d FR=%d RL=%d RR=%d\n", fl, fr, rl, rr);
  }

  setSingleMotor(fl, PIN_FL_IN1, PIN_FL_IN2, PIN_FL_EN, CH_FL);
  setSingleMotor(fr, PIN_FR_IN1, PIN_FR_IN2, PIN_FR_EN, CH_FR);
  setSingleMotor(rl, PIN_RL_IN1, PIN_RL_IN2, PIN_RL_EN, CH_RL);
  setSingleMotor(rr, PIN_RR_IN1, PIN_RR_IN2, PIN_RR_EN, CH_RR);
}

void stopMotors() {
  setMotors(0, 0, 0, 0);
}

void setMotorRaw(int leftSpeed, int rightSpeed) {
  setMotors(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
}

// Mecanum omnidirectional mapping
// vx: Forward/Backward (-255 to 255)
// vy: Strafe Left/Right (-255 to 255)
// w:  Rotate CW/CCW     (-255 to 255)
void mecanumDrive(int vx, int vy, int w) {
  int fl = vx + vy + w;
  int fr = vx - vy - w;
  int rl = vx - vy + w;
  int rr = vx + vy - w;

  // Scale down proportionately if any wheel exceeds MAX_PWM
  int maxMag = max(max(abs(fl), abs(fr)), max(abs(rl), abs(rr)));
  if (maxMag > MAX_PWM) {
    fl = (fl * MAX_PWM) / maxMag;
    fr = (fr * MAX_PWM) / maxMag;
    rl = (rl * MAX_PWM) / maxMag;
    rr = (rr * MAX_PWM) / maxMag;
  }
  setMotors(fl, fr, rl, rr);
}

// =============================================================================
// SERVO LID ACTUATION (MG90S @ 50Hz)
// =============================================================================
void setLidAngle(int angle) {
  angle = constrain(angle, 0, 180);
  uint32_t pulseUs = map(angle, 0, 180, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  // Convert pulse in microseconds to 14-bit duty count (period = 20,000 us)
  uint32_t duty = (pulseUs * ((1 << SERVO_RES_BITS) - 1)) / 20000;
  writePwm(PIN_SERVO, CH_SERVO, duty);
}

// =============================================================================
// ULTRASONIC SENSOR (Non-blocking Trigger & Reading)
// =============================================================================
void updateUltrasonic() {
  unsigned long now = millis();
  if (now - lastPingTime < PING_INTERVAL_MS) {
    return;
  }
  lastPingTime = now;

  // 10us trigger pulse
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);

  // Read ECHO with a safe 25ms timeout (~4.2m maximum range)
  unsigned long duration = pulseIn(PIN_US_ECHO, HIGH, 25000);
  if (duration == 0) {
    currentDistanceCm = 999.0f; // Out of range or no echo
  } else {
    currentDistanceCm = (duration * 0.0343f) / 2.0f;
  }
}

// =============================================================================
// LINE SENSOR READINGS & CALIBRATION
// =============================================================================
void calibrateLineSensors() {
  Serial.println("[CALIB] Place IR sensors over BLACK line now...");
  digitalWrite(PIN_LED_PWR, HIGH);
  delay(1000);

  // Sample for 2.5 seconds
  unsigned long startTime = millis();
  while (millis() - startTime < 2500) {
    int left = analogRead(PIN_IR_LEFT);
    int right = analogRead(PIN_IR_RIGHT);

    if (left < irLeftMin) irLeftMin = left;
    if (left > irLeftMax) irLeftMax = left;
    if (right < irRightMin) irRightMin = right;
    if (right > irRightMax) irRightMax = right;

    digitalWrite(PIN_LED_PWR, (millis() / 100) % 2 == 0 ? HIGH : LOW);
    delay(10);
  }

  // Set midpoints as detection thresholds
  irThresholdLeft = (irLeftMin + irLeftMax) / 2;
  irThresholdRight = (irRightMin + irRightMax) / 2;
  digitalWrite(PIN_LED_PWR, HIGH);

  Serial.printf("[CALIB] Left Thresh=%d (Min=%d Max=%d) | Right Thresh=%d (Min=%d Max=%d)\n",
                irThresholdLeft, irLeftMin, irLeftMax,
                irThresholdRight, irRightMin, irRightMax);
}

void readLineSensors() {
  rawIRLeft = analogRead(PIN_IR_LEFT);
  rawIRRight = analogRead(PIN_IR_RIGHT);

  // Typical TCRT5000: Black line absorbs light -> higher voltage / ADC count
  onLineLeft = (rawIRLeft > irThresholdLeft);
  onLineRight = (rawIRRight > irThresholdRight);

  if (onLineLeft && onLineRight) {
    lineError = 0.0f;
  } else if (onLineLeft && !onLineRight) {
    lineError = -1.0f;
    lastKnownLineSide = -1;
    lineLostTimestamp = millis();
  } else if (!onLineLeft && onLineRight) {
    lineError = 1.0f;
    lastKnownLineSide = 1;
    lineLostTimestamp = millis();
  } else {
    // Both off line
    lineError = (lastKnownLineSide == -1) ? -1.8f : 1.8f;
  }
}

// =============================================================================
// ESP-NOW COMMUNICATION
// =============================================================================
typedef struct __attribute__((packed)) {
  char payload[32];
} EspNowMessage;

void onEspNowDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char buffer[33];
  int copyLen = min(data_len, 32);
  memcpy(buffer, data, copyLen);
  buffer[copyLen] = '\0';

  if (strncmp(buffer, "HUMAN,1", 7) == 0) {
    humanDetectedCam = true;
    lastCamHeartbeat = millis();
  } else if (strncmp(buffer, "HUMAN,0", 7) == 0) {
    humanDetectedCam = false;
    lastCamHeartbeat = millis();
  }
}

void setupNetwork() {
  WiFi.mode(WIFI_AP_STA);

  IPAddress apIP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, gateway, subnet);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL);

  Serial.printf("[NET] SoftAP Started: %s on %s\n", WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init Failed!");
  } else {
    esp_now_register_recv_cb(onEspNowDataRecv);
    Serial.println("[ESP-NOW] Initialized and Listening.");
  }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== BINZO SMART DUSTBIN INITIALIZING ===");

  // Pin Modes
  pinMode(PIN_LED_PWR, OUTPUT);
  pinMode(PIN_SW_RUN, INPUT); // GPIO36 input-only, requires external 10k pull-up
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT); // GPIO39 input-only
  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);

  pinMode(PIN_FL_IN1, OUTPUT);
  pinMode(PIN_FL_IN2, OUTPUT);
  pinMode(PIN_FR_IN1, OUTPUT);
  pinMode(PIN_FR_IN2, OUTPUT);
  pinMode(PIN_RL_IN1, OUTPUT);
  pinMode(PIN_RL_IN2, OUTPUT);
  pinMode(PIN_RR_IN1, OUTPUT);
  pinMode(PIN_RR_IN2, OUTPUT);

  // Initialize PWM channels
  initPwmPin(PIN_FL_EN, CH_FL, PWM_FREQ, PWM_RES_BITS);
  initPwmPin(PIN_FR_EN, CH_FR, PWM_FREQ, PWM_RES_BITS);
  initPwmPin(PIN_RL_EN, CH_RL, PWM_FREQ, PWM_RES_BITS);
  initPwmPin(PIN_RR_EN, CH_RR, PWM_FREQ, PWM_RES_BITS);
  initPwmPin(PIN_SERVO, CH_SERVO, SERVO_FREQ, SERVO_RES_BITS);

  stopMotors();
  setLidAngle(SERVO_ANGLE_CLOSED);
  digitalWrite(PIN_LED_PWR, HIGH);

  setupNetwork();
  calibrateLineSensors();

  currentState = STATE_IDLE;
  Serial.println("[SYS] Boot Complete. Ready.");
}

// =============================================================================
// MAIN LOOP & FINITE STATE MACHINE
// =============================================================================
void loop() {
  unsigned long currentMillis = millis();

  // 1. Read Inputs
  isRunSwitchActive = (digitalRead(PIN_SW_RUN) == LOW); // Active LOW
  isCamOnline = (currentMillis - lastCamHeartbeat < CAM_TIMEOUT_MS);
  updateUltrasonic();
  readLineSensors();

  // Determine person detection trigger:
  // Primary: ESP-NOW message from CAM
  // Fallback: Proximity distance if CAM is offline
  bool isPersonDetected = false;
  if (isCamOnline) {
    isPersonDetected = humanDetectedCam;
  } else {
    isPersonDetected = (currentDistanceCm < US_FALLBACK_DIST_CM && currentDistanceCm > OBSTACLE_STOP_CM);
  }

  // Release person detection latch once the person moves away
  if (!isPersonDetected) {
    personLidLatched = false;
  }

  // 2. Global Safety / Obstacle Guard
  if (currentDistanceCm <= OBSTACLE_STOP_CM && currentState != STATE_BOOT && currentState != STATE_OPEN && currentState != STATE_HOLD) {
    if (currentState != STATE_OBSTACLE) {
      stopMotors();
      currentState = STATE_OBSTACLE;
    }
  }

  // 3. FSM Execution
  switch (currentState) {
    case STATE_IDLE:
      stopMotors();
      if (isRunSwitchActive) {
        currentState = STATE_FOLLOW;
      }
      break;

    case STATE_FOLLOW: {
      if (!isRunSwitchActive) {
        currentState = STATE_IDLE;
        break;
      }

      // Check for human detection to stop and open lid
      if (isPersonDetected && !personLidLatched) {
        stopMotors();
        setLidAngle(SERVO_ANGLE_OPEN);
        stateHoldStartTime = currentMillis;
        personLidLatched = true;
        currentState = STATE_HOLD;
        break;
      }

      // Check if both sensors lost the line
      if (!onLineLeft && !onLineRight) {
        if (currentMillis - lineLostTimestamp > 150) {
          currentState = STATE_SEARCH;
          break;
        }
      } else {
        lineLostTimestamp = currentMillis;
      }

      // PD Control for Differential Line Following
      float pTerm = KP * lineError;
      float dTerm = KD * (lineError - lastLineError);
      lastLineError = lineError;
      float correction = pTerm + dTerm;

      int leftSpeed  = BASE_SPEED + (int)correction;
      int rightSpeed = BASE_SPEED - (int)correction;

      setMotorRaw(leftSpeed, rightSpeed);
      break;
    }

    case STATE_HOLD:
      stopMotors();
      if (currentMillis - stateHoldStartTime >= LID_HOLD_TIME_MS) {
        currentState = STATE_CLOSE;
      }
      break;

    case STATE_CLOSE:
      setLidAngle(SERVO_ANGLE_CLOSED);
      delay(350); // Allow servo to mechanically close
      currentState = isRunSwitchActive ? STATE_FOLLOW : STATE_IDLE;
      break;

    case STATE_SEARCH:
      if (!isRunSwitchActive) {
        currentState = STATE_IDLE;
        break;
      }

      // Line regained
      if (onLineLeft || onLineRight) {
        currentState = STATE_FOLLOW;
        break;
      }

      // If search timeout expired, enter LOST state
      if (currentMillis - lineLostTimestamp > LINE_LOST_TIMEOUT) {
        stopMotors();
        currentState = STATE_LOST;
        break;
      }

      // Search maneuver: rotate in the direction line was last seen
      if (lastKnownLineSide == -1) {
        setMotorRaw(-SEARCH_SPEED, SEARCH_SPEED); // Spin CCW
      } else {
        setMotorRaw(SEARCH_SPEED, -SEARCH_SPEED); // Spin CW
      }
      break;

    case STATE_LOST:
      stopMotors();
      // Blink LED while lost
      digitalWrite(PIN_LED_PWR, (currentMillis / 250) % 2 == 0 ? HIGH : LOW);
      if (onLineLeft || onLineRight) {
        digitalWrite(PIN_LED_PWR, HIGH);
        currentState = STATE_FOLLOW;
      }
      if (!isRunSwitchActive) {
        digitalWrite(PIN_LED_PWR, HIGH);
        currentState = STATE_IDLE;
      }
      break;

    case STATE_OBSTACLE:
      stopMotors();
      // Clear obstacle when distance exceeds clearance threshold
      if (currentDistanceCm >= OBSTACLE_CLEAR_CM) {
        currentState = isRunSwitchActive ? STATE_FOLLOW : STATE_IDLE;
      }
      break;

    case STATE_BOOT:
    case STATE_LOW_BATTERY:
    default:
      stopMotors();
      break;
  }

  // 4. Debug Serial Stream
  if (currentMillis - lastDebugPrintTime >= DEBUG_INTERVAL_MS) {
    lastDebugPrintTime = currentMillis;
    Serial.printf("st=%s run=%d IR=%d/%d line=%.2f d=%.1f human=%d cam=%d\n",
                  stateNames[currentState],
                  isRunSwitchActive ? 1 : 0,
                  rawIRLeft, rawIRRight,
                  lineError,
                  currentDistanceCm,
                  isPersonDetected ? 1 : 0,
                  isCamOnline ? 1 : 0);
  }
}
