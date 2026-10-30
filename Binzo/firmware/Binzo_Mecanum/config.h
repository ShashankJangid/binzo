#pragma once
#include <Arduino.h>

// =============================================================================
// PIN DEFINITIONS - MAIN ESP32-WROOM (30-pin)
// =============================================================================

// Motors (2x L298N drivers - 4 DC Motors with Mecanum Wheels)
// L298N #1: Front Motors
#define PIN_FL_EN     25    // Front-Left PWM (ENA)
#define PIN_FL_IN1    26    // Front-Left Direction IN1
#define PIN_FL_IN2    27    // Front-Left Direction IN2
#define PIN_FR_EN     17    // Front-Right PWM (ENB)
#define PIN_FR_IN1    14    // Front-Right Direction IN1
#define PIN_FR_IN2    16    // Front-Right Direction IN2

// L298N #2: Rear Motors
#define PIN_RL_EN     21    // Rear-Left PWM (ENA)
#define PIN_RL_IN1    32    // Rear-Left Direction IN1
#define PIN_RL_IN2    33    // Rear-Left Direction IN2
#define PIN_RR_EN     22    // Rear-Right PWM (ENB)
#define PIN_RR_IN1     4    // Rear-Right Direction IN1
#define PIN_RR_IN2    13    // Rear-Right Direction IN2

// Servo (MG90S Lid Mechanism)
#define PIN_SERVO     18    // 50Hz PWM

// Ultrasonic Sensor (HC-SR04)
#define PIN_US_TRIG   19    // 10us trigger output
#define PIN_US_ECHO   39    // Echo input (via 1k/2k voltage divider)

// Indicators & Switches
#define PIN_LED_PWR   23    // Power Status LED
#define PIN_SW_RUN    36    // Run/Stop switch (Active LOW; External 10k Pull-up to 3.3V)

// Line Sensors (TCRT5000 IR Modules, 3.3V Powered)
#define PIN_IR_LEFT   34    // Left Line Sensor (Input-only)
#define PIN_IR_RIGHT  35    // Right Line Sensor (Input-only)

// =============================================================================
// TUNING PARAMETERS & THRESHOLDS
// =============================================================================

#define PWM_FREQ            1000  // 1 kHz for motor driver PWM
#define PWM_RES_BITS        8     // 8-bit resolution (0-255)

#define SERVO_FREQ          50    // 50 Hz standard servo frequency
#define SERVO_RES_BITS      14    // 14-bit resolution (16384 counts)
#define SERVO_PULSE_MIN_US  500   // 0.5ms (~0 deg)
#define SERVO_PULSE_MAX_US  2400  // 2.4ms (~180 deg)
#define SERVO_ANGLE_CLOSED  0
#define SERVO_ANGLE_OPEN    95

#define BASE_SPEED          150   // Default cruising PWM (0-255)
#define MIN_PWM             65    // Minimum PWM to overcome motor stiction
#define MAX_PWM             220   // Maximum PWM cap
#define SEARCH_SPEED        95    // Recovery spin speed when line is lost
#define KP                  70.0f // Proportional gain
#define KD                  20.0f // Derivative gain

#define OBSTACLE_STOP_CM    12    // Emergency stop distance (cm)
#define OBSTACLE_CLEAR_CM   25    // Hysteresis release distance (cm)
#define US_FALLBACK_DIST_CM 35    // Proximity trigger distance if CAM is offline

#define LID_HOLD_TIME_MS    4000  // Duration to keep lid open for trash deposit
#define CAM_TIMEOUT_MS      2500  // Fallback to HC-SR04 if CAM heartbeat ceases
#define LINE_LOST_TIMEOUT   1200  // Duration before transitioning to LOST state

#define WIFI_AP_SSID        "Binzo-Bin"
#define WIFI_AP_PASS        "binzo1234"
#define WIFI_AP_CHANNEL     1
