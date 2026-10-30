/*
 * BINZO - Smart Dustbin Robot (Standard 2-Motor L298N Differential Variant)
 * Hardware: ESP32-WROOM-32 (30-pin DevKit)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "../Binzo_Mecanum/config.h"

// 2-Motor Pin Mapping
#define PIN_M1_EN     PIN_FL_EN
#define PIN_M1_IN1    PIN_FL_IN1
#define PIN_M1_IN2    PIN_FL_IN2

#define PIN_M2_EN     PIN_FR_EN
#define PIN_M2_IN1    PIN_FR_IN1
#define PIN_M2_IN2    PIN_FR_IN2

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define USE_ESP32_CORE_V3 1
#else
  #define USE_ESP32_CORE_V3 0
#endif

void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  uint8_t absSpeed = abs(speed);
  if (absSpeed > 0 && absSpeed < MIN_PWM) absSpeed = MIN_PWM;

  if (speed > 0) {
    digitalWrite(PIN_M1_IN1, HIGH);
    digitalWrite(PIN_M1_IN2, LOW);
#if USE_ESP32_CORE_V3
    ledcWrite(PIN_M1_EN, absSpeed);
#else
    ledcWrite(0, absSpeed);
#endif
  } else if (speed < 0) {
    digitalWrite(PIN_M1_IN1, LOW);
    digitalWrite(PIN_M1_IN2, HIGH);
#if USE_ESP32_CORE_V3
    ledcWrite(PIN_M1_EN, absSpeed);
#else
    ledcWrite(0, absSpeed);
#endif
  } else {
    digitalWrite(PIN_M1_IN1, LOW);
    digitalWrite(PIN_M1_IN2, LOW);
#if USE_ESP32_CORE_V3
    ledcWrite(PIN_M1_EN, 0);
#else
    ledcWrite(0, 0);
#endif
  }
}

void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  uint8_t absSpeed = abs(speed);
  if (absSpeed > 0 && absSpeed < MIN_PWM) absSpeed = MIN_PWM;

  if (speed > 0) {
    digitalWrite(PIN_M2_IN1, HIGH);
    digitalWrite(PIN_M2_IN2, LOW);
#if USE_ESP32_CORE_V3
    ledcWrite(PIN_M2_EN, absSpeed);
#else
    ledcWrite(1, absSpeed);
#endif
  } else if (speed < 0) {
    digitalWrite(PIN_M2_IN1, LOW);
    digitalWrite(PIN_M2_IN2, HIGH);
#if USE_ESP32_CORE_V3
    ledcWrite(PIN_M2_EN, absSpeed);
#else
    ledcWrite(1, absSpeed);
#endif
  } else {
    digitalWrite(PIN_M2_IN1, LOW);
    digitalWrite(PIN_M2_IN2, LOW);
#if USE_ESP32_CORE_V3
    ledcWrite(PIN_M2_EN, 0);
#else
    ledcWrite(1, 0);
#endif
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_M1_IN1, OUTPUT);
  pinMode(PIN_M1_IN2, OUTPUT);
  pinMode(PIN_M2_IN1, OUTPUT);
  pinMode(PIN_M2_IN2, OUTPUT);

#if USE_ESP32_CORE_V3
  ledcAttach(PIN_M1_EN, PWM_FREQ, PWM_RES_BITS);
  ledcAttach(PIN_M2_EN, PWM_FREQ, PWM_RES_BITS);
#else
  ledcSetup(0, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(PIN_M1_EN, 0);
  ledcSetup(1, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(PIN_M2_EN, 1);
#endif
  Serial.println("[SYS] 2-Motor Differential Baseline ready.");
}

void loop() {
  // Baseline test loop
  delay(100);
}
