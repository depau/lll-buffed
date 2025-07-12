#pragma once

#include <Arduino.h>
#include <TMCStepper.h>

#include "hardware_iface.h"

inline constexpr uint32_t OPTICAL_SENSOR_1 = PB4;
inline constexpr uint32_t OPTICAL_SENSOR_2 = PB3;
inline constexpr uint32_t OPTICAL_SENSOR_3 = PB2;
inline constexpr uint32_t PRESENCE_SWITCH = PB7;
inline constexpr uint32_t PRESENCE_LED = PA8;
inline constexpr uint32_t PRESENCE_OUTPUT = PB15;
inline constexpr uint32_t ERROR_LED = PA15;
inline constexpr uint32_t BTN_BACKWARD = PB13;
inline constexpr uint32_t BTN_FORWARD = PB12;
inline constexpr uint32_t STEPPER_EN = PA6;
inline constexpr uint32_t STEPPER_DIR = PA7;
inline constexpr uint32_t STEPPER_STEP = PC13;
inline constexpr uint32_t STEPPER_UART = PB1;
inline constexpr uint8_t STEPPER_ADDR = 0b00;
inline constexpr float STEPPER_R_SENSE = 0.11f;
inline constexpr int32_t STEPPER_MICROSTEPS = 64;
constexpr float SPEED_RPM_FACTOR = 9.1463414634f;
constexpr float STEPS_PER_REV = 200.0f;
constexpr float SCREW_PITCH = 0.715f;

class BoardHardware : public BufferHardware<BoardHardware> {
  TMC2209Stepper driver{ STEPPER_UART, STEPPER_UART, STEPPER_R_SENSE, STEPPER_ADDR };

public:
  void initHardwareImpl() {
    pinMode(OPTICAL_SENSOR_1, INPUT);
    pinMode(OPTICAL_SENSOR_2, INPUT);
    pinMode(OPTICAL_SENSOR_3, INPUT);
    pinMode(PRESENCE_SWITCH, INPUT);
    pinMode(BTN_BACKWARD, INPUT);
    pinMode(BTN_FORWARD, INPUT);
    pinMode(PRESENCE_OUTPUT, OUTPUT);
    pinMode(ERROR_LED, OUTPUT);
    pinMode(PRESENCE_LED, OUTPUT);

    pinMode(STEPPER_EN, OUTPUT);
    pinMode(STEPPER_STEP, OUTPUT);
    pinMode(STEPPER_DIR, OUTPUT);
    digitalWrite(STEPPER_EN, LOW);
    driver.begin();
    driver.beginSerial(9600);
    driver.I_scale_analog(false);
    driver.toff(5);
    driver.rms_current(600);
    driver.microsteps(STEPPER_MICROSTEPS);
    driver.VACTUAL(0);
    driver.en_spreadCycle(true);
    driver.pwm_autoscale(true);
  }

  static bool optical1Impl() { return digitalRead(OPTICAL_SENSOR_1) != 0; }
  static bool optical2Impl() { return digitalRead(OPTICAL_SENSOR_2) != 0; }
  static bool optical3Impl() { return digitalRead(OPTICAL_SENSOR_3) != 0; }
  static bool filamentPresentImpl() { return digitalRead(PRESENCE_SWITCH) == 0; }
  static bool buttonForwardImpl() { return digitalRead(BTN_FORWARD) == LOW; }
  static bool buttonBackwardImpl() { return digitalRead(BTN_BACKWARD) == LOW; }
  static void setErrorLedImpl(bool on) { digitalWrite(ERROR_LED, on ? HIGH : LOW); }
  static void setPresenceLedImpl(bool on) { digitalWrite(PRESENCE_LED, on ? HIGH : LOW); }
  static void setPresenceOutputImpl(bool on) { digitalWrite(PRESENCE_OUTPUT, on ? HIGH : LOW); }
  void stepperPushImpl(float speed) { runStepper(true, speed); }
  void stepperRetractImpl(float speed) { runStepper(false, speed); }
  void stepperHoldImpl() {
    digitalWrite(STEPPER_EN, LOW);
    driver.VACTUAL(0);
  }
  void stepperOffImpl() {
    digitalWrite(STEPPER_EN, HIGH);
    driver.VACTUAL(0);
  }
  static void writeLineImpl(const std::string &l) {
    SerialUSB.println(l.c_str());
    Serial2.println(l.c_str());
  }
  static bool readCharImpl(char &c) {
    if (SerialUSB.available()) {
      c = static_cast<char>(SerialUSB.read());
      return true;
    }
    if (Serial2.available()) {
      c = static_cast<char>(Serial2.read());
      return true;
    }
    return false;
  }
  static uint32_t timeMsImpl() { return millis(); }

private:
  void runStepper(bool forward, float speed) {
    digitalWrite(STEPPER_EN, LOW);
    driver.shaft(forward);
    const float rpm = speed * SPEED_RPM_FACTOR;
    const auto v = static_cast<uint32_t>(rpm * STEPPER_MICROSTEPS * STEPS_PER_REV / 60.0f / SCREW_PITCH);
    driver.VACTUAL(v);
  }
};
