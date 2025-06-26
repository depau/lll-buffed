/**
  ***************************************************************************************
  * @file    buffer.cpp
  * @author  lijihu
  * @version V1.0.0
  * @date    2025/05/10
 * @brief   Implementation of the buffer functionality
        *Buffer description
        Optical sensor: 1 when blocked, 0 when unblocked
        Filament switch: 0 when filament present, 1 when absent
        Button: 0 when pressed, 1 when released

        Pins:
        HALL1 --> PB2 (optical sensor 3)
        HALL2 --> PB3 (optical sensor 2)
        HALL3 --> PB4 (optical sensor 1)
        ENDSTOP_3 --> PB7 (filament switch)
        KEY1 --> PB13 (backward)
        KEY2 --> PB12 (forward)
  *
  * @note
  ***************************************************************************************
 * Copyright 2025 xxx@126.com
  ***************************************************************************************
**/

#include <USBSerial.h>
#include <array>
#include <cstdio>

#include "buffer.h"

namespace {
TMC2209Stepper driver(UART, UART, R_SENSE, DRIVER_ADDRESS);
Buffer buffer{}; // stores sensor states
Motor_State motor_state = Stop;

bool is_front = false; // forward flag
uint32_t front_time = 0; // forward time
constexpr uint32_t DEFAULT_TIMEOUT = 90000U;
constexpr uint32_t TIMER_PRESCALE = 48U;
constexpr uint32_t TIMER_OVERFLOW = 1000U;
constexpr uint32_t ONE_SECOND_MS = 1000U;
constexpr int MAX_INDEX = 0x1ff;
constexpr uint32_t TOGGLE_INTERVAL_MS = 500U;
constexpr uint32_t SERIAL_BAUDRATE = 9600U;
constexpr uint8_t DRIVER_TOFF = 5U;
constexpr uint32_t MAX_TIMEOUT = 0xFFFFFFFFU;

// number of presses required to enable continuous movement
constexpr uint8_t MULTI_PRESS_COUNT = 3U;
// minimum time between presses to be considered part of the same sequence (ms)
constexpr uint32_t MULTI_PRESS_MIN_INTERVAL_MS = 50U;
// maximum time between presses to be considered part of the same sequence (ms)
constexpr uint32_t MULTI_PRESS_MAX_INTERVAL_MS = 500U;

uint32_t timeout = DEFAULT_TIMEOUT; // timeout in ms
bool is_timeout = false; // error flag, set if pushing filament for 30s without stopping
bool continuous_run = false; // flag for continuous movement
Motor_State continuous_direction = Stop; // direction for continuous movement
} // namespace

void buffer_init() {
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-cstyle-cast, performance-no-int-to-ptr)
  static HardwareTimer timer(TIM6); // timer for timeout handling

  buffer_sensor_init();
  buffer_motor_init();
  delay(ONE_SECOND_MS);

  timer.pause();
  timer.setPrescaleFactor(TIMER_PRESCALE); // divide by 48 -> 48 MHz / 48 = 1 MHz
  timer.setOverflow(TIMER_OVERFLOW); // 1ms
  timer.attachInterrupt(&timer_it_callback);
  timer.resume();
}

void buffer_loop() {

  static String serial_buf;
  uint32_t lastToggleTime = millis(); // remember the last toggle time
  while (true) {

    if (millis() - lastToggleTime >= TOGGLE_INTERVAL_MS) // toggle every 500ms
    {
      lastToggleTime = millis(); // update current time
      digitalToggle(ERR_LED);
    }
    // 1. read values from all sensors
    read_sensor_state();

#if DEBUG
    buffer_debug();
    while (SerialUSB.available() > 0) {
      const char incoming_char = SerialUSB.read();
      serial_buf += incoming_char;
      int pos_enter = -1;
      pos_enter = serial_buf.indexOf("\n");
      if (pos_enter != -1) {
        String str = serial_buf.substring(0, pos_enter);
        serial_buf = serial_buf.substring(pos_enter + 1);
        if (strstr(str.c_str(), "gconf") != NULL) {
          TMC2208_n::CHOPCONF_t gconf{ 0 };

          // Extract the hexadecimal string after "gconf"
          int pos = str.indexOf("gconf");
          if (pos != -1) {
            String hexPart = str.substring(pos + 5); // skip "gconf"
            hexPart.trim(); // remove whitespace

            // Convert the string to a 32-bit unsigned integer
            uint32_t hexValue = strtoul(hexPart.c_str(), NULL, 16);

            // Assign it to the struct (according to your definition)
            gconf.sr = hexValue; // assume sr is the raw register field
          }
          driver.GCONF(gconf.sr);
          SerialUSB.print("write GCONF:0x");
          SerialUSB.println(gconf.sr, HEX);
          SerialUSB.print("read GCONF: 0x");
          SerialUSB.println(driver.GCONF(), HEX);
        }
      }
    }
#else
    motor_control();

    while (SerialUSB.available() > 0) {
      const char incoming_char = SerialUSB.read();
      serial_buf += incoming_char;
    }
    if (serial_buf.length() > 0) {

      if (serial_buf == "rt") {
        SerialUSB.print("read timeout=");
        SerialUSB.println(timeout);
        serial_buf = "";
      } else if (serial_buf.startsWith("set")) {
        serial_buf.remove(0, 3);
        const int64_t num = serial_buf.toInt();
        if (num < 0 || num > MAX_TIMEOUT) { // keep numeric limit check
          serial_buf = "";
          SerialUSB.println("Error: Invalid timeout value.");
          continue;
        }
        timeout = num;
        serial_buf = "";
        SerialUSB.print("set succeed! timeout=");
        SerialUSB.println(timeout);
      } else {
        SerialUSB.println(serial_buf.c_str());
        SerialUSB.println("command error!");
        serial_buf = "";
      }
    }

#endif
  }
}

void buffer_sensor_init() {
  // Initialize sensors
  pinMode(HALL1, INPUT);
  pinMode(HALL2, INPUT);
  pinMode(HALL3, INPUT);
  pinMode(ENDSTOP_3, INPUT);
  pinMode(KEY1, INPUT);
  pinMode(KEY2, INPUT);

  // Initialize filament indicator LEDs
  pinMode(DUANLIAO, OUTPUT);
  pinMode(ERR_LED, OUTPUT);
  pinMode(START_LED, OUTPUT);
}

void buffer_motor_init() {
  // Initialize motor driver pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // Enable driver in hardware

  // Initialize motor driver
  driver.begin(); // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.beginSerial(SERIAL_BAUDRATE);
  driver.I_scale_analog(false);
  driver.toff(DRIVER_TOFF); // Enables driver in software
  driver.rms_current(I_CURRENT); // Set motor RMS current
  driver.microsteps(MOVE_DIVIDE_NUM); // Set microsteps to 1/16th
  driver.VACTUAL(STOP); // Set velocity
  driver.en_spreadCycle(true);
  driver.pwm_autoscale(true);
}

/**
 * @brief  Read the state of all sensors
 * @param  NULL
 * @retval NULL
 **/
void read_sensor_state() {
  buffer.buffer1_pos1_sensor_state = digitalRead(HALL3) != 0;
  buffer.buffer1_pos2_sensor_state = digitalRead(HALL2) != 0;
  buffer.buffer1_pos3_sensor_state = digitalRead(HALL1) != 0;
  buffer.buffer1_material_swtich_state = digitalRead(ENDSTOP_3) != 0;
  buffer.key1 = digitalRead(KEY1) != 0;
  buffer.key2 = digitalRead(KEY2) != 0;
}

namespace {
void startMotor(Motor_State dir, Motor_State last_state) {
  WRITE_EN_PIN(0);
  if (dir != last_state) {
    driver.VACTUAL(STOP);
  }
  driver.shaft(dir == Forward ? FORWARD : BACK);
  driver.VACTUAL(VACTUAL_VALUE);
}

void stopMotor() {
  WRITE_EN_PIN(1);
  driver.VACTUAL(STOP);
}

auto handleContinuousRun(Motor_State &last_motor_state) -> bool {
  if (!continuous_run) {
    return false;
  }
  if (digitalRead(KEY1) == LOW || digitalRead(KEY2) == LOW || is_timeout) {
    stopMotor();
    motor_state = Stop;
    continuous_run = false;
    is_front = false;
    front_time = 0;
    is_timeout = false;
    while (digitalRead(KEY1) == LOW || digitalRead(KEY2) == LOW) {
    }
    return true;
  }
  startMotor(continuous_direction, last_motor_state);
  is_front = continuous_direction == Forward;
  last_motor_state = continuous_direction;
  return true;
}

auto handleButton(uint8_t pin, Motor_State dir, uint32_t &last_time, uint8_t &count, Motor_State &last_motor_state)
  -> bool {
  if (digitalRead(pin) != LOW) {
    return false;
  }
  const uint32_t now = millis();
  if (now - last_time >= MULTI_PRESS_MIN_INTERVAL_MS && now - last_time <= MULTI_PRESS_MAX_INTERVAL_MS) {
    count++;
  } else {
    count = 1;
  }
  last_time = now;

  startMotor(dir, last_motor_state);
  while (digitalRead(pin) == LOW) {
  }
  stopMotor();
  motor_state = Stop;
  is_front = false;
  front_time = 0;
  is_timeout = false;

  if (count >= MULTI_PRESS_COUNT) {
    continuous_run = true;
    continuous_direction = dir;
    count = 0;
  }
  return true;
}
} // namespace

/**
 * @brief  电机控制
 * @param  NULL
 * @retval NULL
 **/
void motor_control() {
  static Motor_State last_motor_state = Stop;
  static uint32_t last_key1_time = 0;
  static uint8_t key1_count = 0;
  static uint32_t last_key2_time = 0;
  static uint8_t key2_count = 0;

  // stop and trigger error when both buttons are pressed simultaneously
  if (digitalRead(KEY1) == LOW && digitalRead(KEY2) == LOW) {
    stopMotor();
    motor_state = Stop;
    continuous_run = false;
    is_front = false;
    front_time = 0;
    is_timeout = true;
    while (digitalRead(KEY1) == LOW || digitalRead(KEY2) == LOW) {
    }
    return;
  }

  if (handleContinuousRun(last_motor_state)) {
    return;
  }

  if (handleButton(KEY1, Back, last_key1_time, key1_count, last_motor_state)) {
    return;
  }
  if (handleButton(KEY2, Forward, last_key2_time, key2_count, last_motor_state)) {
    return;
  }

  // Check filament presence
  if (digitalRead(ENDSTOP_3) != 0) {
    // No filament, stop motor
    driver.VACTUAL(STOP); // stop
    motor_state = Stop;

    // Pull filament-break pin low
    digitalWrite(DUANLIAO, LOW);

    // Turn off indicator LED
    digitalWrite(START_LED, LOW);

    is_front = false;
    front_time = 0;
    is_timeout = false;
    WRITE_EN_PIN(1);

    return; // no filament, exit
  }

  // Filament present, set filament-break pin high
  digitalWrite(DUANLIAO, HIGH);

  // Turn on indicator LED
  digitalWrite(START_LED, HIGH);

  // Check for error condition
  if (is_timeout) {
    // Stop motor
    driver.VACTUAL(STOP); // stop
    motor_state = Stop;
    WRITE_EN_PIN(1);
    return;
  }

  // Buffer position handling
  if (buffer.buffer1_pos1_sensor_state) // buffer position 1 -> push filament forward
  {
    last_motor_state = motor_state; // remember previous state
    motor_state = Forward;
    is_front = true;

  } else if (buffer.buffer1_pos2_sensor_state) // buffer position 2 -> stop motor
  {
    last_motor_state = motor_state; // remember previous state
    motor_state = Stop;
    is_front = false;
    front_time = 0;
  } else if (buffer.buffer1_pos3_sensor_state) // buffer position 3 -> retract filament
  {
    last_motor_state = motor_state; // remember previous state
    motor_state = Back;
    is_front = false;
    front_time = 0;
  }

  if (motor_state == last_motor_state) { // same as last state, no need to send command
    return;
  }

  // Motor control
  switch (motor_state) {
  case Forward: // forward
  {
    WRITE_EN_PIN(0);
    if (last_motor_state == Back) {
      driver.VACTUAL(STOP); // was moving back, stop before forwarding
    }
    driver.shaft(FORWARD);
    driver.VACTUAL(VACTUAL_VALUE);

  } break;
  case Stop: // stop
  {
    WRITE_EN_PIN(1);
    driver.VACTUAL(STOP);

  } break;
  case Back: // backward
  {
    WRITE_EN_PIN(0);
    if (last_motor_state == Forward) {
      driver.VACTUAL(STOP); // was moving forward, stop before reversing
    }
    driver.shaft(BACK);
    driver.VACTUAL(VACTUAL_VALUE);
  } break;
  }
}

void timer_it_callback() {
  if (is_front) { // when pushing forward
    front_time++;
    if (front_time > timeout) { // timeout reached
      is_timeout = true;
    }
  }
}

void buffer_debug() {
  // SerialUSB.print("buffer1_pos1_sensor_state:");
  // SerialUSB.println(buffer.buffer1_pos1_sensor_state);
  // SerialUSB.print("buffer1_pos2_sensor_state:");
  // SerialUSB.println(buffer.buffer1_pos2_sensor_state);
  // SerialUSB.print("buffer1_pos3_sensor_state:");
  // SerialUSB.println(buffer.buffer1_pos3_sensor_state);
  // SerialUSB.print("buffer1_material_swtich_state:");
  // SerialUSB.println(buffer.buffer1_material_swtich_state);
  // SerialUSB.print("key1:");
  // SerialUSB.println(buffer.key1);
  // SerialUSB.print("key2:");
  // SerialUSB.println(buffer.key2);
  static int idx = 0;
  if (idx < MAX_INDEX) {
    SerialUSB.print("i:");
    SerialUSB.println(idx);
    driver.GCONF(idx);
    driver.PWMCONF(idx);
    idx++;
  }
  const uint32_t gconf = driver.GCONF();
  const uint32_t chopconf = driver.CHOPCONF();
  const uint32_t pwmconf = driver.PWMCONF();
  if (driver.CRCerror) {
    SerialUSB.println("CRCerror");
  } else {
    SerialUSB.print("GCONF():0x");
    SerialUSB.println(gconf, HEX);
    SerialUSB.print("CHOPCONF():0x");
    String buf = String(chopconf, HEX);
    buf.toUpperCase();
    SerialUSB.println(buf);
    SerialUSB.print("PWMCONF():0x");
    buf = String(pwmconf, HEX);
    buf.toUpperCase();
    SerialUSB.println(buf);
    SerialUSB.println("");
  }
  delay(ONE_SECOND_MS);
}
