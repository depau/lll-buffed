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

#include "buffer.h"
TMC2209Stepper driver(UART, UART, R_SENSE, DRIVER_ADDRESS);
Buffer buffer = { 0 }; // stores sensor states
Motor_State motor_state = Stop;

bool is_front = false; // forward flag
uint32_t front_time = 0; // forward time
const int EEPROM_ADDR_TIMEOUT = 0;
const uint32_t DEFAULT_TIMEOUT = 30000;
uint32_t timeout = 30000; // timeout in ms
bool is_error = false; // error flag, set if pushing filament for 30s without stopping
String serial_buf;

static HardwareTimer timer(TIM6); // timer for timeout handling

void buffer_init() {
  buffer_sensor_init();
  buffer_motor_init();
  delay(1000);

  EEPROM.get(EEPROM_ADDR_TIMEOUT, timeout);
  // Check whether the read value is valid (e.g. before first write it may be 0xFFFFFFFF or 0)
  if (timeout == 0xFFFFFFFF || timeout == 0) {
    timeout = DEFAULT_TIMEOUT;
    EEPROM.put(EEPROM_ADDR_TIMEOUT, timeout);
    Serial.println("EEPROM is empty");
  } else {
    Serial.print("read timeout: ");
    Serial.println(timeout);
  }

  timer.pause();
  timer.setPrescaleFactor(48); // divide by 48 -> 48 MHz / 48 = 1 MHz
  timer.setOverflow(1000); // 1ms
  timer.attachInterrupt(&timer_it_callback);
  timer.resume();
}

void buffer_loop() {

  uint32_t lastToggleTime = millis(); // remember the last toggle time
  while (1) {

    if (millis() - lastToggleTime >= 500) // toggle every 500ms
    {
      lastToggleTime = millis(); // update current time
      digitalToggle(ERR_LED);
    }
    // 1. read values from all sensors
    read_sensor_state();

#if DEBUG
    buffer_debug();
    while (Serial.available() > 0) {
      char c = Serial.read();
      serial_buf += c;
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
          Serial.print("write GCONF:0x");
          Serial.println(gconf.sr, HEX);
          Serial.print("read GCONF: 0x");
          Serial.println(driver.GCONF(), HEX);
        }
      }
    }
#else
    motor_control();

    while (Serial.available() > 0) {
      char c = Serial.read();
      serial_buf += c;
    }
    if (serial_buf.length() > 0) {

      if (serial_buf == "rt") {
        Serial.print("read timeout=");
        Serial.println(timeout);
        serial_buf = "";
      } else if (serial_buf.startsWith("set")) {
        serial_buf.remove(0, 3);
        int64_t num = serial_buf.toInt();
        if (num < 0 || num > 0xffffffff) {
          serial_buf = "";
          Serial.println("Error: Invalid timeout value.");
          continue;
        }
        timeout = num;
        EEPROM.put(EEPROM_ADDR_TIMEOUT, timeout);
        serial_buf = "";
        Serial.print("set succeed! timeout=");
        Serial.println(timeout);
      } else {
        Serial.println(serial_buf.c_str());
        Serial.println("command error!");
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
  driver.beginSerial(9600);
  driver.I_scale_analog(false);
  driver.toff(5); // Enables driver in software
  driver.rms_current(I_CURRENT); // Set motor RMS current
  driver.microsteps(Move_Divide_NUM); // Set microsteps to 1/16th
  driver.VACTUAL(STOP); // Set velocity
  driver.en_spreadCycle(true);
  driver.pwm_autoscale(true);
}

/**
 * @brief  Read the state of all sensors
 * @param  NULL
 * @retval NULL
 **/
void read_sensor_state(void) {
  buffer.buffer1_pos1_sensor_state = digitalRead(HALL3);
  buffer.buffer1_pos2_sensor_state = digitalRead(HALL2);
  buffer.buffer1_pos3_sensor_state = digitalRead(HALL1);
  buffer.buffer1_material_swtich_state = digitalRead(ENDSTOP_3);
  buffer.key1 = digitalRead(KEY1);
  buffer.key2 = digitalRead(KEY2);
}

/**
 * @brief  电机控制
 * @param  NULL
 * @retval NULL
 **/
void motor_control(void) {

  static Motor_State last_motor_state = Stop;

  // Button-controlled motor
  // KEY1 pressed
  if (!digitalRead(KEY1)) {
    WRITE_EN_PIN(0); // enable
    driver.VACTUAL(STOP); // stop

    driver.shaft(BACK);
    driver.VACTUAL(VACTRUAL_VALUE);
    while (!digitalRead(KEY1))
      ; // wait for release

    driver.VACTUAL(STOP); // stop
    motor_state = Stop;

    is_front = false;
    front_time = 0;
    is_error = false;
    WRITE_EN_PIN(1); // disable

  } else if (!digitalRead(KEY2)) // KEY2 pressed
  {
    WRITE_EN_PIN(0);
    driver.VACTUAL(STOP); // stop

    driver.shaft(FORWARD);
    driver.VACTUAL(VACTRUAL_VALUE);
    while (!digitalRead(KEY2))
      ;

    driver.VACTUAL(STOP); // stop
    motor_state = Stop;

    is_front = false;
    front_time = 0;
    is_error = false;
    WRITE_EN_PIN(1);
  }

  // Check filament presence
  if (digitalRead(ENDSTOP_3)) {
    // No filament, stop motor
    driver.VACTUAL(STOP); // stop
    motor_state = Stop;

    // Pull filament-break pin low
    digitalWrite(DUANLIAO, 0);

    // Turn off indicator LED
    digitalWrite(START_LED, 0);

    is_front = false;
    front_time = 0;
    is_error = false;
    WRITE_EN_PIN(1);

    return; // no filament, exit
  }

  // Filament present, set filament-break pin high
  digitalWrite(DUANLIAO, 1);

  // Turn on indicator LED
  digitalWrite(START_LED, 1);

  // Check for error condition
  if (is_error) {
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

  if (motor_state == last_motor_state) // same as last state, no need to send command
    return;

  // Motor control
  switch (motor_state) {
  case Forward: // forward
  {
    WRITE_EN_PIN(0);
    if (last_motor_state == Back)
      driver.VACTUAL(STOP); // was moving back, stop before forwarding
    driver.shaft(FORWARD);
    driver.VACTUAL(VACTRUAL_VALUE);

  } break;
  case Stop: // stop
  {
    WRITE_EN_PIN(1);
    driver.VACTUAL(STOP);

  } break;
  case Back: // backward
  {
    WRITE_EN_PIN(0);
    if (last_motor_state == Forward)
      driver.VACTUAL(STOP);
    ; // was moving forward, stop before reversing
    driver.shaft(BACK);
    driver.VACTUAL(VACTRUAL_VALUE);
  } break;
  }
}

void timer_it_callback() {
  if (is_front) { // when pushing forward
    front_time++;
    if (front_time > timeout) { // timeout reached
      is_error = true;
    }
  }
}

void buffer_debug(void) {
  // Serial.print("buffer1_pos1_sensor_state:");Serial.println(buffer.buffer1_pos1_sensor_state);
  // Serial.print("buffer1_pos2_sensor_state:");Serial.println(buffer.buffer1_pos2_sensor_state);
  // Serial.print("buffer1_pos3_sensor_state:");Serial.println(buffer.buffer1_pos3_sensor_state);
  // Serial.print("buffer1_material_swtich_state:");Serial.println(buffer.buffer1_material_swtich_state);
  // Serial.print("key1:");Serial.println(buffer.key1);
  // Serial.print("key2:");Serial.println(buffer.key2);
  static int i = 0;
  if (i < 0x1ff) {
    Serial.print("i:");
    Serial.println(i);
    driver.GCONF(i);
    driver.PWMCONF(i);
    i++;
  }
  uint32_t gconf = driver.GCONF();
  uint32_t chopconf = driver.CHOPCONF();
  uint32_t pwmconf = driver.PWMCONF();
  if (driver.CRCerror) {
    Serial.println("CRCerror");
  } else {
    Serial.print("GCONF():0x");
    Serial.println(gconf, HEX);
    Serial.print("CHOPCONF():0x");
    char buf[11]; // "0x" + 8 digits + null terminator
    sprintf(buf, "%08lX", chopconf); // %08lX -> 8-digit uppercase hex (long unsigned)
    Serial.println(buf);
    Serial.print("PWMCONF():0x");
    sprintf(buf, "%08lX", pwmconf); // %08lX -> 8-digit uppercase hex (long unsigned)
    Serial.println(buf);
    Serial.println("");
  }
  delay(1000);
}
