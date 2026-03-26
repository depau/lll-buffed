#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>
#include <cstdarg>
#include <cstdio>

#include "tiny_printf.hpp"

static constexpr uint8_t SIMULATOR_SENTINEL = 0xAA;
extern "C" volatile uint8_t simulator_buffer_id __attribute__((used, unused));

#ifdef ENABLE_I2C_PROTOCOL
#include <SoftI2CPeripheral.h>

extern "C" volatile uint8_t simulator_i2c_address __attribute__((unused));

#endif

inline constexpr uint32_t OPTICAL_SENSOR_1 = A2; // PF2
inline constexpr uint32_t OPTICAL_SENSOR_2 = A3; // PF3
inline constexpr uint32_t OPTICAL_SENSOR_3 = 4; // PG5
inline constexpr uint32_t PRESENCE_SWITCH = 5; // PE3
inline constexpr uint32_t PRESENCE_LED = LED_BUILTIN; // PB7
inline constexpr uint32_t PRESENCE_OUTPUT = 7; // PH4
inline constexpr uint32_t ERROR_LED = 8; // PH5
inline constexpr uint32_t BTN_BACKWARD = 9; // PH6
inline constexpr uint32_t BTN_FORWARD = 10; // PB4
inline constexpr uint32_t STEPPER_EN = 11; // PB5
inline constexpr uint32_t STEPPER_DIR = 12; // PB6
inline constexpr uint32_t STEPPER_STEP = A0; // PF0
inline constexpr uint32_t STEPPER_UART = A1; // PF1
inline constexpr uint8_t STEPPER_ADDR = 0b00;

// Doesn't really matter but let's fully emulate it
inline constexpr float STEPPER_R_SENSE = 0.11f;
inline constexpr int32_t STEPPER_MICROSTEPS = 64;
inline constexpr float SPEED_RPM_FACTOR = 9.1463414634f;
inline constexpr float STEPS_PER_REV = 200.0f;
inline constexpr float SCREW_PITCH = 0.715f;

#ifdef ENABLE_UART_PROTOCOL
inline constexpr uint32_t UART_RX_PIN = 0; // PE0
inline constexpr uint32_t UART_TX_PIN = 1; // PE1
#endif

#ifdef ENABLE_I2C_PROTOCOL
inline constexpr uint32_t I2C_INT_PIN = 21; // PD0 / INT0
inline constexpr uint32_t I2C_SCL_PIN = 2; // PE4 / INT4
inline constexpr uint32_t I2C_SDA_PIN = 3; // PE5 / INT5
#endif

class scoped_irq_lock {
  uint8_t sreg = 0;

public:
  scoped_irq_lock() {
    sreg = SREG;
    cli();
  }
  ~scoped_irq_lock() { SREG = sreg; }
};

class SimulatorHardware {
  int bufferID = BUFFER_ID;
#ifdef ENABLE_UART_PROTOCOL
  SoftwareSerial serial{ UART_RX_PIN, UART_TX_PIN };
#endif
  TMC2209Stepper driver{ STEPPER_UART, STEPPER_UART, STEPPER_R_SENSE, STEPPER_ADDR };

#ifdef ENABLE_I2C_PROTOCOL
  SoftI2CPeripheral i2c;

  void *i2cContext = nullptr;
  size_t (*onI2CRead)(void *ctx, uint8_t reg) = nullptr;
  void (*onI2CWrite)(void *ctx, uint8_t reg, size_t size, const uint8_t *data) = nullptr;
  volatile uint8_t pendingReadRegister = 0xFF;

  // TX capture buffer — filled by i2cWrite() during onI2CRead callback
  uint8_t i2cTxBuffer[SOFT_I2C_PERIPHERAL_BUF_SIZE]{};
  uint8_t i2cTxLen = 0;
  bool i2cCapturingTx = false;

  void handleI2C() {
    if (!i2c.available())
      return;

    // Read ALL write bytes BEFORE consume()/respond() — buffer may be overwritten after release
    const uint8_t rxLen = i2c.getWriteLength();
    uint8_t rxBuf[SOFT_I2C_PERIPHERAL_BUF_SIZE];
    for (uint8_t i = 0; i < rxLen; i++)
      rxBuf[i] = static_cast<uint8_t>(i2c.read());

    if (i2c.needsResponse()) {
      // Compound write+read: rxBuf[0] is the register address
      if (rxLen >= 1)
        pendingReadRegister = rxBuf[0];

      // Capture i2cWrite() calls into tx buffer
      i2cTxLen = 0;
      i2cCapturingTx = true;
      if (onI2CRead)
        pendingReadRegister += onI2CRead(i2cContext, pendingReadRegister);
      i2cCapturingTx = false;

      i2c.respond(i2cTxBuffer, i2cTxLen);
    } else {
      // Write-only: 1 byte = register pointer update; >1 bytes = register write
      if (rxLen == 1) {
        pendingReadRegister = rxBuf[0];
      } else if (rxLen > 1 && onI2CWrite) {
        onI2CWrite(i2cContext, rxBuf[0], rxLen - 1, &rxBuf[1]);
      }
      i2c.consume();
    }
  }
#endif

public:
  void loop() {
    if (simulator_buffer_id != SIMULATOR_SENTINEL) {
      bufferID = simulator_buffer_id;
      simulator_buffer_id = SIMULATOR_SENTINEL;
      writeLineF("buffer_id=%d", bufferID);
    }

#ifdef ENABLE_I2C_PROTOCOL
    uint8_t new_i2c_addr = 0;
    {
      scoped_irq_lock lock;
      if (simulator_i2c_address != SIMULATOR_SENTINEL) {
        i2c.setAddress(simulator_i2c_address);
        new_i2c_addr = simulator_i2c_address;
        simulator_i2c_address = SIMULATOR_SENTINEL;
      }
    }
    if (new_i2c_addr != 0) {
      writeLineF("i2c_addr=0x%x", new_i2c_addr);
    }

    handleI2C();
#endif
  }

  void initHardware() {
#ifdef ENABLE_UART_PROTOCOL
    serial.begin(115200);
#endif
#ifdef ENABLE_I2C_PROTOCOL
    pinMode(I2C_INT_PIN, OUTPUT);
    digitalWrite(I2C_INT_PIN, HIGH);

    i2c.begin(I2C_ADDR, I2C_SCL_PIN, I2C_SDA_PIN, 4, 5); // INT4 (pin 2), INT5 (pin 3)
    simulator_i2c_address = SIMULATOR_SENTINEL; // signal: I2C ready for address assignment
#endif
    simulator_buffer_id = SIMULATOR_SENTINEL;

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

  static constexpr int getBufferID() { return BUFFER_ID; }

  static bool optical1() { return digitalRead(OPTICAL_SENSOR_1) != 0; }
  static bool optical2() { return digitalRead(OPTICAL_SENSOR_2) != 0; }
  static bool optical3() { return digitalRead(OPTICAL_SENSOR_3) != 0; }
  static bool filamentPresent() { return digitalRead(PRESENCE_SWITCH) == 0; }
  static bool buttonForward() { return digitalRead(BTN_FORWARD) == LOW; }
  static bool buttonBackward() { return digitalRead(BTN_BACKWARD) == LOW; }
  static void setErrorLed(const bool on) { digitalWrite(ERROR_LED, on ? HIGH : LOW); }
  static void setPresenceLed(const bool on) { digitalWrite(PRESENCE_LED, on ? HIGH : LOW); }
  static void setPresenceOutput(const bool on) { digitalWrite(PRESENCE_OUTPUT, on ? HIGH : LOW); }

  void stepperPush(const float speed) { runStepper(true, speed); }
  void stepperRetract(const float speed) { runStepper(false, speed); }

  void stepperHold() {
    digitalWrite(STEPPER_EN, LOW);
    driver.VACTUAL(0);
  }

  void stepperOff() {
    digitalWrite(STEPPER_EN, HIGH);
    driver.VACTUAL(0);
  }

  [[noreturn]] static void rebootDFU() { abort(); }

#ifdef ENABLE_UART_PROTOCOL
  void writeLine(const char *l) {
    serial.write(l);
    serial.write('\n');
  }

  template<size_t BufSize = 64>
  [[gnu::format(printf, 2, 3)]]
  void writeLineF(const char *fmt, ...) {
    char buf[BufSize + 1];
    va_list args;
    va_start(args, fmt);
    tiny::vsnprintf(buf, BufSize, fmt, args);
    va_end(args);
    writeLine(buf);
  }

  bool readChar(char &c) {
    if (serial.available()) {
      c = static_cast<char>(serial.read());
      return true;
    }
    return false;
  }
#endif

#ifdef ENABLE_I2C_PROTOCOL
  void setI2CCallbacks(void *ctx,
                       size_t (*onRead)(void *ctx, uint8_t reg),
                       void (*onWrite)(void *ctx, uint8_t reg, size_t size, const uint8_t *data)) {
    i2cContext = ctx;
    onI2CRead = onRead;
    onI2CWrite = onWrite;
  }

  static void setInterrupt(const bool active) { digitalWrite(I2C_INT_PIN, active ? LOW : HIGH); }

  size_t i2cWrite(const uint8_t data) {
    if (i2cCapturingTx && i2cTxLen < sizeof(i2cTxBuffer)) {
      i2cTxBuffer[i2cTxLen++] = data;
      return 0; // 0 = success (matches BufferHardware convention)
    }
    return 1;
  }

  size_t i2cWrite(const uint8_t *data, const size_t len) {
    if (!i2cCapturingTx)
      return len;
    size_t written = 0;
    while (written < len && i2cTxLen < sizeof(i2cTxBuffer))
      i2cTxBuffer[i2cTxLen++] = data[written++];
    return (written < len) ? len - written : 0;
  }

  template<typename T>
  size_t i2cWriteValue(const T &value) {
    if (i2cWrite(reinterpret_cast<const uint8_t *>(&value), sizeof(T)) == 0)
      return sizeof(T);
    return 0;
  }
#endif

  static uint32_t timeMs() { return millis(); }

private:
  void runStepper(const bool forward, const float speed) {
    digitalWrite(STEPPER_EN, LOW);
    driver.shaft(forward);
    const float rpm = speed * SPEED_RPM_FACTOR;
    const auto v = static_cast<uint32_t>(rpm * STEPPER_MICROSTEPS * STEPS_PER_REV / 60.0f / SCREW_PITCH);
    driver.VACTUAL(v);
  }
};
