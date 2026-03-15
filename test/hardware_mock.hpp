#pragma once

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include "tiny_printf.hpp"

// FakeHardware does not inherit from BufferHardware anymore because BufferHardware is now a concrete class.
// It implements the same implicit interface required by the Buffer template.

class FakeHardware {
public:
  bool opt1{ false };
  bool opt2{ false };
  bool opt3{ false };
  bool presence{ true };
  bool btnF{ false };
  bool btnB{ false };
  bool errorLed{ false };
  bool presLed{ false };
  bool presOut{ false };
  enum class TestMotor {
    Push,
    Retract,
    Hold,
    Off
  };
  TestMotor lastMotor{ TestMotor::Off };
  std::vector<std::string> lines;
  uint32_t now{ 0 };
  std::vector<char> input;

  static void loop() {}
  static void initHardware() {}
  [[noreturn]] static void rebootDFU() { throw std::runtime_error("Reboot to DFU called"); }

  bool optical1() const { return opt1; }
  bool optical2() const { return opt2; }
  bool optical3() const { return opt3; }
  bool filamentPresent() const { return presence; }
  bool buttonForward() const { return btnF; }
  bool buttonBackward() const { return btnB; }
  void setErrorLed(bool on) { errorLed = on; }
  void setPresenceLed(bool on) { presLed = on; }
  void setPresenceOutput(bool on) { presOut = on; }
  void stepperPush(float) { lastMotor = TestMotor::Push; }
  void stepperRetract(float) { lastMotor = TestMotor::Retract; }
  void stepperHold() { lastMotor = TestMotor::Hold; }
  void stepperOff() { lastMotor = TestMotor::Off; }
  void writeLine(const char *l) { lines.push_back(l); }

  template<size_t BufSize = 64>
  [[gnu::format(printf, 2, 3)]]
  void writeLineF(const char *fmt, ...) {
    char buf[BufSize + 1];
    va_list args;
    va_start(args, fmt);
    tiny_vsnprintf(buf, BufSize, fmt, args);
    va_end(args);
    writeLine(buf);
  }

  bool readChar(char &c) {
    if (input.empty())
      return false;
    c = input.front();
    input.erase(input.begin());
    return true;
  }
  uint32_t timeMs() const { return now; }

  void serialSend(const std::string &line) { input.insert(input.end(), line.begin(), line.end()); }

#ifdef ENABLE_I2C_PROTOCOL
  size_t (*onI2CRead)(void *, uint8_t){ nullptr };
  void (*onI2CWrite)(void *, uint8_t, size_t, const uint8_t *){ nullptr };
  void *requestCtx{ nullptr };
  void *receiveCtx{ nullptr };
  uint8_t lastReg{ 0 };
  bool intActive{ false };
  std::vector<uint8_t> i2cTxBuffer;
  // i2cRxBuffer no longer needed for buffering, we pass directly

  void setI2CCallbacks(void *ctx,
                       size_t (*onRead)(void *ctx, uint8_t reg),
                       void (*onWrite)(void *ctx, uint8_t reg, size_t size, const uint8_t *data)) {
    requestCtx = ctx;
    onI2CRead = onRead;
    receiveCtx = ctx;
    onI2CWrite = onWrite;
  }
  void setInterrupt(bool active) { intActive = active; }

  size_t i2cWrite(uint8_t data) {
    i2cTxBuffer.push_back(data);
    return 1;
  }

  size_t i2cWriteBuffer(const uint8_t *data, size_t len) {
    i2cTxBuffer.insert(i2cTxBuffer.end(), data, data + len);
    return len;
  }

  template<typename T>
  size_t i2cWriteValue(const T &value) {
    i2cWriteBuffer(reinterpret_cast<const uint8_t *>(&value), sizeof(T));
    return sizeof(T);
  }

  // Helpers to simulate I2C events
  void simulateI2CRequest() {
    if (onI2CRead) {
      i2cTxBuffer.clear();
      onI2CRead(requestCtx, lastReg);
    }
  }
  void simulateI2CReceive(const std::vector<uint8_t> &data) {
    if (onI2CWrite && !data.empty()) {
      lastReg = data[0];
      if (data.size() > 1) {
        onI2CWrite(receiveCtx, lastReg, data.size() - 1, &data[1]);
      }
    }
  }
#endif
};
