#pragma once

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

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

  static void initHardware() {}

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
    vsnprintf(buf, BufSize, fmt, args);
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
};
