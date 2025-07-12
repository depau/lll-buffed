#pragma once

#include <string>
#include <vector>

#include "hardware_iface.h"

class FakeHardware : public BufferHardware<FakeHardware> {
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

  void initHardwareImpl() {}

  bool optical1Impl() { return opt1; }
  bool optical2Impl() { return opt2; }
  bool optical3Impl() { return opt3; }
  bool filamentPresentImpl() { return presence; }
  bool buttonForwardImpl() { return btnF; }
  bool buttonBackwardImpl() { return btnB; }
  void setErrorLedImpl(bool on) { errorLed = on; }
  void setPresenceLedImpl(bool on) { presLed = on; }
  void setPresenceOutputImpl(bool on) { presOut = on; }
  void stepperPushImpl(float) { lastMotor = TestMotor::Push; }
  void stepperRetractImpl(float) { lastMotor = TestMotor::Retract; }
  void stepperHoldImpl() { lastMotor = TestMotor::Hold; }
  void stepperOffImpl() { lastMotor = TestMotor::Off; }
  void writeLineImpl(const std::string &l) { lines.push_back(l); }
  bool readCharImpl(char &c) {
    if (input.empty())
      return false;
    c = input.front();
    input.erase(input.begin());
    return true;
  }
  uint32_t timeMsImpl() { return now; }

  void serialSend(const std::string &line) { input.insert(input.end(), line.begin(), line.end()); }
};