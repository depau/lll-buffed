#pragma once

#include <string>

template<class Derived>
class BufferHardware {
public:
  void initHardware() { static_cast<Derived *>(this)->initHardwareImpl(); }

  bool optical1() { return static_cast<Derived *>(this)->optical1Impl(); }
  bool optical2() { return static_cast<Derived *>(this)->optical2Impl(); }
  bool optical3() { return static_cast<Derived *>(this)->optical3Impl(); }
  bool filamentPresent() { return static_cast<Derived *>(this)->filamentPresentImpl(); }
  bool buttonForward() { return static_cast<Derived *>(this)->buttonForwardImpl(); }
  bool buttonBackward() { return static_cast<Derived *>(this)->buttonBackwardImpl(); }
  void setErrorLed(bool on) { static_cast<Derived *>(this)->setErrorLedImpl(on); }
  void setPresenceLed(bool on) { static_cast<Derived *>(this)->setPresenceLedImpl(on); }
  void setPresenceOutput(bool on) { static_cast<Derived *>(this)->setPresenceOutputImpl(on); }
  void stepperPush(float speed) { static_cast<Derived *>(this)->stepperPushImpl(speed); }
  void stepperRetract(float speed) { static_cast<Derived *>(this)->stepperRetractImpl(speed); }
  void stepperHold() { static_cast<Derived *>(this)->stepperHoldImpl(); }
  void stepperOff() { static_cast<Derived *>(this)->stepperOffImpl(); }
  void writeLine(const std::string &l) { static_cast<Derived *>(this)->writeLineImpl(l); }
  bool readChar(char &c) { return static_cast<Derived *>(this)->readCharImpl(c); }
  uint32_t timeMs() { return static_cast<Derived *>(this)->timeMsImpl(); }
};
