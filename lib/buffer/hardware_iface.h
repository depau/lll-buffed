#pragma once

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
  bool readChar(char &c) { return static_cast<Derived *>(this)->readCharImpl(c); }
  uint32_t timeMs() { return static_cast<Derived *>(this)->timeMsImpl(); }
  void writeLine(const char *l) { static_cast<Derived *>(this)->writeLineImpl(l); }

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
};
