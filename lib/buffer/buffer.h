#pragma once

#include <cmath>
#include <cstdint>
#include <string>

#include "buffer_hardware.h"

constexpr uint32_t CMD_TIMEOUT = 3000;
constexpr uint32_t SHORT_PRESS_MS = 150;
constexpr uint32_t MULTI_PRESS_MIN_MS = 50;
constexpr uint32_t MULTI_PRESS_MAX_MS = 500;

template<class HW>
class Buffer {
public:
  Buffer();
#ifdef UNIT_TEST
  HW &getHardware() { return hw; }
#endif
  void init();
  void loop();

private:
  enum class Mode {
    Regular,
    Continuous,
    SerialMove,
    Hold,
    Manual
  };
  enum class Motor {
    Push,
    Retract,
    Hold,
    Off
  };
  struct ButtonState {
    bool pressed{ false };
    uint32_t pressStart{ 0 };
    uint32_t lastRelease{ 0 };
    uint8_t count{ 0 };
  };

  void processSerial();
  void handleCommand(const std::string &cmd);
  void doHandleButton(bool pressed, Motor dir, ButtonState &s, uint32_t now);
  void handleButtons();
  void handleRegular();
  void handleSerialMove();
  void handleContinuous();
  void updateHoldTimeout();
  void updateStatus(bool force = false);
  void setMotor(Motor m);

  HW hw;
  Mode mode{ Mode::Regular };
  Motor motor{ Motor::Off };

  uint32_t timeoutMs{ 90000 };
  uint32_t holdTimeoutMs{ 60000 };
  bool holdTimeoutEnabled{ false };
  uint8_t multiPressCount{ 2 };
  float speedMmS{ 30.0f };

  bool filamentPresent{ false };
  bool timedOut{ false };

  ButtonState btnFwd;
  ButtonState btnBack;

  std::string cmdBuf;
  uint32_t lastCharTime{ 0 };

  uint32_t moveEnd{ 0 };
  Motor moveDir{ Motor::Off };

  uint32_t moveStart{ 0 };
  uint32_t holdStart{ 0 };
  uint32_t continuousStart{ 0 };

  Mode lastMode{ Mode::Regular };
  Motor lastMotor{ Motor::Off };
  bool lastFilament{ false };
  bool lastTimedOut{ false };
};

#ifdef ARDUINO
void buffer_init();
void buffer_loop();
#endif

#include "buffer.tpp"
