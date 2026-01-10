#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

constexpr uint32_t CMD_TIMEOUT = 3000;
constexpr uint32_t SHORT_PRESS_MS = 150;
constexpr uint32_t MULTI_PRESS_MIN_MS = 50;
constexpr uint32_t MULTI_PRESS_MAX_MS = 500;

template<class HW>
class Buffer {
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

  HW hw;
  Mode mode{ Mode::Regular };
  Motor motor{ Motor::Off };

  uint32_t timeoutMs{ 90000 };
  uint32_t holdTimeoutMs{ 10000 };
  bool holdTimeoutEnabled{ false };
  uint8_t multiPressCount{ 2 };
  float speedMmS{ 30.0f };

  bool filamentPresent{ false };
  bool timedOut{ false };

  ButtonState btnFwd;
  ButtonState btnBack;

  char cmdBuf[64]{};
  size_t cmdLen{ 0 };
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

  uint32_t lastTimeoutMs{ 0 };
  uint32_t lastHoldTimeoutMs{ 0 };
  bool lastHoldTimeoutEnabled{ false };
  uint32_t lastMultiPressCount{ 0 };
  float lastSpeedMmS{ 0.0f };

public:
  Buffer() = default;

  void init() {
    hw.initHardware();
    filamentPresent = hw.filamentPresent();
    mode = Mode::Regular;
    lastMode = mode;
    motor = Motor::Off;
    setMotor(filamentPresent ? Motor::Hold : Motor::Off);
    lastMotor = motor;
    hw.setPresenceLed(filamentPresent);
    hw.setPresenceOutput(filamentPresent);
    updateStatus(true);
  }

  void loop() {
    processSerial();
    filamentPresent = hw.filamentPresent();
    handleButtons();

    switch (mode) {
    case Mode::Regular:
      handleRegular();
      break;
    case Mode::SerialMove:
      handleSerialMove();
      break;
    case Mode::Continuous:
      handleContinuous();
      break;
    case Mode::Hold:
      setMotor(Motor::Hold);
      break;
    case Mode::Manual:
      break;
    }

    updateHoldTimeout();
    updateStatus();
  }

#ifdef UNIT_TEST
  HW &getHardware() { return hw; }
#endif

private:
  void processSerial() {
    char c;
    while (hw.readChar(c)) {
      if (c == '\r') {
        continue;
      }
      lastCharTime = hw.timeMs();
      if (c == '\n') {
        if (cmdLen > 0) {
          cmdBuf[cmdLen] = '\0';
          handleCommand(cmdBuf);
          cmdLen = 0;
        }
      } else {
        if (cmdLen < sizeof(cmdBuf) - 1) {
          cmdBuf[cmdLen++] = c;
        }
      }
    }
    if (cmdLen > 0 && hw.timeMs() - lastCharTime > CMD_TIMEOUT) {
      cmdLen = 0;
    }
  }

  static bool startsWith(const char *str, const char *prefix) { return strncmp(str, prefix, strlen(prefix)) == 0; }

  void handleCommand(const char *cmd) {
    if (strcmp(cmd, "push") == 0 || strcmp(cmd, "p") == 0) {
      mode = Mode::Continuous;
      continuousStart = hw.timeMs();
      setMotor(Motor::Push);
    } else if (strcmp(cmd, "retract") == 0 || strcmp(cmd, "r") == 0) {
      mode = Mode::Continuous;
      continuousStart = hw.timeMs();
      setMotor(Motor::Retract);
    } else if (strcmp(cmd, "hold") == 0 || strcmp(cmd, "h") == 0) {
      holdTimeoutEnabled = false;
      mode = Mode::Hold;
      setMotor(Motor::Hold);
    } else if (strcmp(cmd, "regular") == 0 || strcmp(cmd, "n") == 0) {
      mode = Mode::Regular;
      setMotor(hw.filamentPresent() ? Motor::Hold : Motor::Off);
    } else if (strcmp(cmd, "off") == 0 || strcmp(cmd, "o") == 0) {
      mode = Mode::Regular;
      setMotor(Motor::Off);
    } else if (strcmp(cmd, "query") == 0 || strcmp(cmd, "q") == 0) {
      updateStatus(true);
    } else if (startsWith(cmd, "move") || startsWith(cmd, "m ")) {
      if (const char *space = strchr(cmd, ' ')) {
        if (const float val = strtof(space + 1, nullptr); val != 0.0f && speedMmS > 0.0f) {
          moveDir = val > 0 ? Motor::Push : Motor::Retract;
          const float ms = std::fabs(val) * 1000.0f / speedMmS;
          moveEnd = hw.timeMs() + static_cast<uint32_t>(ms);
          mode = Mode::SerialMove;
          setMotor(moveDir);
        }
      }
    } else if (startsWith(cmd, "set_timeout")) {
      timeoutMs = strtoul(cmd + 11, nullptr, 10);
    } else if (startsWith(cmd, "set_hold_timeout")) {
      if (startsWith(cmd, "set_hold_timeout_enabled")) {
        holdTimeoutEnabled = static_cast<bool>(strtoul(cmd + 24, nullptr, 10));
      } else {
        holdTimeoutMs = strtoul(cmd + 16, nullptr, 10);
      }
    } else if (startsWith(cmd, "set_multi_press_count")) {
      multiPressCount = static_cast<uint8_t>(strtoul(cmd + 21, nullptr, 10));
    } else if (startsWith(cmd, "set_speed")) {
      speedMmS = strtof(cmd + 9, nullptr);
    }
    updateStatus();
  }

  void doHandleButton(bool pressed, Motor dir, ButtonState &s, uint32_t now) {
    if (pressed) {
      if (!s.pressed) {
        s.pressed = true;
        s.pressStart = now;
        mode = Mode::Manual;
        setMotor(dir);
      }
      return;
    }
    if (!s.pressed) {
      return;
    }
    const uint32_t dur = now - s.pressStart;
    s.pressed = false;
    if (dur <= SHORT_PRESS_MS) {
      if (now - s.lastRelease >= MULTI_PRESS_MIN_MS && now - s.lastRelease <= MULTI_PRESS_MAX_MS) {
        ++s.count;
      } else {
        s.count = 1;
      }
      s.lastRelease = now;
      if (s.count >= multiPressCount) {
        mode = Mode::Continuous;
        continuousStart = now;
        setMotor(dir);
        s.count = 0;
        return;
      }
      holdTimeoutEnabled = false;
      if (hw.filamentPresent()) {
        mode = Mode::Hold;
        setMotor(Motor::Hold);
      } else {
        mode = Mode::Regular;
        setMotor(Motor::Off);
      }
    } else {
      s.count = 0;
      if (hw.filamentPresent()) {
        mode = Mode::Regular;
        setMotor(Motor::Hold);
      } else {
        mode = Mode::Regular;
        setMotor(Motor::Off);
      }
    }
  }
  void handleButtons() {
    const uint32_t now = hw.timeMs();
    doHandleButton(hw.buttonForward(), Motor::Push, btnFwd, now);
    doHandleButton(hw.buttonBackward(), Motor::Retract, btnBack, now);
  }
  void handleRegular() {
    if (!hw.filamentPresent()) {
      hw.setPresenceLed(false);
      hw.setPresenceOutput(false);
      setMotor(Motor::Off);
      timedOut = false;
      return;
    }
    hw.setPresenceLed(true);
    hw.setPresenceOutput(true);

    bool move = false;
    if (hw.optical1()) {
      setMotor(Motor::Push);
      moveStart = hw.timeMs();
      move = true;
    } else if (hw.optical3()) {
      setMotor(Motor::Retract);
      moveStart = hw.timeMs();
      move = true;
    } else if (hw.optical2()) {
      bool newFilament = filamentPresent && !lastFilament;
      if (motor != Motor::Off || newFilament) {
        setMotor(Motor::Hold);
      }
    }

    if (motor == Motor::Push || motor == Motor::Retract) {
      if (!move && hw.timeMs() - moveStart > timeoutMs) {
        timedOut = true;
        mode = Mode::Hold;
        setMotor(Motor::Hold);
      }
    } else {
      timedOut = false;
    }
  }

  void handleSerialMove() {
    if (hw.timeMs() >= moveEnd) {
      mode = hw.filamentPresent() ? Mode::Hold : Mode::Regular;
      setMotor(hw.filamentPresent() ? Motor::Hold : Motor::Off);
    }
  }

  void handleContinuous() {
    if (!hw.filamentPresent()) {
      mode = Mode::Regular;
      setMotor(Motor::Off);
      return;
    }
    if (timeoutMs > 0 && hw.timeMs() - continuousStart > timeoutMs) {
      mode = Mode::Hold;
      setMotor(Motor::Hold);
    }
  }

  void updateHoldTimeout() {
    if (!holdTimeoutEnabled || motor != Motor::Hold) {
      return;
    }
    if (hw.timeMs() - holdStart >= holdTimeoutMs) {
      mode = Mode::Regular;
      setMotor(Motor::Off);
    }
  }

  void updateStatus(bool force = false) {
    if (const bool fil = hw.filamentPresent(); fil != lastFilament || force) {
      hw.writeLineF("filament_present=%d", fil ? 1 : 0);
      lastFilament = fil;
    }
    if (mode != lastMode || force) {
      auto modeStr = "";
      switch (mode) {
      case Mode::Regular:
        modeStr = "regular";
        break;
      case Mode::Continuous:
        modeStr = "continuous";
        break;
      case Mode::SerialMove:
        modeStr = "serial";
        break;
      case Mode::Hold:
        modeStr = "hold";
        break;
      case Mode::Manual:
        modeStr = "manual";
        break;
      }
      hw.writeLineF("mode=%s", modeStr);
      lastMode = mode;
    }
    if (motor != lastMotor || force) {
      auto statusStr = "";
      switch (motor) {
      case Motor::Push:
        statusStr = "push";
        break;
      case Motor::Retract:
        statusStr = "retract";
        break;
      case Motor::Hold:
        statusStr = "hold";
        break;
      case Motor::Off:
        statusStr = "off";
        break;
      }
      hw.writeLineF("status=%s", statusStr);
      lastMotor = motor;
    }
    if (timedOut != lastTimedOut || force) {
      hw.writeLineF("timed_out=%d", timedOut ? 1 : 0);
      lastTimedOut = timedOut;
    }
    if (lastTimeoutMs != timeoutMs || force) {
      hw.writeLineF("timeout=%lu", timeoutMs);
      lastTimeoutMs = timeoutMs;
    }
    if (lastHoldTimeoutMs != holdTimeoutMs || force) {
      hw.writeLineF("hold_timeout=%lu", holdTimeoutMs);
      lastHoldTimeoutMs = holdTimeoutMs;
    }
    if (lastHoldTimeoutEnabled != holdTimeoutEnabled || force) {
      hw.writeLineF("hold_timeout_enabled=%d", holdTimeoutEnabled ? 1 : 0);
      lastHoldTimeoutEnabled = holdTimeoutEnabled;
    }
    if (lastMultiPressCount != multiPressCount || force) {
      hw.writeLineF("multi_press_count=%u", multiPressCount);
      lastMultiPressCount = multiPressCount;
    }
    if (std::fabs(lastSpeedMmS - speedMmS) > 0.01f || force) {
      hw.writeLineF("speed=%.2f", speedMmS);
      lastSpeedMmS = speedMmS;
    }
  }

  void setMotor(Motor m) {
    if (motor == m) {
      return;
    }
    motor = m;
    switch (motor) {
    case Motor::Push:
      hw.stepperPush(speedMmS);
      break;
    case Motor::Retract:
      hw.stepperRetract(speedMmS);
      break;
    case Motor::Hold:
      hw.stepperHold();
      holdStart = hw.timeMs();
      break;
    case Motor::Off:
      hw.stepperOff();
      break;
    }
  }
};
