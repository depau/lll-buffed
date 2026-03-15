#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

#include "tiny_printf.hpp"

#ifndef BUFFER_ID
#define BUFFER_ID 0
#endif

#if BUFFER_ID < 0 || BUFFER_ID > 9
#error "BUFFER_ID must be between 0 and 9"
#endif

constexpr uint32_t CMD_TIMEOUT = 3000;
constexpr uint32_t SHORT_PRESS_MS = 150;
constexpr uint32_t MULTI_PRESS_MIN_MS = 50;
constexpr uint32_t MULTI_PRESS_MAX_MS = 500;

constexpr uint32_t DEFAULT_TIMEOUT_MS = 90000;
constexpr uint32_t DEFAULT_HOLD_TIMEOUT_MS = 10000;
constexpr uint8_t DEFAULT_MULTI_PRESS_COUNT = 2;
constexpr float DEFAULT_SPEED_MM_S = 30.0F;
constexpr uint32_t DEFAULT_EMPTYING_PUSH_TIMEOUT_MS = 2500;

constexpr size_t UART_CMD_BUF_SIZE = 64;

#ifdef ENABLE_I2C_PROTOCOL
enum I2CRegister : uint8_t {
  REG_COMMAND = 0x00, // 1 byte
  REG_MOVE_DIST = 0x01, // 4 bytes (float)
  REG_STATUS = 0x05, // 1 byte
  REG_MODE = 0x06, // 1 byte
  REG_MOTOR = 0x07, // 1 byte
  REG_PARAM_SPEED = 0x08, // 4 bytes (float)
  REG_PARAM_TIMEOUT = 0x0C, // 4 bytes (uint32_t)
  REG_PARAM_EMPTYING_TIMEOUT = 0x10, // 4 bytes (uint32_t)
  REG_PARAM_HOLD_TIMEOUT = 0x14, // 4 bytes (uint32_t)
  REG_PARAM_HOLD_TIMEOUT_ENABLED = 0x18, // 1 byte
  REG_PARAM_MULTI_PRESS_COUNT = 0x19, // 1 byte
};

enum I2CCommand : uint8_t {
  CMD_OFF = 0,
  CMD_REGULAR = 1,
  CMD_HOLD = 2,
  CMD_PUSH = 3,
  CMD_RETRACT = 4,
  CMD_REBOOT_DFU = 0xDF,
};
#endif

template<typename T>
void reinterpret_assign(T &dest, const uint8_t *src) {
  std::memcpy(&dest, src, sizeof(T));
}

template<class HW>
class Buffer {
public:
  enum class Mode : uint8_t {
    Regular = 0,
    Continuous,
    MoveCommand,
    Hold,
    Manual,
    Emptying
  };

private:
  enum class Motor : uint8_t {
    Off = 0,
    Push,
    Retract,
    Hold,
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

  uint32_t timeoutMs{ DEFAULT_TIMEOUT_MS };
  uint32_t holdTimeoutMs{ DEFAULT_HOLD_TIMEOUT_MS };
  bool holdTimeoutEnabled{ false };
  uint8_t multiPressCount{ DEFAULT_MULTI_PRESS_COUNT };
  float speedMmS{ DEFAULT_SPEED_MM_S };
  uint32_t emptyingPushTimeoutMs{ DEFAULT_EMPTYING_PUSH_TIMEOUT_MS };

  bool filamentPresent{ false };
  bool timedOut{ false };

  ButtonState btnFwd;
  ButtonState btnBack;

  char cmdBuf[UART_CMD_BUF_SIZE]{};
  size_t cmdLen{ 0 };
  uint32_t lastCharTime{ 0 };

  uint32_t moveEnd{ 0 };
  Motor moveDir{ Motor::Off };

  uint32_t moveStart{ 0 };
  uint32_t holdStart{ 0 };
  uint32_t continuousStart{ 0 };
  uint32_t emptyingStart{ 0 };
  uint32_t emptyingPushStart{ 0 };

  Mode lastMode{ Mode::Regular };
  Motor lastMotor{ Motor::Off };
  bool lastFilament{ false };
  bool lastTimedOut{ false };

  uint32_t lastTimeoutMs{ 0 };
  uint32_t lastHoldTimeoutMs{ 0 };
  bool lastHoldTimeoutEnabled{ false };
  uint32_t lastMultiPressCount{ 0 };
  float lastSpeedMmS{ 0.0F };
  uint32_t lastEmptyingPushTimeoutMs{ 0 };

public:
  Buffer() = default;

  void init() {
    hw.initHardware();
    filamentPresent = hw.filamentPresent();
    lastFilament = filamentPresent;
    setMode(Mode::Regular);
    setMotor(filamentPresent ? Motor::Hold : Motor::Off);
    lastMotor = motor;
    hw.setPresenceLed(filamentPresent);
    hw.setPresenceOutput(filamentPresent);
    updateStatus(true);

#ifdef ENABLE_I2C_PROTOCOL
    hw.setI2CCallbacks(this, onI2CReadTrampoline, onI2CWriteTrampoline);
#endif
  }

  Mode getMode() const { return mode; }

  void loop() {
    hw.loop();
#ifdef ENABLE_UART_PROTOCOL
    processUartCommands();
#endif

    lastFilament = filamentPresent;
    filamentPresent = hw.filamentPresent();
    if (filamentPresent != lastFilament) {
      hw.setPresenceLed(filamentPresent);
      hw.setPresenceOutput(filamentPresent);
    }

    handleButtons();

    switch (mode) {
    case Mode::Regular:
      handleRegular();
      break;
    case Mode::Emptying:
      handleEmptying();
      break;
    case Mode::MoveCommand:
      handleMoveCommand();
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
#ifdef ENABLE_UART_PROTOCOL
  void processUartCommands() {
    char c = '\0';
    while (hw.readChar(c)) {
      if (c == '\r') {
        continue;
      }
      lastCharTime = hw.timeMs();
      if (c == '\n') {
        if (cmdLen > 0) {
          cmdBuf[cmdLen] = '\0';

          if (cmdBuf[0] >= '0' && cmdBuf[0] <= '9') {
            // A specific buffer is being addressed: check if it matches ours
            if (const int bufID = cmdBuf[0] - '0'; bufID != BUFFER_ID) {
              cmdLen = 0;
              continue;
            }
          }

          handleUartCommand(cmdBuf);
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

  static const char *startsWith(const char *str, ...) {
    va_list args;
    va_start(args, str);
    size_t offset = 0;
    while (const char *prefix = va_arg(args, const char *)) {
      const size_t len = std::strlen(prefix);
      if (std::strncmp(str + offset, prefix, len) != 0) {
        va_end(args);
        return nullptr;
      }
      offset += len;
    }
    va_end(args);
    return str + offset;
  }

  void handleUartCommand(const char *cmd) {
    const char *arg = nullptr;
    if (strcmp(cmd, "push") == 0 || strcmp(cmd, "p") == 0) {
      if (!hw.filamentPresent())
        return;
      setMode(Mode::Continuous);
      setMotor(Motor::Push);
    } else if (strcmp(cmd, "retract") == 0 || strcmp(cmd, "r") == 0) {
      if (!hw.filamentPresent())
        return;
      setMode(Mode::Continuous);
      setMotor(Motor::Retract);
    } else if (strcmp(cmd, "hold") == 0 || strcmp(cmd, "h") == 0) {
      holdTimeoutEnabled = false;
      setMode(Mode::Hold);
      setMotor(Motor::Hold);
    } else if (strcmp(cmd, "regular") == 0 || strcmp(cmd, "n") == 0) {
      setMode(Mode::Regular);
      setMotor(hw.filamentPresent() ? Motor::Hold : Motor::Off);
    } else if (strcmp(cmd, "off") == 0 || strcmp(cmd, "o") == 0) {
      setMode(Mode::Regular);
      setMotor(Motor::Off);
    } else if (strcmp(cmd, "query") == 0 || strcmp(cmd, "q") == 0) {
      updateStatus(true);
    } else if (strcmp(cmd, "reboot_dfu") == 0) {
      hw.writeLineF("mode=dfu");
      HW::rebootDFU();
    } else if (((arg = startsWith(cmd, "move ", nullptr))) || ((arg = startsWith(cmd, "m ", nullptr)))) {
      if (const float val = tiny_strtof(arg); val != 0.0F && speedMmS > 0.0F) {
        moveDir = val > 0 ? Motor::Push : Motor::Retract;
        const float ms = std::fabs(val) * 1000.0F / speedMmS;
        moveEnd = hw.timeMs() + static_cast<uint32_t>(ms);
        setMode(Mode::MoveCommand);
        setMotor(moveDir);
      }
    } else if ((arg = startsWith(cmd, "set_", "timeout", " ", nullptr))) {
      timeoutMs = tiny_strtoul(arg);
    } else if ((arg = startsWith(cmd, "set_", "hold_", "timeout", nullptr))) {
      const char *oldArg = arg;
      if ((arg = startsWith(arg, "_en ", nullptr))) {
        holdTimeoutEnabled = static_cast<bool>(tiny_strtoul(arg));
      } else {
        holdTimeoutMs = tiny_strtoul(oldArg);
      }
    } else if ((arg = startsWith(cmd, "set_", "multi_press_count", " ", nullptr))) {
      multiPressCount = static_cast<uint8_t>(tiny_strtoul(arg));
    } else if ((arg = startsWith(cmd, "set_", "speed", " ", nullptr))) {
      speedMmS = tiny_strtof(arg);
    } else if ((arg = startsWith(cmd, "set_", "emptying_", "timeout", " ", nullptr))) {
      emptyingPushTimeoutMs = tiny_strtoul(arg);
    }
    updateStatus();
  }
#endif

#ifdef ENABLE_I2C_PROTOCOL
  static void onI2CReadTrampoline(void *ctx, const uint8_t reg) {
    if (auto *self = static_cast<Buffer *>(ctx))
      self->onI2CRead(reg);
  }

  static void onI2CWriteTrampoline(void *ctx, const uint8_t reg, const size_t size, const uint8_t *data) {
    if (auto *self = static_cast<Buffer *>(ctx))
      self->onI2CWrite(reg, size, data);
  }

  void onI2CRead(const uint8_t reg) {
    if (reg == REG_STATUS)
      hw.setInterrupt(false);

    switch (reg) {
    case REG_STATUS: {
      uint8_t status = 0;
      if (filamentPresent)
        status |= 0x01;
      if (timedOut)
        status |= 0x02;
      hw.i2cWrite(status);
      break;
    }
    case REG_MODE:
      hw.i2cWrite(static_cast<uint8_t>(mode));
      break;
    case REG_MOTOR:
      hw.i2cWrite(static_cast<uint8_t>(motor));
      break;
    case REG_PARAM_SPEED:
      hw.i2cWriteValue(speedMmS);
      break;
    case REG_PARAM_TIMEOUT:
      hw.i2cWriteValue(timeoutMs);
      break;
    case REG_PARAM_EMPTYING_TIMEOUT:
      hw.i2cWriteValue(emptyingPushTimeoutMs);
      break;
    case REG_PARAM_HOLD_TIMEOUT:
      hw.i2cWriteValue(holdTimeoutMs);
      break;
    case REG_PARAM_HOLD_TIMEOUT_ENABLED:
      hw.i2cWrite(holdTimeoutEnabled ? 1 : 0);
      break;
    case REG_PARAM_MULTI_PRESS_COUNT:
      hw.i2cWrite(multiPressCount);
      break;
    default:
      hw.i2cWrite(0);
      break;
    }
  }

  void onI2CWrite(const uint8_t reg, const size_t size, const uint8_t *data) {
    if (size == 0)
      return;

    switch (reg) {
    case REG_COMMAND:
      handleI2CCommand(data[0]);
      break;
    case REG_MOVE_DIST:
      if (size >= sizeof(float)) {
        const auto dist = *reinterpret_cast<const float *>(data);
        if (dist != 0.0F && speedMmS > 0.0F) {
          moveDir = dist > 0 ? Motor::Push : Motor::Retract;
          const float ms = std::fabs(dist) * 1000.0F / speedMmS;
          moveEnd = hw.timeMs() + static_cast<uint32_t>(ms);
          setMode(Mode::MoveCommand);
          setMotor(moveDir);
          updateStatus();
        }
      }
      break;
    case REG_PARAM_SPEED:
      if (size >= sizeof(speedMmS)) {
        reinterpret_assign(speedMmS, data);
        updateStatus();
      }
      break;
    case REG_PARAM_TIMEOUT:
      if (size >= sizeof(timeoutMs)) {
        reinterpret_assign(timeoutMs, data);
        updateStatus();
      }
      break;
    case REG_PARAM_EMPTYING_TIMEOUT:
      if (size >= sizeof(emptyingPushTimeoutMs)) {
        reinterpret_assign(emptyingPushTimeoutMs, data);
        updateStatus();
      }
      break;
    case REG_PARAM_HOLD_TIMEOUT:
      if (size >= sizeof(holdTimeoutMs)) {
        reinterpret_assign(holdTimeoutMs, data);
        updateStatus();
      }
      break;
    case REG_PARAM_HOLD_TIMEOUT_ENABLED:
      if (size >= 1) {
        holdTimeoutEnabled = data[0] != 0;
        updateStatus();
      }
      break;
    case REG_PARAM_MULTI_PRESS_COUNT:
      if (size >= sizeof(multiPressCount)) {
        reinterpret_assign(multiPressCount, data);
        updateStatus();
      }
      break;
    default:
      // Unknown register, ignore
      break;
    }
  }

  void handleI2CCommand(const uint8_t cmd) {
    switch (cmd) {
    case CMD_PUSH:
      if (!hw.filamentPresent())
        break;
      setMode(Mode::Continuous);
      setMotor(Motor::Push);
      break;
    case CMD_RETRACT:
      if (!hw.filamentPresent())
        break;
      setMode(Mode::Continuous);
      setMotor(Motor::Retract);
      break;
    case CMD_HOLD:
      holdTimeoutEnabled = false;
      setMode(Mode::Hold);
      setMotor(Motor::Hold);
      break;
    case CMD_REGULAR:
      setMode(Mode::Regular);
      setMotor(hw.filamentPresent() ? Motor::Hold : Motor::Off);
      break;
    case CMD_OFF:
      setMode(Mode::Regular);
      setMotor(Motor::Off);
      break;
    case CMD_REBOOT_DFU:
      hw.writeLineF("mode=dfu");
      HW::rebootDFU();
      break;
    default:
      // Unknown command, ignore;
      break;
    }
    updateStatus();
  }
#endif

  void doHandleButton(bool pressed, Motor dir, ButtonState &s, uint32_t now) {
    if (pressed) {
      if (!s.pressed) {
        s.pressed = true;
        s.pressStart = now;
        setMode(Mode::Manual);
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
        if (hw.filamentPresent()) {
          setMode(Mode::Continuous);
          setMotor(dir);
        }
        s.count = 0;
        return;
      }
      holdTimeoutEnabled = false;
      if (hw.filamentPresent()) {
        setMode(Mode::Hold);
        setMotor(Motor::Hold);
      } else {
        setMode(Mode::Regular);
        setMotor(Motor::Off);
      }
    } else {
      s.count = 0;
      if (hw.filamentPresent()) {
        setMode(Mode::Regular);
        setMotor(Motor::Hold);
      } else {
        setMode(Mode::Regular);
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
      if (lastFilament) {
        setMode(Mode::Emptying);
      } else {
        setMotor(Motor::Off);
        timedOut = false;
      }
      return;
    }

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
      if (motor != Motor::Off) {
        setMotor(Motor::Hold);
      }
    }

    if (motor == Motor::Push || motor == Motor::Retract) {
      if (!move && hw.timeMs() - moveStart >= timeoutMs) {
        timedOut = true;
        setMode(Mode::Hold);
        setMotor(Motor::Hold);
      }
    } else {
      timedOut = false;
    }
  }

  void handleEmptying() {
    if (hw.filamentPresent()) {
      setMode(Mode::Regular);
      return;
    }

    if (hw.timeMs() - emptyingStart >= timeoutMs) {
      setMode(Mode::Regular);
      setMotor(Motor::Off);
      return;
    }

    // Work similarly to regular mode
    if (hw.optical1() || (motor == Motor::Push && !hw.optical1() && !hw.optical2() && !hw.optical3())) {
      if (motor != Motor::Push || emptyingPushStart == 0) {
        emptyingPushStart = hw.timeMs();
      }
      setMotor(Motor::Push);
    } else if (hw.optical2()) {
      setMotor(Motor::Hold);
    } else if (hw.optical3()) {
      setMotor(Motor::Retract);
    }

    // If we've been pushing without moving for a while, assume the tail of the filament has left the gear and
    // therefore the spring is no longer under tension
    if (motor == Motor::Push && emptyingPushTimeoutMs > 0 && hw.timeMs() - emptyingPushStart >= emptyingPushTimeoutMs) {
      setMode(Mode::Regular);
      setMotor(Motor::Off);
    }
  }

  void handleMoveCommand() {
    if (hw.timeMs() >= moveEnd) {
      if (hw.filamentPresent()) {
        setMode(Mode::Hold);
        setMotor(Motor::Hold);
      } else {
        setMode(Mode::Regular);
        setMotor(Motor::Off);
      }
    }
  }

  void handleContinuous() {
    if (!hw.filamentPresent()) {
      if (motor == Motor::Retract) {
        setMode(Mode::Regular);
        setMotor(Motor::Off);
      } else {
        setMode(Mode::Emptying);
      }
      return;
    }
    if (timeoutMs > 0 && hw.timeMs() - continuousStart >= timeoutMs) {
      setMode(Mode::Hold);
      setMotor(Motor::Hold);
    }
  }

  void updateHoldTimeout() {
    if (!holdTimeoutEnabled || motor != Motor::Hold) {
      return;
    }
    if (hw.timeMs() - holdStart >= holdTimeoutMs) {
      setMode(Mode::Regular);
      setMotor(Motor::Off);
    }
  }

  void updateStatus(bool force = false) {
#ifdef ENABLE_UART_PROTOCOL
    if (filamentPresent != lastFilament || force) {
      hw.writeLineF("filament_present=%d", filamentPresent ? 1 : 0);
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
      case Mode::MoveCommand:
        modeStr = "move_command";
        break;
      case Mode::Hold:
        modeStr = "hold";
        break;
      case Mode::Manual:
        modeStr = "manual";
        break;
      case Mode::Emptying:
        modeStr = "emptying";
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
      hw.writeLineF("%s=%u", "timeout", timeoutMs);
      lastTimeoutMs = timeoutMs;
    }
    if (lastHoldTimeoutMs != holdTimeoutMs || force) {
      hw.writeLineF("%s=%u", "hold_timeout", holdTimeoutMs);
      lastHoldTimeoutMs = holdTimeoutMs;
    }
    if (lastHoldTimeoutEnabled != holdTimeoutEnabled || force) {
      hw.writeLineF("%s%s=%d", "hold_timeout", "_en", holdTimeoutEnabled ? 1 : 0);
      lastHoldTimeoutEnabled = holdTimeoutEnabled;
    }
    if (lastMultiPressCount != multiPressCount || force) {
      hw.writeLineF("%s=%u", "multi_press_count", multiPressCount);
      lastMultiPressCount = multiPressCount;
    }
    if (std::fabs(lastSpeedMmS - speedMmS) > 0.01F || force) {
      hw.writeLineF("speed=%f", speedMmS);
      lastSpeedMmS = speedMmS;
    }
    if (lastEmptyingPushTimeoutMs != emptyingPushTimeoutMs || force) {
      hw.writeLineF("%s%s=%u", "emptying_", "timeout", emptyingPushTimeoutMs);
      lastEmptyingPushTimeoutMs = emptyingPushTimeoutMs;
    }
#endif
#ifdef ENABLE_I2C_PROTOCOL
    bool changed = false;
    if (hw.filamentPresent() != lastFilament || force)
      changed = true;
    if (mode != lastMode || force)
      changed = true;
    if (motor != lastMotor || force)
      changed = true;
    if (timedOut != lastTimedOut || force)
      changed = true;
    if (lastTimeoutMs != timeoutMs || force)
      changed = true;
    if (lastHoldTimeoutMs != holdTimeoutMs || force)
      changed = true;
    if (lastHoldTimeoutEnabled != holdTimeoutEnabled || force)
      changed = true;
    if (lastMultiPressCount != multiPressCount || force)
      changed = true;
    if (std::fabs(lastSpeedMmS - speedMmS) > 0.01F || force)
      changed = true;
    if (lastEmptyingPushTimeoutMs != emptyingPushTimeoutMs || force)
      changed = true;

    if (changed) {
      hw.setInterrupt(true);
    }
#endif
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

  void setMode(Mode m) {
    if (mode == m) {
      return;
    }
    const uint32_t now = hw.timeMs();

    switch (m) {
    case Mode::Continuous:
      continuousStart = now;
      break;
    case Mode::Emptying:
      emptyingStart = now;
      emptyingPushStart = 0;
      break;
    default:
      break;
    }

    mode = m;
  }
};
