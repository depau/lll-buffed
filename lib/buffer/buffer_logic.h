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

template<class HW>
Buffer<HW>::Buffer() = default;

template<class HW>
void Buffer<HW>::init() {
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

template<class HW>
void Buffer<HW>::setMotor(Motor m) {
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

template<class HW>
void Buffer<HW>::processSerial() {
  char c;
  while (hw.readChar(c)) {
    if (c == '\r') {
      continue;
    }
    lastCharTime = hw.timeMs();
    if (c == '\n') {
      std::string cmd = cmdBuf;
      cmdBuf.clear();
      if (!cmd.empty()) {
        handleCommand(cmd);
      }
    } else {
      if (cmdBuf.size() < 64) {
        cmdBuf += c;
      }
    }
  }
  if (!cmdBuf.empty() && hw.timeMs() - lastCharTime > CMD_TIMEOUT) {
    cmdBuf.clear();
  }
}

template<class HW>
void Buffer<HW>::handleCommand(const std::string &cmd) {
  if (cmd == "push" || cmd == "p") {
    mode = Mode::Continuous;
    continuousStart = hw.timeMs();
    setMotor(Motor::Push);
  } else if (cmd == "retract" || cmd == "r") {
    mode = Mode::Continuous;
    continuousStart = hw.timeMs();
    setMotor(Motor::Retract);
  } else if (cmd == "hold" || cmd == "h") {
    holdTimeoutEnabled = false;
    mode = Mode::Hold;
    setMotor(Motor::Hold);
  } else if (cmd == "regular" || cmd == "n") {
    mode = Mode::Regular;
    setMotor(hw.filamentPresent() ? Motor::Hold : Motor::Off);
  } else if (cmd == "off" || cmd == "o") {
    mode = Mode::Regular;
    setMotor(Motor::Off);
  } else if (cmd == "query" || cmd == "q") {
    updateStatus(true);
  } else if (cmd.rfind("move", 0) == 0 || cmd.rfind("m ", 0) == 0) {
    size_t pos = cmd.find(' ');
    if (pos != std::string::npos) {
      float val = std::stof(cmd.substr(pos + 1));
      if (val != 0.0f && speedMmS > 0.0f) {
        moveDir = val > 0 ? Motor::Push : Motor::Retract;
        const float ms = std::fabs(val) / speedMmS * 1000.0f;
        moveEnd = hw.timeMs() + static_cast<uint32_t>(ms);
        mode = Mode::SerialMove;
        setMotor(moveDir);
      }
    }
  } else if (cmd.rfind("set_timeout", 0) == 0) {
    timeoutMs = static_cast<uint32_t>(std::stoul(cmd.substr(11)));
  } else if (cmd.rfind("set_hold_timeout", 0) == 0) {
    holdTimeoutMs = static_cast<uint32_t>(std::stoul(cmd.substr(16)));
  } else if (cmd.rfind("set_multi_press_count", 0) == 0) {
    multiPressCount = static_cast<uint8_t>(std::stoul(cmd.substr(21)));
  } else if (cmd.rfind("set_speed", 0) == 0) {
    speedMmS = std::stof(cmd.substr(9));
  } else if (cmd == "enable_hold_timeout") {
    holdTimeoutEnabled = true;
  } else if (cmd == "disable_hold_timeout") {
    holdTimeoutEnabled = false;
  }
  updateStatus();
}

template<class HW>
void Buffer<HW>::doHandleButton(bool pressed, Motor dir, ButtonState &s, uint32_t now) {
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
      s.count++;
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

template<class HW>
void Buffer<HW>::handleButtons() {
  const uint32_t now = hw.timeMs();
  doHandleButton(hw.buttonForward(), Motor::Push, btnFwd, now);
  doHandleButton(hw.buttonBackward(), Motor::Retract, btnBack, now);
}

template<class HW>
void Buffer<HW>::handleRegular() {
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

template<class HW>
void Buffer<HW>::handleSerialMove() {
  if (hw.timeMs() >= moveEnd) {
    mode = Mode::Regular;
    setMotor(hw.filamentPresent() ? Motor::Hold : Motor::Off);
    return;
  }
}

template<class HW>
void Buffer<HW>::handleContinuous() {
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

template<class HW>
void Buffer<HW>::updateHoldTimeout() {
  if (!holdTimeoutEnabled || motor != Motor::Hold) {
    return;
  }
  if (hw.timeMs() - holdStart >= holdTimeoutMs) {
    mode = Mode::Regular;
    setMotor(Motor::Off);
  }
}

template<class HW>
void Buffer<HW>::updateStatus(bool force) {
  bool fil = hw.filamentPresent();
  if (fil != lastFilament || force) {
    hw.writeLine(std::string("filament_present=") + (fil ? "1" : "0"));
    lastFilament = fil;
  }
  if (mode != lastMode || force) {
    const char *modeStr = "";
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
    hw.writeLine(std::string("mode=") + modeStr);
    lastMode = mode;
  }
  if (motor != lastMotor || force) {
    const char *statusStr = "";
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
    hw.writeLine(std::string("status=") + statusStr);
    lastMotor = motor;
  }
  if (timedOut != lastTimedOut || force) {
    hw.writeLine(std::string("timed_out=") + (timedOut ? "1" : "0"));
    lastTimedOut = timedOut;
  }
}

template<class HW>
void Buffer<HW>::loop() {
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

#ifdef ARDUINO
#include <Arduino.h>
#include <TMCStepper.h>

namespace {
inline constexpr uint32_t OPTICAL_SENSOR_1 = PB4;
inline constexpr uint32_t OPTICAL_SENSOR_2 = PB3;
inline constexpr uint32_t OPTICAL_SENSOR_3 = PB2;
inline constexpr uint32_t PRESENCE_SWITCH = PB7;
inline constexpr uint32_t PRESENCE_LED = PA8;
inline constexpr uint32_t PRESENCE_OUTPUT = PB15;
inline constexpr uint32_t ERROR_LED = PA15;
inline constexpr uint32_t BTN_BACKWARD = PB13;
inline constexpr uint32_t BTN_FORWARD = PB12;
inline constexpr uint32_t STEPPER_EN = PA6;
inline constexpr uint32_t STEPPER_DIR = PA7;
inline constexpr uint32_t STEPPER_STEP = PC13;
inline constexpr uint32_t STEPPER_UART = PB1;
inline constexpr uint8_t STEPPER_ADDR = 0b00;
inline constexpr float STEPPER_R_SENSE = 0.11f;
inline constexpr int32_t STEPPER_MICROSTEPS = 64;
constexpr float SPEED_RPM_FACTOR = 9.1463414634f;
constexpr float STEPS_PER_REV = 200.0f;
constexpr float SCREW_PITCH = 0.715f;

TMC2209Stepper driver(STEPPER_UART, STEPPER_UART, STEPPER_R_SENSE, STEPPER_ADDR);

class BoardHardware : public BufferHardware<BoardHardware> {
public:
  void initHardwareImpl() {
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

  bool optical1Impl() { return digitalRead(OPTICAL_SENSOR_1) != 0; }
  bool optical2Impl() { return digitalRead(OPTICAL_SENSOR_2) != 0; }
  bool optical3Impl() { return digitalRead(OPTICAL_SENSOR_3) != 0; }
  bool filamentPresentImpl() { return digitalRead(PRESENCE_SWITCH) == 0; }
  bool buttonForwardImpl() { return digitalRead(BTN_FORWARD) == LOW; }
  bool buttonBackwardImpl() { return digitalRead(BTN_BACKWARD) == LOW; }
  void setErrorLedImpl(bool on) { digitalWrite(ERROR_LED, on ? HIGH : LOW); }
  void setPresenceLedImpl(bool on) { digitalWrite(PRESENCE_LED, on ? HIGH : LOW); }
  void setPresenceOutputImpl(bool on) { digitalWrite(PRESENCE_OUTPUT, on ? HIGH : LOW); }
  void stepperPushImpl(float speed) { runStepper(true, speed); }
  void stepperRetractImpl(float speed) { runStepper(false, speed); }
  void stepperHoldImpl() {
    digitalWrite(STEPPER_EN, LOW);
    driver.VACTUAL(0);
  }
  void stepperOffImpl() {
    digitalWrite(STEPPER_EN, HIGH);
    driver.VACTUAL(0);
  }
  void writeLineImpl(const std::string &l) {
    SerialUSB.println(l.c_str());
    Serial2.println(l.c_str());
  }
  bool readCharImpl(char &c) {
    if (SerialUSB.available()) {
      c = static_cast<char>(SerialUSB.read());
      return true;
    }
    if (Serial2.available()) {
      c = static_cast<char>(Serial2.read());
      return true;
    }
    return false;
  }
  uint32_t timeMsImpl() { return millis(); }

private:
  void runStepper(bool forward, float speed) {
    digitalWrite(STEPPER_EN, LOW);
    driver.shaft(forward ? 1 : 0);
    const float rpm = speed * SPEED_RPM_FACTOR;
    const uint32_t v = static_cast<uint32_t>(rpm * STEPPER_MICROSTEPS * STEPS_PER_REV / 60.0f / SCREW_PITCH);
    driver.VACTUAL(v);
  }
};
} // namespace

template class Buffer<BoardHardware>;
static Buffer<BoardHardware> buffer;

inline void buffer_init() {
  buffer.init();
}
inline void buffer_loop() {
  buffer.loop();
}

#endif // ARDUINO
