#include <cmath>

#include "buffer.h"

constexpr uint32_t CMD_TIMEOUT = 3000;
constexpr uint32_t SHORT_PRESS_MS = 150;
constexpr uint32_t MULTI_PRESS_MIN_MS = 50;
constexpr uint32_t MULTI_PRESS_MAX_MS = 500;

#ifdef ARDUINO
#include <Arduino.h>
#include <TMCStepper.h>

namespace {
// pin mappings
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

class BoardHardware : public BufferHardware {
public:
  void initHardware() override {
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

  bool optical1() override { return digitalRead(OPTICAL_SENSOR_1) != 0; }
  bool optical2() override { return digitalRead(OPTICAL_SENSOR_2) != 0; }
  bool optical3() override { return digitalRead(OPTICAL_SENSOR_3) != 0; }
  bool filamentPresent() override { return digitalRead(PRESENCE_SWITCH) == 0; }
  bool buttonForward() override { return digitalRead(BTN_FORWARD) == LOW; }
  bool buttonBackward() override { return digitalRead(BTN_BACKWARD) == LOW; }
  void setErrorLed(bool on) override { digitalWrite(ERROR_LED, on ? HIGH : LOW); }
  void setPresenceLed(bool on) override { digitalWrite(PRESENCE_LED, on ? HIGH : LOW); }
  void setPresenceOutput(bool on) override { digitalWrite(PRESENCE_OUTPUT, on ? HIGH : LOW); }

  void stepperPush(float speed) override { runStepper(true, speed); }
  void stepperRetract(float speed) override { runStepper(false, speed); }
  void stepperHold() override {
    digitalWrite(STEPPER_EN, LOW);
    driver.VACTUAL(0);
  }
  void stepperOff() override {
    digitalWrite(STEPPER_EN, HIGH);
    driver.VACTUAL(0);
  }
  void writeLine(const std::string &l) override {
    SerialUSB.println(l.c_str());
    Serial2.println(l.c_str());
  }
  bool readChar(char &c) override {
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
  uint32_t timeMs() override { return millis(); }

private:
  void runStepper(bool forward, float speed) {
    digitalWrite(STEPPER_EN, LOW);
    driver.shaft(forward ? 1 : 0);
    const float rpm = speed * SPEED_RPM_FACTOR;
    const uint32_t v = static_cast<uint32_t>(rpm * STEPPER_MICROSTEPS * STEPS_PER_REV / 60.0f / SCREW_PITCH);
    driver.VACTUAL(v);
  }
};

BoardHardware boardHw;
} // namespace
#endif

Buffer::Buffer(BufferHardware &h) : hw(h) {
}

void Buffer::init() {
  hw.initHardware();
  filamentPresent = hw.filamentPresent();
  mode = filamentPresent ? Mode::Regular : Mode::Off;
  lastMode = mode;
  motor = Motor::Off;
  setMotor(filamentPresent ? Motor::Hold : Motor::Off);
  lastMotor = motor;
  hw.setPresenceLed(filamentPresent);
  hw.setPresenceOutput(filamentPresent);
  updateStatus();
}

void Buffer::setMotor(Motor m) {
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

void Buffer::processSerial() {
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

void Buffer::handleCommand(const std::string &cmd) {
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
    mode = Mode::Off;
    setMotor(Motor::Off);
  } else if (cmd == "query" || cmd == "q") {
    updateStatus();
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

void Buffer::handleButtons() {
  const uint32_t now = hw.timeMs();
  auto handle = [&](bool pressed, Motor dir, ButtonState &s) {
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
        mode = Mode::Regular;
        setMotor(Motor::Hold);
      } else {
        mode = Mode::Off;
        setMotor(Motor::Off);
      }
    } else { // long press
      s.count = 0;
      if (hw.filamentPresent()) {
        mode = Mode::Regular;
        setMotor(Motor::Hold);
      } else {
        mode = Mode::Off;
        setMotor(Motor::Off);
      }
    }
  };

  handle(hw.buttonForward(), Motor::Push, btnFwd);
  handle(hw.buttonBackward(), Motor::Retract, btnBack);
}

void Buffer::handleRegular() {
  if (!hw.filamentPresent()) {
    mode = Mode::Off;
    setMotor(Motor::Off);
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
    setMotor(Motor::Hold);
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

void Buffer::handleSerialMove() {
  if (hw.timeMs() >= moveEnd) {
    if (hw.filamentPresent()) {
      mode = Mode::Regular;
      setMotor(Motor::Hold);
    } else {
      mode = Mode::Off;
      setMotor(Motor::Off);
    }
    return;
  }
}

void Buffer::handleContinuous() {
  if (!hw.filamentPresent()) {
    mode = Mode::Off;
    setMotor(Motor::Off);
    return;
  }
  if (timeoutMs > 0 && hw.timeMs() - continuousStart > timeoutMs) {
    mode = Mode::Hold;
    setMotor(Motor::Hold);
  }
}

void Buffer::updateHoldTimeout() {
  if (!holdTimeoutEnabled || motor != Motor::Hold) {
    return;
  }
  if (hw.timeMs() - holdStart > holdTimeoutMs) {
    mode = Mode::Off;
    setMotor(Motor::Off);
  }
}

void Buffer::updateStatus() {
  bool fil = hw.filamentPresent();
  if (fil != lastFilament) {
    hw.writeLine(std::string("filament_present=") + (fil ? "1" : "0"));
    lastFilament = fil;
  }
  if (mode != lastMode) {
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
    case Mode::Off:
      modeStr = "off";
      break;
    }
    hw.writeLine(std::string("mode=") + modeStr);
    lastMode = mode;
  }
  if (motor != lastMotor) {
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
  if (timedOut != lastTimedOut) {
    hw.writeLine(std::string("timed_out=") + (timedOut ? "1" : "0"));
    lastTimedOut = timedOut;
  }
}

void Buffer::loop() {
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
    // motor already set in button handler
    break;
  case Mode::Off:
    setMotor(Motor::Off);
    break;
  }

  updateHoldTimeout();
  updateStatus();
}

#ifdef ARDUINO
static Buffer buffer(boardHw);

void buffer_init() {
  buffer.init();
}
void buffer_loop() {
  buffer.loop();
}
#endif
