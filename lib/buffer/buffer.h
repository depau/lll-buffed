#pragma once

#include <cstdint>
#include <string>

/** Hardware abstraction for the filament buffer */
class BufferHardware {
public:
  virtual ~BufferHardware() = default;
  virtual bool optical1() = 0; ///< sensor near the spool
  virtual bool optical2() = 0; ///< middle sensor
  virtual bool optical3() = 0; ///< sensor near the extruder
  virtual bool filamentPresent() = 0; ///< true if filament detected
  virtual bool buttonForward() = 0; ///< forward jog button pressed
  virtual bool buttonBackward() = 0; ///< backward jog button pressed
  virtual void setErrorLed(bool on) = 0; ///< control error LED
  virtual void setPresenceLed(bool on) = 0; ///< control presence LED
  virtual void setPresenceOutput(bool on) = 0; ///< signal filament presence
  virtual void stepperPush(float speed) = 0; ///< run stepper forward
  virtual void stepperRetract(float speed) = 0; ///< run stepper backward
  virtual void stepperHold() = 0; ///< enable driver holding torque
  virtual void stepperOff() = 0; ///< disable driver
  virtual void writeLine(const std::string &l) = 0; ///< send status line
  virtual bool readChar(char &c) = 0; ///< read a byte from serial if available
  virtual uint32_t timeMs() = 0; ///< milliseconds counter
  virtual void initHardware() {}
};

/** High level buffer controller */
class Buffer {
public:
  explicit Buffer(BufferHardware &h);
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

  BufferHardware &hw;
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
