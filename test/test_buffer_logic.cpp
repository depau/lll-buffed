#include <gtest/gtest.h>
#include <vector>

#include "buffer.h"

class FakeHardware : public BufferHardware {
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

  bool optical1() override { return opt1; }
  bool optical2() override { return opt2; }
  bool optical3() override { return opt3; }
  bool filamentPresent() override { return presence; }
  bool buttonForward() override { return btnF; }
  bool buttonBackward() override { return btnB; }
  void setErrorLed(bool on) override { errorLed = on; }
  void setPresenceLed(bool on) override { presLed = on; }
  void setPresenceOutput(bool on) override { presOut = on; }
  void stepperPush(float) override { lastMotor = TestMotor::Push; }
  void stepperRetract(float) override { lastMotor = TestMotor::Retract; }
  void stepperHold() override { lastMotor = TestMotor::Hold; }
  void stepperOff() override { lastMotor = TestMotor::Off; }
  void writeLine(const std::string &l) override { lines.push_back(l); }
  bool readChar(char &c) override {
    if (input.empty())
      return false;
    c = input.front();
    input.erase(input.begin());
    return true;
  }
  uint32_t timeMs() override { return now; }
};

TEST(BufferLogic, StartupWithoutFilament) {
  FakeHardware hw;
  hw.presence = false;
  Buffer buf(hw);
  buf.init();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Off);
}

TEST(BufferLogic, StartupWithFilament) {
  FakeHardware hw;
  hw.presence = true;
  hw.opt2 = true;
  Buffer buf(hw);
  buf.init();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);
}

TEST(BufferLogic, ButtonManualMove) {
  FakeHardware hw;
  hw.presence = true;
  hw.opt2 = true;
  Buffer buf(hw);
  buf.init();
  hw.btnF = true;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push);
  hw.now += 200;
  hw.btnF = false;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);
}

TEST(BufferLogic, SerialMoveCommand) {
  FakeHardware hw;
  hw.presence = true;
  hw.opt2 = true;
  Buffer buf(hw);
  buf.init();
  std::string cmd = "move 2\n";
  hw.input = std::vector<char>(cmd.begin(), cmd.end());
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push);
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);
}
