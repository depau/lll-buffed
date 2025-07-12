#include <algorithm>
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

  void serialSend(const std::string &line) {
    input.insert(input.end(), line.begin(), line.end());
  }
};

TEST(BufferLogic, StartupWithoutFilament) {
  FakeHardware hw;
  hw.presence = false;
  Buffer buf(hw);
  buf.init();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Off);
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("mode=regular") != std::string::npos;
  }));
}

TEST(BufferLogic, StartupWithFilament) {
  FakeHardware hw;
  hw.presence = true;
  hw.opt2 = true;
  Buffer buf(hw);
  buf.init();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("mode=regular") != std::string::npos;
  }));
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
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("mode=manual") != std::string::npos;
  }));
}

TEST(BufferLogic, SerialMoveCommand) {
  FakeHardware hw;
  hw.presence = true;
  hw.opt2 = true;
  Buffer buf(hw);
  buf.init();
  hw.serialSend("move 2\n");
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push);
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("mode=serial") != std::string::npos;
  }));
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);
  hw.serialSend("move 5000\n");
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push);
  hw.btnF = true;
  hw.now += 100;
  buf.loop();
  hw.btnF = false;
  hw.now += 100;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);
}

TEST(BufferLogic, ReactToOpticalSensors) {
  FakeHardware hw;
  hw.presence = true;
  hw.opt2 = true;
  Buffer buf(hw);
  buf.init();
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);

  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("mode=regular") != std::string::npos;
  }));

  hw.opt1 = true;
  hw.opt2 = false;
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push);
  EXPECT_EQ(hw.lines.back(), "status=push");
  hw.opt1 = false;
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push);
  EXPECT_EQ(hw.lines.back(), "status=push");
  hw.opt2 = true;
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);
  EXPECT_EQ(hw.lines.back(), "status=hold");
  hw.opt1 = false;
  hw.opt2 = false;
  hw.opt3 = false;
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);
  EXPECT_EQ(hw.lines.back(), "status=hold");
  hw.opt3 = true;
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Retract);
  EXPECT_EQ(hw.lines.back(), "status=retract");
  hw.opt3 = false;
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Retract);
  EXPECT_EQ(hw.lines.back(), "status=retract");
  hw.opt2 = true;
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);
  EXPECT_EQ(hw.lines.back(), "status=hold");
  hw.presence = false;
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Off);
}

TEST(BufferLogic, ContinuousMode) {
  FakeHardware hw;
  hw.presence = false;
  Buffer buf(hw);
  buf.init();
  hw.serialSend("set_timeout 2000\r\r\r\n");
  hw.serialSend("set_multi_press_count 3\r\n");
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Off);
  hw.serialSend("push\n");
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Off); // No filament, should not move
  hw.presence = true;
  hw.opt1 = false;
  hw.opt2 = true;
  hw.opt3 = false;
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold); // Should be in regular mode, with hold state
  hw.serialSend("push\n");
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push); // Now it should move
  hw.now += 3000; // Wait for timeout
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold); // Should stop after timeout
  hw.serialSend("retract\n");
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Retract); // Should retract
  hw.now += 3000; // Wait for timeout
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold); // Should stop after timeout
  hw.serialSend("off\n");
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Off); // Should turn off
  hw.opt1 = true;
  hw.opt2 = false;
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push); // Should push due to optical sensor
  hw.opt1 = false;
  hw.opt2 = true;
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold); // Should hold when optical sensor is back
  hw.serialSend("push\n");
  buf.loop();
  hw.now += 1000;
  hw.serialSend("hold\n");
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold); // Should hold after push command
  hw.serialSend("retract\n");
  buf.loop();
  hw.now += 1000;
  hw.serialSend("move 2\n");
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push); // Should move forward
  hw.serialSend("push\n");
  hw.now += 1000;
  buf.loop();
  hw.btnF = true; // Simulate button press
  hw.now += 50;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push); // Should still push due to button
  hw.btnF = false; // Release button
  hw.now += 10;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold); // Should hold after button release

  // Simulate multi-press
  hw.btnF = true;
  for (int i = 0; i < 3 * 2; ++i) {
    hw.btnF = !hw.btnF; // Toggle button state
    hw.now += 100;
    buf.loop();
  }
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push); // Should have enabled continuous mode

  // Simulate multi-press
  hw.btnF = false;
  hw.btnB = true;
  for (int i = 0; i < 3 * 2; ++i) {
    hw.btnB = !hw.btnB; // Toggle button state
    hw.now += 100;
    buf.loop();
  }
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Retract); // Should have enabled continuous mode

  hw.serialSend("push\n");
  hw.now += 1000;
  buf.loop();
  hw.presence = false;
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Off); // Should turn off when no filament
}

TEST(BufferLogic, ManualStopAndHoldTimeout) {
  FakeHardware hw;
  hw.presence = true;
  hw.opt1 = false;
  hw.opt2 = true;
  hw.opt3 = false;

  Buffer buf(hw);
  buf.init();
  hw.serialSend("set_hold_timeout 5000\n");
  hw.serialSend("enable_hold_timeout\n");
  buf.loop();

  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold); // Should start in hold mode
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold); // Still holding
  hw.now += 5000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Off); // Should turn off after hold timeout
  hw.opt1 = true;
  hw.opt2 = false;
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push); // Should push due to optical sensor
  hw.opt1 = false;
  hw.opt2 = true;
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold); // Should hold when optical sensor is back
  hw.now += 5000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Off); // Should still time out
  hw.opt1 = true;
  hw.opt2 = false;
  hw.now += 1000;
  buf.loop();
  hw.now += 100;
  hw.btnB = true;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Retract); // Should retract due to button
  hw.now += 100;
  hw.btnB = false;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold); // Should hold after quick button press
  hw.now += 5000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold); // Button should have disabled timeout
  hw.opt1 = false;
  hw.opt2 = false;
  hw.opt3 = true;
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold); // Should still be in hold mode
}

TEST(BufferLogic, PresenceReporting) {
  FakeHardware hw;
  hw.presence = true;
  Buffer buf(hw);
  buf.init();
  buf.loop();
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("filament_present=1") != std::string::npos;
  }));
  hw.presence = false;
  hw.now += 1000;
  buf.loop();
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("filament_present=0") != std::string::npos;
  }));
}
