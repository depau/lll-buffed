#include <algorithm>
#include <gtest/gtest.h>
#include <vector>

#include "buffer_logic.h"
#include "hardware_mock.h"

TEST(BufferLogic, StartupWithoutFilament) {
  Buffer<FakeHardware> buf;
  FakeHardware &hw = buf.getHardware();
  hw.presence = false;
  buf.init();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Off);
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("mode=regular") != std::string::npos;
  }));
}

TEST(BufferLogic, StartupWithFilament) {
  Buffer<FakeHardware> buf;
  FakeHardware &hw = buf.getHardware();
  hw.presence = true;
  hw.opt2 = true;
  buf.init();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);
  EXPECT_EQ(buf.getMode(), Buffer<FakeHardware>::Mode::Regular);
}

TEST(BufferLogic, ButtonManualMove) {
  Buffer<FakeHardware> buf;
  FakeHardware &hw = buf.getHardware();
  hw.presence = true;
  hw.opt2 = true;
  buf.init();
  hw.btnF = true;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push);
  EXPECT_EQ(buf.getMode(), Buffer<FakeHardware>::Mode::Manual);
  hw.now += 200;
  hw.btnF = false;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);
  EXPECT_EQ(buf.getMode(), Buffer<FakeHardware>::Mode::Regular);
}

TEST(BufferLogic, SerialMoveCommand) {
  Buffer<FakeHardware> buf;
  FakeHardware &hw = buf.getHardware();
  hw.presence = true;
  hw.opt2 = true;
  buf.init();
  hw.serialSend("move 2\n");
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push);
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("mode=move_command") != std::string::npos;
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
  Buffer<FakeHardware> buf;
  FakeHardware &hw = buf.getHardware();
  hw.presence = true;
  hw.opt2 = true;
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
}

TEST(BufferLogic, ContinuousMode) {
  Buffer<FakeHardware> buf;
  FakeHardware &hw = buf.getHardware();
  hw.presence = false;
  buf.init();
  hw.serialSend("set_timeout 2000\r\r\r\n");
  hw.serialSend("set_multi_press_count 3\r\n");
  buf.loop();
  hw.serialSend("push\n");
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Off); // No filament, should not move
  hw.presence = true;
  hw.opt1 = false;
  hw.opt2 = true;
  hw.opt3 = false;
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(buf.getMode(), Buffer<FakeHardware>::Mode::Regular);
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Off); // Should be in regular mode, with off state
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
}

TEST(BufferLogic, ManualStopAndHoldTimeout) {
  Buffer<FakeHardware> buf;
  FakeHardware &hw = buf.getHardware();
  hw.presence = true;
  hw.opt1 = false;
  hw.opt2 = true;
  hw.opt3 = false;

  buf.init();
  hw.serialSend("set_hold_timeout 5000\n");
  hw.serialSend("set_hold_timeout_en 1\n");
  buf.loop();

  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold); // Should start in hold mode
  hw.now += 1000;
  buf.loop();
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold); // Still holding
  hw.now += 5001;
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
  Buffer<FakeHardware> buf;
  FakeHardware &hw = buf.getHardware();
  hw.presence = true;
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

TEST(BufferLogic, RegularModeFilamentRunout) {
  Buffer<FakeHardware> buf;
  FakeHardware &hw = buf.getHardware();
  hw.presence = true;
  hw.opt2 = true;
  buf.init();
  hw.serialSend("set_emptying_timeout 5000\n");
  buf.loop();

  SCOPED_TRACE("The buffer should be in regular mode with the motor halted");
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);

  SCOPED_TRACE("Filament runout");
  hw.presence = false;
  hw.now += 100;
  buf.loop();

  SCOPED_TRACE("The motor should still be stopped since, when hitting opt2, the spring is still tensioned. We should "
               "however be in emptying mode");
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("mode=emptying") != std::string::npos;
  }));

  hw.opt2 = false;
  hw.now += 100;
  buf.loop();

  SCOPED_TRACE("Still holding, we consider the spring to be still tensioned until opt1 is hit and held");
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);

  hw.opt1 = true;
  hw.now += 100;
  buf.loop();

  SCOPED_TRACE("Should start pushing");
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push);

  hw.opt1 = false;
  hw.now += 100;
  buf.loop();

  SCOPED_TRACE("Still pushing since the spring is still tensioned, else we wouldn't have left opt1");
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push);

  hw.opt2 = true;
  hw.now += 100;
  buf.loop();

  SCOPED_TRACE("Should start holding");
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);

  hw.opt2 = false;
  hw.opt3 = true;
  hw.now += 100;
  buf.loop();

  SCOPED_TRACE("Should start retracting");
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Retract);

  hw.opt3 = false;
  hw.opt2 = true;
  hw.now += 100;
  buf.loop();

  SCOPED_TRACE("Should start holding");
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);

  hw.opt1 = true;
  hw.opt2 = false;
  hw.now += 100;
  buf.loop();

  SCOPED_TRACE("Should start pushing");
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push);

  hw.now += 5100;
  buf.loop();

  SCOPED_TRACE("Should be back in regular mode, with the motor off since there's no filament");
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Off);
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("mode=regular") != std::string::npos;
  }));
}

TEST(BufferLogic, EmptyingModeTimeout) {
  Buffer<FakeHardware> buf;
  FakeHardware &hw = buf.getHardware();
  hw.presence = true;
  hw.opt2 = true;
  buf.init();
  hw.serialSend("set_timeout 100000\n");
  buf.loop();

  SCOPED_TRACE("The buffer should be in regular mode with the motor halted");
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);

  SCOPED_TRACE("Filament runout");
  hw.presence = false;
  hw.now += 100;
  buf.loop();

  SCOPED_TRACE("The motor should still be stopped since, when hitting opt2, the spring is still tensioned. We should "
               "however be in emptying mode");
  EXPECT_EQ(buf.getMode(), Buffer<FakeHardware>::Mode::Emptying);
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Hold);

  hw.now += 100100;
  buf.loop();
  SCOPED_TRACE("Should be back in regular mode, with the motor off since the timeout has elapsed");
  EXPECT_EQ(buf.getMode(), Buffer<FakeHardware>::Mode::Regular);
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Off);
}

TEST(BufferLogic, ContinuousPushFilamentRunout) {
  Buffer<FakeHardware> buf;
  FakeHardware &hw = buf.getHardware();
  hw.presence = true;
  hw.opt1 = true;
  buf.init();
  hw.serialSend("set_emptying_timeout 5000\n");
  hw.serialSend("push\n");
  buf.loop();

  SCOPED_TRACE("Filament present, should start pushing");
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push);

  SCOPED_TRACE("Filament runout");
  hw.presence = false;
  hw.now += 100;
  buf.loop();  // Trigger emptying mode
  buf.loop();  // Run emptying mode logic once

  SCOPED_TRACE("Should keep pushing until the emptying mode timeout");
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push);

  hw.now += 5001;
  buf.loop();

  SCOPED_TRACE("Should shut down after the emptying mode timeout since there's no filament");
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Off);
}

TEST(BufferLogic, ContinuousRetractFilamentRunout) {
  Buffer<FakeHardware> buf;
  FakeHardware &hw = buf.getHardware();
  hw.presence = true;
  hw.opt1 = true;
  buf.init();
  hw.serialSend("set_emptying_timeout 5000\n");
  hw.serialSend("retract\n");
  buf.loop();

  SCOPED_TRACE("Filament present, should start retracting");
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Retract);

  SCOPED_TRACE("Filament runout");
  hw.presence = false;
  hw.now += 100;
  buf.loop();

  SCOPED_TRACE("Should immediately shut down since there's no filament");
  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Off);
}

TEST(BufferLogic, UARTSettings) {
  Buffer<FakeHardware> buf;
  FakeHardware &hw = buf.getHardware();
  hw.presence = true;
  buf.init();
  buf.loop();
  
  hw.serialSend("set_speed 123\n");
  hw.serialSend("set_timeout 12345\n");
  hw.serialSend("set_hold_timeout 5456\n");
  hw.serialSend("set_hold_timeout_en 0\n");
  hw.serialSend("set_emptying_timeout 6789\n");
  hw.serialSend("set_multi_press_count 7\n");
  buf.loop();

  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("filament_present=1") != std::string::npos;
  }));
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("speed=123.00") != std::string::npos;
  }));
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("timeout=12345") != std::string::npos;
  }));
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("hold_timeout=5456") != std::string::npos;
  }));
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("hold_timeout_en=0") != std::string::npos;
  }));
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("emptying_timeout=6789") != std::string::npos;
  }));
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("multi_press_count=7") != std::string::npos;
  }));
}

TEST(BufferLogic, I2CCommandPush) {
  Buffer<FakeHardware> buf;
  FakeHardware &hw = buf.getHardware();
  hw.presence = true;
  buf.init();

  // Set Command: Push (0x03) to REG_COMMAND (0x00)
  // [RegPtr=0x00] [Val=0x03]
  std::vector<uint8_t> data = { 0x00, 0x03 };
  hw.simulateI2CReceive(data);
  hw.simulateI2CRequest(); // Simulate request to drain response if any (though command doesn't reply)

  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push);
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("mode=continuous") != std::string::npos;
  }));
}

TEST(BufferLogic, I2CStatusRead) {
  Buffer<FakeHardware> buf;
  FakeHardware &hw = buf.getHardware();
  hw.presence = true;
  buf.init();

  // Force a status change to trigger interrupt
  hw.presence = false;
  buf.loop(); // Update status
  EXPECT_TRUE(hw.intActive);

  // Read Status Register: Write Ptr 0x01
  std::vector<uint8_t> ptr = { 0x01 };
  hw.simulateI2CReceive(ptr);

  // Read Response
  hw.simulateI2CRequest();

  // Verify interrupt cleared
  EXPECT_FALSE(hw.intActive);

  // Verify status byte (Filament=0)
  // expected status: 0 (since no bits set for empty, no timeout, etc)
  EXPECT_EQ(hw.i2cTxBuffer.size(), 1);
  if (!hw.i2cTxBuffer.empty())
    EXPECT_EQ(hw.i2cTxBuffer[0], 0x00);
}

TEST(BufferLogic, I2CMoveCommand) {
  Buffer<FakeHardware> buf;
  FakeHardware &hw = buf.getHardware();
  hw.presence = true;
  buf.init();

  // Set Speed first (optional, default is 30)

  // Write Move Dist: 100.0mm
  // Register 0x04
  float dist = 100.0f;
  std::vector<uint8_t> frame;
  frame.push_back(0x04); // REG_MOVE_DIST
  uint8_t *p = (uint8_t *) &dist;
  for (int i = 0; i < 4; i++)
    frame.push_back(p[i]);

  hw.simulateI2CReceive(frame);

  EXPECT_EQ(hw.lastMotor, FakeHardware::TestMotor::Push);
  EXPECT_TRUE(std::any_of(hw.lines.begin(), hw.lines.end(), [](const std::string &line) {
    return line.find("mode=move_command") != std::string::npos;
  }));
}
