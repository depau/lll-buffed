#!/usr/bin/env python3
import sys
import argparse
import struct
import time
import cmd
from smbus2 import SMBus

# Register Map
REG_COMMAND = 0x00  # 1 byte
REG_MOVE_DIST = 0x01  # 4 bytes (float)
REG_STATUS = 0x05  # 1 byte
REG_MODE = 0x06  # 1 byte
REG_MOTOR = 0x07  # 1 byte
REG_PARAM_SPEED = 0x08  # 4 bytes (float)
REG_PARAM_TIMEOUT = 0x0C  # 4 bytes (uint32_t)
REG_PARAM_EMPTYING_TIMEOUT = 0x10  # 4 bytes (uint32_t)
REG_PARAM_HOLD_TIMEOUT = 0x14  # 4 bytes (uint32_t)
REG_PARAM_HOLD_TIMEOUT_ENABLED = 0x18  # 1 byte
REG_PARAM_MULTI_PRESS_COUNT = 0x19  # 1 byte

# Command Codes
CMD_OFF = 0
CMD_REGULAR = 1
CMD_HOLD = 2
CMD_PUSH = 3
CMD_RETRACT = 4


class BufferI2C:
    def __init__(self, bus_id, address, debug=False):
        self.bus_id = bus_id
        self.address = address
        self.debug = debug
        try:
            self.bus = SMBus(bus_id)
            if self.debug:
                print(f"Connected to I2C bus {bus_id}, device address 0x{address:02X}")
        except Exception as e:
            print(f"Error opening I2C bus {bus_id}: {e}")
            sys.exit(1)

    def write_byte(self, reg, val):
        if self.debug:
            print(f"WRITE [0x{reg:02X}] <- 0x{val:02X}")
        try:
            self.bus.write_byte_data(self.address, reg, val)
        except Exception as e:
            print(f"Write error: {e}")

    def write_block(self, reg, data):
        if self.debug:
            hex_data = " ".join([f"0x{b:02X}" for b in data])
            print(f"WRITE [0x{reg:02X}] <- [{hex_data}]")
        try:
            self.bus.write_i2c_block_data(self.address, reg, list(data))
        except Exception as e:
            print(f"Write block error: {e}")

    def read_byte(self, reg):
        try:
            val = self.bus.read_byte_data(self.address, reg)
            if self.debug:
                print(f"READ  [0x{reg:02X}] -> 0x{val:02X}")
            return val
        except Exception as e:
            print(f"Read error: {e}")
            return None

    def read_block(self, reg, length):
        try:
            data = self.bus.read_i2c_block_data(self.address, reg, length)
            if self.debug:
                hex_data = " ".join([f"0x{b:02X}" for b in data])
                print(f"READ  [0x{reg:02X}] -> [{hex_data}]")
            return bytes(data)
        except Exception as e:
            print(f"Read block error: {e}")
            return None

    def send_command(self, cmd_code):
        self.write_byte(REG_COMMAND, cmd_code)

    def set_float(self, reg, val):
        data = struct.pack("<f", val)
        self.write_block(reg, data)

    def set_uint32(self, reg, val):
        data = struct.pack("<I", val)
        self.write_block(reg, data)

    def get_float(self, reg):
        data = self.read_block(reg, 4)
        if data:
            return struct.unpack("<f", data)[0]
        return 0.0

    def get_uint32(self, reg):
        data = self.read_block(reg, 4)
        if data:
            return struct.unpack("<I", data)[0]
        return 0


class Shell(cmd.Cmd):
    intro = "Welcome to the Buffer I2C Shell. Type help or ? to list commands.\n"
    prompt = "> "

    def __init__(self, buffer):
        super().__init__()
        self.buffer = buffer

    def do_push(self, arg):
        """Push filament (Continuous Mode)"""
        self.buffer.send_command(CMD_PUSH)
        print("Command sent: PUSH")

    def do_retract(self, arg):
        """Retract filament (Continuous Mode)"""
        self.buffer.send_command(CMD_RETRACT)
        print("Command sent: RETRACT")

    def do_hold(self, arg):
        """Hold filament (Hold Mode)"""
        self.buffer.send_command(CMD_HOLD)
        print("Command sent: HOLD")

    def do_regular(self, arg):
        """Switch to Regular Mode"""
        self.buffer.send_command(CMD_REGULAR)
        print("Command sent: REGULAR")

    def do_off(self, arg):
        """Turn motor off"""
        self.buffer.send_command(CMD_OFF)
        print("Command sent: OFF")

    def do_move(self, arg):
        """Move filament by distance (mm). Usage: move <dist>"""
        try:
            dist = float(arg)
            self.buffer.set_float(REG_MOVE_DIST, dist)
            print(f"Move command sent: {dist} mm")
        except ValueError:
            print("Invalid distance. Usage: move <dist>")

    def do_set_speed(self, arg):
        """Set speed (mm/s). Usage: set_speed <val>"""
        try:
            val = float(arg)
            self.buffer.set_float(REG_PARAM_SPEED, val)
            print(f"Speed set to {val} mm/s")
        except ValueError:
            print("Invalid value")

    def do_set_timeout(self, arg):
        """Set timeout (ms). Usage: set_timeout <val>"""
        try:
            val = int(arg)
            self.buffer.set_uint32(REG_PARAM_TIMEOUT, val)
            print(f"Timeout set to {val} ms")
        except ValueError:
            print("Invalid value")

    def do_set_emptying_timeout(self, arg):
        """Set emptying timeout (ms). Usage: set_emptying_timeout <val>"""
        try:
            val = int(arg)
            self.buffer.set_uint32(REG_PARAM_EMPTYING_TIMEOUT, val)
            print(f"Emptying timeout set to {val} ms")
        except ValueError:
            print("Invalid value")

    def do_set_hold_timeout(self, arg):
        """Set hold timeout (ms). Usage: set_hold_timeout <val>"""
        try:
            val = int(arg)
            self.buffer.set_uint32(REG_PARAM_HOLD_TIMEOUT, val)
            print(f"Hold timeout set to {val} ms")
        except ValueError:
            print("Invalid value")

    def do_set_hold_timeout_en(self, arg):
        """Enable/Disable hold timeout. Usage: set_hold_timeout_en <0|1>"""
        try:
            val = int(arg)
            self.buffer.write_byte(REG_PARAM_HOLD_TIMEOUT_ENABLED, 1 if val else 0)
            print(f"Hold timeout {'enabled' if val else 'disabled'}")
        except ValueError:
            print("Invalid value")

    def do_set_multi_press(self, arg):
        """Set multi-press count. Usage: set_multi_press <val>"""
        try:
            val = int(arg)
            self.buffer.write_byte(REG_PARAM_MULTI_PRESS_COUNT, val)
            print(f"Multi-press count set to {val}")
        except ValueError:
            print("Invalid value")

    def do_status(self, arg):
        """Print current device status"""
        # Read all status registers
        status = self.buffer.read_byte(REG_STATUS)
        mode = self.buffer.read_byte(REG_MODE)
        motor = self.buffer.read_byte(REG_MOTOR)

        if status is None:
            return

        print("--- Status ---")
        print(f"Registers: STATUS=0x{status:02X} MODE=0x{mode:02X} MOTOR=0x{motor:02X}")

        # Parse Flags
        filament = bool(status & 0x01)
        timed_out = bool(status & 0x02)
        hold_to_en = bool(status & 0x04)
        print(f"Filament Present: {filament}")
        print(f"Timed Out:        {timed_out}")
        print(f"Hold Timeout En:  {hold_to_en}")

        # Parse Mode
        modes = {0: "Regular", 1: "Continuous", 2: "MoveCmd", 3: "Hold", 4: "Manual"}
        print(f"Mode:             {modes.get(mode, 'Unknown')}")

        # Parse Motor
        motors = {0: "Push", 1: "Retract", 2: "Hold", 3: "Off"}
        print(f"Motor:            {motors.get(motor, 'Unknown')}")

    def do_query(self, arg):
        """Alias for status"""
        self.do_status(arg)

    def do_params(self, arg):
        """Read all parameters"""
        speed = self.buffer.get_float(REG_PARAM_SPEED)
        timeout = self.buffer.get_uint32(REG_PARAM_TIMEOUT)
        e_timeout = self.buffer.get_uint32(REG_PARAM_EMPTYING_TIMEOUT)
        hold_timeout = self.buffer.get_uint32(REG_PARAM_HOLD_TIMEOUT)
        hold_to_en = bool(self.buffer.read_byte(REG_PARAM_HOLD_TIMEOUT_ENABLED))
        mp_count = self.buffer.read_byte(REG_PARAM_MULTI_PRESS_COUNT)

        print("--- Parameters ---")
        print(f"Speed:            {speed:.2f} mm/s")
        print(f"Timeout:          {timeout} ms")
        print(f"Emptying Timeout: {e_timeout} ms")
        print(f"Hold Timeout:     {hold_timeout} ms")
        print(f"Hold Timeout En:  {hold_to_en}")
        print(f"Multi-Press:      {mp_count}")

    def do_exit(self, arg):
        """Exit shell"""
        print("Bye")
        return True

    def do_quit(self, arg):
        return self.do_exit(arg)


def main():
    parser = argparse.ArgumentParser(description="I2C CLI for Filament Buffer")
    parser.add_argument("bus", type=int, help="I2C Bus Number (e.g. 1)")
    parser.add_argument(
        "--addr",
        type=lambda x: int(x, 0),
        default=0x10,
        help="Device Address (default 0x10)",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Print I2C transactions"
    )

    args = parser.parse_args()

    buf = BufferI2C(args.bus, args.addr, args.verbose)
    Shell(buf).cmdloop()


if __name__ == "__main__":
    main()
