#!/usr/bin/env python3
# Klipper module for the Mellow LLL Plus filament buffer
#
# Copyright (C) 2024  Davide Depau
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import struct

from . import bus
from .filament_switch_sensor import RunoutHelper

# Register addresses
REG_COMMAND = 0x00
REG_MOVE_DIST = 0x01
REG_STATUS = 0x05
REG_MODE = 0x06
REG_MOTOR = 0x07
REG_PARAM_SPEED = 0x08
REG_PARAM_TIMEOUT = 0x0C
REG_PARAM_EMPTYING_TIMEOUT = 0x10
REG_PARAM_HOLD_TIMEOUT = 0x14
REG_PARAM_HOLD_TIMEOUT_ENABLED = 0x18
REG_PARAM_MULTI_PRESS_COUNT = 0x19

# Commands
CMD_OFF = 0x00
CMD_REGULAR = 0x01
CMD_HOLD = 0x02
CMD_PUSH = 0x03
CMD_RETRACT = 0x04

# STATUS bits
STATUS_FILAMENT_PRESENT = 0x01
STATUS_TIMED_OUT = 0x02

MODE_NAMES = {
    0: "OFF",
    1: "REGULAR",
    2: "HOLD",
    3: "PUSH",
    4: "RETRACT",
    5: "MOVE_COMMAND",
}

MOTOR_NAMES = {
    0: "IDLE",
    1: "RUNNING",
}

VALID_MODES = {"OFF", "REGULAR", "HOLD", "PUSH", "RETRACT"}
VALID_SETTINGS = {
    "SPEED",
    "TIMEOUT",
    "EMPTYING_TIMEOUT",
    "HOLD_TIMEOUT",
    "HOLD_TIMEOUT_ENABLED",
    "MULTI_PRESS_COUNT",
}


class _InterruptDispatcher:
    def __init__(self, pin_desc, printer, config):
        self.reactor = printer.get_reactor()
        self.buffers = []
        buttons = printer.load_object(config, "buttons")
        buttons.register_buttons([pin_desc], self._handle_interrupt)

    def register_buffer(self, buf):
        self.buffers.append(buf)

    def _handle_interrupt(self, eventtime, state):
        if state == 0:
            # Active-low: asserted means a state change occurred on the device
            self.reactor.register_async_callback(self._poll_all)

    def _poll_all(self, eventtime):
        for buf in self.buffers:
            buf.service_interrupt(eventtime)


class _LLLBufferGroup:
    def __init__(self, printer):
        self.printer = printer
        self.buffers = {}
        gcode = printer.lookup_object("gcode")
        gcode.register_command(
            "SET_LLL_BUFFER",
            self.cmd_SET_LLL_BUFFER,
            desc=self.cmd_SET_LLL_BUFFER_help,
        )
        gcode.register_command(
            "LLL_BUFFER_STATUS",
            self.cmd_LLL_BUFFER_STATUS,
            desc=self.cmd_LLL_BUFFER_STATUS_help,
        )

    def register_buffer(self, name, buf):
        self.buffers[name] = buf

    def _get_buffer(self, gcmd):
        name = gcmd.get("BUFFER", None)
        if name is not None:
            if name not in self.buffers:
                raise gcmd.error("Unknown LLL buffer '%s'" % (name,))
            return self.buffers[name]
        if len(self.buffers) == 1:
            return next(iter(self.buffers.values()))
        if len(self.buffers) == 0:
            raise gcmd.error("No LLL buffers configured")
        raise gcmd.error("Multiple LLL buffers configured, specify BUFFER=<name>")

    cmd_SET_LLL_BUFFER_help = "Set LLL buffer mode or parameter"

    def cmd_SET_LLL_BUFFER(self, gcmd):
        buf = self._get_buffer(gcmd)
        mode = gcmd.get("MODE", None)
        setting = gcmd.get("SETTING", None)
        if mode is not None and setting is not None:
            raise gcmd.error("Cannot specify both MODE and SETTING")
        if mode is not None:
            buf.set_mode(gcmd)
        elif setting is not None:
            buf.set_setting(gcmd)
        else:
            raise gcmd.error("Must specify either MODE or SETTING")

    cmd_LLL_BUFFER_STATUS_help = "Query LLL buffer status"

    def cmd_LLL_BUFFER_STATUS(self, gcmd):
        buf = self._get_buffer(gcmd)
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        st = buf.get_full_status(eventtime)
        msg = (
            "LLL Buffer '%s':\n"
            "  Mode: %s\n"
            "  Motor: %s\n"
            "  Filament present: %s\n"
            "  Timed out: %s\n"
            "  Speed: %.1f mm/s\n"
            "  Timeout: %.3f s\n"
            "  Emptying timeout: %.3f s\n"
            "  Hold timeout: %.3f s\n"
            "  Hold timeout enabled: %s\n"
            "  Multi press count: %d"
        ) % (
            buf.name,
            st["mode"],
            st["motor"],
            st["filament_present"],
            st["timed_out"],
            st["speed"],
            st["timeout"],
            st["emptying_timeout"],
            st["hold_timeout"],
            st["hold_timeout_enabled"],
            st["multi_press_count"],
        )
        gcmd.respond_info(msg)


class LLLBuffer:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        # I2C setup
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=0x10, default_speed=400000
        )
        # Interrupt pin (optional)
        self.has_interrupt = False
        interrupt_pin = config.get("interrupt_pin", None)
        if interrupt_pin is not None:
            self.has_interrupt = True
            ppins = self.printer.lookup_object("pins")
            pin_params = ppins.lookup_pin(interrupt_pin)
            chip = pin_params["chip"]
            pin = pin_params["pin"]
            irq_key = "_lll_buffer_irq:%s:%s" % (chip.get_name(), pin)
            dispatcher = self.printer.lookup_object(irq_key, None)
            if dispatcher is None:
                dispatcher = _InterruptDispatcher(interrupt_pin, self.printer, config)
                self.printer.add_object(irq_key, dispatcher)
            dispatcher.register_buffer(self)
        # Polling (used only when no interrupt pin)
        self.poll_interval = config.getfloat("poll_interval", 0.5, minval=0.05)
        self.poll_timer = None
        # RunoutHelper handles filament runout/insert events
        self.runout_helper = RunoutHelper(config)
        self.get_status = self.runout_helper.get_status
        # Internal state (initialized in _handle_connect)
        self._mode = 0
        self._motor = 0
        self._timed_out = False
        self._speed = 0.0
        self._timeout = 0
        self._emptying_timeout = 0
        self._hold_timeout = 0
        self._hold_timeout_enabled = False
        self._multi_press_count = 0
        self._estimated_print_time = None
        # Lifecycle handlers
        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_connect(self):
        status = self._read_status()
        self.runout_helper.note_filament_present(bool(status & STATUS_FILAMENT_PRESENT))
        self._timed_out = bool(status & STATUS_TIMED_OUT)
        self._read_all_params()
        if not self.has_interrupt:
            self.poll_timer = self.reactor.register_timer(
                self._poll_sensor, self.reactor.NOW
            )

    def _handle_ready(self):
        self._estimated_print_time = self.i2c.get_mcu().estimated_print_time

    # I2C helpers

    def _read_reg(self, reg, length):
        params = self.i2c.i2c_read([reg], length)
        return bytearray(params["response"])

    def _write_reg(self, reg, data_list):
        self.i2c.i2c_write([reg] + data_list)

    def _read_status(self):
        data = self._read_reg(REG_STATUS, 1)
        return data[0]

    def _read_all_params(self):
        self._mode = self._read_reg(REG_MODE, 1)[0]
        self._motor = self._read_reg(REG_MOTOR, 1)[0]
        self._speed = struct.unpack("<f", bytes(self._read_reg(REG_PARAM_SPEED, 4)))[0]
        self._timeout = struct.unpack(
            "<I", bytes(self._read_reg(REG_PARAM_TIMEOUT, 4))
        )[0]
        self._emptying_timeout = struct.unpack(
            "<I", bytes(self._read_reg(REG_PARAM_EMPTYING_TIMEOUT, 4))
        )[0]
        self._hold_timeout = struct.unpack(
            "<I", bytes(self._read_reg(REG_PARAM_HOLD_TIMEOUT, 4))
        )[0]
        self._hold_timeout_enabled = bool(
            self._read_reg(REG_PARAM_HOLD_TIMEOUT_ENABLED, 1)[0]
        )
        self._multi_press_count = self._read_reg(REG_PARAM_MULTI_PRESS_COUNT, 1)[0]

    def service_interrupt(self, eventtime):
        try:
            status = self._read_status()
            self.runout_helper.note_filament_present(
                bool(status & STATUS_FILAMENT_PRESENT)
            )
            self._timed_out = bool(status & STATUS_TIMED_OUT)
        except Exception:
            logging.exception("LLLBuffer %s: error reading status", self.name)

    def _poll_sensor(self, eventtime):
        try:
            status = self._read_status()
            self.runout_helper.note_filament_present(
                bool(status & STATUS_FILAMENT_PRESENT)
            )
            self._timed_out = bool(status & STATUS_TIMED_OUT)
        except Exception:
            logging.exception("LLLBuffer %s: error polling status", self.name)
        return eventtime + self.poll_interval

    def set_mode(self, gcmd):
        mode_str = gcmd.get("MODE").upper()
        if mode_str not in VALID_MODES:
            raise gcmd.error(
                "Invalid MODE '%s', must be one of: %s"
                % (mode_str, ", ".join(sorted(VALID_MODES)))
            )
        distance = gcmd.get_float("DISTANCE", None)
        if distance is not None and mode_str not in ("PUSH", "RETRACT"):
            raise gcmd.error("DISTANCE is only valid with MODE=PUSH or MODE=RETRACT")
        if mode_str == "OFF":
            self._write_reg(REG_COMMAND, [CMD_OFF])
        elif mode_str == "REGULAR":
            self._write_reg(REG_COMMAND, [CMD_REGULAR])
        elif mode_str == "HOLD":
            self._write_reg(REG_COMMAND, [CMD_HOLD])
        elif mode_str == "PUSH":
            if distance is not None:
                self._write_reg(REG_MOVE_DIST, list(struct.pack("<f", distance)))
            else:
                self._write_reg(REG_COMMAND, [CMD_PUSH])
        elif mode_str == "RETRACT":
            if distance is not None:
                self._write_reg(REG_MOVE_DIST, list(struct.pack("<f", -distance)))
            else:
                self._write_reg(REG_COMMAND, [CMD_RETRACT])

    def set_setting(self, gcmd):
        setting = gcmd.get("SETTING").upper()
        if setting not in VALID_SETTINGS:
            raise gcmd.error(
                "Invalid SETTING '%s', must be one of: %s"
                % (setting, ", ".join(sorted(VALID_SETTINGS)))
            )
        if setting == "SPEED":
            value = gcmd.get_float("VALUE", minval=0.0)
            self._write_reg(REG_PARAM_SPEED, list(struct.pack("<f", value)))
            self._speed = value
        elif setting == "TIMEOUT":
            value = gcmd.get_float("VALUE", minval=0.0)
            ms = int(value * 1000)
            self._write_reg(REG_PARAM_TIMEOUT, list(struct.pack("<I", ms)))
            self._timeout = ms
        elif setting == "EMPTYING_TIMEOUT":
            value = gcmd.get_float("VALUE", minval=0.0)
            ms = int(value * 1000)
            self._write_reg(REG_PARAM_EMPTYING_TIMEOUT, list(struct.pack("<I", ms)))
            self._emptying_timeout = ms
        elif setting == "HOLD_TIMEOUT":
            value = gcmd.get_float("VALUE", minval=0.0)
            ms = int(value * 1000)
            self._write_reg(REG_PARAM_HOLD_TIMEOUT, list(struct.pack("<I", ms)))
            self._hold_timeout = ms
        elif setting == "HOLD_TIMEOUT_ENABLED":
            value = gcmd.get_int("VALUE", minval=0, maxval=1)
            self._write_reg(REG_PARAM_HOLD_TIMEOUT_ENABLED, [value])
            self._hold_timeout_enabled = bool(value)
        elif setting == "MULTI_PRESS_COUNT":
            value = gcmd.get_int("VALUE", minval=0, maxval=255)
            self._write_reg(REG_PARAM_MULTI_PRESS_COUNT, [value])
            self._multi_press_count = value

    def get_full_status(self, eventtime):
        try:
            status = self._read_status()
            filament_present = bool(status & STATUS_FILAMENT_PRESENT)
            timed_out = bool(status & STATUS_TIMED_OUT)
            self.runout_helper.note_filament_present(filament_present)
            self._timed_out = timed_out
            self._read_all_params()
        except Exception:
            logging.exception("LLLBuffer %s: error reading full status", self.name)
            filament_present = self.runout_helper.filament_present
            timed_out = self._timed_out
        return {
            "mode": MODE_NAMES.get(self._mode, "UNKNOWN(%d)" % self._mode),
            "motor": MOTOR_NAMES.get(self._motor, "UNKNOWN(%d)" % self._motor),
            "filament_present": filament_present,
            "timed_out": timed_out,
            "speed": self._speed,
            "timeout": self._timeout / 1000.0,
            "emptying_timeout": self._emptying_timeout / 1000.0,
            "hold_timeout": self._hold_timeout / 1000.0,
            "hold_timeout_enabled": self._hold_timeout_enabled,
            "multi_press_count": self._multi_press_count,
        }

    def get_extruder_pos(self, eventtime=None):
        toolhead = self.printer.lookup_object("toolhead", None)
        if toolhead is None:
            return 0.0
        extruder = toolhead.get_extruder()
        if extruder is None:
            return 0.0
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        if self._estimated_print_time is None:
            return 0.0
        print_time = self._estimated_print_time(eventtime)
        return extruder.find_past_position(print_time)

    def sensor_get_status(self, eventtime):
        return {
            "has_interrupt": self.has_interrupt,
            "poll_interval": self.poll_interval,
            "mode": MODE_NAMES.get(self._mode, "UNKNOWN(%d)" % self._mode),
            "motor": MOTOR_NAMES.get(self._motor, "UNKNOWN(%d)" % self._motor),
            "timed_out": self._timed_out,
        }


def _get_or_create_group(printer):
    group = printer.lookup_object("_lll_buffer_group", None)
    if group is None:
        group = _LLLBufferGroup(printer)
        printer.add_object("_lll_buffer_group", group)
    return group


def load_config(config):
    group = _get_or_create_group(config.get_printer())
    buf = LLLBuffer(config)
    group.register_buffer(buf.name, buf)
    return buf


def load_config_prefix(config):
    group = _get_or_create_group(config.get_printer())
    buf = LLLBuffer(config)
    group.register_buffer(buf.name, buf)
    return buf
