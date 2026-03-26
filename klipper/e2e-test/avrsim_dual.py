#!/usr/bin/env python3
# Simulate Klipper MCU + filament buffer firmware connected via I2C.
# Klipper communicates with klippy via a PTY; buffer firmware UART is printed
# to stdout with a [BUFF] prefix.
#
# Based on avrsim.py by Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2015-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import errno
import fcntl
import optparse
import os
import pty
import signal
import struct
import sys
import termios
import threading

import pysimulavr

SERIALBITS = 10  # 8N1 = 1 start, 8 data, 1 stop
SIMULAVR_FREQ = 10**9


class LineBufferedOutput:
    """Buffers serial bytes and writes complete lines with a prefix."""

    def __init__(self, prefix, stream):
        self.prefix = prefix
        self.stream = stream
        self.buf = b""

    def write(self, data):
        self.buf += bytes(data)
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            line = line.replace(b"\r", b"")  # strip CR to avoid overwriting prefix
            if line:
                out = self.prefix + line.decode("utf-8", errors="replace") + "\n"
                self.stream.write(out)
                self.stream.flush()


# noinspection PyPep8Naming
class SerialRxPin(pysimulavr.PySimulationMember, pysimulavr.Pin):
    """Reads serial data from an AVR transmit pin and writes it to an output stream."""

    def __init__(self, baud, output):
        pysimulavr.Pin.__init__(self)
        pysimulavr.PySimulationMember.__init__(self)
        self.state = None
        self.output = output
        self.sc = pysimulavr.SystemClock.Instance()
        self.delay = SIMULAVR_FREQ // baud
        self.current = 0
        self.pos = -1

    def SetInState(self, pin):
        pysimulavr.Pin.SetInState(self, pin)
        self.state = pin.outState
        if self.pos < 0 and pin.outState == pin.LOW:
            self.pos = 0
            self.sc.Add(self)

    def DoStep(self, trueHwStep):
        ishigh = self.state == self.HIGH
        self.current |= ishigh << self.pos
        self.pos += 1
        if self.pos == 1:
            return int(self.delay * 1.5)
        if self.pos >= SERIALBITS:
            data = bytearray([(self.current >> 1) & 0xFF])
            self.output.write(data)
            self.pos = -1
            self.current = 0
            return -1
        return self.delay


# noinspection PyPep8Naming
class SerialTxPin(pysimulavr.PySimulationMember, pysimulavr.Pin):
    """Sends serial data from a terminal to an AVR receive pin."""

    def __init__(self, baud, terminal):
        pysimulavr.Pin.__init__(self)
        pysimulavr.PySimulationMember.__init__(self)
        self.terminal = terminal
        self.SetPin("H")
        self.sc = pysimulavr.SystemClock.Instance()
        self.delay = SIMULAVR_FREQ // baud
        self.current = 0
        self.pos = 0
        self.queue = bytearray()
        self.sc.Add(self)

    def DoStep(self, trueHwStep):
        if not self.pos:
            if not self.queue:
                data = self.terminal.read()
                if not data:
                    return self.delay * 100
                self.queue.extend(data)
            self.current = (self.queue.pop(0) << 1) | 0x200
        newstate = "L"
        if self.current & (1 << self.pos):
            newstate = "H"
        self.SetPin(newstate)
        self.pos += 1
        if self.pos >= SERIALBITS:
            self.pos = 0
        return self.delay


class TerminalIO:
    """Wraps a file descriptor for bidirectional PTY I/O."""

    def __init__(self, fd: int):
        self.fd = fd

    def write(self, data):
        os.write(self.fd, data)

    def read(self):
        try:
            return os.read(self.fd, 64)
        except os.error as e:
            if e.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                pysimulavr.SystemClock.Instance().stop()
        return ""


# noinspection PyProtectedMember
class _I2CLineMonitor(pysimulavr.Pin):
    """Internal: monitors one I2C line; notifies decoder on level change."""

    def __init__(self, decoder):
        pysimulavr.Pin.__init__(self)
        self._decoder = decoder
        self.level = None  # "H" or "L"

    def SetInState(self, pin):
        pysimulavr.Pin.SetInState(self, pin)
        s = pin.outState
        if s in (self.HIGH, self.PULLUP):
            new = "H"
        elif s in (self.LOW, self.PULLDOWN):
            new = "L"
        else:
            new = None  # TRISTATE / unknown
        if new == self.level:
            return
        self.level = new
        self._decoder._on_change()


class I2CDecoder:
    """Monitors SCL and SDA lines and reconstructs I2C transactions.

    Calls on_transaction(segments) when a STOP condition is detected.
    segments is a list of (addr, is_read, bytes_list) tuples.
    """

    def __init__(self, on_transaction=None):
        self.scl_mon = _I2CLineMonitor(self)
        self.sda_mon = _I2CLineMonitor(self)
        self._on_transaction_cb = on_transaction
        self._prev_scl = None
        self._prev_sda = None
        self._reset()

    def _reset(self):
        self._in_frame = False
        self._bits = []
        self._cur_addr = None
        self._cur_is_read = False
        self._cur_bytes = []
        self._segments = []

    def _finish_segment(self):
        if self._cur_addr is not None:
            self._segments.append(
                (self._cur_addr, self._cur_is_read, list(self._cur_bytes))
            )
        self._cur_addr = None
        self._cur_is_read = False
        self._cur_bytes = []
        self._bits = []

    def _process_byte(self, byte):
        if self._cur_addr is None:
            self._cur_addr = byte >> 1
            self._cur_is_read = bool(byte & 1)
        else:
            self._cur_bytes.append(byte)

    def _handle_scl_rise(self, sda):
        bit = 1 if sda == "H" else 0
        self._bits.append(bit)
        if len(self._bits) == 9:
            byte = 0
            for b in self._bits[:8]:
                byte = (byte << 1) | b
            self._bits = []
            self._process_byte(byte)

    def _handle_start(self):
        if self._in_frame:
            self._finish_segment()
        else:
            self._in_frame = True
        self._cur_addr = None
        self._cur_is_read = False
        self._cur_bytes = []
        self._bits = []

    def _handle_stop(self):
        self._finish_segment()
        if self._on_transaction_cb is not None:
            self._on_transaction_cb(list(self._segments))
        else:
            for addr, is_read, data in self._segments:
                direction = "R" if is_read else "W"
                hex_data = " ".join(f"{b:02x}" for b in data)
                sys.stderr.write(f"[I2C] [0x{addr:02x} {direction}] {hex_data}\n")
                sys.stderr.flush()
        self._reset()

    def _on_change(self):
        scl = self.scl_mon.level
        sda = self.sda_mon.level
        prev_scl = self._prev_scl
        prev_sda = self._prev_sda
        self._prev_scl = scl
        self._prev_sda = sda

        if scl is None or sda is None:
            return

        # START condition: SDA falls while SCL is high
        if scl == "H" and prev_sda == "H" and sda == "L":
            self._handle_start()
        # STOP condition: SDA rises while SCL is high
        elif self._in_frame and scl == "H" and prev_sda == "L" and sda == "H":
            self._handle_stop()
        # SCL rising edge: sample data bit
        elif self._in_frame and prev_scl == "L" and scl == "H":
            self._handle_scl_rise(sda)


class LLLProtocolDecoder:
    """Decodes I2C transactions for the LLL buffer firmware protocol."""

    _REG_NAMES = {
        0x00: "REG_COMMAND",
        0x01: "REG_MOVE_DIST",
        0x05: "REG_STATUS",
        0x06: "REG_MODE",
        0x07: "REG_MOTOR",
        0x08: "REG_PARAM_SPEED",
        0x0C: "REG_PARAM_TIMEOUT",
        0x10: "REG_PARAM_EMPTYING_TIMEOUT",
        0x14: "REG_PARAM_HOLD_TIMEOUT",
        0x18: "REG_PARAM_HOLD_TIMEOUT_ENABLED",
        0x19: "REG_PARAM_MULTI_PRESS_COUNT",
    }

    _CMD_NAMES = {
        0: "CMD_OFF",
        1: "CMD_REGULAR",
        2: "CMD_HOLD",
        3: "CMD_PUSH",
        4: "CMD_RETRACT",
        0xDF: "CMD_REBOOT_DFU",
    }

    _MODE_NAMES = {
        0: "Regular",
        1: "Continuous",
        2: "MoveCommand",
        3: "Hold",
        4: "Manual",
        5: "Emptying",
    }

    _MOTOR_NAMES = {
        0: "Off",
        1: "Push",
        2: "Retract",
        3: "Hold",
    }

    def __init__(self, num_buffers=1):
        self._lll_addrs = set(range(0x10, 0x10 + num_buffers))
        self.i2c_decoder = I2CDecoder(on_transaction=self._on_transaction)

    def _on_transaction(self, segments):
        # Print raw hex for each segment
        for addr, is_read, data in segments:
            direction = "R" if is_read else "W"
            hex_data = " ".join(f"{b:02x}" for b in data)
            sys.stderr.write(f"[I2C] [0x{addr:02x} {direction}] {hex_data}\n")

        # Only decode LLL device addresses
        if not segments or segments[0][0] not in self._lll_addrs:
            sys.stderr.flush()
            return

        # Register write: 1 segment, write, data non-empty
        if len(segments) == 1 and not segments[0][1] and len(segments[0][2]) >= 1:
            _, _, data = segments[0]
            reg = data[0]
            payload = data[1:]
            line = self._decode_write(reg, payload)
            if line:
                sys.stderr.write(f"[LLL] {line}\n")

        # Register read: 2 segments, first write + second read
        elif len(segments) == 2 and not segments[0][1] and segments[1][1]:
            _, _, write_data = segments[0]
            _, _, read_data = segments[1]
            if write_data:
                reg = write_data[0]
                line = self._decode_read(reg, read_data)
                if line:
                    sys.stderr.write(f"[LLL] {line}\n")

        sys.stderr.write("\n")
        sys.stderr.flush()

    def _format_reg(self, reg):
        return self._REG_NAMES.get(reg, f"REG_0x{reg:02x}")

    def _decode_write(self, reg, payload):
        reg_name = self._format_reg(reg)
        if reg == 0x00:  # REG_COMMAND
            if len(payload) >= 1:
                cmd_name = self._CMD_NAMES.get(payload[0], f"0x{payload[0]:02x}")
                return f"WRITE {reg_name} → {cmd_name}"
            return f"WRITE {reg_name} (no data)"
        if reg == 0x01:  # REG_MOVE_DIST (float)
            if len(payload) >= 4:
                (val,) = struct.unpack("<f", bytes(payload[:4]))
                return f"WRITE {reg_name} → {val:.4f} mm"
            return f"WRITE {reg_name} (short data: {payload!r})"
        if reg == 0x08:  # REG_PARAM_SPEED (float)
            if len(payload) >= 4:
                (val,) = struct.unpack("<f", bytes(payload[:4]))
                return f"WRITE {reg_name} → {val:.4f} mm/s"
            return f"WRITE {reg_name} (short data: {payload!r})"
        if reg in (0x0C, 0x10, 0x14):  # uint32 timeout registers
            if len(payload) >= 4:
                (val,) = struct.unpack("<I", bytes(payload[:4]))
                return f"WRITE {reg_name} → {val} ms"
            return f"WRITE {reg_name} (short data: {payload!r})"
        if reg == 0x18:  # REG_PARAM_HOLD_TIMEOUT_ENABLED
            if len(payload) >= 1:
                enabled = "enabled" if payload[0] else "disabled"
                return f"WRITE {reg_name} → {enabled} (0x{payload[0]:02x})"
            return f"WRITE {reg_name} (no data)"
        if reg == 0x19:  # REG_PARAM_MULTI_PRESS_COUNT
            if len(payload) >= 1:
                return f"WRITE {reg_name} → {payload[0]}"
            return f"WRITE {reg_name} (no data)"
        hex_payload = " ".join(f"{b:02x}" for b in payload)
        return f"WRITE {reg_name} → {hex_payload}"

    def _decode_read(self, reg, data):
        reg_name = self._format_reg(reg)
        if reg == 0x00:  # REG_COMMAND
            if len(data) >= 1:
                cmd_name = self._CMD_NAMES.get(data[0], f"0x{data[0]:02x}")
                return f"READ {reg_name} → {cmd_name}"
        elif reg == 0x01:  # REG_MOVE_DIST (float)
            if len(data) >= 4:
                (val,) = struct.unpack("<f", bytes(data[:4]))
                return f"READ {reg_name} → {val:.4f} mm"
        elif reg == 0x05:  # REG_STATUS
            if len(data) >= 1:
                flags = data[0]
                filament = "present" if flags & 0x01 else "absent"
                timed_out = " timed_out" if flags & 0x02 else ""
                return (
                    f"READ {reg_name} → filament={filament}{timed_out} (0x{flags:02x})"
                )
        elif reg == 0x06:  # REG_MODE
            if len(data) >= 1:
                mode_name = self._MODE_NAMES.get(data[0], f"0x{data[0]:02x}")
                return f"READ {reg_name} → {mode_name}"
        elif reg == 0x07:  # REG_MOTOR
            if len(data) >= 1:
                motor_name = self._MOTOR_NAMES.get(data[0], f"0x{data[0]:02x}")
                return f"READ {reg_name} → {motor_name}"
        elif reg == 0x08:  # REG_PARAM_SPEED (float)
            if len(data) >= 4:
                (val,) = struct.unpack("<f", bytes(data[:4]))
                return f"READ {reg_name} → {val:.4f} mm/s"
        elif reg in (0x0C, 0x10, 0x14):  # uint32 timeout registers
            if len(data) >= 4:
                (val,) = struct.unpack("<I", bytes(data[:4]))
                return f"READ {reg_name} → {val} ms"
        elif reg == 0x18:  # REG_PARAM_HOLD_TIMEOUT_ENABLED
            if len(data) >= 1:
                enabled = "enabled" if data[0] else "disabled"
                return f"READ {reg_name} → {enabled} (0x{data[0]:02x})"
        elif reg == 0x19:  # REG_PARAM_MULTI_PRESS_COUNT
            if len(data) >= 1:
                return f"READ {reg_name} → {data[0]}"
        hex_data = " ".join(f"{b:02x}" for b in data)
        return f"READ {reg_name} → {hex_data}"


def create_pty(ptyname):
    """Creates a pseudo-TTY and symlinks it to ptyname."""
    mfd, sfd = pty.openpty()
    try:
        os.unlink(ptyname)
    except os.error:
        pass
    os.symlink(os.ttyname(sfd), ptyname)
    fcntl.fcntl(mfd, fcntl.F_SETFL, fcntl.fcntl(mfd, fcntl.F_GETFL) | os.O_NONBLOCK)
    tcattr = termios.tcgetattr(mfd)
    tcattr[0] &= ~(
        termios.IGNBRK
        | termios.BRKINT
        | termios.PARMRK
        | termios.ISTRIP
        | termios.INLCR
        | termios.IGNCR
        | termios.ICRNL
        | termios.IXON
    )
    tcattr[1] &= ~termios.OPOST
    tcattr[3] &= ~(
        termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN
    )
    tcattr[2] &= ~(termios.CSIZE | termios.PARENB)
    tcattr[2] |= termios.CS8
    tcattr[6][termios.VMIN] = 0
    tcattr[6][termios.VTIME] = 0
    termios.tcsetattr(mfd, termios.TCSAFLUSH, tcattr)
    return mfd


# Internal trace: captures simulavr's internal device register/port state via DumpManager.
class InternalTrace:
    def __init__(self, filename, signals):
        self.filename = filename
        self.signals = signals
        if not signals:
            self.dman = None
            return
        self.dman = pysimulavr.DumpManager.Instance()
        # self.dman.SetSingleDeviceApp()  # Skip for multi-device simulation

    def show_help(self):
        ostr = pysimulavr.ostringstream()
        self.dman.save(ostr)
        sys.stdout.write(
            "Available signals for internal tracing:\n"
            "Mapping: Dev1 = Klipper, Dev2 = Buffer\n\n"
        )
        sys.stdout.write(ostr.str())
        sys.exit(0)

    def load_options(self):
        if self.dman is None:
            return
        if self.signals.strip() == "?":
            self.show_help()
        sigs = "\n".join(["+ " + s for s in self.signals.split(",")])
        self.dman.addDumpVCD(self.filename, sigs, "ns", False, False)

    def start(self):
        if self.dman is not None:
            self.dman.start()

    def finish(self):
        if self.dman is not None:
            self.dman.stopApplication()


# External trace: observes Net/pin state as seen on the simulated wires via VCD.
# noinspection PyProtectedMember
class _ExternalTracePin(pysimulavr.Pin):
    """Monitor pin attached to a Net that records level changes to an ExternalTrace.

    If observed_pin is set, records that pin's outState (what one specific device
    is driving) rather than the net's computed state.  The monitor is added to the
    shared net so it receives callbacks on every bus transition — this avoids
    disconnecting the device pin from its original net.
    """

    def __init__(self, tracer, vcd_id, observed_pin=None):
        pysimulavr.Pin.__init__(self)
        self._tracer = tracer
        self._vcd_id = vcd_id
        self._last_state = None
        self._observed_pin = observed_pin  # None → net state; Pin → that pin's outState

    def SetInState(self, pin):
        pysimulavr.Pin.SetInState(self, pin)
        state = (
            self._observed_pin.outState
            if self._observed_pin is not None
            else pin.outState
        )
        if state == self._last_state:
            return
        self._last_state = state
        if state in (self.HIGH, self.PULLUP):
            level = "1"
        elif state in (self.LOW, self.PULLDOWN):
            level = "0"
        elif state == self.TRISTATE:
            level = "z"
        else:
            level = "x"
        self._tracer._record(level, self._vcd_id)


class ExternalTrace:
    """Writes a VCD file capturing signals observed on Nets and/or device pins."""

    # 94 printable ASCII characters usable as VCD signal identifiers
    _ID_CHARS = "!\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~"

    def __init__(self, filename):
        self._filename = filename
        self._signals = []  # [(name, vcd_id, net, observed_pin)]
        self._monitors = []  # keep _ExternalTracePin refs alive
        self._file = None
        self._current_time = -1
        self.sc = pysimulavr.SystemClock.Instance()

    def add_net(self, name, net):
        """Attach to a Net — records its computed bus state (all devices + pullups)."""
        vcd_id = self._ID_CHARS[len(self._signals)]
        self._signals.append((name, vcd_id, net, None))

    def add_pin(self, name, dev_pin, existing_net):
        """Monitor a device pin's driving state via its existing shared net.

        The monitor is added to existing_net (not a new one), so dev_pin stays
        connected to the bus.  Records dev_pin.outState on each bus transition,
        showing what that specific device is driving (TRISTATE when silent).
        """
        vcd_id = self._ID_CHARS[len(self._signals)]
        self._signals.append((name, vcd_id, existing_net, dev_pin))

    def _record(self, level, vcd_id):
        t = self.sc.GetCurrentTime()
        if t != self._current_time:
            self._file.write(f"#{t}\n")
            self._current_time = t
        self._file.write(f"{level}{vcd_id}\n")

    def start(self):
        self._file = open(self._filename, "w", buffering=1)  # line-buffered
        self._file.write("$timescale 1 ns $end\n")
        self._file.write("$scope module signals $end\n")
        for name, vcd_id, _, _ in self._signals:
            self._file.write(f"$var wire 1 {vcd_id} {name} $end\n")
        self._file.write("$upscope $end\n")
        self._file.write("$enddefinitions $end\n")
        self._file.write("$dumpvars\n")
        for _, vcd_id, _, _ in self._signals:
            self._file.write(f"1{vcd_id}\n")  # assume pulled high at t=0
        self._file.write("$end\n")
        for _, vcd_id, net, observed_pin in self._signals:
            mon = _ExternalTracePin(self, vcd_id, observed_pin)
            self._monitors.append(mon)
            net.Add(mon)

    def finish(self):
        if self._file:
            self._file.write(f"#{self.sc.GetCurrentTime()}\n")
            self._file.close()
            self._file = None


class TrackedNet:
    """Wraps pysimulavr.Net and records which (dev_name, pin_name) pairs are connected.

    Use add_device_pin() for device I/O pins; use Add() for everything else
    (pullups, SerialRxPin, monitors, etc.).  The _device_pins dict is the single
    source of truth consumed later to build the external-trace lookup table.
    """

    def __init__(self):
        self._net = pysimulavr.Net()
        self.device_pins = {}  # (dev_name, pin_name) -> Pin

    def add_device_pin(self, dev_name, dev, pin_name):
        pin = dev.GetPin(pin_name)
        self._net.Add(pin)
        self.device_pins[(dev_name, pin_name)] = pin

    def Add(self, pin):
        self._net.Add(pin)


# noinspection PyPep8Naming
class I2CAddressSetter(pysimulavr.PySimulationMember):
    _INVALID = 0xAA
    _POLL = SIMULAVR_FREQ // 1000  # 1 ms sim time

    def __init__(self, dev, target_addr, label):
        pysimulavr.PySimulationMember.__init__(self)
        self._dev = dev
        self._target_addr = target_addr
        self._label = label
        self._sym_addr = dev.data.GetAddressAtSymbol("simulator_i2c_address")
        # Write sentinel 0x00 before simulation starts
        self._dev.setRWMem(self._sym_addr, 0x00)
        self._done = False
        pysimulavr.SystemClock.Instance().Add(self)

    def DoStep(self, trueHwStep):
        if self._done:
            return -1
        if self._dev.getRWMem(self._sym_addr) == self._INVALID:
            sys.stderr.write(
                f"[{self._label}] Device initialized; setting I2C address to 0x{self._target_addr:02x}\n"
            )
            sys.stderr.flush()
            self._dev.setRWMem(self._sym_addr, self._target_addr)
            self._done = True
            return -1
        return self._POLL


def main():
    usage = "%prog [options] <klipper.elf> <simulator.elf>"
    opts = optparse.OptionParser(usage)
    opts.add_option(
        "--klipper-clock",
        type="int",
        dest="klipper_clock",
        default=20000000,
        help="machine speed",
    )
    opts.add_option(
        "--buffer-clock",
        type="int",
        dest="buffer_clock",
        default=16000000,
        help="machine speed",
    )
    opts.add_option(
        "-b",
        "--baud",
        type="int",
        dest="baud",
        default=115200,
        help="baud rate of the simulator (buffer) UART",
    )
    opts.add_option(
        "--klipper-baud",
        type="int",
        dest="klipper_baud",
        default=250000,
        help="baud rate of the Klipper UART (default: 250000)",
    )
    opts.add_option(
        "--num-buffers",
        type="int",
        dest="num_buffers",
        default=1,
        help="number of buffer devices to simulate (default: 1)",
    )
    opts.add_option(
        "-i",
        "--interrupt-line",
        action="append",
        dest="interrupt_lines",
        metavar="GPIO=i,j,...",
        help=(
            "wire klipper GPIO to buffer INT pins; repeatable. "
            "Default (no flag): all buffers share one net on klipper C4. "
            "Any occurrence disables the default. "
            "Use GPIO= to connect only the klipper pin (no buffers). "
            "Use '' to create no INT nets at all. "
            "Example: --interrupt-line C4=0,1 --interrupt-line C5=2"
        ),
    )
    opts.add_option(
        "-p",
        "--port",
        type="string",
        dest="port",
        default="/tmp/pseudoserial",
        help="pseudo-tty device to create for Klipper serial port",
    )
    opts.add_option(
        "-n",
        "--timeout",
        type="float",
        dest="timeout",
        default=0.0,
        help="stop after this many real-time seconds (0 = run forever)",
    )
    deffile = os.path.splitext(os.path.basename(sys.argv[0]))[0] + ".vcd"
    opts.add_option(
        "-t",
        "--internal-trace",
        type="string",
        dest="internal_trace",
        help="simulavr internal signals to trace (? for help)",
    )
    opts.add_option(
        "-f",
        "--internal-tracefile",
        type="string",
        dest="internal_tracefile",
        default=deffile,
        help=f"output filename for internal trace (default: {deffile})",
    )
    defextfile = os.path.splitext(os.path.basename(sys.argv[0]))[0] + "_external.vcd"
    opts.add_option(
        "-e",
        "--external-trace",
        type="string",
        dest="external_trace",
        help="comma-separated Net/pin signals for external VCD trace (? for help)",
    )
    opts.add_option(
        "-E",
        "--external-tracefile",
        type="string",
        dest="external_tracefile",
        default=defextfile,
        help=f"output filename for external trace (default: {defextfile})",
    )
    options, args = opts.parse_args()
    if len(args) == 0:
        klipper_elf = "klipper.elf"
        buffer_elf = "../../.pio/build/simulator/firmware.elf"
    elif len(args) == 2:
        klipper_elf, buffer_elf = args
    else:
        opts.error("Expected 0 or 2 arguments (klipper.elf simulator.elf)")

    klipper_clock = options.klipper_clock
    buffer_clock = options.buffer_clock
    buffer_baud = options.baud
    klipper_baud = options.klipper_baud
    ptyname = options.port
    num_buffers = options.num_buffers

    sc = pysimulavr.SystemClock.Instance()
    internal_trace = InternalTrace(options.internal_tracefile, options.internal_trace)
    pysimulavr.cvar.sysConHandler.SetUseExit(True)

    # Create and load both devices
    factory = pysimulavr.AvrFactory.instance()

    klipper_dev = factory.makeDevice("atmega644p")
    klipper_dev.Load(klipper_elf)
    klipper_dev.SetClockFreq(SIMULAVR_FREQ // klipper_clock)
    sc.Add(klipper_dev)

    buffer_devs = []
    buffer_addr_setters = []  # keep alive (prevent GC)
    buffer_tx_nets = []

    for i in range(num_buffers):
        buf_dev = factory.makeDevice("atmega2560")
        buf_dev.Load(buffer_elf)
        buf_dev.SetClockFreq(SIMULAVR_FREQ // buffer_clock)
        sc.Add(buf_dev)
        buffer_devs.append(buf_dev)

        # I2C address setter
        setter = I2CAddressSetter(buf_dev, 0x10 + i, f"BUFF{i}")
        buffer_addr_setters.append(setter)

        # UART output
        buf_out = LineBufferedOutput(f"[BUFF{i}] ", sys.stdout)
        buf_rxpin = SerialRxPin(buffer_baud, buf_out)
        buf_tx_net = TrackedNet()
        buf_tx_net.add_device_pin(f"buffer{i}", buf_dev, "E1")
        buf_tx_net.Add(buf_rxpin)
        buffer_tx_nets.append(buf_tx_net)

    # INT lines: default connects all buffer D0 to klipper C4 on one shared net.
    # Override with --interrupt-line GPIO=i,j,... (repeatable); any occurrence
    # disables the default.  Use GPIO= to wire the klipper pin with no buffers.
    # Use '' (empty) to create no INT nets at all.
    int_nets = {}  # gpio_name → TrackedNet
    if options.interrupt_lines is None:
        net = TrackedNet()
        net.add_device_pin("klipper", klipper_dev, "C4")
        for i, buf_dev in enumerate(buffer_devs):
            net.add_device_pin(f"buffer{i}", buf_dev, "D0")
        int_nets["C4"] = net
    else:
        for spec in options.interrupt_lines:
            spec = spec.strip()
            if not spec:
                continue
            if "=" not in spec:
                opts.error(f"Invalid --interrupt-line {spec!r}: expected GPIO=indices")
            gpio, _, indices_str = spec.partition("=")
            gpio = gpio.strip().upper()
            if not gpio:
                continue
            net = TrackedNet()
            net.add_device_pin("klipper", klipper_dev, gpio)
            for idx_str in indices_str.split(","):
                idx_str = idx_str.strip()
                if not idx_str:
                    continue
                idx = int(idx_str)
                if idx < 0 or idx >= num_buffers:
                    opts.error(
                        f"Buffer index {idx} out of range (0..{num_buffers - 1})"
                    )
                net.add_device_pin(f"buffer{idx}", buffer_devs[idx], "D0")
            int_nets[gpio] = net

    internal_trace.load_options()

    # I2C SCL: klipper_dev PC0 (HW TWI SCL) <-> all buffer PE4 (SoftI2C SCL / INT4)
    i2c_scl_net = TrackedNet()
    i2c_scl_net.add_device_pin("klipper", klipper_dev, "C0")

    # I2C SDA: klipper_dev PC1 (HW TWI SDA) <-> all buffer PE5 (SoftI2C SDA / INT5)
    i2c_sda_net = TrackedNet()
    i2c_sda_net.add_device_pin("klipper", klipper_dev, "C1")

    # Connect all buffer devices to the shared I2C bus
    for i, buf_dev in enumerate(buffer_devs):
        i2c_scl_net.add_device_pin(f"buffer{i}", buf_dev, "E4")
        i2c_sda_net.add_device_pin(f"buffer{i}", buf_dev, "E5")

    # I2C protocol decoder: always active, prints decoded transactions to stderr
    lll_decoder = LLLProtocolDecoder(num_buffers=num_buffers)
    i2c_scl_net.Add(lll_decoder.i2c_decoder.scl_mon)
    i2c_sda_net.Add(lll_decoder.i2c_decoder.sda_mon)

    # Klipper UART: bidirectional via PTY (TX=D1 out, RX=D0 in)
    fd = create_pty(ptyname)
    klipper_io = TerminalIO(fd)
    klipper_rxpin = SerialRxPin(klipper_baud, klipper_io)  # AVR TX (D1) → PTY
    klipper_txpin = SerialTxPin(klipper_baud, klipper_io)  # PTY → AVR RX (D0)

    klipper_uart_tx_net = TrackedNet()
    klipper_uart_tx_net.add_device_pin("klipper", klipper_dev, "D1")
    klipper_uart_tx_net.Add(klipper_rxpin)

    klipper_uart_rx_net = TrackedNet()
    klipper_uart_rx_net.add_device_pin("klipper", klipper_dev, "D0")
    klipper_uart_rx_net.Add(klipper_txpin)

    # External trace: observe Net/pin state as seen on the simulated wires.
    external_trace = None
    if options.external_trace:
        named_nets = {
            "scl": i2c_scl_net,
            "sda": i2c_sda_net,
            "klipper_to_pty_tx": klipper_uart_tx_net,
            "klipper_from_pty_rx": klipper_uart_rx_net,
        }
        for i, net in enumerate(buffer_tx_nets):
            named_nets[f"buffer{i}_tx_to_stdout"] = net
        for gpio_name, net in int_nets.items():
            named_nets[f"int_{gpio_name.lower()}"] = net

        # Build dev_pin_nets automatically from pins registered in each TrackedNet.
        # This is the single source of truth — no manual duplication needed.
        dev_pin_nets = {}
        for net in named_nets.values():
            for key in net.device_pins:
                dev_pin_nets[key] = net

        if options.external_trace.strip() == "?":
            net_names = ", ".join(k.upper() for k in named_nets)
            pin_list = "\n".join(f"  {dev}.{pin}" for dev, pin in sorted(dev_pin_nets))
            example = ",".join(
                [k.upper() for k in list(named_nets)[:3]]
                + [f"{dev}.{pin}" for dev, pin in list(dev_pin_nets)[:2]]
            )
            sys.stdout.write(
                f"Named nets (computed bus state — ideal for I2C decoders):\n"
                f"  {net_names}\n\n"
                f"Device pins (per-device driving state; TRISTATE when not driving):\n"
                f"{pin_list}\n\n"
                f"Example: --external-trace {example} -E i2c.vcd\n"
            )
            sys.exit(0)
        external_trace = ExternalTrace(options.external_tracefile)
        for token in options.external_trace.split(","):
            token = token.strip()
            if token.lower() in named_nets:
                external_trace.add_net(token.upper(), named_nets[token.lower()])
            elif "." in token:
                dev, pin = token.split(".", 1)
                tracked = dev_pin_nets.get((dev, pin))
                if tracked is None:
                    known = ", ".join(f"{d}.{p}" for d, p in sorted(dev_pin_nets))
                    sys.exit(f"Unknown signal {token!r} — known device pins: {known}")
                external_trace.add_pin(
                    token.upper(), tracked.device_pins[(dev, pin)], tracked
                )
            else:
                sys.exit(f"Unknown signal: {token!r} (use --external-trace ? for help)")

    sys.stdout.write("Starting Klipper + Buffer dual AVR simulation\n")
    sys.stdout.write(
        f"  Klipper: atmega644p {klipper_elf}  clock={klipper_clock}  baud={klipper_baud}  PTY={ptyname}\n"
    )
    for i in range(num_buffers):
        sys.stdout.write(
            f"  Buffer[{i}]: atmega2560 {buffer_elf}  i2c=0x{0x10 + i:02x}  clock={buffer_clock}  baud={buffer_baud}\n"
        )
    sys.stdout.flush()

    signal.signal(signal.SIGHUP, lambda sig, frame: sc.stop())
    signal.signal(signal.SIGINT, lambda sig, frame: sc.stop())

    # Real-time timeout via a background thread (RunTimeRange blocks and is
    # unreliable for finite durations with multi-device setups).
    if options.timeout:
        t = threading.Timer(
            options.timeout, lambda: os.kill(os.getpid(), signal.SIGINT)
        )
        t.start()

    try:
        internal_trace.start()
        if external_trace:
            external_trace.start()
        sc.RunTimeRange(0x7FFF0000FFFF0000)
    except KeyboardInterrupt:
        pass
    finally:
        internal_trace.finish()
        if external_trace:
            external_trace.finish()
        try:
            os.close(fd)
            os.unlink(ptyname)
        except os.error:
            pass

    sys.stdout.write("Simulation finished\n")


if __name__ == "__main__":
    main()
