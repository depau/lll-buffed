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
import sys
import termios
import threading
import time

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


class SerialRxPin(pysimulavr.PySimulationMember, pysimulavr.Pin):
    """Reads serial data from an AVR transmit pin and writes it to an output stream."""

    def __init__(self, baud, output):
        pysimulavr.Pin.__init__(self)
        pysimulavr.PySimulationMember.__init__(self)
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


class I2CMonitor(pysimulavr.Pin):
    """Attaches to an I2C Net and logs state transitions to stderr."""

    MAX_EVENTS = 500

    def __init__(self, name):
        pysimulavr.Pin.__init__(self)
        self.name = name
        self.sc = pysimulavr.SystemClock.Instance()
        self.last_state = None
        self.count = 0

    def SetInState(self, pin):
        pysimulavr.Pin.SetInState(self, pin)
        state = pin.outState
        if state == self.last_state:
            return
        self.last_state = state
        self.count += 1
        if self.count > self.MAX_EVENTS:
            return
        if self.count == self.MAX_EVENTS:
            sys.stderr.write("[I2C] (suppressing further events)\n")
            sys.stderr.flush()
            return
        level = "H" if state == self.HIGH else "L" if state == self.LOW else "?"
        t = self.sc.GetCurrentTime()
        sys.stderr.write(f"[I2C] {self.name}={level}  t={t}ns\n")
        sys.stderr.flush()


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
        self._device_pins = {}  # (dev_name, pin_name) -> Pin

    def add_device_pin(self, dev_name, dev, pin_name):
        pin = dev.GetPin(pin_name)
        self._net.Add(pin)
        self._device_pins[(dev_name, pin_name)] = pin

    def Add(self, pin):
        self._net.Add(pin)


# Pace the simulation scaled to real time
class Pacing(pysimulavr.PySimulationMember):
    def __init__(self, rate):
        pysimulavr.PySimulationMember.__init__(self)
        self.sc = pysimulavr.SystemClock.Instance()
        self.pacing_rate = 1.0 / (rate * SIMULAVR_FREQ)
        self.next_check_clock = 0
        self.rel_time = time.time()
        self.best_offset = 0.0
        self.delay = SIMULAVR_FREQ // 10000
        self.sc.Add(self)

    def DoStep(self, trueHwStep):
        curtime = time.time()
        clock = self.sc.GetCurrentTime()
        offset = clock * self.pacing_rate - (curtime - self.rel_time)
        self.best_offset = max(self.best_offset, offset)
        if offset > 0.000050:
            time.sleep(offset - 0.000040)
        if clock >= self.next_check_clock:
            self.rel_time -= min(self.best_offset, 0.0)
            self.next_check_clock = clock + self.delay * 500
            self.best_offset = -999999999.0
        return self.delay


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
        "-r",
        "--rate",
        type="float",
        dest="pacing_rate",
        default=0.0,
        help="real-time pacing rate",
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
    opts.add_option(
        "-i",
        "--debug-i2c",
        action="store_true",
        dest="debug_i2c",
        default=False,
        help="log I2C SCL/SDA transitions to stderr",
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

    sc = pysimulavr.SystemClock.Instance()
    internal_trace = InternalTrace(options.internal_tracefile, options.internal_trace)
    pysimulavr.cvar.sysConHandler.SetUseExit(True)

    # Create and load both devices
    factory = pysimulavr.AvrFactory.instance()

    klipper_dev = factory.makeDevice("atmega644p")
    klipper_dev.Load(klipper_elf)
    klipper_dev.SetClockFreq(SIMULAVR_FREQ // klipper_clock)
    sc.Add(klipper_dev)

    buffer_dev = factory.makeDevice("atmega2560")
    buffer_dev.Load(buffer_elf)
    buffer_dev.SetClockFreq(SIMULAVR_FREQ // buffer_clock)
    sc.Add(buffer_dev)

    internal_trace.load_options()

    # I2C SCL: klipper_dev PC0 (HW TWI SCL) <-> buffer_dev PE4 (SoftI2C SCL / INT4)
    i2c_scl_net = TrackedNet()
    i2c_scl_net.add_device_pin("klipper", klipper_dev, "C0")
    i2c_scl_net.add_device_pin("buffer", buffer_dev, "E4")

    # I2C SDA: klipper_dev PC1 (HW TWI SDA) <-> buffer_dev PE5 (SoftI2C SDA / INT5)
    i2c_sda_net = TrackedNet()
    i2c_sda_net.add_device_pin("klipper", klipper_dev, "C1")
    i2c_sda_net.add_device_pin("buffer", buffer_dev, "E5")

    # INT line (active-low open-drain): buffer_dev PD0 (I2C_INT_PIN) -> klipper_dev PC4
    i2c_int_net = TrackedNet()
    i2c_int_net.add_device_pin("klipper", klipper_dev, "C4")
    i2c_int_net.add_device_pin("buffer", buffer_dev, "D0")

    if options.debug_i2c:
        scl_mon = I2CMonitor("SCL")
        sda_mon = I2CMonitor("SDA")
        int_mon = I2CMonitor("INT")
        i2c_scl_net.Add(scl_mon)
        i2c_sda_net.Add(sda_mon)
        i2c_int_net.Add(int_mon)

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

    # Buffer firmware UART: TX only, printed to stdout with [BUFF] prefix
    buffer_out = LineBufferedOutput("[BUFF] ", sys.stdout)
    buffer_rxpin = SerialRxPin(buffer_baud, buffer_out)
    buffer_tx_net = TrackedNet()
    buffer_tx_net.add_device_pin("buffer", buffer_dev, "E1")
    buffer_tx_net.Add(buffer_rxpin)

    # External trace: observe Net/pin state as seen on the simulated wires.
    external_trace = None
    if options.external_trace:
        named_nets = {
            "scl": i2c_scl_net,
            "sda": i2c_sda_net,
            "int": i2c_int_net,
            "klipper_to_pty_tx": klipper_uart_tx_net,
            "klipper_from_pty_rx": klipper_uart_rx_net,
            "buffer_tx_to_stdout": buffer_tx_net,
        }

        # Build dev_pin_nets automatically from pins registered in each TrackedNet.
        # This is the single source of truth — no manual duplication needed.
        dev_pin_nets = {}
        for net in named_nets.values():
            for key in net._device_pins:
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
                    token.upper(), tracked._device_pins[(dev, pin)], tracked
                )
            else:
                sys.exit(f"Unknown signal: {token!r} (use --external-trace ? for help)")

    if options.pacing_rate:
        pacing = Pacing(options.pacing_rate)  # noqa: F841

    sys.stdout.write("Starting Klipper + Simulator dual AVR simulation\n")
    sys.stdout.write(
        f"  Klipper: atmega644p {klipper_elf}  clock={klipper_clock}  baud={klipper_baud}  PTY={ptyname}\n"
    )
    sys.stdout.write(
        f"  Simulator: atmega2560 {buffer_elf}  clock={buffer_clock}  baud={buffer_baud}\n"
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
