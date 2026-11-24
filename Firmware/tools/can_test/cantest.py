#!/usr/bin/env python3
"""
CAN bench test tool for boards running the BLDC-Driver firmware
(BLDC-Driver, MiniFOC, ...). See docs/can-tools.md.

Talks to the drivers over CAN (1 Mbit/s classic CAN, 11-bit IDs, protocol in
Firmware/include/can_protocol.h) and either runs an automatic test sequence
or lets you poke a driver by hand.

Commands
  auto     Run the test sequence, print PASS / FAIL, exit code 0 if all pass.
  monitor  Print every motor-driver frame on the bus, decoded. Sends nothing
           (add --listen-only to keep the adapter fully silent, e.g. with the
           robot's controller on the bus: it then doesn't even ACK frames).
  run      Start FOC if needed, then hold --current until Ctrl+C. Prints the
           mean velocity, state and flags once a second.
  busoff   Bus-off recovery test: spins the motor, asks you to unplug the
           CAN cable for a couple of seconds, and checks that the driver
           drops the current, keeps running and comes back on the bus.
  info     Read the firmware version, board (model name and capabilities,
           protocol 4+) and settings (CAN protocol 2+).
  config   Change the board's settings over CAN (protocol 2+), e.g.
             config --set-node 3 --save      give node 1 the node ID 3
             config --set-invert on --save
             config --cal clear --save       next Start FOC re-aligns
             config --cal store --save       keep this boot's alignment
             config --revert / --reboot
           Saved settings are PENDING until --save, which stops the motor,
           writes flash and reboots the board.
  list     List the connected adapters (elmue) or channels (python-can).

Boards are addressed by CAN node ID 1-15 (firmware 1.2+, `set node N` on the
serial console); --side left/right is shorthand for node 1/2.

Bit timing defaults to what the driver uses (1 Mbit/s, 87.5 %, classic CAN);
--bitrate, --sample-point, --fd, --data-bitrate, --data-sample-point change it.
The prescaler/segments are calculated from the adapter's clock and limits.
GUI: python cantest_gui.py

Adapters (--backend)
  elmue      (default) Candlelight adapter with ElmuSoft's CANable 2.5
             firmware, e.g. WeAct USB2CANFD v2. Uses ElmuSoft's Python classes
             (BSD-3), bundled in ./elmue (see elmue/README.md).
             No pip packages needed.
             Close HUD ECU Hacker first (only one program can open the adapter).
  python-can Any python-can interface (pip install python-can), e.g.
               --backend python-can --interface slcan --channel COM5
               --backend python-can --interface socketcan --channel can0
             (for socketcan, set the bitrate with `ip link` beforehand).

Examples
  python cantest.py --node 2 auto
  python cantest.py --side left run --current 0.5
  python cantest.py --node 3 info
  python cantest.py monitor
  python cantest.py monitor --listen-only

SAFETY: the motor WILL spin. Clamp it or lift the wheel. `auto` sends
current to the other half of the node's current frame (its partner node: 1/2,
3/4, ...) to check that it is ignored, so that partner must not be on the bus.
"""

import argparse
import math
import os
import struct
import sys
import time

sys.dont_write_bytecode = True

# ----------------------------------------------------------------------------
# Protocol (Firmware/include/can_protocol.h)
# ----------------------------------------------------------------------------

# CAN protocol 3 (firmware 1.2): node IDs 1-15. For node n:
#   current frame  0x010 + (n-1)/2, odd n bytes 0-3, even n bytes 4-7 (NaN = no command)
#   commands       0x020 + n
#   telemetry      0x100 + n
CURRENT_FRAME_BASE = 0x010
COMMAND_ID_BASE    = 0x020
TELEMETRY_ID_BASE  = 0x100
NODE_MIN, NODE_MAX = 1, 15
SIDE_NODES = {"left": 1, "right": 2}   # a robot's wheels
NAN = math.nan


def node_ids(node):
    """(current frame ID, byte offset in it, command ID, telemetry ID)."""
    return CURRENT_FRAME_BASE + (node - 1) // 2, 0 if node % 2 else 4, COMMAND_ID_BASE + node, TELEMETRY_ID_BASE + node


def partner_node(node):
    """The node that shares this node's current frame."""
    return node + 1 if node % 2 else node - 1


def is_current_frame(can_id):
    return CURRENT_FRAME_BASE <= can_id <= CURRENT_FRAME_BASE + (NODE_MAX - 1) // 2


def current_frame_nodes(can_id):
    """(node in bytes 0-3, node in bytes 4-7) of a current frame."""
    k = can_id - CURRENT_FRAME_BASE
    return 2 * k + 1, 2 * k + 2


CMD_SHAFT_VELOCITY = 0x02
CMD_DRIVER_STATUS  = 0x03
CMD_START_FOC      = 0x10
CMD_GET_INFO       = 0x20
CMD_GET_CONFIG     = 0x21
CMD_SET_PARAM      = 0x22
CMD_CALIBRATION    = 0x23
CMD_SAVE           = 0x24
CMD_REVERT         = 0x25
CMD_REBOOT         = 0x26
CMD_GET_BOARD_INFO = 0x27   # protocol 4+
CMD_GET_PARAM      = 0x28   # protocol 5+
CMD_COGGING        = 0x29   # protocol 7+
CMD_NACK           = 0xFE
CMD_ERROR          = 0xFF

CMD_NAMES = {CMD_START_FOC: "START_FOC", CMD_GET_INFO: "GET_INFO", CMD_GET_CONFIG: "GET_CONFIG",
             CMD_SET_PARAM: "SET_PARAM", CMD_CALIBRATION: "CALIBRATION", CMD_SAVE: "SAVE",
             CMD_REVERT: "REVERT", CMD_REBOOT: "REBOOT", CMD_GET_BOARD_INFO: "GET_BOARD_INFO",
             CMD_GET_PARAM: "GET_PARAM", CMD_COGGING: "COGGING"}
PARAM_NODE_ID, PARAM_INVERT, PARAM_VELOCITY_INTERVAL_MS = 1, 2, 0x10
PARAM_CURRENT_P, PARAM_CURRENT_I, PARAM_CURRENT_TF, PARAM_FOC_LOOP_HZ = 0x20, 0x21, 0x22, 0x30  # protocol 5+
PARAM_COG_COMP = 0x23  # protocol 7+
PARAM_NAMES = {PARAM_NODE_ID: "node ID", PARAM_INVERT: "invert", PARAM_VELOCITY_INTERVAL_MS: "velocity interval ms",
               PARAM_CURRENT_P: "current loop P", PARAM_CURRENT_I: "current loop I",
               PARAM_CURRENT_TF: "current filter Tf", PARAM_FOC_LOOP_HZ: "FOC loop Hz",
               PARAM_COG_COMP: "anti-cogging"}
# CMD_COGGING (protocol 7): operations, calibration states, failures, flags.
COG_OPS = {0: "status", 1: "calibrate", 2: "refine", 3: "abort", 4: "clear", 5: "result", 6: "read", 7: "info"}
COG_STATES = {0: "idle", 1: "settling", 2: "forward", 3: "backward", 4: "done", 5: "failed"}
COG_FAILURES = {0: "-", 1: "FOC not running", 2: "host sent a current", 3: "rotor didn't follow",
                4: "aborted", 5: "angles missed", 6: "no table to refine"}
COG_FLAGS = ((1, "table"), (2, "enabled"), (4, "UNSAVED"), (8, "other-sensor-direction"), (16, "applied"),
             (32, "refined"), (64, "slots-only"))
CAL_STORE, CAL_CLEAR = 1, 2


def set_param_payload(param, value):
    return bytes([CMD_SET_PARAM, param]) + struct.pack(">f", float(value))


def parse_config(data):
    """CMD_GET_CONFIG reply -> dict, or None."""
    if len(data) < 8 or data[0] != CMD_GET_CONFIG:
        return None
    f = data[3]
    return {"pending": bool(data[1]), "node": data[2], "invert": bool(f & CONFIG_FLAG_INVERT),
            "aligned": bool(f & CONFIG_FLAG_CALIBRATION), "sensor_ccw": bool(f & CONFIG_FLAG_SENSOR_CCW),
            "unsaved": bool(f & CONFIG_FLAG_UNSAVED), "zero_angle": be_float(data, 4)}


def reply_status(data):
    """Status byte of a configuration reply (0 = OK), or None for a NACK/unknown."""
    if not data or data[0] == CMD_NACK:
        return None
    c = data[0]
    if c == CMD_SET_PARAM and len(data) >= 3:
        return data[2]
    if c == CMD_CALIBRATION and len(data) >= 3:
        return data[2]
    if c == CMD_COGGING and len(data) >= 3:
        return data[2]
    if c in (CMD_SAVE, CMD_REVERT, CMD_REBOOT) and len(data) >= 2:
        return data[1]
    return 0
CONFIG_STATUS = {0: "OK", 1: "unknown command/parameter", 2: "bad value", 3: "no alignment to store",
                 4: "flash overlaps settings", 5: "FOC not running"}
CONFIG_FLAG_INVERT, CONFIG_FLAG_CALIBRATION, CONFIG_FLAG_SENSOR_CCW, CONFIG_FLAG_UNSAVED = 1, 2, 4, 8

STATE_NAMES = {0: "unconfigured", 1: "idle", 2: "running", 3: "fault (FOC start failed)"}
STATE_IDLE, STATE_RUNNING, STATE_FAULT = 1, 2, 3

FLAG_TIMEOUT, FLAG_CALIBRATION, FLAG_INVERTED, FLAG_DRV_FAULT = 1, 2, 4, 8

ERROR_NAMES = {
    1: "motor init failed",
    2: "current sense init failed",
    3: "FOC init / sensor alignment failed",
    4: "DRV8316 fault",
}

# CMD_GET_BOARD_INFO capability bits (protocol 4+). The tools decide what to
# show from these, never from the model name, so a new board needs no changes
# here. Unknown bits are shown as numbers.
CAP_MEASURES_SUPPLY_VOLTAGE = 1 << 0   # heartbeat supply mV is a measurement
CAP_MEASURES_INPUT_CURRENT  = 1 << 1   # heartbeat bytes 5-6: input current, int16 mA
CAP_NAMES = {CAP_MEASURES_SUPPLY_VOLTAGE: "supply voltage", CAP_MEASURES_INPUT_CURRENT: "input current"}


def cap_names(caps):
    names = [n for bit, n in CAP_NAMES.items() if caps & bit]
    names += ["bit %d" % i for i in range(32) if caps & (1 << i) and (1 << i) not in CAP_NAMES]
    return ", ".join(names) or "none"

# initFOC() blocks (motor init, current sense offset calibration, encoder and
# current sense alignment) and sends nothing on CAN meanwhile. Well over 5 s
# on Mk2 without a stored alignment.
START_TIMEOUT_S = 20.0


def be_float(data, pos):
    return struct.unpack(">f", bytes(data[pos:pos + 4]))[0]


def flag_names(flags):
    names = [n for bit, n in ((FLAG_TIMEOUT, "timeout"), (FLAG_CALIBRATION, "stored-alignment"),
                              (FLAG_INVERTED, "inverted"), (FLAG_DRV_FAULT, "DRV-fault")) if flags & bit]
    return " ".join(names) if names else "none"


def state_name(state):
    if state is None:
        return "unknown (no status received)"
    return STATE_NAMES.get(state, "invalid (%d)" % state)


def status_millivolts(data):
    """Supply mV from a CMD_DRIVER_STATUS frame (0 = no reading / old firmware)."""
    return (data[3] << 8) | data[4] if len(data) >= 5 else 0


def status_milliamps(data):
    """Input mA from a CMD_DRIVER_STATUS frame, or None (no reading, or a board
    without CAP_MEASURES_INPUT_CURRENT, which sends 5 bytes)."""
    if len(data) < 7:
        return None
    ma = int.from_bytes(bytes(data[5:7]), "big", signed=True)
    return None if ma == -32768 else ma


def fmt_input(mv, ma, places=3):
    """Input current and power from the heartbeat, e.g. "0.520 A, 12.5 W" (power
    = supply voltage x input current, negative while the motor regenerates).
    "" on a board that doesn't report the current (ma None)."""
    if ma is None:
        return ""
    text = "%.*f A" % (places, ma / 1000)
    if mv:
        text += ", %.1f W" % (mv * ma / 1e6)
    return text


class BoardInfoJoiner:
    """Joins the CMD_GET_BOARD_INFO replies seen on the bus into one line per
    board, for logs that show replies as they arrive. Page 0 (capabilities,
    hardware rev) is held until the model name is complete: a page containing a
    NUL, or page 5. Name pages after that (a host may ask for all five at once)
    show nothing. Keyed by anything that identifies the board (node, side)."""

    def __init__(self):
        self._page0 = {}   # key -> page 0 reply
        self._name = {}    # key -> name bytes so far (absent = not reading a name)

    def feed(self, key, data):
        """The line to show for this reply, or "" for nothing (yet)."""
        if len(data) < 8:
            return ""
        page = data[1]
        if page == 0:
            self._page0[key] = bytes(data)
            return ""
        if page == 1:
            self._name[key] = b""
        elif key not in self._name:
            return ""
        chunk = bytes(data[2:8])
        self._name[key] += chunk.split(b"\0")[0]
        if b"\0" not in chunk and page < 5:
            return ""
        text = "board %r" % self._name.pop(key).decode("ascii", "replace")
        p0 = self._page0.pop(key, None)
        if p0:
            text += ", hardware rev %d, capabilities: %s" % (p0[6], cap_names(int.from_bytes(p0[2:6], "big")))
        return text


def fmt_amps(v):
    return "-" if v != v else "%.3f A" % v   # NaN = no command


def decode_reply(data):
    """Configuration replies (protocol 2+), or None."""
    c = data[0]
    if c == CMD_GET_INFO and len(data) >= 6:
        return "info: protocol %d, firmware %d.%d.%d, image %d KB" % tuple(data[1:6])
    if c == CMD_GET_BOARD_INFO and len(data) >= 8:
        if data[1] == 0:
            return "board info: hardware rev %d, capabilities %s" % (
                data[6], cap_names(int.from_bytes(bytes(data[2:6]), "big")))
        if data[1] in (6, 7):
            return "board serial number part %d: %s" % (data[1] - 5, bytes(data[2:8]).hex().upper())
        return "board name part %d: %r" % (data[1], bytes(data[2:8]).split(b"\0")[0].decode("ascii", "replace"))
    if c == CMD_GET_CONFIG and len(data) >= 8:
        f = data[3]
        names = [n for bit, n in ((CONFIG_FLAG_INVERT, "invert"), (CONFIG_FLAG_CALIBRATION, "aligned"),
                                  (CONFIG_FLAG_SENSOR_CCW, "sensor-CCW"), (CONFIG_FLAG_UNSAVED, "UNSAVED")) if f & bit]
        return "config (%s): node %s, %s, zero angle %.4f" % (
            "pending" if data[1] else "active", data[2] or "not set", " ".join(names) or "no flags", be_float(data, 4))
    if c == CMD_SET_PARAM and len(data) >= 7:
        return "SET_PARAM %s: %s, now %g" % (PARAM_NAMES.get(data[1], "0x%02X" % data[1]),
                                             CONFIG_STATUS.get(data[2], data[2]), be_float(data, 3))
    if c == CMD_GET_PARAM and len(data) >= 7:
        return "GET_PARAM %s: %s, value %g" % (PARAM_NAMES.get(data[1], "0x%02X" % data[1]),
                                               CONFIG_STATUS.get(data[2], data[2]), be_float(data, 3))
    if c == CMD_CALIBRATION and len(data) >= 3:
        return "CALIBRATION %s: %s" % ({1: "store", 2: "clear"}.get(data[1], data[1]),
                                       CONFIG_STATUS.get(data[2], data[2]))
    if c in (CMD_SAVE, CMD_REVERT, CMD_REBOOT) and len(data) >= 2:
        return "%s: %s" % (CMD_NAMES[c], CONFIG_STATUS.get(data[1], data[1]))
    if c == CMD_COGGING and len(data) >= 8 and data[1] == 6:
        if data[2] == 0xFF:
            return "COGGING read: no table"
        vals = [b - 256 if b > 127 else b for b in data[4:8]]
        return "COGGING read: bins %d-%d: %s (counts)" % ((data[2] << 8) | data[3], ((data[2] << 8) | data[3]) + 3, vals)
    if c == CMD_COGGING and len(data) >= 8 and data[1] == 7:
        return "COGGING info: %s, %.3f mA per count, %d bins" % (CONFIG_STATUS.get(data[2], data[2]),
                                                               be_float(data, 3) * 1000, data[7] * 16)
    if c == CMD_COGGING and len(data) >= 8:
        head = "COGGING %s: %s" % (COG_OPS.get(data[1], data[1]), CONFIG_STATUS.get(data[2], data[2]))
        if data[1] == 5:
            err = "> 25.4" if data[5] == 255 else "%.1f" % (data[5] / 10)
            return "%s, coverage %d%%, agreement %d%%, max position error %s mrad, noise left out %.1f mA" % (
                head, data[3], data[4], err, data[6] / 2)
        state = COG_STATES.get(data[3], data[3])
        detail = COG_FAILURES.get(data[4], data[4]) if data[3] == 5 else "%d%%" % data[4]
        flags = " ".join(n for bit, n in COG_FLAGS if data[5] & bit) or "no table"
        return "%s, calibration %s (%s), %s, table %d mA p-p, friction %d mA" % (
            head, state, detail, flags, data[6] * 4, data[7] * 2)
    if c == CMD_NACK and len(data) >= 2:
        return "NACK: command 0x%02X not understood" % data[1]
    return None


def decode(can_id, data):
    """Human-readable decoding of a motor-driver frame, or None if not one."""
    d = bytes(data)
    if is_current_frame(can_id):
        a, b = current_frame_nodes(can_id)
        if len(d) != 8:
            return "currents node %d/%d, INVALID length %d" % (a, b, len(d))
        return "currents  node %d: %s, node %d: %s" % (a, fmt_amps(be_float(d, 0)), b, fmt_amps(be_float(d, 4)))
    if COMMAND_ID_BASE + NODE_MIN <= can_id <= COMMAND_ID_BASE + NODE_MAX and d:
        n = can_id - COMMAND_ID_BASE
        return "node %d command 0x%02X (%s)" % (n, d[0], CMD_NAMES.get(d[0], "?"))
    if TELEMETRY_ID_BASE + NODE_MIN <= can_id <= TELEMETRY_ID_BASE + NODE_MAX and d:
        n = can_id - TELEMETRY_ID_BASE
        c = d[0]
        if c == CMD_DRIVER_STATUS and len(d) >= 3:
            mv, ma = status_millivolts(d), status_milliamps(d)
            return "node %d status   %s, flags %s, supply %s%s" % (
                n, state_name(d[1]), flag_names(d[2]), "%.2f V" % (mv / 1000) if mv else "-",
                ", input " + fmt_input(mv, ma) if ma is not None else "")
        if c == CMD_SHAFT_VELOCITY and len(d) >= 8:
            iq = int.from_bytes(bytes(d[5:7]), "big", signed=True) / 1000.0
            idd = int.from_bytes(bytes(d[7:8]), "big", signed=True) * 0.016
            return "node %d velocity %.2f rad/s, current q %+.3f A d %+.3f A" % (n, be_float(d, 1), iq, idd)
        if c == CMD_SHAFT_VELOCITY and len(d) >= 5:
            return "node %d velocity %.2f rad/s" % (n, be_float(d, 1))
        if c == CMD_START_FOC and len(d) >= 5:
            return "node %d START_FOC reply %s" % (n, "OK" if be_float(d, 1) == 1.0 else "FAILED")
        if c == CMD_ERROR and len(d) >= 5:
            code = int(be_float(d, 1))
            return "node %d ERROR %d (%s)" % (n, code, ERROR_NAMES.get(code, "unknown"))
        r = decode_reply(d)
        if r:
            return "node %d %s" % (n, r)
        return "node %d telemetry, unknown command 0x%02X" % (n, c)
    return None


# ----------------------------------------------------------------------------
# Console colours (ANSI; os.system("") enables them in the Windows console)
# ----------------------------------------------------------------------------

if sys.platform == "win32":
    os.system("")
USE_COLOR = sys.stdout.isatty()


def color(text, code):
    return "\033[%sm%s\033[0m" % (code, text) if USE_COLOR else text


_ANSI = {"info": None, "warn": "33", "error": "31", "pass": "32", "fail": "31;1", "dim": "90", "prompt": "36;1"}


def _print_output(text, kind):
    code = _ANSI.get(kind)
    print(color(text, code) if code else text, flush=True)


# All output goes through here; the GUI replaces it with set_output().
_output = _print_output


def set_output(func):
    """func(text, kind) with kind in info/warn/error/pass/fail/dim/prompt."""
    global _output
    _output = func


def emit(text, kind="info"):
    _output(text, kind)


def info(msg):
    emit(msg, "info")


def warn(msg):
    emit(msg, "warn")


def error(msg):
    emit(msg, "error")


# ----------------------------------------------------------------------------
# Bus settings and bit timing
# ----------------------------------------------------------------------------

# What the firmware uses (can_protocol.h). Other settings are allowed
# (e.g. to sniff another bus) but the driver won't talk to the adapter.
DRIVER_BITRATE = 1_000_000
DRIVER_SAMPLE_POINT = 87.5


class CanSettings:
    def __init__(self, bitrate=DRIVER_BITRATE, sample_point=DRIVER_SAMPLE_POINT,
                 fd=False, data_bitrate=2_000_000, data_sample_point=75.0):
        self.bitrate = int(bitrate)
        self.sample_point = float(sample_point)
        self.fd = bool(fd)
        self.data_bitrate = int(data_bitrate)
        self.data_sample_point = float(data_sample_point)

    def describe(self):
        s = "%s @ %.1f %%" % (fmt_bitrate(self.bitrate), self.sample_point)
        if self.fd:
            s += ", CAN FD data %s @ %.1f %%" % (fmt_bitrate(self.data_bitrate), self.data_sample_point)
        return s

    def warn_if_not_driver(self):
        if self.bitrate != DRIVER_BITRATE or abs(self.sample_point - DRIVER_SAMPLE_POINT) > 0.1:
            warn("Note: the firmware uses %s @ %.1f %% classic CAN; these settings won't match it."
                 % (fmt_bitrate(DRIVER_BITRATE), DRIVER_SAMPLE_POINT))


def fmt_bitrate(b):
    if b >= 1_000_000 and b % 1_000_000 == 0:
        return "%d Mbit/s" % (b // 1_000_000)
    if b >= 1000 and b % 1000 == 0:
        return "%d kbit/s" % (b // 1000)
    return "%d bit/s" % b


def parse_bitrate(text):
    """'1M', '500k', '1000000' -> int."""
    t = str(text).strip().lower().replace("bit/s", "").replace("bps", "").strip()
    mult = 1
    if t.endswith("m"):
        mult, t = 1_000_000, t[:-1]
    elif t.endswith("k"):
        mult, t = 1000, t[:-1]
    return int(round(float(t) * mult))


def _segments(clk, brp, bitrate, sample_point, lim):
    """(brp, seg1, seg2, actual_sample_point) for the bitrate (within 0.1 %, e.g.
    83.333k), or None."""
    n = int(round(clk / (brp * bitrate)))  # time quanta per bit = 1 + seg1 + seg2
    if n < 3 or abs(clk / (brp * n) - bitrate) > bitrate * 0.001:
        return None
    seg2 = int(round(n * (100.0 - sample_point) / 100.0))
    seg2 = max(lim.seg2_min, min(lim.seg2_max, seg2))
    seg1 = n - 1 - seg2
    if not lim.seg1_min <= seg1 <= lim.seg1_max:
        return None
    return brp, seg1, seg2, 100.0 * (1 + seg1) / n


def calc_timing(clk, bitrate, sample_point, lim):
    """Best sample point match; on a tie the smallest prescaler (ElmuSoft's advice)."""
    best = None
    for brp in range(max(1, lim.brp_min), lim.brp_max + 1):
        r = _segments(clk, brp, bitrate, sample_point, lim)
        if r and (best is None or abs(r[3] - sample_point) < abs(best[3] - sample_point) - 1e-9):
            best = r
    return best


def calc_timing_fd(clk, s, lim_nom, lim_data):
    """Nominal + data timing, preferring one shared prescaler (ElmuSoft's advice)."""
    best = None
    for brp in range(max(1, lim_nom.brp_min, lim_data.brp_min), min(lim_nom.brp_max, lim_data.brp_max) + 1):
        n = _segments(clk, brp, s.bitrate, s.sample_point, lim_nom)
        d = _segments(clk, brp, s.data_bitrate, s.data_sample_point, lim_data)
        if n and d:
            err = abs(n[3] - s.sample_point) + abs(d[3] - s.data_sample_point)
            if best is None or err < best[0] - 1e-9:
                best = (err, n, d)
    if best:
        return best[1], best[2]
    return (calc_timing(clk, s.bitrate, s.sample_point, lim_nom),
            calc_timing(clk, s.data_bitrate, s.data_sample_point, lim_data))


# ----------------------------------------------------------------------------
# Adapter backends: send(id, data), recv(timeout_s) -> (id, data) | None, close()
# ----------------------------------------------------------------------------

def _fix_elmue_setupapi_types():
    """ElmuSoft's OsLibrary_Windows.py calls the SetupAPI functions through
    ctypes without argtypes/restype, so ctypes treats the HDEVINFO handle as a
    32-bit int. On 64-bit Python the handle gets truncated and EnumDevices()
    fails with "API Error 6: The handle is invalid". Declare the real
    signatures (ctypes caches the function objects, so this fixes their calls
    too). HDEVINFO is declared as c_ssize_t so INVALID_HANDLE_VALUE stays -1,
    which is what their code compares against."""
    import ctypes
    from ctypes import wintypes
    import OsLibrary_Windows as osl
    sa = osl.setupapi
    H, P, D, B = ctypes.c_ssize_t, ctypes.c_void_p, wintypes.DWORD, wintypes.BOOL
    sigs = {
        "SetupDiGetClassDevsW":              (H, [P, P, P, D]),
        "SetupDiCreateDeviceInfoList":       (H, [P, P]),
        "SetupDiDestroyDeviceInfoList":      (B, [H]),
        "SetupDiEnumDeviceInterfaces":       (B, [H, P, P, D, P]),
        "SetupDiGetDeviceInterfaceDetailW":  (B, [H, P, P, D, P, P]),
        "SetupDiGetDeviceRegistryPropertyW": (B, [H, P, D, P, P, D, P]),
        "SetupDiGetDevicePropertyW":         (B, [H, P, P, P, P, D, P, D]),
        "SetupDiOpenDeviceInfoW":            (B, [H, P, P, D, P]),
    }
    for name, (restype, argtypes) in sigs.items():
        fn = getattr(sa, name)
        fn.restype = restype
        fn.argtypes = argtypes


# ElmuSoft's Candlelight classes, bundled next to this script.
ELMUE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "elmue")


def _load_elmue():
    elmue_dir = ELMUE_DIR
    if not os.path.isfile(os.path.join(elmue_dir, "Candlelight.py")):
        raise SystemExit(
            "ElmuSoft's Candlelight.py not found in %s\n"
            "It should be bundled in tools/can_test/elmue (see elmue/README.md).\n"
            "Upstream: 'SampleApplication Python/Source' of\n"
            "https://github.com/Elmue/CANable-2.5-firmware-Slcan-and-Candlelight" % elmue_dir)
    if elmue_dir not in sys.path:
        sys.path.insert(0, elmue_dir)
    import Candlelight as cl  # noqa: E402  (ElmuSoft, BSD-3)
    if sys.platform == "win32":
        _fix_elmue_setupapi_types()
    return cl


def list_elmue_devices():
    """Descriptions of the connected Candlelight 2.5 adapters, in --device index order."""
    cl = _load_elmue()
    devs = cl.Candlelight().EnumDevices(True)
    return ["%s | %s | %s" % (d.ms_Product, d.ms_Interface, d.ms_SerialNo or "no serial") for d in devs]


def list_pycan_channels(interface):
    """Channels python-can can find for an interface (COM ports for slcan)."""
    chans = []
    try:
        import can
        for cfg in can.detect_available_configs(interfaces=[interface]):
            ch = str(cfg.get("channel", ""))
            if ch and ch not in chans:
                chans.append(ch)
    except Exception:
        pass
    if not chans and interface in ("slcan", "serial", "robotell", "seeedstudio"):
        try:
            from serial.tools import list_ports
            chans = [p.device for p in list_ports.comports()]
        except Exception:
            pass
    return chans


class ElmueBus:
    """Candlelight adapter with ElmuSoft CANable 2.5 firmware, via ElmuSoft's Python classes."""

    def __init__(self, device_index=0, settings=None, listen_only=False):
        settings = settings or CanSettings()
        cl = _load_elmue()
        self.cl = cl

        self.candle = cl.Candlelight()
        devices = self.candle.EnumDevices(True)
        if not devices:
            raise SystemExit("No Candlelight 2.5 adapter found (is HUD ECU Hacker or another tool holding it?).")
        if device_index >= len(devices):
            raise SystemExit("Adapter %d selected but only %d found." % (device_index, len(devices)))
        self.candle.Open(devices[device_index])
        for detail in self.candle.GetDetails():
            info("  " + detail.Format(22))

        dev = self.candle.GetDeviceInfo()
        clk = dev.mk_Capability.fclk_can
        if settings.fd:
            if not dev.mb_SupportsFD:
                self.candle.Close()
                raise SystemExit("This adapter does not support CAN FD.")
            nom, data = calc_timing_fd(clk, settings, dev.mk_CapabilityFD.time_nom, dev.mk_CapabilityFD.time_data)
        else:
            nom, data = calc_timing(clk, settings.bitrate, settings.sample_point, dev.mk_Capability.time), None

        for name, t, rate, sp in (("nominal", nom, settings.bitrate, settings.sample_point),
                                  ("data", data, settings.data_bitrate, settings.data_sample_point)):
            if name == "data" and not settings.fd:
                continue
            if t is None:
                self.candle.Close()
                raise SystemExit("Can't make an exact %s %s from the %d MHz CAN clock."
                                 % (name, fmt_bitrate(rate), clk // 1_000_000))
            if abs(t[3] - sp) > 1.0:
                warn("  %s sample point %.1f %% requested, nearest possible is %.1f %%" % (name, sp, t[3]))

        self.timing_text = self.candle.SetBitrate(False, *nom[:3])
        info("  %s  (BRP %d, Seg1 %d, Seg2 %d)" % ((self.timing_text,) + tuple(nom[:3])))
        if data:
            t = self.candle.SetBitrate(True, *data[:3])
            info("  %s  (BRP %d, Seg1 %d, Seg2 %d)" % ((t,) + tuple(data[:3])))
            self.timing_text += "; " + t
        if listen_only:
            # Silent: no ACKs, no error frames, can't send. The other nodes on
            # the bus (drivers + controller) must ACK each other.
            self.candle.Start(cl.eDeviceFlags.GS_DevFlagListenOnly)
            self.timing_text += " (listen-only)"
        else:
            self.candle.Start(cl.eDeviceFlags.GS_DevFlagNone)  # normal mode: ACKs frames, auto-retransmit
        self.adapter_errors = 0
        self._last_err_print = 0.0

    def send(self, can_id, data):
        p = self.cl.kCanPacket()
        p.mu32_ID = can_id
        p.mu8_Data = bytearray(data)
        self.candle.SendPacket(p)

    def recv(self, timeout_s):
        r = self.candle.ReceiveData(max(1, int(timeout_s * 1000)))
        if r is None:
            return None
        if r.mk_RxFrame is not None:
            p = self.candle.RxFrameToCanPacket(r)
            if p.mb_29bit:
                return None
            return p.mu32_ID, bytes(p.mu8_Data)
        if r.mk_Error is not None:
            self.adapter_errors += 1
            now = time.monotonic()
            if now - self._last_err_print > 1.0:
                self._last_err_print = now
                msg = self.candle.FormatCanErrors(r)[0]
                warn("  [adapter] CAN error: %s" % msg)
        return None

    def close(self):
        if sys.platform == "win32":
            self._stop_pipe_thread()
        try:
            self.candle.Close()
        except Exception:
            pass

    def _stop_pipe_thread(self):
        """Work around a use-after-free in ElmuSoft's OsLibrary_Windows.Close().

        Their _PipeThread keeps an overlapped WinUsb_ReadPipe() pending, with
        the OVERLAPPED struct as a local of the thread. Close() just wakes the
        thread and lets it exit with the read still pending, then WinUsb_Free()
        cancels it and Windows writes the result into the freed struct: the
        process dies with an access violation, usually on the next open or a
        little later (seen as the GUI closing itself on Disconnect).

        Fix: ask the thread to stop, then cancel the pending read with
        WinUsb_AbortPipe() WHILE the thread (and its OVERLAPPED) still exist.
        The read completes as aborted, the thread wakes, sees the stop flag and
        exits with nothing pending. Repeated in case the thread had just
        started a new read."""
        try:
            import ctypes
            import OsLibrary_Windows as osl
            lib = self.candle.mi_OsLibrary
            abort = osl.winusb.WinUsb_AbortPipe
            abort.argtypes = [ctypes.c_void_p, ctypes.c_ubyte]
            abort.restype = ctypes.c_int
            end = time.monotonic() + 1.0
            while lib.mb_ThreadRuns and lib.mh_WinUsb and time.monotonic() < end:
                lib.mb_AbortThread = True
                abort(lib.mh_WinUsb, lib.mk_Info.mu8_EndpointIN)
                time.sleep(0.01)
        except Exception as ex:
            warn("  [adapter] could not stop the USB receive thread cleanly: %s" % ex)


class PyCanBus:
    """Any python-can interface (slcan COM port, socketcan, gs_usb, pcan, ...)."""

    def __init__(self, interface, channel, settings=None, listen_only=False):
        settings = settings or CanSettings()
        try:
            import can
        except ImportError:
            raise SystemExit("python-can is not installed: pip install python-can")
        kwargs = {"interface": interface, "channel": channel}
        if interface != "socketcan":            # socketcan: set with `ip link` beforehand
            kwargs["bitrate"] = settings.bitrate
        if settings.fd:
            kwargs["fd"] = True
            if interface != "socketcan":
                kwargs["data_bitrate"] = settings.data_bitrate
        warn("  python-can: sample point is not set by this tool (the interface's default is used).")
        if listen_only:
            warn("  python-can: listen-only is not set by this tool; nothing is sent, but the adapter still ACKs.")
        self.bus = can.Bus(**kwargs)
        self.can = can
        self.timing_text = settings.describe()
        self.adapter_errors = 0

    def send(self, can_id, data):
        self.bus.send(self.can.Message(arbitration_id=can_id, data=bytes(data), is_extended_id=False))

    def recv(self, timeout_s):
        msg = self.bus.recv(timeout_s)
        if msg is None:
            return None
        if msg.is_error_frame:
            self.adapter_errors += 1
            return None
        if msg.is_extended_id or msg.is_remote_frame:
            return None
        return msg.arbitration_id, bytes(msg.data)

    def close(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass


# ----------------------------------------------------------------------------
# Driver session: sends commands on a fixed interval and tracks telemetry
# ----------------------------------------------------------------------------

class Aborted(Exception):
    pass


class Driver:
    def __init__(self, bus, node, interval_ms, verbose):
        self.bus = bus
        self.node = node
        self.cur_id, self.offset, self.cmd_id, self.tele_id = node_ids(node)
        self.config = {}             # 0 = active, 1 = pending: parse_config() of the last GET_CONFIG replies
        self.interval = interval_ms / 1000.0
        self.verbose = verbose

        self.last_state = None
        self.last_flags = 0
        self.last_vel = 0.0
        self.last_status_time = None
        self.last_mv = 0             # supply mV from the heartbeat (0 = no reading)
        self.replies = {}            # last configuration reply per command byte
        self.info = None             # (protocol, major, minor, patch, image KB) from CMD_GET_INFO
        self.board = None            # {"caps", "rev", "model"} from CMD_GET_BOARD_INFO (protocol 4+)
        self.last_ma = None          # input mA from the heartbeat (None = not reported)
        self.start_reply = None
        self.errors = []
        self.drv_fault_seen = False
        self.tx_failures = 0
        self._last_tx_fail_print = 0.0
        self.should_abort = None  # optional callable; run() raises Aborted when it returns True
        # Callables (can_id, data) that also get every received frame, e.g. the
        # other side's Driver on the same bus, or a CommandWatch.
        self.observers = []
        self.reset_window()

    # --- tracking ---

    def reset_window(self):
        self.flags_seen = 0
        self.status_count = 0
        self.vel_count = 0
        self.vel_sum = 0.0

    def vel_avg(self):
        return self.vel_sum / self.vel_count if self.vel_count else 0.0

    def _process(self, can_id, data):
        if self.verbose:
            text = decode(can_id, data)
            if text:
                emit("  rx 0x%03X  %s" % (can_id, text), "dim")
        if can_id != self.tele_id or not data:
            return
        cmd = data[0]
        if cmd == CMD_DRIVER_STATUS and len(data) >= 3:
            self.last_state, self.last_flags = data[1], data[2]
            self.last_mv = status_millivolts(data)
            self.last_ma = status_milliamps(data)
            self.flags_seen |= data[2]
            self.status_count += 1
            self.last_status_time = time.monotonic()
            if data[2] & FLAG_DRV_FAULT:
                self.drv_fault_seen = True
        elif cmd == CMD_SHAFT_VELOCITY and len(data) >= 5:
            self.last_vel = be_float(data, 1)
            self.vel_sum += self.last_vel
            self.vel_count += 1
        elif cmd == CMD_START_FOC and len(data) >= 5:
            self.start_reply = be_float(data, 1)
        elif cmd == CMD_ERROR and len(data) >= 5:
            code = int(be_float(data, 1))
            self.errors.append(code)
            error("  node %d reported error %d (%s)" % (self.node, code, ERROR_NAMES.get(code, "unknown")))
        elif cmd in CMD_NAMES or cmd == CMD_NACK:
            if cmd == CMD_GET_INFO and len(data) >= 6:
                self.info = tuple(data[1:6])
            if cmd == CMD_GET_CONFIG:
                cfg = parse_config(data)
                if cfg:
                    self.config[1 if cfg["pending"] else 0] = cfg
            # A NACK is filed under the command it rejects, so request() sees it.
            self.replies[data[1] if cmd == CMD_NACK and len(data) >= 2 else cmd] = bytes(data)

    # --- sending ---

    def _send(self, can_id, data):
        try:
            self.bus.send(can_id, data)
        except Exception as ex:  # e.g. adapter Tx buffer full while the bus is down
            self.tx_failures += 1
            now = time.monotonic()
            if now - self._last_tx_fail_print > 1.0:
                self._last_tx_fail_print = now
                warn("  [adapter] send failed: %s" % ex)

    def send_start(self):
        self._send(self.cmd_id, bytes([CMD_START_FOC]))

    def send_current(self, mine, other=NAN, mode="normal"):
        """This node's current frame: `mine` in our slot, `other` (default NaN =
        no command) in the partner node's slot. Mode 'nan' puts NaN in ours."""
        if mode == "short":
            data = struct.pack(">f", mine)  # 4 bytes: must be ignored by the driver
        else:
            buf = bytearray(8)
            buf[self.offset:self.offset + 4] = struct.pack(">f", math.nan if mode == "nan" else mine)
            buf[4 - self.offset:8 - self.offset] = struct.pack(">f", other)
            data = bytes(buf)
        self._send(self.cur_id, data)

    def run(self, seconds, mine=0.0, other=NAN, mode="normal", until=None, tx=None):
        """Send the command every interval (mode 'none' sends nothing) and
        process received frames for `seconds`. Returns True if `until()`
        became true first. `tx`, if given, is called every interval instead
        of sending this node's current frame (e.g. to drive several nodes)."""
        now = time.monotonic()
        end = now + seconds
        next_tx = now
        while now < end:
            if self.should_abort is not None and self.should_abort():
                raise Aborted()
            if mode != "none" and now >= next_tx:
                if tx is not None:
                    tx()
                else:
                    self.send_current(mine, other, mode)
                next_tx += self.interval
                if next_tx < now:
                    next_tx = now + self.interval
            frame = self.bus.recv(0.001)
            if frame is not None:
                self._process(*frame)
                for obs in self.observers:
                    obs(*frame)
            if until is not None and until():
                return True
            now = time.monotonic()
        return False

    def request(self, payload, timeout_s=0.3):
        """Send a configuration command, return its reply frame (or None).
        Firmware 1.0 (protocol 1) doesn't answer them."""
        cmd = payload[0]
        self.replies.pop(cmd, None)
        self._send(self.cmd_id, bytes(payload))
        self.run(timeout_s, mode="none", until=lambda: cmd in self.replies)
        return self.replies.get(cmd)

    def set_node(self, node):
        """Talk to another node ID from now on (e.g. after its node ID was changed and saved)."""
        self.node = node
        self.cur_id, self.offset, self.cmd_id, self.tele_id = node_ids(node)
        self.last_state, self.last_status_time, self.info, self.config = None, None, None, {}
        self.board, self.last_ma = None, None

    def configure(self, ops):
        """Run configuration commands in order (each a payload, e.g.
        set_param_payload(PARAM_INVERT, 1) or [CMD_SAVE]); stops at the first
        failure. CMD_SAVE with a changed node ID follows the board to its new
        ID once it has rebooted. Returns True if all succeeded."""
        for op in ops:
            op = bytes(op)
            name = CMD_NAMES.get(op[0], "0x%02X" % op[0])
            new_node = None
            if op[0] == CMD_SAVE:
                r = self.request([CMD_GET_CONFIG, 1])
                pending = parse_config(r) if r else None
                if pending and pending["node"] and pending["node"] != self.node:
                    new_node = pending["node"]
            r = self.request(op)
            if r is None:
                error("  node %d: no reply to %s (firmware 1.0 has no configuration over CAN)" % (self.node, name))
                return False
            status = reply_status(r)
            text = decode_reply(r) or name
            if status != 0:
                error("  node %d: %s" % (self.node, text))
                return False
            info("  node %d: %s" % (self.node, text))
            if op[0] in (CMD_SAVE, CMD_REBOOT):
                old = self.node
                if new_node:
                    info("  node %d is now node %d: following it to commands 0x%03X / telemetry 0x%03X"
                         % (old, new_node, *node_ids(new_node)[2:]))
                    self.set_node(new_node)
                info("  waiting for node %d to reboot..." % self.node)
                self.run(0.5, mode="none")   # the old heartbeat stops ~20 ms after the reply
                self.last_status_time = None
                back = self.run(10.0, mode="none", until=lambda: self.last_status_time is not None)
                if not back:
                    error("  node %d did not come back within 10 s (check the serial console)" % self.node)
                    return False
                info("  node %d is back (state %s)" % (self.node, state_name(self.last_state)))
        return True

    def read_board_info(self):
        """CMD_GET_BOARD_INFO pages into self.board (protocol 4+). Returns it,
        or None if the board doesn't support it (call read GET_INFO first: on
        older firmware this is skipped without waiting for a reply)."""
        if self.info is not None and self.info[0] < 4:
            self.board = None
            return None
        r = self.request([CMD_GET_BOARD_INFO, 0])
        if r is None or r[0] != CMD_GET_BOARD_INFO or len(r) < 8:
            self.board = None
            return None
        board = {"caps": int.from_bytes(bytes(r[2:6]), "big"), "rev": r[6], "model": ""}
        name = b""
        for page in range(1, 6):
            p = self.request([CMD_GET_BOARD_INFO, page])
            if p is None or p[0] != CMD_GET_BOARD_INFO or len(p) < 8:
                break
            chunk = bytes(p[2:8])
            name += chunk.split(b"\0")[0]
            if b"\0" in chunk:
                break
        board["model"] = name.decode("ascii", "replace")
        self.board = board
        return board

    def board_label(self):
        """Model name as the board reports it, or a fallback for older firmware."""
        if self.board and self.board["model"]:
            return self.board["model"]
        return "board not reported (firmware before 1.3)" if self.info else ""

    def has_cap(self, bit):
        return bool(self.board and self.board["caps"] & bit)

    def read_config(self):
        """GET_CONFIG active + pending into self.config; False if no reply."""
        ok = True
        for which in (0, 1):
            ok = self.request([CMD_GET_CONFIG, which]) is not None and ok
        return ok

    def start_foc(self, timeout_s=START_TIMEOUT_S):
        """Returns (ok, seconds) - ok is None if no reply arrived."""
        self.start_reply = None
        t0 = time.monotonic()
        self.send_start()
        self.run(timeout_s, mode="none", until=lambda: self.start_reply is not None)
        dt = time.monotonic() - t0
        if self.start_reply is None:
            return None, dt
        return self.start_reply == 1.0, dt


class CommandWatch:
    """Tracks the commands ANOTHER node (the robot's controller) sends: current
    frames (0x010-0x017) and commands (0x021-0x02F), per target node. The
    adapter never receives its own frames (elmue: they come back as Tx echoes;
    python-can: receive_own_messages is off), so everything seen here came
    from someone else."""

    LATE_S = 0.1  # driver command timeout (COMMAND_TIMEOUT_MS)

    def __init__(self):
        self.current = {}       # node -> last commanded current (A); NaN slots are not commands
        self.last_time = {}     # node -> time of the last real (non-NaN) command
        self.late = {}          # node -> gaps longer than the driver's command timeout
        self.max_gap = {}       # node -> longest gap since take_max_gap()
        self.count = 0          # valid current frames, total
        self.invalid = 0        # current frames with the wrong length
        self.last_frame_time = None
        self.commands = {}      # node -> number of command frames (START_FOC, config, ...)
        self.nodes_heard = {}   # node -> time its telemetry was last seen

    def process(self, can_id, data):
        now = time.monotonic()
        if TELEMETRY_ID_BASE + NODE_MIN <= can_id <= TELEMETRY_ID_BASE + NODE_MAX:
            self.nodes_heard[can_id - TELEMETRY_ID_BASE] = now
            return
        if is_current_frame(can_id):
            if len(data) != 8:
                self.invalid += 1
                return
            self.count += 1
            self.last_frame_time = now
            for node, pos in zip(current_frame_nodes(can_id), (0, 4)):
                v = be_float(data, pos)
                if v != v:
                    continue  # NaN: no command for this node
                prev = self.last_time.get(node)
                if prev is not None:
                    gap = now - prev
                    self.max_gap[node] = max(self.max_gap.get(node, 0.0), gap)
                    if gap > self.LATE_S:
                        self.late[node] = self.late.get(node, 0) + 1
                self.last_time[node] = now
                self.current[node] = v
            return
        if COMMAND_ID_BASE + NODE_MIN <= can_id <= COMMAND_ID_BASE + NODE_MAX and data:
            node = can_id - COMMAND_ID_BASE
            self.commands[node] = self.commands.get(node, 0) + 1
            emit("  another node sent %s to node %d" % (CMD_NAMES.get(data[0], "command 0x%02X" % data[0]), node),
                 "prompt")

    def active(self, node=None, max_age=0.5):
        """Commands seen recently for `node`, or any current frame if node is None."""
        t = self.last_frame_time if node is None else self.last_time.get(node)
        return t is not None and time.monotonic() - t < max_age

    def heard_recently(self, node, max_age=1.0):
        t = self.nodes_heard.get(node)
        return t is not None and time.monotonic() - t < max_age

    def take_max_gap(self, node):
        return self.max_gap.pop(node, 0.0)


# ----------------------------------------------------------------------------
# Results
# ----------------------------------------------------------------------------

class Results:
    def __init__(self):
        self.passed = 0
        self.failed = 0

    def check(self, ok, name, detail=""):
        if ok:
            self.passed += 1
            emit("PASS  %s - %s" % (name, detail), "pass")
        else:
            self.failed += 1
            emit("FAIL  %s - %s" % (name, detail), "fail")
        return ok

    def summary(self):
        text = "==== %d passed, %d failed ====" % (self.passed, self.failed)
        emit(text, "pass" if self.failed == 0 else "fail")
        return self.failed == 0


def start_or_fail(drv, res, name="Start FOC"):
    info("Sending START_FOC, waiting up to %.0f s (the driver sends nothing while it aligns)..." % START_TIMEOUT_S)
    n_err = len(drv.errors)
    ok, dt = drv.start_foc()
    if ok is None:
        return res.check(False, name, "no reply within %.0f s" % START_TIMEOUT_S)
    detail = "reply %s after %.1f s" % ("OK" if ok else "FAILED", dt)
    if not ok and len(drv.errors) > n_err:
        code = drv.errors[-1]
        detail += ", error %d (%s)" % (code, ERROR_NAMES.get(code, "unknown"))
    return res.check(ok, name, detail)


# ----------------------------------------------------------------------------
# Commands
# ----------------------------------------------------------------------------

def cmd_auto(drv, args):
    res = Results()
    I = abs(args.current) if abs(args.current) >= 0.05 else 0.3

    # 1. Heartbeat
    drv.reset_window()
    drv.run(1.0, mode="none")
    if drv.status_count == 0:
        res.check(False, "Status heartbeat",
                  "nothing received on 0x%03X. Check the node ID (`config` on serial), bitrate, "
                  "termination, GND, and that nothing else holds the adapter." % drv.tele_id)
        return res.summary()
    res.check(8 <= drv.status_count <= 12, "Status heartbeat", "%d frames in 1 s (expect 10)" % drv.status_count)
    info("Driver state: %s, flags: %s" % (state_name(drv.last_state), flag_names(drv.last_flags)))
    res.check(6000 <= drv.last_mv <= 30000, "Supply voltage in the heartbeat",
              "%.2f V (expect 6-30 V; 0 = no reading, or firmware without the field)" % (drv.last_mv / 1000))
    reply = drv.request([CMD_GET_INFO])
    if res.check(reply is not None, "CMD_GET_INFO answered", decode_reply(reply) if reply else
                 "no reply: firmware 1.0 (no configuration over CAN)"):
        res.check(drv.info is not None and drv.info[0] >= 3, "CAN protocol 3 or later (node IDs)",
                  "protocol %s" % (drv.info[0] if drv.info else "?"))

    # 2. Start FOC
    if not start_or_fail(drv, res):
        return res.summary()
    drv.reset_window()
    drv.run(0.3, mode="none")
    res.check(drv.last_state == STATE_RUNNING, "State is running after start", state_name(drv.last_state))
    res.check(drv.flags_seen & FLAG_TIMEOUT, "Timeout flag set while no commands are sent", flag_names(drv.flags_seen))
    if drv.last_flags & FLAG_CALIBRATION:
        info("  (stored alignment used)")

    # 3. Zero current clears the timeout, velocity telemetry rate
    drv.reset_window()
    drv.run(1.0, 0.0)
    res.check(not drv.last_flags & FLAG_TIMEOUT, "Current commands clear the timeout flag", flag_names(drv.last_flags))
    res.check(80 <= drv.vel_count <= 110, "Velocity telemetry rate", "%d frames in 1 s (expect ~100)" % drv.vel_count)

    # 4./5. Direction
    for sign, name in ((1, "Positive current spins forward"), (-1, "Negative current spins backward")):
        drv.run(2.0, sign * I)
        drv.reset_window()
        drv.run(1.0, sign * I)
        v = drv.vel_avg()
        res.check(sign * v > 2.0, name, "%+.2f A -> mean %+.2f rad/s (expect %s2)" % (sign * I, v, ">+" if sign > 0 else "<-"))

    # 6. Other motor's half of 0x020 is ignored
    drv.run(3.0, 0.0)  # coast
    v0 = abs(drv.last_vel)
    drv.run(2.0, 0.0, I)
    v1 = abs(drv.last_vel)
    res.check(v1 <= v0 + 1.0, "Partner node's half of the current frame is ignored",
              "|vel| %.2f -> %.2f rad/s while %.2f A sent to node %d" % (v0, v1, I, partner_node(drv.node)))

    # 7. Command timeout while spinning
    drv.run(2.0, I)
    drv.reset_window()
    drv.run(0.4, mode="none")
    res.check(drv.flags_seen & FLAG_TIMEOUT, "Command timeout after 400 ms of silence", flag_names(drv.flags_seen))
    drv.reset_window()
    drv.run(0.5, 0.0)
    res.check(not drv.last_flags & FLAG_TIMEOUT, "Timeout clears when commands resume", flag_names(drv.last_flags))

    # 8. Short frames, 9. NaN
    for mode, name in (("short", "4-byte current frames are ignored"), ("nan", "NaN current is ignored")):
        drv.reset_window()
        drv.run(0.4, 0.0, mode=mode)
        res.check(drv.flags_seen & FLAG_TIMEOUT, name, flag_names(drv.flags_seen))
        drv.run(0.5, 0.0)

    # 10. START_FOC while running replies at once
    drv.start_reply = None
    t0 = time.monotonic()
    drv.send_start()
    drv.run(1.0, 0.0, until=lambda: drv.start_reply is not None)
    dt = time.monotonic() - t0
    res.check(drv.start_reply == 1.0 and dt < 0.2, "START_FOC while running replies immediately",
              ("no reply" if drv.start_reply is None else "reply %.1f" % drv.start_reply) + " after %d ms" % (dt * 1000))

    # 11. Whole run
    res.check(not drv.errors, "No CMD_ERROR frames during the test", "%d received" % len(drv.errors))
    res.check(not drv.drv_fault_seen, "No DRV8316 fault flag during the test", "seen" if drv.drv_fault_seen else "none")
    res.check(drv.bus.adapter_errors == 0, "No CAN bus errors reported by the adapter", "%d reported" % drv.bus.adapter_errors)

    drv.run(0.3, 0.0)
    return res.summary()


def ensure_running(drv):
    drv.reset_window()
    drv.run(0.3, mode="none")
    if drv.last_state == STATE_RUNNING:
        return True
    if drv.last_state is None:
        error("No status from the driver on 0x%03X." % drv.tele_id)
        return False
    return start_or_fail(drv, Results())


def cmd_run(drv, args):
    if not ensure_running(drv):
        return False
    info("Sending %.3f A every %d ms. Ctrl+C to stop." % (args.current, args.interval))
    while True:
        drv.reset_window()
        drv.run(1.0, args.current)
        info("vel %+7.2f rad/s (%3d frames/s)  state %s  flags %s" %
             (drv.vel_avg(), drv.vel_count, state_name(drv.last_state), flag_names(drv.last_flags)))


def cmd_monitor(bus, args):
    info("Monitoring. Ctrl+C to stop.")
    counts = {}
    last_print = time.monotonic()
    while True:
        frame = bus.recv(0.05)
        if frame is not None:
            can_id, data = frame
            text = decode(can_id, data)
            # Velocity is ~100 Hz: summarise it unless --verbose.
            if text and (args.verbose or "velocity" not in text):
                info("%10.3f  0x%03X  %-24s %s" % (time.monotonic(), can_id, data.hex(" "), text))
            elif text:
                counts[can_id] = counts.get(can_id, 0) + 1
            elif args.verbose:
                info("%10.3f  0x%03X  %s" % (time.monotonic(), can_id, data.hex(" ")))
        now = time.monotonic()
        if counts and now - last_print >= 5.0:
            info("  (velocity frames in the last %.0f s: %s)" %
                 (now - last_print, ", ".join("0x%03X: %d" % kv for kv in sorted(counts.items()))))
            counts.clear()
            last_print = now


def cmd_info(drv, args):
    drv.run(0.3, mode="none")
    if drv.last_status_time is None:
        error("No status from node %d on 0x%03X." % (drv.node, drv.tele_id))
        return False
    mv = drv.last_mv
    info("Node %d: state %s, flags %s, supply %s%s" % (
        drv.node, state_name(drv.last_state), flag_names(drv.last_flags),
        "%.2f V" % (mv / 1000) if mv else "no reading",
        ", input " + fmt_input(mv, drv.last_ma) if drv.last_ma is not None else ""))
    reply = drv.request([CMD_GET_INFO])
    if reply is None:
        warn("No CMD_GET_INFO reply: firmware 1.0 (no configuration over CAN).")
        return True
    info("  " + decode_reply(reply))
    board = drv.read_board_info()
    if board:
        info("  board: %s (hardware rev %d), capabilities: %s" % (
            board["model"] or "?", board["rev"], cap_names(board["caps"])))
    else:
        info("  board: %s" % drv.board_label())
    for which in (0, 1):
        r = drv.request([CMD_GET_CONFIG, which])
        info("  " + (decode_reply(r) if r else "no GET_CONFIG reply"))
    return True


def cmd_config(drv, args):
    drv.run(0.3, mode="none")
    if drv.last_status_time is None:
        error("No status from node %d on 0x%03X." % (drv.node, drv.tele_id))
        return False
    ops = []
    if args.set_node is not None:
        if not NODE_MIN <= args.set_node <= NODE_MAX:
            error("--set-node must be %d..%d" % (NODE_MIN, NODE_MAX))
            return False
        if args.set_node != drv.node:
            other = Driver(drv.bus, args.set_node, args.interval, False)
            drv.observers.append(other._process)
            drv.run(0.3, mode="none")
            drv.observers.remove(other._process)
            if other.last_status_time is not None:
                error("Node %d is already on the bus: two boards with one node ID can't be told apart." % args.set_node)
                return False
        ops.append(set_param_payload(PARAM_NODE_ID, args.set_node))
    if args.set_invert is not None:
        ops.append(set_param_payload(PARAM_INVERT, 1 if args.set_invert == "on" else 0))
    if args.velocity_interval is not None:
        ops.append(set_param_payload(PARAM_VELOCITY_INTERVAL_MS, args.velocity_interval))
    if args.cal:
        ops.append([CMD_CALIBRATION, CAL_STORE if args.cal == "store" else CAL_CLEAR])
    if args.revert:
        ops.append([CMD_REVERT])
    if args.save:
        ops.append([CMD_SAVE])
    elif args.reboot:
        ops.append([CMD_REBOOT])
    if not ops:
        warn("Nothing to change. Options: --set-node N, --set-invert on|off, --velocity-interval MS, "
             "--cal store|clear, --revert, --save, --reboot")
    ok = drv.configure(ops)
    if drv.read_config():
        for which in (0, 1):
            c = drv.config.get(which)
            if c:
                info("  %s: node %s, invert %s, alignment %s%s" % (
                    "pending" if which else "active ", c["node"] or "not set", "on" if c["invert"] else "off",
                    ("stored (%s, zero angle %.4f)" % ("CCW" if c["sensor_ccw"] else "CW", c["zero_angle"]))
                    if c["aligned"] else "none (aligns at Start FOC)", "   UNSAVED CHANGES" if c["unsaved"] and which else ""))
        if drv.config.get(1, {}).get("unsaved") and not args.save:
            warn("Pending changes are not saved yet: add --save (stops the motor and reboots the board).")
    return ok


def cmd_busoff(drv, args):
    res = Results()
    if not ensure_running(drv):
        return False
    I = args.current if abs(args.current) >= 0.05 else 0.3
    drv.run(1.5, I)
    emit("Now UNPLUG the CAN cable for 2-5 s, then plug it back in. (waiting up to 60 s)", "prompt")

    # Wait for the status heartbeat to stop...
    lost = drv.run(60.0, I, until=lambda: time.monotonic() - (drv.last_status_time or time.monotonic()) > 0.5)
    if not res.check(lost, "Cable unplugged (heartbeat stopped)", "detected" if lost else "no gap within 60 s"):
        return res.summary()
    t_lost = time.monotonic()

    # ...and come back.
    drv.reset_window()
    back = drv.run(60.0, I, until=lambda: drv.status_count > 0)
    if not res.check(back, "Driver back on the bus", "after %.1f s" % (time.monotonic() - t_lost) if back
                     else "nothing within 60 s - did it reset into DFU? (CAN_RX is BOOT0)"):
        return res.summary()

    drv.reset_window()
    drv.run(1.0, I)
    res.check(drv.last_state == STATE_RUNNING, "Still running (no reset)", state_name(drv.last_state))
    res.check(not drv.last_flags & FLAG_TIMEOUT, "Commands accepted again", flag_names(drv.last_flags))
    res.check(drv.vel_count >= 80, "Velocity telemetry resumed", "%d frames in 1 s" % drv.vel_count)
    info("Check the serial log: it should show 'No current command for 100 ms - applying 0 A' while unplugged,")
    info("and 'CAN bus-off - attempting recovery' / 'CAN bus recovered' if the driver went bus-off.")
    drv.run(0.3, 0.0)
    return res.summary()


# ----------------------------------------------------------------------------

def add_bus_args(ap, here):
    g = ap.add_argument_group("adapter")
    g.add_argument("--backend", choices=["elmue", "python-can"], default="elmue")
    g.add_argument("--device", type=int, default=0, help="adapter index from `list` (elmue backend)")
    g.add_argument("--interface", default="slcan", help="python-can interface (default slcan)")
    g.add_argument("--channel", help="python-can channel, e.g. COM5 or can0")
    b = ap.add_argument_group("bit timing (the firmware uses 1M @ 87.5 %, classic)")
    b.add_argument("--bitrate", type=parse_bitrate, default=DRIVER_BITRATE, help="nominal bitrate, e.g. 1M, 500k")
    b.add_argument("--sample-point", type=float, default=DRIVER_SAMPLE_POINT,
                   help="nominal sample point %% (default 87.5)")
    b.add_argument("--fd", action="store_true", help="enable CAN FD (data phase bitrate)")
    b.add_argument("--data-bitrate", type=parse_bitrate, default=2_000_000, help="CAN FD data bitrate (default 2M)")
    b.add_argument("--data-sample-point", type=float, default=75.0, help="CAN FD data sample point %% (default 75)")


def settings_from_args(args):
    return CanSettings(args.bitrate, args.sample_point, args.fd, args.data_bitrate, args.data_sample_point)


def open_bus(args, settings, listen_only=False):
    if args.backend == "elmue":
        return ElmueBus(args.device, settings, listen_only)
    if not args.channel:
        raise SystemExit("--channel is required with --backend python-can")
    return PyCanBus(args.interface, args.channel, settings, listen_only)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["auto", "monitor", "run", "busoff", "info", "config", "list"])
    ap.add_argument("--node", type=int, help="CAN node ID of the driver, 1-15 (default 1)")
    ap.add_argument("--side", choices=["left", "right"], help="shorthand for --node 1 (left) / 2 (right)")
    ap.add_argument("--current", type=float, default=0.3, help="current in A for run/auto/busoff (default 0.3)")
    ap.add_argument("--interval", type=int, default=10, help="command interval in ms, 1..90 (default 10)")
    ap.add_argument("-v", "--verbose", action="store_true", help="print every decoded frame")
    c = ap.add_argument_group("config command")
    c.add_argument("--set-node", type=int, help="new node ID 1-15 (pending until --save)")
    c.add_argument("--set-invert", choices=["on", "off"], help="flip current/velocity sign (pending until --save)")
    c.add_argument("--velocity-interval", type=int, help="velocity telemetry period 1-100 ms (runtime, not saved)")
    c.add_argument("--cal", choices=["store", "clear"], help="store this boot's alignment / clear it (pending)")
    c.add_argument("--revert", action="store_true", help="discard unsaved changes")
    c.add_argument("--save", action="store_true", help="save pending settings (stops the motor, reboots)")
    c.add_argument("--reboot", action="store_true", help="reboot the board (stops the motor)")
    ap.add_argument("--listen-only", action="store_true",
                    help="monitor: open the adapter in listen-only (silent) mode, no ACKs")
    add_bus_args(ap, here)
    args = ap.parse_args()

    if args.node is not None and args.side is not None:
        ap.error("use --node or --side, not both")
    node = args.node if args.node is not None else SIDE_NODES.get(args.side, 1)
    if not NODE_MIN <= node <= NODE_MAX:
        ap.error("--node must be %d..%d" % (NODE_MIN, NODE_MAX))
    if not 1 <= args.interval <= 90:
        ap.error("--interval must be 1..90 ms (the driver's command timeout is 100 ms)")
    if abs(args.current) > 5:
        ap.error("--current must be within +-5 A (the firmware clamps to 2 A)")

    if args.command == "list":
        if args.backend == "elmue":
            devs = list_elmue_devices()
            info("Candlelight 2.5 adapters (use --device N):" if devs else "No Candlelight 2.5 adapter found.")
            for i, d in enumerate(devs):
                info("  %d: %s" % (i, d))
        else:
            chans = list_pycan_channels(args.interface)
            info("python-can '%s' channels: %s" % (args.interface, ", ".join(chans) if chans else "none found"))
        return

    settings = settings_from_args(args)
    settings.warn_if_not_driver()
    if args.listen_only and args.command != "monitor":
        ap.error("--listen-only only works with monitor (the other commands have to send)")
    bus = open_bus(args, settings, args.listen_only)

    drv = Driver(bus, node, args.interval, args.verbose)
    info("Node %d: current frame 0x%03X bytes %d-%d, commands 0x%03X, telemetry 0x%03X"
         % (node, drv.cur_id, drv.offset, drv.offset + 3, drv.cmd_id, drv.tele_id))
    ok = True
    try:
        if args.command == "auto":
            ok = cmd_auto(drv, args)
        elif args.command == "run":
            ok = cmd_run(drv, args)
        elif args.command == "busoff":
            ok = cmd_busoff(drv, args)
        elif args.command == "info":
            ok = cmd_info(drv, args)
        elif args.command == "config":
            ok = cmd_config(drv, args)
        else:
            cmd_monitor(bus, args)
    except KeyboardInterrupt:
        info("Stopped.")
    finally:
        if args.command not in ("monitor", "info", "config"):
            try:
                drv.send_current(0.0)  # leave the motor at 0 A
            except Exception:
                pass
        bus.close()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
