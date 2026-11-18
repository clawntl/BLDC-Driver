#!/usr/bin/env python3
"""
CAN test GUI for the BLDC-Driver firmware (tkinter, no extra packages).
See docs/can-tools.md.

Same adapters as cantest.py (Candlelight 2.5 via the bundled
ElmuSoft classes in ./elmue, or python-can). Run:  python cantest_gui.py

Two drivers are shown side by side, each by CAN node ID (firmware 1.2+,
protocol 3; default node 1 = left wheel, node 2 = right wheel). Two modes,
picked before connecting:

  Control  The GUI is the controller (unplug the robot's controller). Both
           motors can be driven at the same time: per side Start FOC, "Send"
           and a current slider, or "Start both" / "Link sliders". Nodes 1/2
           share current frame 0x010, so both currents go out in one frame;
           a side with Send off gets NaN (= no command, the driver times out
           to 0 A). STOP (or Esc) sends 0 A to both, stops sending and aborts
           a running test. Auto test / Bus-off test run per side, the same
           sequences as the CLI. "Info" reads the firmware version and the board
           (model name; input current and power are shown if the board
           measures the current).
           "Settings..." changes the board's settings over CAN (node ID,
           invert, alignment, save/revert/reboot; firmware 1.1+).
  Monitor  The robot's controller stays on the bus and in charge; the GUI
           sends nothing. Shows what the controller commands per node
           (currents, gaps longer than the 100 ms timeout, START_FOC and
           configuration commands) next to each driver's state, flags,
           supply voltage and velocity. "Listen-only" (elmue) keeps the
           adapter completely silent (no ACKs).

The adapter and bit timing: Refresh lists the connected adapters; the firmware
uses 1 Mbit/s, 87.5 %, classic CAN (the defaults).

SAFETY: the motors WILL spin. Clamp them or lift the wheels.
"""

import collections
import os
import queue
import sys
import threading
import time
import tkinter as tk
import types
from tkinter import messagebox, scrolledtext, ttk

sys.dont_write_bytecode = True
import cantest as core  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PLOT_SECONDS = 10.0
UI_PERIOD_MS = 50
FRESH_S = 0.5          # status older than this = driver offline

SIDES = ("left", "right")
SIDE_COLORS = {"left": "#268bd2", "right": "#cb4b16"}
SIDE_TINTS = {"left": "#cfe3f5", "right": "#f5d8cc"}   # slider troughs

BITRATES = ["10k", "20k", "50k", "83.333k", "100k", "125k", "250k", "500k", "800k", "1M"]
DATA_BITRATES = ["1M", "2M", "4M", "5M", "8M"]
SAMPLE_POINTS = ["60", "66.7", "70", "75", "80", "87.5"]
PYCAN_INTERFACES = ["slcan", "socketcan", "gs_usb", "candle", "pcan", "kvaser", "vector", "ixxat", "neovi",
                    "serial", "seeedstudio", "virtual"]


def other_side(side):
    return "right" if side == "left" else "left"


# ----------------------------------------------------------------------------
# CAN worker thread: owns the bus, runs the command loop and the tests
# ----------------------------------------------------------------------------

class Worker(threading.Thread):
    def __init__(self, bus, interval_ms, monitor, nodes):
        super().__init__(daemon=True)
        self.bus = bus
        self.monitor = monitor
        self.interval_ms = interval_ms
        self.drv = {s: core.Driver(bus, nodes[s], interval_ms, verbose=False) for s in SIDES}
        self.watch = core.CommandWatch()
        # Every frame received by either Driver.run() reaches both drivers,
        # the controller watch and the event logger.
        for s in SIDES:
            d = self.drv[s]
            d.should_abort = self._should_abort
            d.observers = [self.drv[other_side(s)]._process, self.watch.process, self._log_events]
        self._last_seen = {s: (None, None) for s in SIDES}   # (state, flags) for change logging
        self._board_info = core.BoardInfoJoiner()            # name pages -> one log line

        self.tasks = queue.Queue()
        self.stop_flag = False
        self.abort_flag = False
        self.busy = None            # description of the running task, or None
        self.send_enabled = {s: False for s in SIDES}
        self.current = {s: 0.0 for s in SIDES}
        self.zero_now = False       # STOP pressed: send one 0 A frame right away

    def _should_abort(self):
        return self.stop_flag or (self.busy is not None and self.abort_flag)

    def submit(self, task, side=None, arg=None):
        self.tasks.put((task, side, arg))

    def sending(self):
        return not self.monitor and any(self.send_enabled.values())

    def _send_currents(self, zero=False):
        """One frame per current-frame ID in use (one for nodes 1+2). A side
        with Send off gets NaN = no command; `zero` sends 0 A to both."""
        frames = {}
        for s in SIDES:
            d = self.drv[s]
            v = 0.0 if zero else (self.current[s] if self.send_enabled[s] else core.NAN)
            frames.setdefault(d.cur_id, [core.NAN, core.NAN])[d.offset // 4] = v
        for can_id, (a, b) in frames.items():
            self.drv["left"]._send(can_id, core.struct.pack(">ff", a, b))

    # --- event log (runs in this thread, emit() is queued to the GUI) ---

    def _log_events(self, can_id, data):
        for s in SIDES:
            d = self.drv[s]
            if can_id != d.tele_id or not data:
                continue
            if data[0] == core.CMD_DRIVER_STATUS and len(data) >= 3:
                now = (data[1], data[2])
                prev = self._last_seen[s]
                if now != prev:
                    self._last_seen[s] = now
                    if prev[0] != now[0]:
                        kind = "error" if now[0] == core.STATE_FAULT else "info"
                        core.emit("%s: state %s, flags %s" % (s, core.state_name(now[0]), core.flag_names(now[1])),
                                  kind)
                    else:
                        changed = now[1] ^ prev[1]
                        kind = "warn" if (changed & now[1] & (core.FLAG_TIMEOUT | core.FLAG_DRV_FAULT)) else "dim"
                        core.emit("%s: flags %s" % (s, core.flag_names(now[1])), kind)
            elif data[0] == core.CMD_START_FOC and len(data) >= 5:
                ok = core.be_float(data, 1) == 1.0
                core.emit("%s: START_FOC reply %s" % (s, "OK" if ok else "FAILED"), "pass" if ok else "fail")
            elif data[0] == core.CMD_GET_BOARD_INFO:
                text = self._board_info.feed(s, data)
                if text:
                    core.emit("%s (node %d): %s" % (s, d.node, text), "dim")
            else:
                text = core.decode_reply(data)
                if text:
                    core.emit("%s (node %d): %s" % (s, d.node, text), "dim")

    # --- main loop ---

    def run(self):
        lead = self.drv["left"]   # any Driver works: both sides are in each frame and observed
        try:
            while not self.stop_flag:
                try:
                    task, side, arg = self.tasks.get_nowait()
                except queue.Empty:
                    task = None

                if task is None:
                    if self.monitor:
                        lead.run(0.05, mode="none")
                        continue
                    if self.zero_now:
                        self.zero_now = False
                        self._send_currents(zero=True)
                    lead.run(0.05, mode="normal" if self.sending() else "none", tx=self._send_currents)
                    continue

                self.busy = task + ((" " + side) if side else "")
                self.abort_flag = False
                try:
                    self._run_task(task, side, arg)
                except core.Aborted:
                    if not self.stop_flag:
                        core.emit("Aborted.", "warn")
                        self._send_currents(zero=True)
                except Exception as ex:
                    core.emit("Error in %s: %s" % (self.busy, ex), "error")
                finally:
                    self.busy = None
                    self.abort_flag = False
        except core.Aborted:
            pass
        except Exception as ex:
            core.emit("CAN connection stopped: %s" % ex, "error")
        finally:
            if not self.monitor:
                try:
                    self._send_currents(zero=True)
                except Exception:
                    pass
            self.bus.close()
            core.emit("Disconnected.", "dim")

    def _run_task(self, task, side, arg=None):
        if task == "start_both":
            self._start_both()
            return
        if task == "info_all":
            for s in SIDES:
                self._info(s, quiet_if_absent=True)
            return
        drv = self.drv[side]
        if task == "info":
            self._info(side)
            return
        if task == "read_config":
            drv.read_config()
            return
        if task == "config":
            core.emit("==== Settings %s (node %d) ====" % (side, drv.node), "prompt")
            if any(op[0] in (core.CMD_SAVE, core.CMD_REBOOT) for op in arg):
                self.send_enabled = {s: False for s in SIDES}   # the board stops its motor anyway
            drv.configure(arg)
            drv.read_config()
            return
        cur = self.current[side]
        args = types.SimpleNamespace(current=cur if abs(cur) >= 0.05 else 0.3,
                                     interval=self.interval_ms, verbose=False)
        if task == "start":
            core.emit("==== Start FOC %s ====" % side, "prompt")
            core.start_or_fail(drv, core.Results(), "Start FOC %s" % side)
        elif task == "auto":
            self.send_enabled = {s: False for s in SIDES}
            core.emit("==== Auto test %s (%.2f A) ====" % (side, abs(args.current)), "prompt")
            core.cmd_auto(drv, args)
        elif task == "busoff":
            self.send_enabled = {s: False for s in SIDES}
            core.emit("==== Bus-off test %s (%.2f A) ====" % (side, args.current), "prompt")
            core.cmd_busoff(drv, args)

    def _info(self, side, quiet_if_absent=False):
        d = self.drv[side]
        d.run(0.25, mode="none")
        if d.last_status_time is None:
            if not quiet_if_absent:
                core.emit("%s (node %d): no status received" % (side, d.node), "warn")
            return
        if d.request([core.CMD_GET_INFO]) is None:
            core.emit("%s (node %d): no GET_INFO reply - firmware 1.0 (no configuration over CAN)" % (side, d.node),
                      "warn")
            return
        d.read_board_info()
        r = d.request([core.CMD_GET_CONFIG, 0])
        if r is not None and r[3] & core.CONFIG_FLAG_UNSAVED:
            core.emit("%s (node %d): has unsaved setting changes" % (side, d.node), "warn")

    def _start_both(self):
        """START_FOC to both drivers at once; they align in parallel."""
        core.emit("==== Start FOC both ====", "prompt")
        res = core.Results()
        lead = self.drv["left"]
        lead.run(0.3, mode="none")
        targets = []
        for s in SIDES:
            d = self.drv[s]
            if d.last_status_time is None:
                core.emit("%s: no status received, not starting it" % s, "warn")
                continue
            d.start_reply = None
            d.send_start()
            targets.append(s)
        if not targets:
            res.check(False, "Start FOC both", "no driver online")
            return
        core.emit("Waiting up to %.0f s for %s (the drivers send nothing while they align)..."
                  % (core.START_TIMEOUT_S, " and ".join(targets)))
        t0 = time.monotonic()
        lead.run(core.START_TIMEOUT_S, mode="none",
                 until=lambda: all(self.drv[s].start_reply is not None for s in targets))
        dt = time.monotonic() - t0
        for s in targets:
            r = self.drv[s].start_reply
            res.check(r == 1.0, "Start FOC %s" % s,
                      "no reply within %.0f s" % core.START_TIMEOUT_S if r is None
                      else "reply %s (%.1f s)" % ("OK" if r == 1.0 else "FAILED", dt))


# ----------------------------------------------------------------------------
# One side's status + controls
# ----------------------------------------------------------------------------

class SidePanel:
    def __init__(self, app, parent, side):
        self.app = app
        self.side = side
        f = ttk.LabelFrame(parent, text="%s motor" % side.capitalize())
        self.frame = f

        ids = ttk.Frame(f)
        ids.pack(fill="x", padx=4, pady=(4, 0))
        ttk.Label(ids, text="Node").pack(side="left")
        self.node_var = tk.IntVar(value=core.SIDE_NODES[side])
        self.sp_node = ttk.Spinbox(ids, from_=core.NODE_MIN, to=core.NODE_MAX, textvariable=self.node_var, width=4,
                                   command=self.update_ids)
        self.sp_node.pack(side="left", padx=4)
        self.sp_node.bind("<KeyRelease>", lambda e: self.update_ids())
        self.lbl_ids = tk.Label(ids, text="", anchor="w", fg="#666666")
        self.lbl_ids.pack(side="left", padx=4)
        self.lbl_fw = tk.Label(ids, text="", anchor="e", fg="#666666")
        self.lbl_fw.pack(side="right", padx=4)
        self.update_ids()

        top = ttk.Frame(f)
        top.pack(fill="x", padx=4, pady=(4, 0))
        tk.Label(top, text="■", fg=SIDE_COLORS[side], font=("Segoe UI", 14)).pack(side="left")
        self.lbl_link = tk.Label(top, text="not connected", width=13, relief="groove")
        self.lbl_link.pack(side="left", padx=4)
        self.lbl_state = tk.Label(top, text="state: -", width=22, anchor="w")
        self.lbl_state.pack(side="left", padx=4)
        self.lbl_vel = tk.Label(top, text="- rad/s", width=13, font=("Segoe UI", 16, "bold"), anchor="e")
        self.lbl_vel.pack(side="right", padx=4)

        fl = ttk.Frame(f)
        fl.pack(fill="x", padx=4, pady=2)
        self.flag_labels = {}
        for bit, name in ((core.FLAG_TIMEOUT, "cmd timeout"), (core.FLAG_CALIBRATION, "stored align"),
                          (core.FLAG_INVERTED, "inverted"), (core.FLAG_DRV_FAULT, "DRV fault")):
            lbl = tk.Label(fl, text=name, width=11, relief="groove")
            lbl.pack(side="left", padx=2)
            self.flag_labels[bit] = lbl
        # Supply readout, sized to its text (packed last, so the flags keep
        # their space when the window is narrow).
        self.lbl_volt = tk.Label(fl, text="- V", font=("Segoe UI", 11), anchor="e")
        self.lbl_volt.pack(side="right", padx=4)

        self.lbl_cmd = tk.Label(f, text="commanded: -", anchor="w", font=("Segoe UI", 10, "bold"))
        self.lbl_cmd.pack(fill="x", padx=4)
        self.lbl_counts = tk.Label(f, text="", anchor="w", fg="#666666")
        self.lbl_counts.pack(fill="x", padx=4)

        c = ttk.Frame(f)
        c.pack(fill="x", padx=4, pady=(4, 0))
        self.btn_start = ttk.Button(c, text="Start FOC", command=lambda: app.submit("start", side))
        self.btn_start.pack(side="left", padx=2)
        self.send_var = tk.BooleanVar(value=False)
        self.chk_send = ttk.Checkbutton(c, text="Send current", variable=self.send_var,
                                        command=lambda: app.on_send_toggled(side))
        self.chk_send.pack(side="left", padx=8)
        self.current_var = tk.DoubleVar(value=0.0)
        self.scale = tk.Scale(c, from_=-2.0, to=2.0, resolution=0.01, orient="horizontal", length=260,
                              variable=self.current_var, command=lambda v: app.on_current_changed(side),
                              label="Current (A)", troughcolor=SIDE_TINTS[side])
        self.scale.pack(side="left", padx=4, fill="x", expand=True)

        q = ttk.Frame(f)
        q.pack(fill="x", padx=4, pady=2)
        self.quick_buttons = []
        for a in (-0.5, -0.3, 0.0, 0.3, 0.5):
            b = ttk.Button(q, text="%+.1f" % a if a else "0", width=5,
                           command=lambda a=a: app.set_current(side, a))
            b.pack(side="left", padx=1)
            self.quick_buttons.append(b)
        self.btn_auto = ttk.Button(q, text="Auto test", command=lambda: app.submit("auto", side))
        self.btn_auto.pack(side="right", padx=2)
        self.btn_busoff = ttk.Button(q, text="Bus-off test", command=lambda: app.submit("busoff", side))
        self.btn_busoff.pack(side="right", padx=2)
        self.btn_info = ttk.Button(q, text="Info", command=lambda: app.submit("info", side))
        self.btn_info.pack(side="right", padx=2)
        self.btn_settings = ttk.Button(q, text="Settings...", command=lambda: app.open_settings(side))
        self.btn_settings.pack(side="right", padx=2)

        self.controls = [self.btn_start, self.chk_send, self.scale, self.btn_auto, self.btn_busoff,
                         self.btn_info, self.btn_settings] + self.quick_buttons

    def node(self):
        """The node ID in the spinbox, or None if it isn't a valid one."""
        try:
            n = int(self.node_var.get())
        except (tk.TclError, ValueError):
            return None
        return n if core.NODE_MIN <= n <= core.NODE_MAX else None

    def update_ids(self):
        n = self.node()
        if n is None:
            self.lbl_ids.configure(text="(node must be %d-%d)" % (core.NODE_MIN, core.NODE_MAX))
            return
        cur_id, off, cmd_id, tele_id = core.node_ids(n)
        self.lbl_ids.configure(text="currents 0x%03X [%d-%d], commands 0x%03X, telemetry 0x%03X"
                                    % (cur_id, off, off + 3, cmd_id, tele_id))


# ----------------------------------------------------------------------------
# Settings window (configuration over CAN, firmware 1.1+ / protocol 2+)
# ----------------------------------------------------------------------------

class SettingsDialog:
    """Reads and changes one board's settings. Node ID, invert and the
    alignment change the PENDING settings; "Save & reboot" makes them active
    (the board stops its motor, writes flash and resets)."""

    def __init__(self, app, side):
        self.app = app
        self.side = side
        self.win = tk.Toplevel(app.root)
        self.win.resizable(False, False)
        self.win.protocol("WM_DELETE_WINDOW", self.close)
        self.win.bind("<Escape>", lambda e: app.stop_motor())
        f = ttk.Frame(self.win, padding=8)
        f.pack(fill="both", expand=True)

        self.lbl_title = tk.Label(f, text="", font=("Segoe UI", 11, "bold"), anchor="w")
        self.lbl_title.grid(row=0, column=0, columnspan=4, sticky="we")
        self.lbl_fw = tk.Label(f, text="", anchor="w", fg="#666666")
        self.lbl_fw.grid(row=1, column=0, columnspan=4, sticky="we")

        g = ttk.LabelFrame(f, text="Current settings")
        g.grid(row=2, column=0, columnspan=4, sticky="we", pady=(6, 4))
        self.lbl_active = tk.Label(g, text="active:  -", anchor="w", font=("Consolas", 9))
        self.lbl_active.pack(fill="x", padx=4)
        self.lbl_pending = tk.Label(g, text="pending: -", anchor="w", font=("Consolas", 9))
        self.lbl_pending.pack(fill="x", padx=4)

        e = ttk.LabelFrame(f, text="Change (pending until Save & reboot)")
        e.grid(row=3, column=0, columnspan=4, sticky="we", pady=4)
        ttk.Label(e, text="Node ID").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        self.node_var = tk.IntVar(value=core.SIDE_NODES[side])
        ttk.Spinbox(e, from_=core.NODE_MIN, to=core.NODE_MAX, textvariable=self.node_var, width=5).grid(
            row=0, column=1, sticky="w", padx=4)
        self.btn_node = ttk.Button(e, text="Set node", command=self.set_node)
        self.btn_node.grid(row=0, column=2, padx=4)
        ttk.Label(e, text="changes the board's CAN IDs after the reboot", foreground="#666666").grid(
            row=0, column=3, sticky="w", padx=4)

        ttk.Label(e, text="Invert").grid(row=1, column=0, sticky="w", padx=4, pady=2)
        self.invert_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(e, text="on", variable=self.invert_var).grid(row=1, column=1, sticky="w", padx=4)
        self.btn_invert = ttk.Button(e, text="Set invert", command=self.set_invert)
        self.btn_invert.grid(row=1, column=2, padx=4)
        ttk.Label(e, text="flips the sign of the current command and the reported velocity",
                  foreground="#666666").grid(row=1, column=3, sticky="w", padx=4)

        ttk.Label(e, text="Alignment").grid(row=2, column=0, sticky="w", padx=4, pady=2)
        self.btn_cal_store = ttk.Button(e, text="Store this boot's",
                                        command=lambda: self.run([[core.CMD_CALIBRATION, core.CAL_STORE]]))
        self.btn_cal_store.grid(row=2, column=1, padx=4, sticky="w")
        self.btn_cal_clear = ttk.Button(e, text="Clear",
                                        command=lambda: self.run([[core.CMD_CALIBRATION, core.CAL_CLEAR]]))
        self.btn_cal_clear.grid(row=2, column=2, padx=4)
        ttk.Label(e, text="store needs FOC running; with none stored, Start FOC aligns (wheel moves)",
                  foreground="#666666").grid(row=2, column=3, sticky="w", padx=4)

        r = ttk.LabelFrame(f, text="Runtime (applies at once, back to default after a reset)")
        r.grid(row=4, column=0, columnspan=4, sticky="we", pady=4)
        ttk.Label(r, text="Velocity telemetry (ms)").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        self.vel_var = tk.IntVar(value=10)
        ttk.Spinbox(r, from_=1, to=100, textvariable=self.vel_var, width=5).grid(row=0, column=1, sticky="w", padx=4)
        self.btn_vel = ttk.Button(r, text="Set", command=self.set_velocity_interval)
        self.btn_vel.grid(row=0, column=2, padx=4)

        b = ttk.Frame(f)
        b.grid(row=5, column=0, columnspan=4, sticky="we", pady=(6, 0))
        self.btn_save = ttk.Button(b, text="Save & reboot", command=self.save)
        self.btn_save.pack(side="left", padx=2)
        self.btn_revert = ttk.Button(b, text="Revert", command=lambda: self.run([[core.CMD_REVERT]]))
        self.btn_revert.pack(side="left", padx=2)
        self.btn_reboot = ttk.Button(b, text="Reboot", command=self.reboot)
        self.btn_reboot.pack(side="left", padx=2)
        self.btn_realign = ttk.Button(b, text="Re-run alignment...", command=self.realign)
        self.btn_realign.pack(side="left", padx=(12, 2))
        self.btn_refresh = ttk.Button(b, text="Refresh", command=self.refresh)
        self.btn_refresh.pack(side="right", padx=2)
        self.buttons = [self.btn_node, self.btn_invert, self.btn_cal_store, self.btn_cal_clear, self.btn_vel,
                        self.btn_save, self.btn_revert, self.btn_reboot, self.btn_realign, self.btn_refresh]

        self._loaded_for = None     # node whose settings filled the edit fields
        self.refresh()

    def drv(self):
        w = self.app.worker
        return w.drv[self.side] if w else None

    def run(self, ops):
        self.app.submit("config", self.side, [bytes(op) for op in ops])

    def refresh(self):
        self.app.submit("read_config", self.side)

    def set_node(self):
        try:
            n = int(self.node_var.get())
        except (tk.TclError, ValueError):
            n = 0
        d = self.drv()
        if d is None:
            return
        if not core.NODE_MIN <= n <= core.NODE_MAX:
            messagebox.showerror("CAN test", "Node ID must be %d..%d." % (core.NODE_MIN, core.NODE_MAX), parent=self.win)
            return
        if n != d.node and self.app.worker.watch.heard_recently(n):
            messagebox.showerror("CAN test", "Node %d is already on the bus. Two boards with the same node ID can't "
                                            "be told apart (only USB serial `set node` can fix that)." % n,
                                 parent=self.win)
            return
        self.run([core.set_param_payload(core.PARAM_NODE_ID, n)])

    def set_invert(self):
        self.run([core.set_param_payload(core.PARAM_INVERT, 1 if self.invert_var.get() else 0)])

    def set_velocity_interval(self):
        try:
            ms = int(self.vel_var.get())
        except (tk.TclError, ValueError):
            ms = 0
        if not 1 <= ms <= 100:
            messagebox.showerror("CAN test", "1..100 ms.", parent=self.win)
            return
        self.run([core.set_param_payload(core.PARAM_VELOCITY_INTERVAL_MS, ms)])

    def save(self):
        d = self.drv()
        pend = d.config.get(1) if d else None
        msg = "Save the pending settings to flash?\n\nThe board stops its motor and reboots."
        if pend and pend["node"] and pend["node"] != d.node:
            msg += ("\n\nIts node ID changes from %d to %d; this panel follows it to the new ID."
                    % (d.node, pend["node"]))
        if messagebox.askokcancel("CAN test", msg, parent=self.win):
            self.run([[core.CMD_SAVE]])

    def reboot(self):
        if messagebox.askokcancel("CAN test", "Reboot the board? Its motor stops. Unsaved changes are lost.",
                                  parent=self.win):
            self.run([[core.CMD_REBOOT]])

    def realign(self):
        d = self.drv()
        if d is None:
            return
        if d.last_state == core.STATE_RUNNING and not (d.config.get(0) or {}).get("aligned"):
            # Step 2: this boot aligned (nothing was stored) -> keep it.
            if messagebox.askokcancel("CAN test", "FOC is running with a fresh alignment (none was stored).\n\n"
                                                 "Store it and save? The board reboots and uses it from then on.",
                                      parent=self.win):
                self.run([[core.CMD_CALIBRATION, core.CAL_STORE], [core.CMD_SAVE]])
            return
        if messagebox.askokcancel(
                "CAN test", "Re-running the sensor alignment takes three steps:\n\n"
                           "1. Now: clear the stored alignment and save (the board reboots).\n"
                           "2. Press Start FOC: with nothing stored the board aligns (the wheel moves a little).\n"
                           "3. Press 'Re-run alignment...' again to store the new alignment and save.\n\n"
                           "Do step 1 now?", parent=self.win):
            self.run([[core.CMD_CALIBRATION, core.CAL_CLEAR], [core.CMD_SAVE]])

    def update(self):
        """Called from the App's UI tick."""
        d = self.drv()
        w = self.app.worker
        can_use = w is not None and not w.monitor and w.busy is None
        for b in self.buttons:
            self.app._set_enabled(b, can_use)
        if d is None:
            self.app._set_label(self.lbl_title, text="%s motor: not connected" % self.side.capitalize())
            return
        self.win.title("Settings - %s (node %d)" % (self.side, d.node))
        self.app._set_label(self.lbl_title, text="%s motor, node %d   (commands 0x%03X, telemetry 0x%03X)" % (
            self.side.capitalize(), d.node, d.cmd_id, d.tele_id))
        self.app._set_label(self.lbl_fw, text=(
            "%s, firmware %d.%d.%d, CAN protocol %d, image %d KB" % (
                d.board_label(), d.info[1], d.info[2], d.info[3], d.info[0], d.info[4])
            if d.info else "firmware: unknown (press Info / Refresh; no reply = firmware 1.0, use USB serial)"))
        for which, lbl, name in ((0, self.lbl_active, "active: "), (1, self.lbl_pending, "pending:")):
            c = d.config.get(which)
            if not c:
                self.app._set_label(lbl, text="%s -" % name, fg="black")
                continue
            text = "%s node %-3s invert %-4s alignment %s" % (
                name, c["node"] or "-", "on" if c["invert"] else "off",
                ("stored (%s, zero angle %.4f)" % ("CCW" if c["sensor_ccw"] else "CW", c["zero_angle"]))
                if c["aligned"] else "none (Start FOC aligns)")
            unsaved = which == 1 and c["unsaved"]
            self.app._set_label(lbl, text=text + ("   <- UNSAVED" if unsaved else ""),
                                fg="#b58900" if unsaved else "black")
        # Fill the edit fields once per node from its pending settings.
        pend = d.config.get(1)
        if pend and self._loaded_for != d.node:
            self._loaded_for = d.node
            self.node_var.set(pend["node"] or d.node)
            self.invert_var.set(pend["invert"])

    def close(self):
        self.app.settings.pop(self.side, None)
        self.win.destroy()


# ----------------------------------------------------------------------------
# GUI
# ----------------------------------------------------------------------------

class App:
    LOG_COLORS = {"info": None, "warn": "#b58900", "error": "#dc322f", "pass": "#2aa12a",
                  "fail": "#dc322f", "dim": "#888888", "prompt": "#268bd2"}

    def __init__(self, root):
        self.root = root
        self.worker = None
        self.log_queue = queue.Queue()
        self.vel_hist = {s: collections.deque() for s in SIDES}
        self.cur_hist = {s: collections.deque() for s in SIDES}
        self._widget_state = {}     # last state applied per widget (see _set_enabled)
        self._was_fresh = {s: False for s in SIDES}
        self._linking = False
        self._rate = {"t": 0.0, "count": 0, "fps": 0.0, "gap": 0.0}
        self._ctrl_was_active = False
        self._ctrl_warned = 0.0
        self._gaps = {}
        self.settings = {}          # side -> open SettingsDialog
        self.device_labels = []     # elmue adapters, index = --device
        core.set_output(lambda text, kind: self.log_queue.put((text, kind)))

        root.title("BLDC-Driver CAN Test")
        root.minsize(1100, 720)
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.bind("<Escape>", lambda e: self.stop_motor())

        self._build_connection()
        self._build_timing()
        self._build_drivers()
        self._build_plot_and_log()
        self._update_widgets()
        root.after(100, self.refresh_devices)
        root.after(UI_PERIOD_MS, self.tick)

    # --- layout ---

    def _build_connection(self):
        f = ttk.LabelFrame(self.root, text="Adapter")
        f.pack(fill="x", padx=8, pady=(8, 4))

        self.backend = tk.StringVar(value="elmue")
        self.device = tk.StringVar(value="")
        self.interface = tk.StringVar(value="slcan")
        self.channel = tk.StringVar(value="")
        self.mode = tk.StringVar(value="control")
        self.listen_only = tk.BooleanVar(value=True)
        self.interval = tk.IntVar(value=10)

        ttk.Label(f, text="Type").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        self.cb_backend = ttk.Combobox(f, textvariable=self.backend, state="readonly", width=24,
                                       values=["elmue", "python-can"])
        self.cb_backend.grid(row=0, column=1, padx=4, sticky="w")
        self.cb_backend.bind("<<ComboboxSelected>>", lambda e: (self._update_widgets(), self.refresh_devices()))
        ttk.Label(f, text="elmue = Candlelight 2.5 (ElmuSoft classes)   python-can = slcan, socketcan, PCAN, ...",
                  foreground="#666666").grid(row=0, column=2, columnspan=4, sticky="w", padx=4)

        # ElmuSoft row
        ttk.Label(f, text="Device").grid(row=1, column=0, sticky="w", padx=4, pady=2)
        self.cb_device = ttk.Combobox(f, textvariable=self.device, state="readonly", width=60)
        self.cb_device.grid(row=1, column=1, columnspan=3, padx=4, sticky="we")
        self.btn_refresh = ttk.Button(f, text="Refresh", command=self.refresh_devices)
        self.btn_refresh.grid(row=1, column=4, padx=4)

        # python-can row
        ttk.Label(f, text="Interface").grid(row=3, column=0, sticky="w", padx=4, pady=2)
        self.cb_iface = ttk.Combobox(f, textvariable=self.interface, values=PYCAN_INTERFACES, width=24)
        self.cb_iface.grid(row=3, column=1, padx=4, sticky="w")
        self.cb_iface.bind("<<ComboboxSelected>>", lambda e: self.refresh_devices())
        ttk.Label(f, text="Channel").grid(row=3, column=2, sticky="e", padx=4)
        self.cb_chan = ttk.Combobox(f, textvariable=self.channel, width=20)
        self.cb_chan.grid(row=3, column=3, padx=4, sticky="w")

        # Mode row
        ttk.Separator(f).grid(row=4, column=0, columnspan=6, sticky="we", pady=4)
        ttk.Label(f, text="Mode").grid(row=5, column=0, sticky="w", padx=4, pady=2)
        m = ttk.Frame(f)
        m.grid(row=5, column=1, columnspan=3, sticky="w", padx=4)
        self.rb_control = ttk.Radiobutton(m, text="Control (GUI drives both motors)", value="control",
                                          variable=self.mode, command=self._update_widgets)
        self.rb_control.pack(side="left")
        self.rb_monitor = ttk.Radiobutton(m, text="Monitor (controller on the bus, GUI sends nothing)",
                                          value="monitor", variable=self.mode, command=self._update_widgets)
        self.rb_monitor.pack(side="left", padx=(12, 0))
        self.chk_listen = ttk.Checkbutton(m, text="Listen-only (no ACKs)", variable=self.listen_only)
        self.chk_listen.pack(side="left", padx=(12, 0))

        ttk.Label(f, text="Command interval (ms)").grid(row=6, column=0, sticky="w", padx=4, pady=2)
        self.sp_interval = ttk.Spinbox(f, from_=1, to=90, textvariable=self.interval, width=6)
        self.sp_interval.grid(row=6, column=1, padx=4, sticky="w")
        self.btn_connect = ttk.Button(f, text="Connect", command=self.toggle_connect, width=14)
        self.btn_connect.grid(row=6, column=4, columnspan=2, padx=4, sticky="e")
        f.columnconfigure(3, weight=1)

    def _build_timing(self):
        f = ttk.LabelFrame(self.root, text="Bit timing  (firmware: 1 Mbit/s, 87.5 %, classic CAN)")
        f.pack(fill="x", padx=8, pady=4)

        self.bitrate = tk.StringVar(value="1M")
        self.sample_point = tk.StringVar(value="87.5")
        self.fd = tk.BooleanVar(value=False)
        self.data_bitrate = tk.StringVar(value="2M")
        self.data_sample_point = tk.StringVar(value="75")

        ttk.Label(f, text="Bitrate").grid(row=0, column=0, padx=4, pady=4)
        self.cb_bitrate = ttk.Combobox(f, textvariable=self.bitrate, values=BITRATES, width=9)
        self.cb_bitrate.grid(row=0, column=1, padx=4)
        ttk.Label(f, text="Sample point %").grid(row=0, column=2, padx=4)
        self.cb_sp = ttk.Combobox(f, textvariable=self.sample_point, values=SAMPLE_POINTS, width=6)
        self.cb_sp.grid(row=0, column=3, padx=4)

        self.chk_fd = ttk.Checkbutton(f, text="CAN FD", variable=self.fd, command=self._update_widgets)
        self.chk_fd.grid(row=0, column=4, padx=(16, 4))
        ttk.Label(f, text="Data bitrate").grid(row=0, column=5, padx=4)
        self.cb_dbitrate = ttk.Combobox(f, textvariable=self.data_bitrate, values=DATA_BITRATES, width=9)
        self.cb_dbitrate.grid(row=0, column=6, padx=4)
        ttk.Label(f, text="Data sample point %").grid(row=0, column=7, padx=4)
        self.cb_dsp = ttk.Combobox(f, textvariable=self.data_sample_point, values=SAMPLE_POINTS, width=6)
        self.cb_dsp.grid(row=0, column=8, padx=4)

        self.btn_defaults = ttk.Button(f, text="Firmware defaults", command=self.timing_defaults)
        self.btn_defaults.grid(row=0, column=9, padx=(16, 4))

        self.lbl_timing = ttk.Label(f, text="", foreground="#666666")
        self.lbl_timing.grid(row=1, column=0, columnspan=10, sticky="w", padx=4, pady=(0, 4))

    def _build_drivers(self):
        f = ttk.Frame(self.root)
        f.pack(fill="x", padx=8, pady=4)
        self.panels = {}
        for i, s in enumerate(SIDES):
            p = SidePanel(self, f, s)
            p.frame.grid(row=0, column=i, sticky="nsew", padx=(0, 4) if i == 0 else (4, 0))
            f.columnconfigure(i, weight=1, uniform="side")
            self.panels[s] = p

        g = ttk.LabelFrame(self.root, text="Both")
        g.pack(fill="x", padx=8, pady=4)
        self.btn_start_both = ttk.Button(g, text="Start FOC both", command=lambda: self.submit("start_both"))
        self.btn_start_both.pack(side="left", padx=4, pady=4)
        self.link_var = tk.BooleanVar(value=False)
        self.chk_link = ttk.Checkbutton(g, text="Link sliders + Send", variable=self.link_var,
                                        command=lambda: self._link_from("left"))
        self.chk_link.pack(side="left", padx=8)
        self.btn_abort = ttk.Button(g, text="Abort test", command=self.abort_task)
        self.btn_abort.pack(side="left", padx=4)
        self.btn_stop = tk.Button(g, text="STOP both  (Esc)", bg="#dc322f", fg="white",
                                  font=("Segoe UI", 11, "bold"), activebackground="#b02020",
                                  activeforeground="white", command=self.stop_motor, width=16)
        self.btn_stop.pack(side="right", padx=8, pady=4)
        self.lbl_bus = tk.Label(g, text="", anchor="w", fg="#666666")
        self.lbl_bus.pack(side="left", padx=8, fill="x", expand=True)

    def _build_plot_and_log(self):
        pw = ttk.PanedWindow(self.root, orient="vertical")
        pw.pack(fill="both", expand=True, padx=8, pady=(4, 8))

        pf = ttk.LabelFrame(pw, text="Shaft velocity and commanded current (last %d s)   "
                                     "blue = left, orange = right" % PLOT_SECONDS)
        self.canvas = tk.Canvas(pf, height=180, bg="white", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        pw.add(pf, weight=2)

        lf = ttk.LabelFrame(pw, text="Log")
        self.log = scrolledtext.ScrolledText(lf, height=8, font=("Consolas", 9), state="disabled")
        self.log.pack(fill="both", expand=True)
        for kind, colr in self.LOG_COLORS.items():
            if colr:
                self.log.tag_configure(kind, foreground=colr)
        self.log.tag_configure("fail", foreground=self.LOG_COLORS["fail"], font=("Consolas", 9, "bold"))
        ttk.Button(lf, text="Clear log", command=self._clear_log).pack(anchor="e")
        pw.add(lf, weight=1)

    # --- adapter selection ---

    def refresh_devices(self):
        if self.worker is not None:
            return
        if self.backend.get() == "elmue":
            try:
                self.device_labels = core.list_elmue_devices()
            except SystemExit as ex:
                self.device_labels = []
                core.emit(str(ex), "error")
            except Exception as ex:
                self.device_labels = []
                core.emit("Listing adapters failed: %s" % ex, "error")
            labels = ["%d: %s" % (i, d) for i, d in enumerate(self.device_labels)]
            self.cb_device.configure(values=labels)
            if labels:
                if self.device.get() not in labels:
                    self.device.set(labels[0])
            else:
                self.device.set("(no Candlelight 2.5 adapter found)")
            core.emit("Found %d Candlelight 2.5 adapter(s)." % len(labels), "dim")
        else:
            chans = core.list_pycan_channels(self.interface.get().strip())
            self.cb_chan.configure(values=chans)
            if chans and not self.channel.get():
                self.channel.set(chans[0])
            core.emit("python-can '%s': %s" % (self.interface.get(), ", ".join(chans) if chans else
                                               "no channels detected (type one in, e.g. COM5 or can0)"), "dim")

    def timing_defaults(self):
        self.bitrate.set("1M")
        self.sample_point.set("87.5")
        self.fd.set(False)
        self._update_widgets()

    def _read_settings(self):
        try:
            return core.CanSettings(core.parse_bitrate(self.bitrate.get()), float(self.sample_point.get()),
                                    self.fd.get(), core.parse_bitrate(self.data_bitrate.get()),
                                    float(self.data_sample_point.get()))
        except ValueError:
            raise ValueError("Bitrates must look like 1M, 500k or 1000000; sample points like 87.5")

    # --- actions ---

    def toggle_connect(self):
        if self.worker is not None:
            self.disconnect()
            return
        monitor = self.mode.get() == "monitor"
        try:
            interval = int(self.interval.get())
        except (tk.TclError, ValueError):
            interval = 0
        if not 1 <= interval <= 90:
            messagebox.showerror("CAN test", "Interval must be 1..90 ms (the driver's command timeout is 100 ms).")
            return
        try:
            settings = self._read_settings()
        except ValueError as ex:
            messagebox.showerror("CAN test", str(ex))
            return
        if not 50 <= settings.sample_point <= 95 or (settings.fd and not 50 <= settings.data_sample_point <= 95):
            messagebox.showerror("CAN test", "Sample points must be between 50 and 95 %.")
            return

        nodes = {s: self.panels[s].node() for s in SIDES}
        if None in nodes.values():
            messagebox.showerror("CAN test", "Node IDs must be %d..%d." % (core.NODE_MIN, core.NODE_MAX))
            return
        if nodes["left"] == nodes["right"]:
            messagebox.showerror("CAN test", "Left and right need different node IDs.")
            return

        listen_only = monitor and self.listen_only.get()
        core.emit("Connecting (%s mode%s): %s" % ("monitor" if monitor else "control",
                                                  ", listen-only" if listen_only else "", settings.describe()),
                  "prompt")
        settings.warn_if_not_driver()
        try:
            if self.backend.get() == "elmue":
                labels = self.cb_device.cget("values")
                if not labels or self.device.get() not in labels:
                    messagebox.showerror("CAN test", "No adapter selected. Plug it in and press Refresh.")
                    return
                index = list(labels).index(self.device.get())
                bus = core.ElmueBus(index, settings, listen_only)
            else:
                if not self.channel.get().strip():
                    messagebox.showerror("CAN test", "Enter a python-can channel, e.g. COM5 or can0.")
                    return
                bus = core.PyCanBus(self.interface.get().strip(), self.channel.get().strip(), settings, listen_only)
        except SystemExit as ex:  # the core reports setup problems this way
            core.emit(str(ex), "error")
            messagebox.showerror("CAN test", str(ex))
            return
        except Exception as ex:
            core.emit("Connect failed: %s" % ex, "error")
            messagebox.showerror("CAN test", "Connect failed:\n%s" % ex)
            return

        self.lbl_timing.configure(text="Active: " + getattr(bus, "timing_text", settings.describe()))
        self.worker = Worker(bus, interval, monitor, nodes)
        for p in self.panels.values():
            p.send_var.set(False)
            p.current_var.set(0.0)
        self._apply_all()
        for s in SIDES:
            self.vel_hist[s].clear()
            self.cur_hist[s].clear()
        self._was_fresh = {s: False for s in SIDES}
        self._rate = {"t": time.monotonic(), "count": 0, "fps": 0.0, "gap": 0.0}
        self._ctrl_was_active = False
        self.worker.start()
        if monitor:
            core.emit("Monitoring nodes %d and %d: the GUI sends nothing. Waiting for the controller's current "
                      "frames and the drivers' telemetry." % (nodes["left"], nodes["right"]), "prompt")
        else:
            d = self.worker.drv
            shared = d["left"].cur_id == d["right"].cur_id
            core.emit("Connected to nodes %d and %d: currents go out every %d ms while a side has 'Send current' "
                      "ticked (%s; a side with Send off gets NaN = no command)."
                      % (nodes["left"], nodes["right"], interval,
                         "one shared frame 0x%03X" % d["left"].cur_id if shared else "two frames"), "prompt")
            self.worker.submit("info_all")
        self._update_widgets()

    def disconnect(self):
        w = self.worker
        if w is None:
            return
        w.send_enabled = {s: False for s in SIDES}
        w.stop_flag = True
        w.join(timeout=3.0)
        self.worker = None
        self.lbl_timing.configure(text="")
        self._update_widgets()

    def _fresh(self, side):
        d = self.worker.drv[side] if self.worker else None
        return d is not None and d.last_status_time is not None and time.monotonic() - d.last_status_time < FRESH_S

    def open_settings(self, side):
        dlg = self.settings.get(side)
        if dlg is not None:
            dlg.win.lift()
            dlg.refresh()
            return
        self.settings[side] = SettingsDialog(self, side)

    def submit(self, task, side=None, arg=None):
        w = self.worker
        if w is None or w.busy or w.monitor:
            return
        if task == "auto":
            partner = core.partner_node(w.drv[side].node)
            if w.watch.heard_recently(partner):
                if not messagebox.askokcancel(
                        "CAN test", "Node %d is on the bus too, and shares node %d's current frame.\n\n"
                                   "The auto test's step 6 sends current to node %d to check that node %d ignores it, "
                                   "so that motor may spin briefly if it is running.\n\nContinue?"
                                   % (partner, w.drv[side].node, partner, w.drv[side].node)):
                    return
        if task in ("auto", "busoff"):
            for p in self.panels.values():
                p.send_var.set(False)
            self._apply_all()
        w.submit(task, side, arg)

    def abort_task(self):
        if self.worker is not None and self.worker.busy:
            self.worker.abort_flag = True

    def stop_motor(self):
        for p in self.panels.values():
            p.send_var.set(False)
            p.current_var.set(0.0)
        self._apply_all()
        w = self.worker
        if w is not None and not w.monitor:
            # The worker thread owns the adapter: it aborts a running test
            # (which sends 0 A) or sends one 0 A frame straight away.
            w.abort_flag = True
            w.zero_now = True

    def set_current(self, side, amps):
        self.panels[side].current_var.set(amps)
        self.on_current_changed(side)

    def on_current_changed(self, side):
        self._link_from(side)
        self._apply_all()

    def on_send_toggled(self, side):
        self._link_from(side)
        self._apply_all()

    def _link_from(self, side):
        """With 'Link' on, copy this side's slider and Send box to the other side."""
        if not self.link_var.get() or self._linking:
            return
        self._linking = True
        try:
            src, dst = self.panels[side], self.panels[other_side(side)]
            dst.current_var.set(src.current_var.get())
            dst.send_var.set(src.send_var.get())
        finally:
            self._linking = False
        self._apply_all()

    def _apply_all(self):
        w = self.worker
        if w is None:
            return
        # Build new dicts and swap them in, so the worker never sees a half update.
        w.current = {s: float(self.panels[s].current_var.get()) for s in SIDES}
        w.send_enabled = {s: bool(self.panels[s].send_var.get()) for s in SIDES}

    def on_close(self):
        self.disconnect()
        self.root.destroy()

    # --- periodic UI update ---

    def tick(self):
        self._drain_log()
        self._update_status()
        self._draw_plot()
        self._update_widgets()
        for dlg in list(self.settings.values()):
            dlg.update()
        self.root.after(UI_PERIOD_MS, self.tick)

    def _drain_log(self):
        lines = []
        try:
            while True:
                lines.append(self.log_queue.get_nowait())
        except queue.Empty:
            pass
        if not lines:
            return
        self.log.configure(state="normal")
        for text, kind in lines:
            self.log.insert("end", "%s  %s\n" % (time.strftime("%H:%M:%S"), text), (kind,))
        self.log.see("end")
        self.log.configure(state="disabled")

    def _clear_log(self):
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def _set_label(self, lbl, **kw):
        # Only touch a label when something changed (avoids needless redraws).
        changed = {k: v for k, v in kw.items() if str(lbl.cget(k)) != str(v)}
        if changed:
            lbl.configure(**changed)

    def _update_status(self):
        w = self.worker
        now = time.monotonic()
        if w is None:
            # Not connected: nothing below is known any more, so show the
            # panels as they start out (not the last values before Disconnect).
            for p in self.panels.values():
                self._set_label(p.lbl_link, text="not connected", bg="#dddddd")
                self._set_label(p.lbl_state, text="state: -", bg="#f0f0f0")
                for lbl in p.flag_labels.values():
                    self._set_label(lbl, bg="#f0f0f0")
                self._set_label(p.lbl_vel, text="- rad/s")
                self._set_label(p.lbl_volt, text="- V")
                self._set_label(p.lbl_fw, text="")
                self._set_label(p.lbl_cmd, text="commanded: -")
                self._set_label(p.lbl_counts, text="")
            self._set_label(self.lbl_bus, text="")
            return

        # Controller (another node sending current frames)
        watch = w.watch
        if now - self._rate["t"] >= 1.0:
            dt = now - self._rate["t"]
            self._rate.update(fps=(watch.count - self._rate["count"]) / dt, count=watch.count, t=now)
            self._gaps = {s: watch.take_max_gap(w.drv[s].node) for s in SIDES}
        ctrl_active = watch.active()
        if ctrl_active != self._ctrl_was_active:
            self._ctrl_was_active = ctrl_active
            if w.monitor:
                core.emit("Controller %s sending current frames." % ("started" if ctrl_active else "STOPPED"),
                          "info" if ctrl_active else "warn")
        if not w.monitor and ctrl_active and now - self._ctrl_warned > 5.0:
            self._ctrl_warned = now
            core.emit("WARNING: another node is sending current frames (the robot's controller?). "
                      "It fights the GUI - disconnect it or use Monitor mode.", "error")

        if watch.count:
            ctrl = "controller current frames: %s, %.0f/s" % ("active" if ctrl_active else "silent", self._rate["fps"])
            if watch.invalid:
                ctrl += ", wrong length: %d" % watch.invalid
        else:
            ctrl = "no current frames from another node" if w.monitor else ""
        heard = sorted(n for n in watch.nodes_heard if watch.heard_recently(n, 2.0))
        if heard:
            ctrl += "   nodes heard: " + ", ".join(str(n) for n in heard)
        task = ("   running: " + w.busy) if w.busy else ""
        self._set_label(self.lbl_bus, text="adapter CAN errors: %d   %s%s" % (w.bus.adapter_errors, ctrl, task),
                        fg="#dc322f" if (not w.monitor and ctrl_active) else "#666666")

        # A board whose node ID was changed and saved: the Driver followed it.
        for s, p in self.panels.items():
            if p.node() != w.drv[s].node:
                p.node_var.set(w.drv[s].node)
                p.update_ids()
                self._was_fresh[s] = False

        # Per side
        state_bg = {core.STATE_RUNNING: "#9be29b", core.STATE_IDLE: "#f0f0f0", core.STATE_FAULT: "#f3a6a6"}
        for s, p in self.panels.items():
            d = w.drv[s]
            fresh = self._fresh(s)
            if fresh != self._was_fresh[s]:
                self._was_fresh[s] = fresh
                core.emit("%s driver %s." % (s, "online" if fresh else "OFFLINE (no status for 0.5 s)"),
                          "info" if fresh else "warn")
            self._set_label(p.lbl_link, text="driver online" if fresh else "no status",
                            bg="#9be29b" if fresh else "#f3a6a6")
            self._set_label(p.lbl_state, text="state: " + core.state_name(d.last_state),
                            bg=state_bg.get(d.last_state, "#f0f0f0"))
            for bit, lbl in p.flag_labels.items():
                on = fresh and bool(d.last_flags & bit)
                warn_bit = bit in (core.FLAG_TIMEOUT, core.FLAG_DRV_FAULT)
                self._set_label(lbl, bg=("#f3a6a6" if warn_bit else "#9bc8f0") if on else "#f0f0f0")
            self._set_label(p.lbl_vel, text="%+.2f rad/s" % d.last_vel if fresh else "- rad/s")
            volt = "%.2f V" % (d.last_mv / 1000) if fresh and d.last_mv else "- V"
            if d.has_cap(core.CAP_MEASURES_INPUT_CURRENT):
                ok = fresh and d.last_ma is not None
                volt += "  " + (core.fmt_input(d.last_mv, d.last_ma, 2).replace(", ", "  ") if ok else "- A  - W")
            self._set_label(p.lbl_volt, text=volt)
            fw = ("%s, fw %d.%d.%d, protocol %d" % (
                d.board_label(), d.info[1], d.info[2], d.info[3], d.info[0]) if d.info else "")
            self._set_label(p.lbl_fw, text=fw)

            # What the motor is being told
            extra = ""
            if w.monitor:
                n = d.node
                cur = watch.current.get(n) if watch.active(n) else None
                cmd = ("commanded by controller: %+.3f A" % cur) if cur is not None else "commanded: - (no command)"
                if watch.commands.get(n):
                    cmd += "   commands seen: %d" % watch.commands[n]
                if n in watch.last_time:
                    extra = "   controller: longest gap %d ms, gaps > 100 ms: %d" % (
                        self._gaps.get(s, 0.0) * 1000, watch.late.get(n, 0))
            elif w.sending() and w.send_enabled[s]:
                cur = w.current[s]
                cmd = "sending: %+.2f A" % cur
            else:
                cur = None
                cmd = "not sending (NaN / nothing: the driver times out to 0 A)"
            self._set_label(p.lbl_cmd, text=cmd)
            self._set_label(p.lbl_counts, text="driver errors: %d   send failures: %d%s" % (
                len(d.errors), d.tx_failures, extra))

            if fresh:
                self.vel_hist[s].append((now, d.last_vel))
            self.cur_hist[s].append((now, cur))
            for hist in (self.vel_hist[s], self.cur_hist[s]):
                while hist and now - hist[0][0] > PLOT_SECONDS:
                    hist.popleft()

    def _draw_plot(self):
        c = self.canvas
        c.delete("all")
        W, H = max(c.winfo_width(), 50), max(c.winfo_height(), 80)
        L, R, T, B, GAP = 56, 10, 8, 18, 14
        ph_total = H - T - B - GAP
        vel_h = int(ph_total * 0.6)
        cur_h = ph_total - vel_h
        pw = W - L - R
        now = time.monotonic()

        def panel(top, ph, hists, min_scale, unit):
            vals = [abs(v) for h in hists.values() for _, v in h if v is not None]
            vmax = max([min_scale] + vals) * 1.1

            def y(v):
                return top + ph / 2 - v / vmax * ph / 2

            c.create_rectangle(L, top, L + pw, top + ph, outline="#cccccc")
            c.create_line(L, y(0), L + pw, y(0), fill="#bbbbbb", dash=(3, 3))
            fmt = "%+.0f" if vmax > 10 else "%+.1f"
            for v in (vmax / 1.1, -vmax / 1.1):
                c.create_text(L - 4, y(v), text=fmt % v, anchor="e", fill="#666666", font=("Segoe UI", 8))
            c.create_text(L - 4, y(0), text=unit, anchor="e", fill="#666666", font=("Segoe UI", 8))
            for s, h in hists.items():
                # Break the line wherever there is no value (driver offline / not commanded).
                seg = []
                for t, v in h:
                    if v is None:
                        if len(seg) >= 4:
                            c.create_line(*seg, fill=SIDE_COLORS[s], width=2)
                        seg = []
                        continue
                    seg += [L + pw - (now - t) / PLOT_SECONDS * pw, y(v)]
                if len(seg) >= 4:
                    c.create_line(*seg, fill=SIDE_COLORS[s], width=2)

        panel(T, vel_h, self.vel_hist, 5.0, "rad/s")
        panel(T + vel_h + GAP, cur_h, self.cur_hist, 0.5, "A")
        c.create_text(L, H - 4, text="-%ds" % PLOT_SECONDS, anchor="sw", fill="#666666", font=("Segoe UI", 8))
        c.create_text(L + pw, H - 4, text="now", anchor="se", fill="#666666", font=("Segoe UI", 8))

    def _set_enabled(self, widget, enabled, on_state="normal"):
        """Change a widget's state only when it actually changes.

        Re-applying the state of a ttk button on every UI tick resets its
        pressed/active state, so clicks get lost between press and release."""
        want = on_state if enabled else "disabled"
        if self._widget_state.get(widget) == want:
            return
        self._widget_state[widget] = want
        if isinstance(widget, ttk.Widget):
            if want == "disabled":
                widget.state(["disabled"])
            elif want == "readonly":
                widget.state(["!disabled", "readonly"])
            else:
                widget.state(["!disabled", "!readonly"])
        else:
            widget.configure(state=want)

    def _update_widgets(self):
        connected = self.worker is not None
        monitor = self.worker.monitor if connected else self.mode.get() == "monitor"
        busy = connected and self.worker.busy is not None
        elmue = self.backend.get() == "elmue"
        idle = not connected
        can_control = connected and not monitor and not busy

        text = "Disconnect" if connected else "Connect"
        if self.btn_connect.cget("text") != text:
            self.btn_connect.configure(text=text)

        self._set_enabled(self.cb_backend, idle, "readonly")
        self._set_enabled(self.cb_device, idle and elmue, "readonly")
        self._set_enabled(self.btn_refresh, idle)
        self._set_enabled(self.cb_iface, idle and not elmue)
        self._set_enabled(self.cb_chan, idle and not elmue)
        for wdg in (self.rb_control, self.rb_monitor):
            self._set_enabled(wdg, idle)
        self._set_enabled(self.chk_listen, idle and monitor and elmue)
        self._set_enabled(self.sp_interval, idle and not monitor)
        for wdg in (self.cb_bitrate, self.cb_sp, self.chk_fd, self.btn_defaults):
            self._set_enabled(wdg, idle)
        for wdg in (self.cb_dbitrate, self.cb_dsp):
            self._set_enabled(wdg, idle and self.fd.get())
        for p in self.panels.values():
            for wdg in p.controls:
                self._set_enabled(wdg, can_control)
            self._set_enabled(p.sp_node, idle)
        for wdg in (self.btn_start_both, self.chk_link):
            self._set_enabled(wdg, can_control)
        self._set_enabled(self.btn_abort, busy)
        self._set_enabled(self.btn_stop, not monitor)


def main():
    if sys.platform == "win32":
        # Without this, Windows bitmap-scales the window on high-DPI displays
        # (blurry text, and hit areas that can end up offset from the drawn widgets).
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
    root = tk.Tk()
    try:
        ttk.Style().theme_use("vista" if sys.platform == "win32" else "clam")
    except tk.TclError:
        pass
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
