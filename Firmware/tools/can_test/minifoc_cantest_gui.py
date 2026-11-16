#!/usr/bin/env python3
"""
MiniFOC CAN test GUI (tkinter, no extra packages).

Same adapters as minifoc_cantest.py (Candlelight 2.5 via the bundled
ElmuSoft classes in ./elmue, or python-can). Run:  python minifoc_cantest_gui.py

  1. Pick the adapter (Refresh lists the connected ones) and the bit timing.
     The MiniFOC firmware uses 1 Mbit/s, 87.5 %, classic CAN (the defaults).
  2. Connect, Start FOC, tick "Send current" and move the slider.
  3. STOP (or Esc) sets 0 A, stops sending and aborts a running test; the
     driver's 100 ms command timeout then holds it at 0 A as well.
  4. "Auto test" / "Bus-off test" run the same sequences as the CLI; the log
     shows PASS / FAIL.

SAFETY: the motor WILL spin. Clamp it or lift the wheel. Auto test needs
only ONE MiniFOC on the bus.
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
import minifoc_cantest as core  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PLOT_SECONDS = 10.0
UI_PERIOD_MS = 50

BITRATES = ["10k", "20k", "50k", "83.333k", "100k", "125k", "250k", "500k", "800k", "1M"]
DATA_BITRATES = ["1M", "2M", "4M", "5M", "8M"]
SAMPLE_POINTS = ["60", "66.7", "70", "75", "80", "87.5"]
PYCAN_INTERFACES = ["slcan", "socketcan", "gs_usb", "candle", "pcan", "kvaser", "vector", "ixxat", "neovi",
                    "serial", "seeedstudio", "virtual"]


# ----------------------------------------------------------------------------
# CAN worker thread: owns the bus, runs the command loop and the tests
# ----------------------------------------------------------------------------

class Worker(threading.Thread):
    def __init__(self, bus, side, interval_ms):
        super().__init__(daemon=True)
        self.drv = core.Driver(bus, side, interval_ms, verbose=False)
        self.drv.should_abort = self._should_abort
        self.tasks = queue.Queue()
        self.stop_flag = False
        self.abort_flag = False
        self.busy = None            # name of the running task, or None
        self.send_enabled = False
        self.current = 0.0
        self.zero_now = False       # STOP pressed: send one 0 A frame right away
        self.interval_ms = interval_ms

    def _should_abort(self):
        return self.stop_flag or (self.busy is not None and self.abort_flag)

    def submit(self, name):
        self.tasks.put(name)

    def run(self):
        try:
            while not self.stop_flag:
                try:
                    task = self.tasks.get_nowait()
                except queue.Empty:
                    task = None

                if task is None:
                    if self.zero_now:
                        self.zero_now = False
                        self.drv.send_current(0.0)
                    self.drv.run(0.05, self.current, mode="normal" if self.send_enabled else "none")
                    continue

                self.busy = task
                self.abort_flag = False
                try:
                    self._run_task(task)
                except core.Aborted:
                    if not self.stop_flag:
                        core.emit("Test aborted.", "warn")
                        self.drv.send_current(0.0)
                except Exception as ex:
                    core.emit("Error in %s: %s" % (task, ex), "error")
                finally:
                    self.busy = None
                    self.abort_flag = False
        except core.Aborted:
            pass
        except Exception as ex:
            core.emit("CAN connection stopped: %s" % ex, "error")
        finally:
            try:
                self.drv.send_current(0.0)
            except Exception:
                pass
            self.drv.bus.close()
            core.emit("Disconnected.", "dim")

    def _run_task(self, task):
        args = types.SimpleNamespace(current=self.current if abs(self.current) >= 0.05 else 0.3,
                                     interval=self.interval_ms, verbose=False)
        if task == "start":
            core.start_or_fail(self.drv, core.Results())
        elif task == "auto":
            self.send_enabled = False
            core.emit("==== Auto test (%.2f A) ====" % abs(args.current), "prompt")
            core.cmd_auto(self.drv, args)
        elif task == "busoff":
            self.send_enabled = False
            core.emit("==== Bus-off test (%.2f A) ====" % args.current, "prompt")
            core.cmd_busoff(self.drv, args)


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
        self.vel_hist = collections.deque()
        self._widget_state = {}     # last state applied per widget (see _set_enabled)
        self.device_labels = []     # elmue adapters, index = --device
        core.set_output(lambda text, kind: self.log_queue.put((text, kind)))

        root.title("MiniFOC CAN Test")
        root.minsize(980, 720)
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.bind("<Escape>", lambda e: self.stop_motor())

        self._build_connection()
        self._build_timing()
        self._build_status()
        self._build_controls()
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
        self.side = tk.StringVar(value="right")
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

        # Driver row
        ttk.Separator(f).grid(row=4, column=0, columnspan=6, sticky="we", pady=4)
        ttk.Label(f, text="MiniFOC side").grid(row=5, column=0, sticky="w", padx=4, pady=2)
        self.cb_side = ttk.Combobox(f, textvariable=self.side, values=["left", "right"], state="readonly", width=24)
        self.cb_side.grid(row=5, column=1, padx=4, sticky="w")
        ttk.Label(f, text="Command interval (ms)").grid(row=5, column=2, sticky="e", padx=4)
        self.sp_interval = ttk.Spinbox(f, from_=1, to=90, textvariable=self.interval, width=6)
        self.sp_interval.grid(row=5, column=3, padx=4, sticky="w")
        self.btn_connect = ttk.Button(f, text="Connect", command=self.toggle_connect, width=14)
        self.btn_connect.grid(row=5, column=4, columnspan=2, padx=4, sticky="e")
        f.columnconfigure(3, weight=1)

    def _build_timing(self):
        f = ttk.LabelFrame(self.root, text="Bit timing  (MiniFOC firmware: 1 Mbit/s, 87.5 %, classic CAN)")
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

        self.btn_defaults = ttk.Button(f, text="MiniFOC defaults", command=self.timing_defaults)
        self.btn_defaults.grid(row=0, column=9, padx=(16, 4))

        self.lbl_timing = ttk.Label(f, text="", foreground="#666666")
        self.lbl_timing.grid(row=1, column=0, columnspan=10, sticky="w", padx=4, pady=(0, 4))

    def _build_status(self):
        f = ttk.LabelFrame(self.root, text="Driver")
        f.pack(fill="x", padx=8, pady=4)

        self.lbl_link = tk.Label(f, text="not connected", width=16, relief="groove")
        self.lbl_link.grid(row=0, column=0, padx=4, pady=4)
        self.lbl_state = tk.Label(f, text="state: -", width=26, anchor="w")
        self.lbl_state.grid(row=0, column=1, padx=4)
        self.lbl_vel = tk.Label(f, text="- rad/s", width=14, font=("Segoe UI", 16, "bold"))
        self.lbl_vel.grid(row=0, column=2, padx=8)

        self.flag_labels = {}
        for i, (bit, name) in enumerate(((core.FLAG_TIMEOUT, "cmd timeout"),
                                         (core.FLAG_CALIBRATION, "stored align"),
                                         (core.FLAG_INVERTED, "inverted"),
                                         (core.FLAG_DRV_FAULT, "DRV fault"))):
            lbl = tk.Label(f, text=name, width=12, relief="groove")
            lbl.grid(row=0, column=3 + i, padx=2)
            self.flag_labels[bit] = lbl

        self.lbl_counts = tk.Label(f, text="", anchor="w", fg="#666666")
        self.lbl_counts.grid(row=1, column=0, columnspan=7, sticky="w", padx=4)

    def _build_controls(self):
        f = ttk.LabelFrame(self.root, text="Control")
        f.pack(fill="x", padx=8, pady=4)

        self.btn_start = ttk.Button(f, text="Start FOC", command=lambda: self.submit("start"))
        self.btn_start.grid(row=0, column=0, padx=4, pady=4)

        self.send_var = tk.BooleanVar(value=False)
        self.chk_send = ttk.Checkbutton(f, text="Send current", variable=self.send_var, command=self._apply_current)
        self.chk_send.grid(row=0, column=1, padx=8)

        self.current_var = tk.DoubleVar(value=0.0)
        self.scale = tk.Scale(f, from_=-2.0, to=2.0, resolution=0.01, orient="horizontal", length=360,
                              variable=self.current_var, command=lambda v: self._apply_current(),
                              label="Current (A)")
        self.scale.grid(row=0, column=2, padx=4)

        q = ttk.Frame(f)
        q.grid(row=0, column=3, padx=4)
        self.quick_buttons = []
        for a in (-0.5, -0.3, 0.0, 0.3, 0.5):
            b = ttk.Button(q, text="%+.1f" % a if a else "0", width=5,
                           command=lambda a=a: (self.current_var.set(a), self._apply_current()))
            b.pack(side="left", padx=1)
            self.quick_buttons.append(b)

        self.btn_stop = tk.Button(f, text="STOP  (Esc)", bg="#dc322f", fg="white", font=("Segoe UI", 11, "bold"),
                                  activebackground="#b02020", activeforeground="white",
                                  command=self.stop_motor, width=12)
        self.btn_stop.grid(row=0, column=4, padx=8)

        t = ttk.Frame(f)
        t.grid(row=1, column=0, columnspan=5, sticky="w", padx=4, pady=(0, 4))
        self.btn_auto = ttk.Button(t, text="Auto test", command=lambda: self.submit("auto"))
        self.btn_auto.pack(side="left", padx=2)
        self.btn_busoff = ttk.Button(t, text="Bus-off test", command=lambda: self.submit("busoff"))
        self.btn_busoff.pack(side="left", padx=2)
        self.btn_abort = ttk.Button(t, text="Abort test", command=self.abort_task)
        self.btn_abort.pack(side="left", padx=2)
        ttk.Label(t, text="Tests use |slider current| (0.3 A if below 0.05). Auto test: only one MiniFOC on the bus.",
                  foreground="#666666").pack(side="left", padx=8)

    def _build_plot_and_log(self):
        pw = ttk.PanedWindow(self.root, orient="vertical")
        pw.pack(fill="both", expand=True, padx=8, pady=(4, 8))

        pf = ttk.LabelFrame(pw, text="Shaft velocity (last %d s)" % PLOT_SECONDS)
        self.canvas = tk.Canvas(pf, height=160, bg="white", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        pw.add(pf, weight=1)

        lf = ttk.LabelFrame(pw, text="Log")
        self.log = scrolledtext.ScrolledText(lf, height=10, font=("Consolas", 9), state="disabled")
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
        try:
            interval = int(self.interval.get())
        except (tk.TclError, ValueError):
            interval = 0
        if not 1 <= interval <= 90:
            messagebox.showerror("MiniFOC", "Interval must be 1..90 ms (the driver's command timeout is 100 ms).")
            return
        try:
            settings = self._read_settings()
        except ValueError as ex:
            messagebox.showerror("MiniFOC", str(ex))
            return
        if not 50 <= settings.sample_point <= 95 or (settings.fd and not 50 <= settings.data_sample_point <= 95):
            messagebox.showerror("MiniFOC", "Sample points must be between 50 and 95 %.")
            return

        core.emit("Connecting: %s" % settings.describe(), "prompt")
        settings.warn_if_not_driver()
        try:
            if self.backend.get() == "elmue":
                labels = self.cb_device.cget("values")
                if not labels or self.device.get() not in labels:
                    messagebox.showerror("MiniFOC", "No adapter selected. Plug it in and press Refresh.")
                    return
                index = list(labels).index(self.device.get())
                bus = core.ElmueBus(index, settings)
            else:
                if not self.channel.get().strip():
                    messagebox.showerror("MiniFOC", "Enter a python-can channel, e.g. COM5 or can0.")
                    return
                bus = core.PyCanBus(self.interface.get().strip(), self.channel.get().strip(), settings)
        except SystemExit as ex:  # the core reports setup problems this way
            core.emit(str(ex), "error")
            messagebox.showerror("MiniFOC", str(ex))
            return
        except Exception as ex:
            core.emit("Connect failed: %s" % ex, "error")
            messagebox.showerror("MiniFOC", "Connect failed:\n%s" % ex)
            return

        self.lbl_timing.configure(text="Active: " + getattr(bus, "timing_text", settings.describe()))
        self.worker = Worker(bus, self.side.get(), interval)
        self.send_var.set(False)
        self.current_var.set(0.0)
        self._apply_current()
        self.vel_hist.clear()
        self.worker.start()
        d = self.worker.drv
        core.emit("Connected: %s side, commands 0x%03X, telemetry 0x%03X" % (self.side.get(), d.cmd_id, d.tele_id),
                  "prompt")
        self._update_widgets()

    def disconnect(self):
        w = self.worker
        if w is None:
            return
        w.send_enabled = False
        w.current = 0.0
        w.stop_flag = True
        w.join(timeout=3.0)
        self.worker = None
        self.lbl_timing.configure(text="")
        self._update_widgets()

    def submit(self, task):
        if self.worker is None or self.worker.busy:
            return
        if task in ("auto", "busoff"):
            self.send_var.set(False)
        self.worker.submit(task)

    def abort_task(self):
        if self.worker is not None and self.worker.busy:
            self.worker.abort_flag = True

    def stop_motor(self):
        self.send_var.set(False)
        self.current_var.set(0.0)
        self._apply_current()
        if self.worker is not None:
            # The worker thread owns the adapter: it aborts a running test
            # (which sends 0 A) or sends one 0 A frame straight away.
            self.worker.abort_flag = True
            self.worker.zero_now = True

    def _apply_current(self):
        if self.worker is not None:
            self.worker.current = float(self.current_var.get())
            self.worker.send_enabled = bool(self.send_var.get())

    def on_close(self):
        self.disconnect()
        self.root.destroy()

    # --- periodic UI update ---

    def tick(self):
        self._drain_log()
        self._update_status()
        self._draw_plot()
        self._update_widgets()
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
            self.log.insert("end", text + "\n", (kind,))
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
        if w is None:
            self._set_label(self.lbl_link, text="not connected", bg="#dddddd")
            return
        d = w.drv
        now = time.monotonic()
        fresh = d.last_status_time is not None and now - d.last_status_time < 0.5
        self._set_label(self.lbl_link, text="driver online" if fresh else "no status",
                        bg="#9be29b" if fresh else "#f3a6a6")
        state_bg = {core.STATE_RUNNING: "#9be29b", core.STATE_IDLE: "#f0f0f0", core.STATE_FAULT: "#f3a6a6"}
        self._set_label(self.lbl_state, text="state: " + core.state_name(d.last_state),
                        bg=state_bg.get(d.last_state, "#f0f0f0"))
        for bit, lbl in self.flag_labels.items():
            on = fresh and bool(d.last_flags & bit)
            warn_bit = bit in (core.FLAG_TIMEOUT, core.FLAG_DRV_FAULT)
            self._set_label(lbl, bg=("#f3a6a6" if warn_bit else "#9bc8f0") if on else "#f0f0f0")
        self._set_label(self.lbl_vel, text="%+.2f rad/s" % d.last_vel)
        self._set_label(self.lbl_counts, text="driver errors: %d   adapter CAN errors: %d   send failures: %d   %s" % (
            len(d.errors), d.bus.adapter_errors, d.tx_failures, ("running: " + w.busy) if w.busy else ""))

        if fresh or d.last_state is not None:
            self.vel_hist.append((now, d.last_vel))
        while self.vel_hist and now - self.vel_hist[0][0] > PLOT_SECONDS:
            self.vel_hist.popleft()

    def _draw_plot(self):
        c = self.canvas
        c.delete("all")
        W, H = max(c.winfo_width(), 50), max(c.winfo_height(), 50)
        L, R, T, B = 50, 10, 10, 20
        pw, ph = W - L - R, H - T - B
        vmax = max([5.0] + [abs(v) for _, v in self.vel_hist]) * 1.1

        def y(v):
            return T + ph / 2 - v / vmax * ph / 2

        c.create_rectangle(L, T, L + pw, T + ph, outline="#cccccc")
        c.create_line(L, y(0), L + pw, y(0), fill="#bbbbbb", dash=(3, 3))
        for v in (vmax / 1.1, -vmax / 1.1):
            c.create_text(L - 4, y(v), text="%+.0f" % v, anchor="e", fill="#666666", font=("Segoe UI", 8))
        c.create_text(L - 4, y(0), text="0", anchor="e", fill="#666666", font=("Segoe UI", 8))
        c.create_text(L, H - 4, text="-%ds" % PLOT_SECONDS, anchor="sw", fill="#666666", font=("Segoe UI", 8))
        c.create_text(L + pw, H - 4, text="now", anchor="se", fill="#666666", font=("Segoe UI", 8))

        if len(self.vel_hist) >= 2:
            now = time.monotonic()
            pts = []
            for t, v in self.vel_hist:
                pts += [L + pw - (now - t) / PLOT_SECONDS * pw, y(v)]
            c.create_line(*pts, fill="#268bd2", width=2)

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
        busy = connected and self.worker.busy is not None
        elmue = self.backend.get() == "elmue"
        idle = not connected

        text = "Disconnect" if connected else "Connect"
        if self.btn_connect.cget("text") != text:
            self.btn_connect.configure(text=text)

        self._set_enabled(self.cb_backend, idle, "readonly")
        self._set_enabled(self.cb_device, idle and elmue, "readonly")
        self._set_enabled(self.btn_refresh, idle)
        self._set_enabled(self.cb_iface, idle and not elmue)
        self._set_enabled(self.cb_chan, idle and not elmue)
        self._set_enabled(self.cb_side, idle, "readonly")
        self._set_enabled(self.sp_interval, idle)
        for wdg in (self.cb_bitrate, self.cb_sp, self.chk_fd, self.btn_defaults):
            self._set_enabled(wdg, idle)
        for wdg in (self.cb_dbitrate, self.cb_dsp):
            self._set_enabled(wdg, idle and self.fd.get())
        for wdg in [self.btn_start, self.chk_send, self.btn_auto, self.btn_busoff, self.scale] + self.quick_buttons:
            self._set_enabled(wdg, connected and not busy)
        self._set_enabled(self.btn_abort, busy)


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
