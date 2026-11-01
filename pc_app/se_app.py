#!/usr/bin/env python3
"""
EMI Shielding Effectiveness - Antenna Positioner + NanoVNA (v3)
==============================================================
PC software for:
  * two antennas on a NanoVNA with the sample between them,
  * antenna A on a motorised stage (our ESP32 firmware, src/main.cpp),
  * S21 with and without the sample at up to 3 antenna positions (0-20 cm),
  * temperature / humidity (DHT22 on the ESP32) logged for every sweep,
  * automatic SE calculation:  SE(f, d) = |S21 no sample| dB - |S21 with sample| dB,
    plus the average SE over the stops of each run.

Based on se_app_v2.py (Version2_SE_System), adapted to our one-motor firmware.

Run:  python se_app.py            (use the "Simulated" tick boxes to try without hardware)
      python se_app.py --demo     (both VNA and antenna stage simulated)
Needs: numpy, matplotlib, pyserial
"""
from __future__ import annotations

import argparse
import datetime as dt
import math
import os
import queue
import sys
import threading
import time

import numpy as np
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nanovna_driver import (NanoVNA, NanoVNAV2, SimulatedFreeSpaceVNA, SweepAborted,  # noqa: E402
                            connect_vna, list_serial_ports)  # noqa: E402
from stage_controller import (StageController, SimulatedStage, StageError, StageAborted,  # noqa: E402
                              MAX_CM)  # noqa: E402
import se_core as core  # noqa: E402

# ===========================================================================
# USER SETTINGS - adjust defaults and display here
# ===========================================================================
APP_TITLE = "Antenna Positioner + NanoVNA SE System v3"
DEFAULT_START_MHZ, DEFAULT_STOP_MHZ = 2300.0, 2600.0     # 2.4 GHz band with margin
DEFAULT_POINTS, DEFAULT_AVERAGES = 201, 5                 # more averages needed above 1.5 GHz
BAND_NAME = "2.4 GHz ISM"
BAND_MHZ = (2400.0, 2483.5)         # shaded on plots; band-average / band-min SE reported
MAX_FREQ_MHZ = 3000.0               # highest frequency the app will request (SAA-2: 3 GHz)
NATIVE_MAX_MHZ = 1500.0             # NanoVNA-H4 fundamental range; harmonics above this
SWEEP_PRESETS = {                   # name -> (start MHz, stop MHz, points)
    "2.4 GHz band (2.30-2.60 GHz)": (2300.0, 2600.0, 201),
    "2.4 GHz ISM only (2.40-2.4835 GHz)": (2400.0, 2483.5, 101),
    "Wide (1 MHz - 3 GHz)": (1.0, 3000.0, 401),
    "Native H4 range (1 MHz - 1.5 GHz)": (1.0, 1500.0, 201),
}
MAX_STOPS = 3                       # antenna positions per run
DEFAULT_STOPS_CM = [5.0, 10.0, 15.0]
DEFAULT_SETTLE_S = 1.0              # wait after each move (vibration)
MARKER_FREQS_MHZ = [2400, 2440, 2480]
STATUS_POLL_S = 1.0                 # live antenna/env readout interval
ENV_LOG_INTERVAL_S = 10.0           # background T/RH logging to env_log.csv
SIM_DIST_B_MM = 100.0               # demo only: fixed antenna B distance for the simulated VNA
SESSIONS_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sessions")

PROMPTS = {
    "reference": "REFERENCE run (no sample)\n\nRemove the sample so the antennas see each other "
                 "through the empty sample window/holder.\n\nPress OK to start.",
    "blocked": "BLOCKED run (optional, measures the system limit)\n\nMount a solid, thick metal plate "
               "in the sample holder.\n\nPress OK to start.",
    "sample": "SAMPLE run: \"{name}\"\n\nMount the sample in the holder.\n\nPress OK to start.",
}
COMMAND_HELP = "zero, home, reset, move <cm>, f <cm>, b <cm>, stop, pos, env, status"
# ===========================================================================

GOOD, MUTED, BAD = "#1a7f37", "#777777", "#b42318"


class App(tk.Tk):
    def __init__(self, demo: bool = False):
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1400x880")
        self.minsize(1150, 720)
        self.vna = None
        self.stage = None
        self.session: core.Session | None = None
        self.q: queue.Queue = queue.Queue()
        self.abort = threading.Event()
        self.closing = False
        self.busy = False
        self.last_status: dict | None = None
        self._last_env_log = 0.0
        self._sim_state = {"dist_a": 50.0, "dist_b": SIM_DIST_B_MM, "fixture": "reference", "sample": ""}
        self.vna_sim = tk.BooleanVar(value=demo)
        self.stage_sim = tk.BooleanVar(value=demo)
        self._build()
        self.refresh_ports()
        self.update_plan_label()
        self.redraw()
        threading.Thread(target=self._monitor, daemon=True).start()
        self.after(100, self._poll)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ================================================================== layout
    def _build(self):
        pw = ttk.PanedWindow(self, orient="horizontal")
        pw.pack(fill="both", expand=True)
        left = ttk.Frame(pw, padding=6, width=410)
        right = ttk.Frame(pw, padding=(0, 6, 6, 0))
        pw.add(left, weight=0)
        pw.add(right, weight=1)
        self.after(400, lambda: (self.update_idletasks(), pw.sashpos(0, 430)))

        nb = ttk.Notebook(left)
        nb.pack(fill="both", expand=True)
        self.tab_setup = ttk.Frame(nb, padding=6)
        self.tab_stage = ttk.Frame(nb, padding=6)
        self.tab_run = ttk.Frame(nb, padding=6)
        nb.add(self.tab_setup, text="1  Setup")
        nb.add(self.tab_stage, text="2  Antenna")
        nb.add(self.tab_run, text="3  Measure")
        self.nb = nb
        self._build_setup(self.tab_setup)
        self._build_stage(self.tab_stage)
        self._build_run(self.tab_run)
        self._build_plots(right)

        bar = ttk.Frame(self, padding=(8, 3))
        bar.pack(fill="x", side="bottom")
        self.sb_vna = ttk.Label(bar, text="VNA: not connected", foreground=MUTED)
        self.sb_stage = ttk.Label(bar, text="Antenna: not connected", foreground=MUTED)
        self.sb_env = ttk.Label(bar, text="T —  RH —")
        self.sb_session = ttk.Label(bar, text="Session: none", foreground=MUTED)
        for w in (self.sb_vna, self.sb_stage, self.sb_env):
            w.pack(side="left", padx=(0, 18))
        self.sb_session.pack(side="right")

    def _port_row(self, parent, row, var, sim_var, connect_cmd):
        combo = ttk.Combobox(parent, textvariable=var, width=36, state="readonly")
        combo.grid(row=row, column=0, columnspan=2, sticky="ew")
        ttk.Button(parent, text="Refresh", command=self.refresh_ports).grid(row=row, column=2, padx=(4, 0))
        ttk.Checkbutton(parent, text="Simulated (no hardware)", variable=sim_var).grid(
            row=row + 1, column=0, columnspan=2, sticky="w", pady=(4, 0))
        btn = ttk.Button(parent, text="Connect", command=connect_cmd)
        btn.grid(row=row + 1, column=2, padx=(4, 0), pady=(4, 0))
        lbl = ttk.Label(parent, text="Not connected", foreground=MUTED, wraplength=360)
        lbl.grid(row=row + 2, column=0, columnspan=3, sticky="w", pady=(4, 0))
        parent.columnconfigure(1, weight=1)
        return combo, btn, lbl

    def _build_setup(self, p):
        f = ttk.LabelFrame(p, text="NanoVNA (USB)", padding=6)
        f.pack(fill="x", pady=(0, 6))
        self.vna_port = tk.StringVar()
        self.vna_combo, self.vna_btn, self.vna_lbl = self._port_row(f, 0, self.vna_port, self.vna_sim,
                                                                    self.toggle_vna)
        f = ttk.LabelFrame(p, text="ESP32 antenna positioner (USB)", padding=6)
        f.pack(fill="x", pady=(0, 6))
        self.stage_port = tk.StringVar()
        self.stage_combo, self.stage_btn, self.stage_lbl = self._port_row(f, 0, self.stage_port, self.stage_sim,
                                                                          self.toggle_stage)
        ttk.Label(f, text="Close the PlatformIO Serial Monitor first - only one program can use the port. "
                          "Without the ESP32 the app asks you to set distances by hand.",
                  foreground=MUTED, wraplength=360).grid(row=3, column=0, columnspan=3, sticky="w", pady=(4, 0))

        f = ttk.LabelFrame(p, text="VNA sweep", padding=6)
        f.pack(fill="x", pady=(0, 6))
        self.start_var = tk.StringVar(value=f"{DEFAULT_START_MHZ:g}")
        self.stop_var = tk.StringVar(value=f"{DEFAULT_STOP_MHZ:g}")
        self.points_var = tk.StringVar(value=str(DEFAULT_POINTS))
        self.avg_var = tk.StringVar(value=str(DEFAULT_AVERAGES))
        ttk.Label(f, text="Preset").grid(row=0, column=0, sticky="w", padx=(0, 4))
        self.preset_var = tk.StringVar(value=next(iter(SWEEP_PRESETS)))
        pc = ttk.Combobox(f, textvariable=self.preset_var, values=list(SWEEP_PRESETS), state="readonly", width=32)
        pc.grid(row=0, column=1, columnspan=3, sticky="w", pady=2)
        pc.bind("<<ComboboxSelected>>", lambda e: self.apply_preset())
        for i, (lbl, var) in enumerate([("Start (MHz)", self.start_var), ("Stop (MHz)", self.stop_var),
                                        ("Points", self.points_var), ("Averages", self.avg_var)]):
            ttk.Label(f, text=lbl).grid(row=1 + i // 2, column=(i % 2) * 2, sticky="w", padx=(0, 4))
            ttk.Entry(f, textvariable=var, width=9).grid(row=1 + i // 2, column=(i % 2) * 2 + 1, sticky="w",
                                                         padx=(0, 10), pady=2)
        ttk.Label(f, text="Calibrate the NanoVNA over the same range (SOLT at the antenna cable ends) "
                          "before measuring.", foreground=MUTED, wraplength=360).grid(
            row=3, column=0, columnspan=4, sticky="w", pady=(4, 0))
        self.range_note = ttk.Label(f, text="", foreground="#b45309", wraplength=360)
        self.range_note.grid(row=4, column=0, columnspan=4, sticky="w", pady=(4, 0))
        self.stop_var.trace_add("write", lambda *a: self.update_range_note())
        self.update_range_note()

        f = ttk.LabelFrame(p, text="Session (all data is saved automatically)", padding=6)
        f.pack(fill="x")
        self.session_lbl = ttk.Label(f, text="A new session starts with your first run.", wraplength=360,
                                     foreground=MUTED)
        self.session_lbl.pack(anchor="w")
        b = ttk.Frame(f)
        b.pack(fill="x", pady=(6, 0))
        ttk.Button(b, text="New session…", command=self.new_session).pack(side="left")
        ttk.Button(b, text="Open session…", command=self.open_session).pack(side="left", padx=4)

    def _build_stage(self, p):
        f = ttk.LabelFrame(p, text="Live position and environment", padding=6)
        f.pack(fill="x", pady=(0, 6))
        ttk.Label(f, text="Antenna A", font=("TkDefaultFont", 10, "bold")).grid(row=0, column=0, sticky="w")
        self.ro_pos = ttk.Label(f, text="—", width=34)
        self.ro_pos.grid(row=0, column=1, sticky="w", padx=6)
        ttk.Label(f, text="Environment", font=("TkDefaultFont", 10, "bold")).grid(row=1, column=0, sticky="w")
        self.ro_env = ttk.Label(f, text="—")
        self.ro_env.grid(row=1, column=1, sticky="w", padx=6)

        f = ttk.LabelFrame(p, text="Command to the ESP32", padding=6)
        f.pack(fill="both", expand=True, pady=(0, 6))
        row = ttk.Frame(f)
        row.pack(fill="x")
        self.cmd_var = tk.StringVar()
        e = ttk.Entry(row, textvariable=self.cmd_var)
        e.pack(side="left", fill="x", expand=True)
        e.bind("<Return>", lambda ev: self.send_command())
        ttk.Button(row, text="Send", command=self.send_command).pack(side="left", padx=(4, 0))
        ttk.Label(f, text=f"Commands: {COMMAND_HELP}.  In demo mode, 'kill' simulates the kill button.",
                  foreground=MUTED, wraplength=360).pack(anchor="w", pady=(4, 4))
        box = ttk.Frame(f)
        box.pack(fill="both", expand=True)
        self.console = tk.Text(box, height=14, wrap="word", font=("TkFixedFont", 9), state="disabled")
        sb = ttk.Scrollbar(box, command=self.console.yview)
        self.console.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.console.pack(fill="both", expand=True)

        stop = tk.Button(p, text="STOP MOTOR", bg="#c62828", fg="white", activebackground="#8e0000",
                         font=("TkDefaultFont", 11, "bold"), command=self.stop_motors)
        stop.pack(fill="x", pady=(0, 6), ipady=4)

    def _build_run(self, p):
        f = ttk.LabelFrame(p, text=f"Sweep plan (antenna position from 0 cm, max {MAX_CM:g} cm)", padding=6)
        f.pack(fill="x", pady=(0, 6))
        self.n_stops = tk.StringVar(value=str(MAX_STOPS))
        ttk.Label(f, text="Number of stops").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(f, from_=1, to=MAX_STOPS, textvariable=self.n_stops, width=4, state="readonly",
                    command=self.update_plan_label).grid(row=0, column=1, sticky="w", padx=4, pady=2)
        self.stop_vars, self.stop_entries = [], []
        for i in range(MAX_STOPS):
            var = tk.StringVar(value=f"{DEFAULT_STOPS_CM[i]:g}")
            ttk.Label(f, text=f"Stop {i + 1} (cm)").grid(row=1 + i, column=0, sticky="w")
            ent = ttk.Entry(f, textvariable=var, width=8)
            ent.grid(row=1 + i, column=1, sticky="w", padx=4, pady=2)
            var.trace_add("write", lambda *a: self.update_plan_label())
            self.stop_vars.append(var)
            self.stop_entries.append(ent)
        self.settle = tk.StringVar(value=f"{DEFAULT_SETTLE_S:g}")
        ttk.Label(f, text="Settle (s)").grid(row=1 + MAX_STOPS, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.settle, width=8).grid(row=1 + MAX_STOPS, column=1, sticky="w",
                                                             padx=4, pady=2)
        self.settle.trace_add("write", lambda *a: self.update_plan_label())
        self.plan_lbl = ttk.Label(f, text="", foreground=MUTED, wraplength=360)
        self.plan_lbl.grid(row=2 + MAX_STOPS, column=0, columnspan=4, sticky="w", pady=(4, 0))

        f = ttk.LabelFrame(p, text="Run", padding=6)
        f.pack(fill="x", pady=(0, 6))
        ttk.Label(f, text="Recommended order: 1 then 2 (optional) then 3 for each sample. "
                          "Repeat 1 if the room or setup changes.", foreground=MUTED, wraplength=360).grid(
            row=0, column=0, columnspan=2, sticky="w")
        self.run_btns = {}
        self.run_btns["reference"] = ttk.Button(f, text="1  Reference run (no sample)",
                                                command=lambda: self.start_run("reference"))
        self.run_btns["blocked"] = ttk.Button(f, text="2  Blocked run - metal plate (optional)",
                                              command=lambda: self.start_run("blocked"))
        for i, k in enumerate(("reference", "blocked")):
            self.run_btns[k].grid(row=1 + i, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        ttk.Label(f, text="Sample name").grid(row=3, column=0, sticky="w", pady=(8, 0))
        self.sample_name = tk.StringVar(value="Sample 1")
        ttk.Entry(f, textvariable=self.sample_name).grid(row=3, column=1, sticky="ew", pady=(8, 0), padx=(4, 0))
        self.run_btns["sample"] = ttk.Button(f, text="3  Sample run", command=lambda: self.start_run("sample"))
        self.run_btns["sample"].grid(row=4, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        f.columnconfigure(1, weight=1)

        f = ttk.Frame(p)
        f.pack(fill="x", pady=(0, 4))
        self.abort_btn = tk.Button(f, text="Abort run", command=self.abort_run, state="disabled",
                                   bg="#fde0dc")
        self.abort_btn.pack(side="right")
        self.run_status = ttk.Label(f, text="Idle", foreground=MUTED)
        self.run_status.pack(side="left")
        self.progress = ttk.Progressbar(p, mode="determinate", maximum=1.0)
        self.progress.pack(fill="x", pady=(0, 6))

        f = ttk.LabelFrame(p, text="Log", padding=4)
        f.pack(fill="both", expand=True)
        self.log = tk.Text(f, height=10, wrap="word", font=("TkFixedFont", 9), state="disabled")
        sb = ttk.Scrollbar(f, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log.pack(fill="both", expand=True)

    def _build_plots(self, p):
        vp = ttk.PanedWindow(p, orient="vertical")
        vp.pack(fill="both", expand=True)
        top = ttk.Frame(vp)
        bottom = ttk.Frame(vp)
        vp.add(top, weight=3)
        vp.add(bottom, weight=1)

        ctl = ttk.Frame(top)
        ctl.pack(fill="x")
        self.view = tk.StringVar(value="se_f")
        for txt, val in [("SE vs frequency", "se_f"), ("SE vs distance", "se_d"), ("Raw |S21|", "s21"),
                         ("Environment", "env")]:
            ttk.Radiobutton(ctl, text=txt, value=val, variable=self.view, command=self.redraw).pack(
                side="left", padx=(0, 8))
        ttk.Label(ctl, text="  Sample").pack(side="left")
        self.sel_sample = tk.StringVar()
        self.sample_combo = ttk.Combobox(ctl, textvariable=self.sel_sample, state="readonly", width=18)
        self.sample_combo.pack(side="left", padx=4)
        self.sample_combo.bind("<<ComboboxSelected>>", lambda e: self.redraw())
        self.logx = tk.BooleanVar(value=False)
        ttk.Checkbutton(ctl, text="Log freq", variable=self.logx, command=self.redraw).pack(side="left", padx=4)

        self.fig = Figure(figsize=(8, 5.5), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=top)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        tb = ttk.Frame(top)
        tb.pack(fill="x")
        NavigationToolbar2Tk(self.canvas, tb).update()

        bar = ttk.Frame(bottom)
        bar.pack(fill="x", pady=(4, 2))
        ttk.Label(bar, text=f"Results (SE in dB; band = {BAND_NAME} {BAND_MHZ[0]:g}-{BAND_MHZ[1]:g} MHz; "
                            "≥ = at system limit; AVG = average over the stops)").pack(side="left")
        ttk.Button(bar, text="Export SE CSV…", command=self.export_csv).pack(side="right")
        ttk.Button(bar, text="Save plot…", command=self.save_plot).pack(side="right", padx=4)
        cols = ["sample", "run", "pos", "bmean", "bmin"] + [f"m{m}" for m in MARKER_FREQS_MHZ] + ["t", "rh", "dt"]
        heads = ["Sample", "Run", "Stop cm", "Band avg", "Band min"] + [
            (f"@{m / 1000:g} GHz" if m >= 1000 else f"@{m:g} MHz") for m in MARKER_FREQS_MHZ] + \
            ["T °C", "RH %", "ΔT ref"]
        self.table = ttk.Treeview(bottom, columns=cols, show="headings", height=6)
        for c, h in zip(cols, heads):
            self.table.heading(c, text=h)
            self.table.column(c, width=130 if c == "sample" else (90 if c == "run" else 70),
                              anchor="w" if c in ("sample", "run") else "e")
        self.table.tag_configure("avg", font=("TkDefaultFont", 9, "bold"))
        sb = ttk.Scrollbar(bottom, command=self.table.yview)
        self.table.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.table.pack(fill="both", expand=True)

    # ================================================================== helpers
    def log_msg(self, text: str):
        self.log.configure(state="normal")
        self.log.insert("end", f"{dt.datetime.now():%H:%M:%S}  {text}\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def console_add(self, text: str):
        self.console.configure(state="normal")
        self.console.insert("end", text + "\n")
        self.console.see("end")
        self.console.configure(state="disabled")

    def refresh_ports(self):
        ports = list_serial_ports()
        items = [f"{d}  —  {desc}{'  [NanoVNA]' if NanoVNA.looks_like_nanovna(v, p) else ''}"
                 for d, desc, v, p in ports]
        for combo, var in ((self.vna_combo, self.vna_port), (self.stage_combo, self.stage_port)):
            combo["values"] = items
            if var.get() not in items:
                var.set("")
        vna_guess = next((i for i in items if "[NanoVNA]" in i), items[0] if items else "")
        if not self.vna_port.get():
            self.vna_port.set(vna_guess)
        if not self.stage_port.get():          # the ESP32 board uses a CP210x USB chip
            self.stage_port.set(next((i for i in items if "CP210" in i), "")
                                or next((i for i in items if i != vna_guess), ""))

    def ensure_session(self) -> core.Session:
        if self.session is None:
            os.makedirs(SESSIONS_ROOT, exist_ok=True)
            self.session = core.Session.create(SESSIONS_ROOT)
            self._session_changed()
        return self.session

    def _session_changed(self):
        s = self.session
        self.session_lbl.config(text=f"Saving to:\n{os.path.abspath(s.folder)}", foreground=GOOD)
        self.sb_session.config(text=f"Session: {s.name}")
        self.log_msg(f"Session: {os.path.abspath(s.folder)}")
        self.refresh_results()

    def read_settings(self) -> dict:
        try:
            start, stop = float(self.start_var.get()), float(self.stop_var.get())
            points, avg = int(self.points_var.get()), int(self.avg_var.get())
        except ValueError:
            raise ValueError("Sweep settings must be numbers.")
        if not 0.01 <= start < stop <= MAX_FREQ_MHZ:
            raise ValueError(f"Frequencies must satisfy 0.01 MHz ≤ start < stop ≤ {MAX_FREQ_MHZ:g} MHz.")
        if not 11 <= points <= 2001:
            raise ValueError("Points must be between 11 and 2001.")
        if not 1 <= avg <= 50:
            raise ValueError("Averages must be between 1 and 50.")
        return dict(start_hz=start * 1e6, stop_hz=stop * 1e6, points=points, averages=avg)

    def read_plan(self):
        """Stop positions in cm (1 to MAX_STOPS of them) and the settle time."""
        n = int(self.n_stops.get())
        try:
            stops = [float(v.get()) for v in self.stop_vars[:n]]
            settle = float(self.settle.get())
        except ValueError:
            raise ValueError("Stop distances and settle time must be numbers.")
        if any(not 0 <= c <= MAX_CM for c in stops):
            raise ValueError(f"Each stop must be between 0 and {MAX_CM:g} cm.")
        return stops, max(0.0, settle)

    def apply_preset(self):
        start, stop, pts = SWEEP_PRESETS[self.preset_var.get()]
        self.start_var.set(f"{start:g}")
        self.stop_var.set(f"{stop:g}")
        self.points_var.set(str(pts))
        if hasattr(self, "logx"):
            self.logx.set(stop / max(start, 1e-3) > 20)   # log axis only for wide sweeps
            self.redraw()

    def update_range_note(self):
        try:
            stop = float(self.stop_var.get())
        except ValueError:
            stop = 0
        if isinstance(self.vna, NanoVNAV2):
            note = ("NanoVNA V2 / SAA-2: measures natively up to 3 GHz (about 60 dB range at 2.4 GHz). "
                    "Data is uncalibrated - fine for SE (no-sample / with-sample ratio). "
                    "Do the blocked (metal plate) run to see your setup's limit.")
        elif stop > NATIVE_MAX_MHZ:
            note = ("NanoVNA-H4 above 1.5 GHz measures on harmonics: much less dynamic range "
                    "(roughly 40-60 dB at 2.4 GHz). Use ≥5 averages and do the blocked (metal plate) "
                    "run so instrument-limited SE is flagged.")
        else:
            note = ""
        self.range_note.config(text=note)

    def update_plan_label(self):
        n = int(self.n_stops.get())
        for i, ent in enumerate(self.stop_entries):
            ent.config(state="normal" if i < n else "disabled")
        try:
            stops, _ = self.read_plan()
            txt = ", ".join(f"{c:g}" for c in stops)
            self.plan_lbl.config(text=f"{len(stops)} stop(s): {txt} cm, then back home (0 cm).", foreground=MUTED)
        except ValueError as ex:
            self.plan_lbl.config(text=str(ex), foreground=BAD)

    # ================================================================== connections
    def toggle_vna(self):
        if self.busy:
            return
        if self.vna is not None:
            self.vna.close()
            self.vna = None
            self.vna_btn.config(text="Connect")
            self.vna_lbl.config(text="Not connected", foreground=MUTED)
            self.sb_vna.config(text="VNA: not connected", foreground=MUTED)
            return
        if self.vna_sim.get():
            self.vna = SimulatedFreeSpaceVNA(lambda: dict(self._sim_state))
        else:
            port = self.vna_port.get().split()[0] if self.vna_port.get() else ""
            if not port:
                messagebox.showwarning(APP_TITLE, "Choose the NanoVNA's port (press Refresh after plugging it in).")
                return
            try:
                self.vna = connect_vna(port)
            except Exception as ex:
                messagebox.showerror(APP_TITLE, f"Could not connect to the NanoVNA on {port}:\n{ex}\n\n"
                                     "Is it switched on? Close NanoVNA-Saver / NanoVNA-App if they are open.")
                return
        self.vna_btn.config(text="Disconnect")
        self.vna_lbl.config(text=f"Connected: {self.vna.port} ({self.vna.version})", foreground=GOOD)
        self.sb_vna.config(text=f"VNA: {self.vna.version}", foreground=GOOD)
        self.log_msg(f"VNA connected ({self.vna.version})")
        self.update_range_note()

    def toggle_stage(self):
        if self.busy:
            return
        if self.stage is not None:
            st, self.stage = self.stage, None
            st.close()
            self.stage_btn.config(text="Connect")
            self.stage_lbl.config(text="Not connected", foreground=MUTED)
            self.sb_stage.config(text="Antenna: not connected", foreground=MUTED)
            self.sb_env.config(text="T —  RH —")
            self.ro_pos.config(text="—", foreground="black")
            self.ro_env.config(text="—")
            self.last_status = None
            return
        on_line = lambda ln: self.q.put(("esp", ln))      # firmware replies -> command console
        if self.stage_sim.get():
            st = SimulatedStage(on_line)
        else:
            port = self.stage_port.get().split()[0] if self.stage_port.get() else ""
            if not port:
                messagebox.showwarning(APP_TITLE, "Choose the ESP32's port (press Refresh after plugging it in).")
                return
            if self.vna is not None and getattr(self.vna, "port", None) == port:
                messagebox.showwarning(APP_TITLE, "That port is already used by the NanoVNA.")
                return
            try:
                self.config(cursor="watch")
                self.update()
                st = StageController(port, on_line)
            except Exception as ex:
                messagebox.showerror(APP_TITLE, f"Could not connect to the ESP32 on {port}:\n{ex}\n\n"
                                     "Check the firmware is uploaded and the Serial Monitor is closed.")
                return
            finally:
                self.config(cursor="")
        self.stage = st
        self.stage_btn.config(text="Disconnect")
        self.stage_lbl.config(text=f"Connected: {st.port} ({st.fw})", foreground=GOOD)
        self.log_msg(f"Antenna positioner connected ({st.fw}). Position 0 cm = where the antenna "
                     "was at power-up, or after 'zero'.")

    # ================================================================== monitor (background thread)
    def _monitor(self):
        while not self.closing:
            st = self.stage
            if st is not None:
                try:
                    s = st.status()
                    self.q.put(("stage_status", s))
                    if (self.session is not None and time.time() - self._last_env_log >= ENV_LOG_INTERVAL_S
                            and not math.isnan(s["t"])):
                        self._last_env_log = time.time()
                        self.session.log_env(s["t"], s["rh"], s["pos_cm"] * 10, 0.0)
                except Exception as ex:
                    self.q.put(("stage_status_err", str(ex)))
            time.sleep(STATUS_POLL_S)

    def _show_status(self, s: dict):
        self.last_status = s
        state = "KILLED - send 'reset'" if s["killed"] else ("moving" if s["busy"] else "idle")
        self.ro_pos.config(text=f"{s['pos_cm']:6.2f} cm   {state}", foreground=BAD if s["killed"] else "black")
        t, rh = s["t"], s["rh"]
        env = "sensor error" if math.isnan(t) else f"{t:.1f} °C    {rh:.1f} %RH"
        self.ro_env.config(text=env)
        self.sb_env.config(text="T/RH: sensor error" if math.isnan(t) else f"T {t:.1f} °C   RH {rh:.1f} %")
        self.sb_stage.config(text=f"Antenna: {s['pos_cm']:.2f} cm" + ("  KILLED" if s["killed"] else ""),
                             foreground=BAD if s["killed"] else GOOD)

    # ================================================================== manual antenna control
    def send_command(self):
        cmd = self.cmd_var.get().strip()
        if not cmd:
            return
        if self.stage is None:
            messagebox.showwarning(APP_TITLE, "Connect the ESP32 first (Setup tab).")
            return
        if self.busy and cmd != "stop":
            messagebox.showinfo(APP_TITLE, "A measurement run is in progress - only 'stop' is allowed.")
            return
        self.cmd_var.set("")
        self.console_add(f">> {cmd}")

        def work():
            try:
                for ln in self.stage.send(cmd):
                    if ln.startswith("STATUS"):          # other replies arrive through on_line
                        self.q.put(("esp", ln))
            except Exception as ex:
                self.q.put(("esp", f"Error: {ex}"))
        threading.Thread(target=work, daemon=True).start()

    def stop_motors(self):
        self.abort.set()
        if self.stage is not None:
            try:
                self.stage.stop()
                self.log_msg("STOP sent to the motor")
            except Exception as ex:
                self.log_msg(f"STOP failed: {ex}")

    # ================================================================== runs
    def start_run(self, run_type: str):
        if self.busy:
            return
        if self.vna is None:
            messagebox.showwarning(APP_TITLE, "Connect the NanoVNA first (Setup tab).")
            return
        try:
            st = self.read_settings()
            stops, settle = self.read_plan()
        except ValueError as ex:
            messagebox.showerror(APP_TITLE, str(ex))
            return
        sample = ""
        if run_type == "sample":
            sample = self.sample_name.get().strip()
            if not sample:
                messagebox.showwarning(APP_TITLE, "Enter a sample name.")
                return
        if self.stage is not None:
            if self.last_status is not None and self.last_status["killed"]:
                messagebox.showerror(APP_TITLE, "The kill button was pressed.\n\n"
                                                "Send 'reset' in the Antenna tab first.")
                return
        elif not messagebox.askyesno(APP_TITLE, "The ESP32 is not connected.\n\n"
                                                "Run in MANUAL mode? You will be asked to set each antenna "
                                                "position by hand; temperature and humidity will not be recorded."):
            return
        if run_type != "reference" and self.session is not None:
            probe = [core.Sweep("", run_type, sample, c * 10, 0.0, np.zeros(1), np.zeros(1), np.zeros(1), st)
                     for c in stops]
            missing = [p for p in probe if core.find_match(self.session.sweeps, p, "reference") is None]
            if missing and not messagebox.askyesno(
                    APP_TITLE, f"{len(missing)} of {len(stops)} stops have no reference sweep with these "
                               "sweep settings, so SE can't be calculated there yet.\n\nContinue anyway?"):
                return
        elif run_type != "reference" and not messagebox.askyesno(
                APP_TITLE, "No reference run in this session yet. SE needs a reference run at the same "
                           "stops.\n\nContinue anyway?"):
            return
        if not messagebox.askokcancel(APP_TITLE, PROMPTS[run_type].format(name=sample)):
            return

        ses = self.ensure_session()
        run_id = ses.new_run_id(run_type)
        self.abort.clear()
        self._set_busy(True)
        self.log_msg(f"{run_id}: {run_type} run{' of ' + sample if sample else ''}, {len(stops)} stop(s)")
        threading.Thread(target=self._run_worker, args=(run_type, sample, stops, st, settle, run_id),
                         daemon=True).start()

    def _ask(self, msg: str) -> bool:
        """Called from the worker thread: ask the user in the GUI thread and wait."""
        ev, holder = threading.Event(), {}
        self.q.put(("ask", msg, ev, holder))
        while not ev.wait(0.2):
            if self.abort.is_set():
                return False
        return holder.get("ok", False)

    def _env(self):
        if self.stage is None:
            return core.NAN, core.NAN
        try:
            return self.stage.env()
        except Exception:
            return core.NAN, core.NAN

    def _wait_settle(self, settle: float):
        t_end = time.time() + settle
        while time.time() < t_end:
            if self.abort.is_set():
                raise StageAborted()
            time.sleep(0.05)

    def _run_worker(self, run_type, sample, stops, st, settle, run_id):
        n = len(stops)
        stage = self.stage
        show = lambda s: self.q.put(("stage_status", s))
        try:
            for i, cm in enumerate(stops):
                if self.abort.is_set():
                    raise StageAborted()
                if stage is not None:
                    self.q.put(("status", f"[{i + 1}/{n}] Moving to {cm:g} cm…"))
                    stage.move_to(cm)
                    s = stage.wait_idle(abort=self.abort, on_status=show)
                    self._wait_settle(settle)
                    pos = round(s["pos_cm"], 2)
                else:
                    if not self._ask(f"Stop {i + 1} of {n}\n\nSet antenna A to {cm:g} cm, then press OK."):
                        raise StageAborted()
                    pos = cm
                self._sim_state.update(dist_a=pos * 10, fixture=run_type, sample=sample)
                self.q.put(("status", f"[{i + 1}/{n}] Sweeping at {pos:g} cm…"))
                t0, rh0 = self._env()
                f, s11, s21 = self.vna.sweep(st["start_hz"], st["stop_hz"], st["points"], st["averages"],
                                             progress=lambda p, i=i: self.q.put(("progress", (i + p) / n)),
                                             abort=self.abort)
                t1, rh1 = self._env()
                sw = core.Sweep(run_id, run_type, sample, pos * 10, 0.0, f, s11, s21, dict(st), t0, rh0, t1, rh1)
                self.session.add_sweep(sw)
                self.q.put(("sweep", sw))
            if stage is not None:
                self.q.put(("status", "Returning home (0 cm)…"))
                stage.home()
                stage.wait_idle(abort=self.abort, on_status=show)
                self.q.put(("log", "Antenna back at 0 cm"))
            self.q.put(("done", f"{run_id} finished ({n} stop(s))"))
        except (StageAborted, SweepAborted):
            self.q.put(("done", f"{run_id} aborted - sweeps already taken are saved"))
        except Exception as ex:
            try:
                if stage is not None:
                    stage.stop()
            except Exception:
                pass
            import traceback
            from nanovna_driver import dbg
            dbg("RUN ERROR:\n" + traceback.format_exc())
            self.q.put(("run_error", f"{run_id} stopped: {type(ex).__name__}: {ex}\n"
                                     "(details in pc_app/vna_debug_log.txt)"))

    def abort_run(self):
        self.abort.set()
        self.run_status.config(text="Aborting…")

    def _set_busy(self, busy: bool):
        self.busy = busy
        st = "disabled" if busy else "normal"
        for b in list(self.run_btns.values()) + [self.vna_btn, self.stage_btn]:
            b.config(state=st)
        self.abort_btn.config(state="normal" if busy else "disabled")
        if not busy:
            self.progress["value"] = 0

    # ================================================================== queue pump (GUI thread)
    def _poll(self):
        try:
            while True:
                item = self.q.get_nowait()
                kind = item[0]
                if kind == "stage_status":
                    if self.stage is not None:
                        self._show_status(item[1])
                elif kind == "stage_status_err":
                    self.sb_stage.config(text=f"Antenna: {item[1][:60]}", foreground=BAD)
                elif kind == "esp":
                    self.console_add(item[1])
                elif kind == "progress":
                    self.progress["value"] = item[1]
                elif kind == "status":
                    self.run_status.config(text=item[1], foreground="black")
                elif kind == "log":
                    self.log_msg(item[1])
                elif kind == "sweep":
                    sw = item[1]
                    env = "" if np.isnan(sw.t_mean) else f", {sw.t_mean:.1f} °C {sw.rh_mean:.1f} %RH"
                    self.log_msg(f"  saved {sw.file}{env}")
                    if sw.run_type == "sample":
                        self.sel_sample.set(sw.sample)
                    self.refresh_results()
                elif kind == "done":
                    self._set_busy(False)
                    self.run_status.config(text=item[1], foreground=GOOD)
                    self.log_msg(item[1])
                    if "finished" in item[1] and self.sample_name.get():
                        self.sample_name.set(self._next_name(self.sample_name.get()))
                elif kind == "run_error":
                    self._set_busy(False)
                    self.run_status.config(text="Run stopped (error)", foreground=BAD)
                    self.log_msg(item[1])
                    messagebox.showerror(APP_TITLE, item[1])
                elif kind == "ask":
                    _, msg, ev, holder = item
                    holder["ok"] = messagebox.askokcancel(APP_TITLE, msg)
                    ev.set()
        except queue.Empty:
            pass
        if not self.closing:
            self.after(100, self._poll)

    def _next_name(self, name: str) -> str:
        import re
        m = re.match(r"^(.*?)(\d+)$", name)
        if not m or self.session is None or name not in self.session.samples():
            return name
        return f"{m.group(1)}{int(m.group(2)) + 1}"

    # ================================================================== results
    @staticmethod
    def _by_run(results) -> dict:
        groups = {}
        for r in results:
            groups.setdefault(r.sweep.run_id, []).append(r)
        return groups

    def _table_row(self, r, pos_txt, t, rh, dT, tags=()):
        s = r.sweep
        row = [s.sample, s.run_id, pos_txt]
        b = core.band_stats(r, BAND_MHZ)
        pre = "≥" if (b and b[2]) else ""
        row += ["—", "—"] if b is None else [f"{pre}{b[0]:.1f}", f"{pre}{b[1]:.1f}"]
        for m in MARKER_FREQS_MHZ:
            v = core.se_at(r, m)
            row.append("—" if v is None else (f"≥{v[0]:.1f}" if v[1] else f"{v[0]:.1f}"))
        row += ["—" if np.isnan(t) else f"{t:.1f}", "—" if np.isnan(rh) else f"{rh:.1f}",
                "—" if np.isnan(dT) else f"{dT:+.2f}"]
        self.table.insert("", "end", values=row, tags=tags)

    def refresh_results(self):
        ses = self.session
        samples = ses.samples() if ses else []
        self.sample_combo["values"] = samples
        if samples and self.sel_sample.get() not in samples:
            self.sel_sample.set(samples[-1])
        self.table.delete(*self.table.get_children())
        if ses:
            for rs in self._by_run(ses.results()).values():
                for r in rs:
                    s = r.sweep
                    self._table_row(r, f"{s.dist_a / 10:g}", s.t_mean, s.rh_mean, s.t_mean - r.ref.t_mean)
                if len(rs) > 1:                          # average over the stops of this run
                    with np.errstate(all="ignore"):
                        t = np.nanmean([r.sweep.t_mean for r in rs]) if any(
                            not np.isnan(r.sweep.t_mean) for r in rs) else core.NAN
                        rh = np.nanmean([r.sweep.rh_mean for r in rs]) if any(
                            not np.isnan(r.sweep.rh_mean) for r in rs) else core.NAN
                        dTs = [r.sweep.t_mean - r.ref.t_mean for r in rs]
                        dT = np.nanmean(dTs) if any(not np.isnan(v) for v in dTs) else core.NAN
                    self._table_row(core.average_se(rs), "AVG", t, rh, dT, tags=("avg",))
            if self.table.get_children():
                self.table.see(self.table.get_children()[-1])
        self.redraw()

    # ================================================================== plots
    def redraw(self):
        self.fig.clf()
        ax = self.fig.add_subplot(111)
        view = self.view.get()
        ses = self.session
        sample = self.sel_sample.get()
        drew = False
        if ses is not None:
            if view == "se_f":
                drew = self._plot_se_f(ax, ses, sample)
            elif view == "se_d":
                drew = self._plot_se_d(ax, ses, sample)
            elif view == "s21":
                drew = self._plot_s21(ax, ses, sample)
            else:
                drew = self._plot_env(ax, ses)
        if not drew:
            msg = {"env": "No temperature / humidity data yet.\nConnect the ESP32 and start a session."}.get(
                view, "No SE data yet.\nRun 1 (reference) and then 3 (sample) at the same stops.")
            ax.text(0.5, 0.5, msg, ha="center", va="center", transform=ax.transAxes, color=MUTED)
            ax.set_xticks([])
            ax.set_yticks([])
        self.fig.tight_layout()
        self.canvas.draw_idle()

    def _dist_colors(self, results):
        keys = sorted({(r.sweep.dist_a, r.sweep.dist_b) for r in results})
        cmap = matplotlib.colormaps["viridis"]
        return {k: cmap(0.9 * i / max(1, len(keys) - 1)) for i, k in enumerate(keys)}

    def _shade_band(self, ax, fmin, fmax):
        if BAND_MHZ[0] < fmax and BAND_MHZ[1] > fmin:
            ax.axvspan(max(BAND_MHZ[0], fmin), min(BAND_MHZ[1], fmax), color="#f59f00", alpha=0.12, lw=0,
                       label=f"{BAND_NAME} band")
        if fmin < NATIVE_MAX_MHZ < fmax and not isinstance(self.vna, NanoVNAV2):
            ax.axvline(NATIVE_MAX_MHZ, color="#888", ls="-.", lw=1, label="1.5 GHz: H4 harmonic mode above")

    def _plot_se_f(self, ax, ses, sample):
        res = ses.results(sample)
        if not res:
            return False
        colors = self._dist_colors(res)
        dup = len({(r.sweep.dist_a, r.sweep.dist_b) for r in res}) < len(res)
        for r in sorted(res, key=lambda r: (r.sweep.dist_a, r.sweep.dist_b)):
            s, col = r.sweep, colors[(r.sweep.dist_a, r.sweep.dist_b)]
            fm = s.f / 1e6
            lbl = s.pos_label + (f" ({s.run_id})" if dup else "")
            ax.plot(fm, np.where(r.at_limit, np.nan, r.se), color=col, lw=1.6, label=lbl)
            if r.at_limit.any():
                ax.plot(fm, np.where(r.at_limit, r.se, np.nan), color=col, lw=1.1, ls=":")
            if r.dr is not None:
                ax.plot(fm, r.dr, color=col, lw=0.8, ls="--", alpha=0.45)
        for run_id, rs in self._by_run(res).items():   # average over the stops of each run
            if len(rs) > 1:
                avg = core.average_se(rs)
                ax.plot(avg.sweep.f / 1e6, avg.se, color="k", lw=2.4,
                        label="Average" + (f" ({run_id})" if dup else ""))
        self._shade_band(ax, min(r.sweep.f[0] for r in res) / 1e6, max(r.sweep.f[-1] for r in res) / 1e6)
        un = ses.unmatched(sample)
        title = f"Shielding effectiveness — {sample}"
        if any(r.dr is not None for r in res):
            title += "   (dashed = system limit, dotted = at limit)"
        ax.set_title(title + (f"\n{un} sweep(s) without a matching reference not shown" if un else ""), fontsize=10)
        ax.set_xlabel("Frequency (MHz)")
        ax.set_ylabel("SE (dB)")
        ax.set_ylim(bottom=min(0, float(np.nanmin([np.min(r.se) for r in res]))))
        if self.logx.get():
            ax.set_xscale("log")
        ax.grid(True, which="both", alpha=0.3)
        ax.legend(fontsize=8, title="Antenna position", title_fontsize=8)
        return True

    def _plot_se_d(self, ax, ses, sample):
        res = ses.results(sample)
        if not res:
            return False
        colors = matplotlib.rcParams["axes.prop_cycle"].by_key()["color"]
        drew = False
        bpts = sorted((r.sweep.dist_a / 10, *core.band_stats(r, BAND_MHZ)) for r in res
                      if core.band_stats(r, BAND_MHZ) is not None)
        if bpts:
            x = np.array([p[0] for p in bpts])
            y = np.array([p[1] for p in bpts])
            lim = np.array([p[3] for p in bpts])
            ax.plot(x, y, "-", color="k", lw=2.2, label=f"{BAND_NAME} band average")
            ax.plot(x[~lim], y[~lim], "s", color="k", ms=6)
            ax.plot(x[lim], y[lim], "s", mfc="white", mec="k", ms=7)
            drew = True
        for j, m in enumerate(MARKER_FREQS_MHZ):
            pts = [(r.sweep.dist_a / 10, *core.se_at(r, m)) for r in res if core.se_at(r, m) is not None]
            if not pts:
                continue
            pts.sort()
            x = np.array([p[0] for p in pts])
            y = np.array([p[1] for p in pts])
            lim = np.array([p[2] for p in pts])
            col = colors[j % len(colors)]
            lbl = f"{m / 1000:g} GHz" if m >= 1000 else f"{m:g} MHz"
            ax.plot(x, y, "-", color=col, lw=1.4, label=lbl)
            ax.plot(x[~lim], y[~lim], "o", color=col, ms=5)
            ax.plot(x[lim], y[lim], "o", mfc="white", mec=col, ms=6)
            drew = True
        if not drew:
            return False
        ax.set_title(f"SE vs distance — {sample}   (open markers = at system limit)", fontsize=10)
        ax.set_xlabel("Antenna A position (cm)")
        ax.set_ylabel("SE (dB)")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, title="Frequency", title_fontsize=8)
        return True

    def _plot_s21(self, ax, ses, sample):
        res = ses.results(sample)
        if res:
            colors = self._dist_colors(res)
            for r in sorted(res, key=lambda r: (r.sweep.dist_a, r.sweep.dist_b)):
                col = colors[(r.sweep.dist_a, r.sweep.dist_b)]
                fm = r.sweep.f / 1e6
                ax.plot(fm, core.db20(r.ref.s21), color=col, lw=1.0, ls="--")
                ax.plot(fm, core.db20(r.sweep.s21), color=col, lw=1.4, label=r.sweep.pos_label)
                if r.blocked is not None:
                    ax.plot(fm, core.db20(r.blocked.s21), color=col, lw=0.8, ls=":", alpha=0.6)
            self._shade_band(ax, min(r.sweep.f[0] for r in res) / 1e6, max(r.sweep.f[-1] for r in res) / 1e6)
            ax.set_title(f"|S21| — {sample}   (dashed = no sample, solid = with sample, dotted = metal plate)",
                         fontsize=10)
            ax.legend(fontsize=8)
        else:
            refs = [s for s in ses.sweeps if s.run_type in ("reference", "blocked")]
            if not refs:
                return False
            for s in refs:
                ax.plot(s.f / 1e6, core.db20(s.s21), lw=1.0, ls="-" if s.run_type == "reference" else ":",
                        label=f"{s.run_type} {s.pos_label}")
            ax.set_title("|S21| of reference / blocked sweeps", fontsize=10)
            ax.legend(fontsize=7, ncol=2)
        ax.set_xlabel("Frequency (MHz)")
        ax.set_ylabel("|S21| (dB)")
        if self.logx.get():
            ax.set_xscale("log")
        ax.grid(True, which="both", alpha=0.3)
        return True

    def _plot_env(self, ax, ses):
        log = list(ses.env_log)
        sw = [s for s in ses.sweeps if not np.isnan(s.t_mean)]
        if not log and not sw:
            return False
        ax2 = ax.twinx()
        if log:
            t = [e[0] for e in log]
            ax.plot(t, [e[1] for e in log], color="#d9480f", lw=1.3, label="Temperature")
            ax2.plot(t, [e[2] for e in log], color="#1c7ed6", lw=1.3, label="Humidity")
        mk = {"reference": ("s", "Reference sweep"), "blocked": ("D", "Blocked sweep"), "sample": ("o", "Sample sweep")}
        for rt, (m, lbl) in mk.items():
            pts = [s for s in sw if s.run_type == rt]
            if pts:
                ts = [dt.datetime.strptime(s.timestamp, "%Y-%m-%d %H:%M:%S") for s in pts]
                ax.plot(ts, [s.t_mean for s in pts], m, color="#d9480f", mfc="white", ms=5, label=lbl)
        ax.set_ylabel("Temperature (°C)", color="#d9480f")
        ax2.set_ylabel("Relative humidity (%)", color="#1c7ed6")
        ax.set_title("Environment during the session (markers = sweeps)", fontsize=10)
        ax.grid(True, alpha=0.3)
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=8, loc="best", framealpha=0.85)
        self.fig.autofmt_xdate()
        return True

    # ================================================================== files
    def new_session(self):
        if self.busy:
            return
        root = filedialog.askdirectory(title="Folder to create the session in", initialdir=SESSIONS_ROOT
                                       if os.path.isdir(SESSIONS_ROOT) else os.getcwd())
        if not root:
            return
        self.session = core.Session.create(root)
        self._session_changed()

    def open_session(self):
        if self.busy:
            return
        folder = filedialog.askdirectory(title="Open a session folder (contains measurement_log.csv)",
                                         initialdir=SESSIONS_ROOT if os.path.isdir(SESSIONS_ROOT) else os.getcwd())
        if not folder:
            return
        try:
            self.session = core.Session.load(folder)
        except Exception as ex:
            messagebox.showerror(APP_TITLE, f"Could not open session:\n{ex}")
            return
        self._session_changed()
        self.log_msg(f"Loaded {len(self.session.sweeps)} sweeps")

    def export_csv(self):
        if self.session is None or not self.session.results():
            messagebox.showinfo(APP_TITLE, "No SE results to export yet.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")],
                                            initialdir=self.session.folder, initialfile="se_results.csv")
        if path:
            summ = self.session.export_se(path, MARKER_FREQS_MHZ, BAND_MHZ, BAND_NAME)
            self.log_msg(f"Exported {os.path.basename(path)} and {os.path.basename(summ)}")
            messagebox.showinfo(APP_TITLE, f"Saved:\n{path}\n{summ}")

    def save_plot(self):
        init = self.session.folder if self.session else os.getcwd()
        path = filedialog.asksaveasfilename(defaultextension=".png", initialdir=init,
                                            filetypes=[("PNG", "*.png"), ("PDF", "*.pdf"), ("SVG", "*.svg")],
                                            initialfile=f"{self.view.get()}_{dt.datetime.now():%Y%m%d_%H%M}.png")
        if path:
            self.fig.savefig(path, dpi=200)

    def on_close(self):
        if self.busy and not messagebox.askyesno(APP_TITLE, "A run is in progress. Stop it and quit?"):
            return
        self.abort.set()
        self.closing = True
        for dev in (self.stage, self.vna):
            try:
                if dev is self.stage and dev is not None:
                    dev.stop()
                if dev is not None:
                    dev.close()
            except Exception:
                pass
        self.destroy()


def main():
    ap = argparse.ArgumentParser(description=APP_TITLE)
    ap.add_argument("--demo", action="store_true", help="simulate both the NanoVNA and the antenna stage")
    args = ap.parse_args()
    App(demo=args.demo).mainloop()


if __name__ == "__main__":
    main()
