"""
Core data model, SE calculation and session storage for the v2 free-space system.

Every sweep is saved as it is taken (Touchstone .s2p + a row in measurement_log.csv),
so a crash or power cut never loses data, and a session can be reopened later.
"""
from __future__ import annotations

import csv
import datetime as dt
import math
import os
import re
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

DIST_TOL_MM = 0.5          # distances within this are treated as the same position
LIMIT_MARGIN_DB = 6.0      # SE within this of the dynamic range is flagged "at limit"
NAN = float("nan")

LOG_FIELDS = ["run_id", "timestamp", "run_type", "sample", "dist_a_mm", "dist_b_mm",
              "t_start_C", "rh_start_pct", "t_end_C", "rh_end_pct",
              "start_hz", "stop_hz", "points", "averages", "file"]
ENV_FIELDS = ["timestamp", "t_C", "rh_pct", "dist_a_mm", "dist_b_mm"]
RUN_TYPES = ("reference", "blocked", "sample")


def now_str() -> str:
    return dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def db20(x) -> np.ndarray:
    return 20.0 * np.log10(np.maximum(np.abs(x), 1e-15))


def same_grid(a: dict, b: dict) -> bool:
    return all(float(a[k]) == float(b[k]) for k in ("start_hz", "stop_hz", "points"))


def safe_filename(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", name).strip("_") or "x"


def _mean2(a, b) -> float:
    vals = [v for v in (a, b) if v is not None and not math.isnan(v)]
    return sum(vals) / len(vals) if vals else NAN


def _num(s) -> float:
    try:
        return float(s)
    except (TypeError, ValueError):
        return NAN


@dataclass
class Sweep:
    run_id: str
    run_type: str                 # "reference" (no sample), "blocked" (metal plate), "sample"
    sample: str
    dist_a: float                 # antenna A to sample plane, mm
    dist_b: float                 # antenna B to sample plane, mm
    f: np.ndarray
    s11: np.ndarray
    s21: np.ndarray
    settings: dict
    t_start: float = NAN
    rh_start: float = NAN
    t_end: float = NAN
    rh_end: float = NAN
    timestamp: str = field(default_factory=now_str)
    file: str = ""

    @property
    def t_mean(self) -> float:
        return _mean2(self.t_start, self.t_end)

    @property
    def rh_mean(self) -> float:
        return _mean2(self.rh_start, self.rh_end)

    @property
    def pos_label(self) -> str:
        if self.dist_b == 0:                       # one motor (antenna B not motorised yet)
            return f"{self.dist_a / 10:g} cm"
        return f"A {self.dist_a / 10:g} / B {self.dist_b / 10:g} cm"


@dataclass
class SEResult:
    sweep: Sweep
    ref: Sweep
    blocked: Optional[Sweep]
    se: np.ndarray
    dr: Optional[np.ndarray]
    at_limit: np.ndarray


def find_match(sweeps: list[Sweep], target: Sweep, run_type: str) -> Optional[Sweep]:
    """Latest sweep of run_type at the same antenna positions and frequency grid
    (preferring ones taken before the target)."""
    c = [s for s in sweeps if s.run_type == run_type
         and abs(s.dist_a - target.dist_a) <= DIST_TOL_MM
         and abs(s.dist_b - target.dist_b) <= DIST_TOL_MM
         and same_grid(s.settings, target.settings)]
    if not c:
        return None
    before = [s for s in c if s.timestamp <= target.timestamp]
    return (before or c)[-1]


def compute_se(sweep: Sweep, sweeps: list[Sweep], margin: float = LIMIT_MARGIN_DB) -> Optional[SEResult]:
    """SE(f) = |S21 without sample| dB - |S21 with sample| dB at the same antenna distances."""
    ref = find_match(sweeps, sweep, "reference")
    if ref is None:
        return None
    se = db20(ref.s21) - db20(sweep.s21)
    blk = find_match(sweeps, sweep, "blocked")
    dr = db20(ref.s21) - db20(blk.s21) if blk is not None else None
    lim = se > (dr - margin) if dr is not None else np.zeros(len(se), dtype=bool)
    return SEResult(sweep, ref, blk, se, dr, lim)


def average_se(results: list[SEResult]) -> SEResult:
    """Mean SE (dB) over the stops of one run (all sweeps share the same frequency grid)."""
    se = np.mean([r.se for r in results], axis=0)
    lim = np.any([r.at_limit for r in results], axis=0)
    return SEResult(results[0].sweep, results[0].ref, None, se, None, lim)


def se_at(res: SEResult, f_mhz: float):
    """(SE in dB, at_limit) at one frequency, or None if outside the sweep."""
    fm = res.sweep.f / 1e6
    if not fm[0] <= f_mhz <= fm[-1]:
        return None
    i = int(np.argmin(np.abs(fm - f_mhz)))
    return float(np.interp(f_mhz, fm, res.se)), bool(res.at_limit[i])


def band_stats(res: SEResult, band_mhz: tuple[float, float]):
    """(mean SE, min SE, any point at limit) over a frequency band, or None if not covered."""
    fm = res.sweep.f / 1e6
    m = (fm >= band_mhz[0]) & (fm <= band_mhz[1])
    if not m.any():
        return None
    return float(np.mean(res.se[m])), float(np.min(res.se[m])), bool(res.at_limit[m].any())


def distance_plan(start: float, stop: float, step: float, mode: str, fixed: float) -> list[tuple[float, float]]:
    """Antenna positions for one run. mode: 'both' (symmetric), 'A' (A moves, B fixed), 'B'."""
    if step <= 0:
        raise ValueError("Step must be greater than 0 mm.")
    sign = 1 if stop >= start else -1
    n = int(math.floor(abs(stop - start) / step + 1e-9)) + 1
    vals = [round(start + sign * i * step, 3) for i in range(n)]
    if mode == "both":
        return [(v, v) for v in vals]
    if mode == "A":
        return [(v, fixed) for v in vals]
    if mode == "B":
        return [(fixed, v) for v in vals]
    raise ValueError(f"Unknown mode {mode}")


# --------------------------------------------------------------------- Touchstone
def write_touchstone(path: str, sw: Sweep):
    with open(path, "w") as fh:
        fh.write(f"! {sw.run_type}: {sw.sample}  run {sw.run_id}  measured {sw.timestamp}\n")
        fh.write(f"! antenna A {sw.dist_a} mm, antenna B {sw.dist_b} mm; "
                 f"T {sw.t_mean:.2f} C, RH {sw.rh_mean:.1f} %\n")
        fh.write("! NanoVNA measures S11 and S21 only; S12 and S22 are set to 0\n")
        fh.write("# HZ S RI R 50\n")
        for f, a, b in zip(sw.f, sw.s11, sw.s21):
            fh.write(f"{f:.0f} {a.real:.9e} {a.imag:.9e} {b.real:.9e} {b.imag:.9e} 0 0 0 0\n")


def read_touchstone(path: str):
    rows = []
    with open(path) as fh:
        for ln in fh:
            ln = ln.strip()
            if not ln or ln[0] in "!#":
                continue
            rows.append([float(v) for v in ln.split()])
    a = np.array(rows)
    return a[:, 0], a[:, 1] + 1j * a[:, 2], a[:, 3] + 1j * a[:, 4]


# --------------------------------------------------------------------- session
class Session:
    def __init__(self, folder: str):
        self.folder = folder
        os.makedirs(os.path.join(folder, "sweeps"), exist_ok=True)
        self.sweeps: list[Sweep] = []
        self.env_log: list[tuple] = []     # (datetime, t, rh, dist_a, dist_b)
        self._runs = 0

    @classmethod
    def create(cls, root: str, name: str = "") -> "Session":
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        return cls(os.path.join(root, stamp + (f"_{safe_filename(name)}" if name else "")))

    @property
    def name(self) -> str:
        return os.path.basename(os.path.normpath(self.folder))

    def new_run_id(self, run_type: str) -> str:
        self._runs += 1
        return f"R{self._runs:03d}_{run_type[:3].upper()}"

    def _append_csv(self, fname, fields, row):
        path = os.path.join(self.folder, fname)
        new = not os.path.exists(path)
        with open(path, "a", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=fields)
            if new:
                w.writeheader()
            w.writerow(row)

    def add_sweep(self, sw: Sweep):
        tag = safe_filename(sw.sample if sw.run_type == "sample" else sw.run_type)
        fname = f"{sw.run_id}_{tag}_A{sw.dist_a:g}_B{sw.dist_b:g}.s2p"
        write_touchstone(os.path.join(self.folder, "sweeps", fname), sw)
        sw.file = f"sweeps/{fname}"
        fmt = lambda v, d: "" if v is None or math.isnan(v) else f"{v:.{d}f}"
        self._append_csv("measurement_log.csv", LOG_FIELDS, {
            "run_id": sw.run_id, "timestamp": sw.timestamp, "run_type": sw.run_type, "sample": sw.sample,
            "dist_a_mm": f"{sw.dist_a:.3f}", "dist_b_mm": f"{sw.dist_b:.3f}",
            "t_start_C": fmt(sw.t_start, 2), "rh_start_pct": fmt(sw.rh_start, 1),
            "t_end_C": fmt(sw.t_end, 2), "rh_end_pct": fmt(sw.rh_end, 1),
            "start_hz": f"{sw.settings['start_hz']:.0f}", "stop_hz": f"{sw.settings['stop_hz']:.0f}",
            "points": sw.settings["points"], "averages": sw.settings.get("averages", 1), "file": sw.file})
        self.sweeps.append(sw)

    def log_env(self, t: float, rh: float, dist_a: float, dist_b: float):
        ts = dt.datetime.now()
        self.env_log.append((ts, t, rh, dist_a, dist_b))
        self._append_csv("env_log.csv", ENV_FIELDS, {
            "timestamp": ts.strftime("%Y-%m-%d %H:%M:%S"), "t_C": f"{t:.2f}", "rh_pct": f"{rh:.1f}",
            "dist_a_mm": f"{dist_a:.2f}", "dist_b_mm": f"{dist_b:.2f}"})

    def samples(self) -> list[str]:
        out = []
        for s in self.sweeps:
            if s.run_type == "sample" and s.sample not in out:
                out.append(s.sample)
        return out

    def results(self, sample: Optional[str] = None) -> list[SEResult]:
        out = []
        for s in list(self.sweeps):
            if s.run_type == "sample" and (sample is None or s.sample == sample):
                r = compute_se(s, self.sweeps)
                if r is not None:
                    out.append(r)
        return out

    def unmatched(self, sample: Optional[str] = None) -> int:
        n = [s for s in self.sweeps if s.run_type == "sample" and (sample is None or s.sample == sample)]
        return sum(1 for s in n if find_match(self.sweeps, s, "reference") is None)

    def export_se(self, path: str, markers_mhz: list[float], band_mhz=None, band_name: str = "band") -> str:
        results = self.results()
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["sample", "run_id", "timestamp", "dist_a_mm", "dist_b_mm", "frequency_Hz",
                        "frequency_MHz", "SE_dB", "S21_sample_dB", "S21_reference_dB",
                        "dynamic_range_dB", "at_system_limit", "T_C", "RH_pct", "ref_T_C", "ref_RH_pct"])
            for r in results:
                s = r.sweep
                smp, ref = db20(s.s21), db20(r.ref.s21)
                for i, f in enumerate(s.f):
                    w.writerow([s.sample, s.run_id, s.timestamp, f"{s.dist_a:.3f}", f"{s.dist_b:.3f}",
                                f"{f:.0f}", f"{f / 1e6:.6f}", f"{r.se[i]:.3f}", f"{smp[i]:.3f}",
                                f"{ref[i]:.3f}", "" if r.dr is None else f"{r.dr[i]:.3f}",
                                int(r.at_limit[i]), f"{s.t_mean:.2f}", f"{s.rh_mean:.1f}",
                                f"{r.ref.t_mean:.2f}", f"{r.ref.rh_mean:.1f}"])
        root, _ = os.path.splitext(path)
        summ = root + "_summary.csv"
        with open(summ, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["sample", "run_id", "timestamp", "dist_a_mm", "dist_b_mm", "mean_SE_dB",
                        "min_SE_dB", "max_SE_dB", "pct_points_at_limit"] +
                       ([f"band_mean_SE_dB ({band_name} {band_mhz[0]:g}-{band_mhz[1]:g} MHz)", "band_min_SE_dB"]
                        if band_mhz is not None else []) +
                       [f"SE_at_{m:g}MHz_dB" for m in markers_mhz] +
                       ["T_C", "RH_pct", "dT_vs_ref_C", "dRH_vs_ref_pct", "reference_run", "blocked_run"])
            for r in results:
                s = r.sweep
                row = [s.sample, s.run_id, s.timestamp, f"{s.dist_a:.3f}", f"{s.dist_b:.3f}",
                       f"{np.mean(r.se):.2f}", f"{np.min(r.se):.2f}", f"{np.max(r.se):.2f}",
                       f"{100 * np.mean(r.at_limit):.1f}"]
                if band_mhz is not None:
                    b = band_stats(r, band_mhz)
                    pre = ">=" if (b and b[2]) else ""
                    row += ["", ""] if b is None else [f"{pre}{b[0]:.2f}", f"{pre}{b[1]:.2f}"]
                for m in markers_mhz:
                    v = se_at(r, m)
                    row.append("" if v is None else (f">={v[0]:.2f}" if v[1] else f"{v[0]:.2f}"))
                row += [f"{s.t_mean:.2f}", f"{s.rh_mean:.1f}", f"{s.t_mean - r.ref.t_mean:.2f}",
                        f"{s.rh_mean - r.ref.rh_mean:.1f}", r.ref.run_id, r.blocked.run_id if r.blocked else ""]
                w.writerow(row)
        return summ

    @classmethod
    def load(cls, folder: str) -> "Session":
        ses = cls(folder)
        log = os.path.join(folder, "measurement_log.csv")
        if not os.path.exists(log):
            raise FileNotFoundError(f"No measurement_log.csv in {folder}")
        with open(log, newline="") as fh:
            for row in csv.DictReader(fh):
                f, s11, s21 = read_touchstone(os.path.join(folder, row["file"]))
                st = {"start_hz": float(row["start_hz"]), "stop_hz": float(row["stop_hz"]),
                      "points": int(row["points"]), "averages": int(row["averages"] or 1)}
                ses.sweeps.append(Sweep(row["run_id"], row["run_type"], row["sample"], float(row["dist_a_mm"]),
                                        float(row["dist_b_mm"]), f, s11, s21, st, _num(row["t_start_C"]),
                                        _num(row["rh_start_pct"]), _num(row["t_end_C"]), _num(row["rh_end_pct"]),
                                        row["timestamp"], row["file"]))
                m = re.match(r"R(\d+)_", row["run_id"])
                if m:
                    ses._runs = max(ses._runs, int(m.group(1)))
        env = os.path.join(folder, "env_log.csv")
        if os.path.exists(env):
            with open(env, newline="") as fh:
                for row in csv.DictReader(fh):
                    ses.env_log.append((dt.datetime.strptime(row["timestamp"], "%Y-%m-%d %H:%M:%S"),
                                        _num(row["t_C"]), _num(row["rh_pct"]),
                                        _num(row["dist_a_mm"]), _num(row["dist_b_mm"])))
        return ses
