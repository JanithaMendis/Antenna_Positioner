"""
NanoVNA drivers: NanoVNA-H / H4 (text shell) and NanoVNA V2 / SAA-2 (binary protocol,
auto-detected by connect_vna), plus a free-space simulator for demo mode.
"""
from __future__ import annotations

import datetime as dt
import math
import os
import re
import struct
import time
from typing import Callable, Optional

import numpy as np

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

POINTS_PER_SCAN = 101                    # safe on all NanoVNA firmware
NANOVNA_USB_IDS = {(0x0483, 0x5740),    # NanoVNA-H / H4 (STM32)
                   (0x04B4, 0x0008)}    # NanoVNA V2 / SAA-2 / LiteVNA
C0 = 299_792_458.0


class SweepAborted(Exception):
    pass


def list_serial_ports():
    """[(device, description, vid, pid)] for all serial ports."""
    if serial is None:
        return []
    return [(p.device, p.description or "", p.vid, p.pid) for p in serial.tools.list_ports.comports()]


class _SweepMixin:
    """Full sweep = several 'scan' commands (<= POINTS_PER_SCAN points each), averaged."""

    def sweep(self, start_hz: float, stop_hz: float, points: int, averages: int = 1,
              progress: Optional[Callable[[float], None]] = None, abort=None):
        grid = np.linspace(start_hz, stop_hz, points)
        n_chunks = max(1, math.ceil(points / POINTS_PER_SCAN))
        chunks = np.array_split(np.arange(points), n_chunks)
        f_out = np.empty(points)
        s11 = np.zeros(points, dtype=complex)
        s21 = np.zeros(points, dtype=complex)
        total, done = n_chunks * averages, 0
        try:
            for idx in chunks:
                i0, i1 = idx[0], idx[-1] + 1
                acc11 = np.zeros(i1 - i0, dtype=complex)
                acc21 = np.zeros(i1 - i0, dtype=complex)
                for _ in range(averages):
                    if abort is not None and abort.is_set():
                        raise SweepAborted()
                    f, a11, a21 = self.scan(grid[i0], grid[i1 - 1], i1 - i0)
                    acc11 += a11
                    acc21 += a21
                    done += 1
                    if progress:
                        progress(done / total)
                f_out[i0:i1] = f
                s11[i0:i1] = acc11 / averages
                s21[i0:i1] = acc21 / averages
        finally:
            self.resume()
        return f_out, s11, s21


class NanoVNA(_SweepMixin):
    """NanoVNA over its USB virtual serial port (text shell with a 'ch> ' prompt)."""

    PROMPT = b"ch> "

    def __init__(self, port: str, timeout: float = 15.0):
        if serial is None:
            raise RuntimeError("pyserial is not installed (pip install pyserial)")
        self.port = port
        self.ser = serial.Serial(port, 115200, timeout=3)
        time.sleep(0.1)
        self._sync()
        self.ser.timeout = timeout
        ver = self.command("version")
        self.version = ver[0] if ver else "unknown firmware"

    @staticmethod
    def looks_like_nanovna(vid, pid) -> bool:
        return (vid, pid) in NANOVNA_USB_IDS

    def _read_prompt(self) -> bytes:
        data = self.ser.read_until(self.PROMPT)
        if not data.endswith(self.PROMPT):
            raise TimeoutError("No response from the NanoVNA (timed out waiting for 'ch>' prompt).")
        return data[: -len(self.PROMPT)]

    def _sync(self):
        self.ser.reset_input_buffer()
        self.ser.write(b"\r")
        self._read_prompt()
        self.ser.reset_input_buffer()

    def command(self, cmd: str) -> list[str]:
        self.ser.write((cmd + "\r").encode("ascii"))
        raw = self._read_prompt().decode("ascii", errors="replace").replace("\r", "")
        lines = [ln.strip() for ln in raw.split("\n")]
        if lines and lines[0] == cmd:
            lines = lines[1:]
        return [ln for ln in lines if ln]

    def scan(self, start_hz: float, stop_hz: float, points: int):
        cmd = f"scan {int(round(start_hz))} {int(round(stop_hz))} {int(points)} 7"
        lines = self.command(cmd)
        try:
            arr = np.array([[float(v) for v in ln.split()] for ln in lines])
        except ValueError:
            raise RuntimeError(f"Unexpected reply to '{cmd}': {lines[:3]}")
        if arr.shape != (points, 5):
            raise RuntimeError(f"Expected {points} rows of 5 values from '{cmd}', got {arr.shape}. "
                               f"First lines: {lines[:3]}")
        return arr[:, 0], arr[:, 1] + 1j * arr[:, 2], arr[:, 3] + 1j * arr[:, 4]

    def resume(self):
        try:
            self.command("resume")
        except Exception:
            pass

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class NotNanoVNAV2(Exception):
    pass


def _read_exact(ser, n: int, timeout: float) -> bytes:
    n = int(n)          # must be a plain int: Windows pyserial rejects numpy ints (TypeError)
    buf = bytearray()
    t_end = time.time() + timeout
    while len(buf) < n and time.time() < t_end:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf += chunk
    return bytes(buf)


DEBUG_LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vna_debug_log.txt")


def dbg(msg: str, new: bool = False):
    """Append a line to vna_debug_log.txt (next to this script) for troubleshooting."""
    try:
        with open(DEBUG_LOG, "w" if new else "a", encoding="utf-8") as fh:
            fh.write(f"{dt.datetime.now():%H:%M:%S.%f}"[:-3] + "  " + msg + "\n")
    except Exception:
        pass


def _hx(b: bytes, limit: int = 64) -> str:
    h = " ".join(f"{x:02x}" for x in b[:limit])
    return h + (f" ... ({len(b)} bytes total)" if len(b) > limit else f"  ({len(b)} bytes)")


class NanoVNAV2(_SweepMixin):
    """NanoVNA V2 / SAA-2 / V2 Plus4 / LiteVNA over USB (binary protocol).

    Follows the same sequence as NanoVNA-Saver: 0x00 NOP, 0x0D INDICATE (device replies '2'),
    0x10 AA = read 1-byte register, 0x21/0x23 AA <LE value> = write 2/8-byte register,
    0x20 30 00 = clear data FIFO, 0x18 30 NN = read NN records of 32 bytes:
    fwd0 re/im, rev0 re/im, rev1 re/im (int32) + freq index (uint16) + 6 spare bytes.
    S11 = rev0/fwd0, S21 = rev1/fwd0.  NOTE: data is UNCALIBRATED (the device does not
    apply its own calibration to USB data). SE = S21_ref / S21_sample cancels this.
    Every transaction is written to vna_debug_log.txt.
    """

    WRITE_SLEEP = 0.05
    SEC_PER_POINT = 0.035          # NanoVNA-Saver: ~3.6 s for 101 points

    def __init__(self, port: str):
        if serial is None:
            raise RuntimeError("pyserial is not installed (pip install pyserial)")
        self.port = port
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        try:
            time.sleep(0.1)
            if not self.detect(self.ser):
                raise NotNanoVNAV2()
            dbg(f"connected to {port}: NanoVNA V2 protocol detected (INDICATE -> '2')", new=True)
            hw = self._read1(0xF2)
            fw = (self._read1(0xF3) or 0, self._read1(0xF4) or 0)
            # NanoVNA-Saver shows the firmware as major.0.minor and, for firmware <= 1.0.1,
            # applies its "S21 hack": the first point of every sweep is unreliable, so it sweeps
            # one extra point below the start frequency and discards it. Do exactly the same.
            self.s21_hack = fw <= (1, 1)
            self.version = (f"NanoVNA V2 / SAA-2, HW rev {hw}, FW {fw[0]}.0.{fw[1]}"
                            + (" (first-point fix on)" if self.s21_hack else ""))
            dbg(self.version)
        except Exception:
            self.ser.close()
            raise

    @staticmethod
    def detect(ser) -> bool:
        ser.reset_input_buffer()
        ser.write(b"\x00" * 8)              # NOPs: reset the device's command parser
        time.sleep(0.05)
        ser.reset_input_buffer()
        ser.write(b"\x0d")                  # INDICATE
        r = _read_exact(ser, 1, 1.0)
        time.sleep(0.1)
        ser.reset_input_buffer()
        return r == b"2"

    def _w(self, data: bytes, what: str):
        dbg(f">> {_hx(data)}  {what}")
        self.ser.write(data)
        time.sleep(self.WRITE_SLEEP)

    def _read1(self, addr: int):
        self._w(bytes([0x10, addr]), f"READ reg {addr:02X}")
        d = _read_exact(self.ser, 1, 1.0)
        dbg(f"<< {_hx(d)}")
        return d[0] if d else None

    def scan(self, start_hz: float, stop_hz: float, points: int):
        want = int(points)
        start = int(round(start_hz))
        step = int(round((stop_hz - start_hz) / (want - 1))) if want > 1 else 0
        hack = 1 if getattr(self, "s21_hack", False) else 0
        points = want + hack                                    # extra first point, discarded below
        dev_start = max(50_000, start - step * hack)
        dbg(f"--- scan start={start} step={step} points={want}" + (" (+1 discarded first point)" if hack else ""))
        self._w(struct.pack("<BBQ", 0x23, 0x00, dev_start) + struct.pack("<BBQ", 0x23, 0x10, step) +
                struct.pack("<BBH", 0x21, 0x20, points) + struct.pack("<BBH", 0x21, 0x22, 1),
                "WRITE sweep start/step/points/valuesPerFreq")
        self.ser.reset_input_buffer()
        self._w(b"\x00" * 8, "8 x NOP (reset parser)")
        self._w(struct.pack("<BBB", 0x20, 0x30, 0), "clear FIFO")
        got = {}
        t_end = time.time() + 10.0 + points * 0.2
        while len(got) < points:
            if time.time() > t_end:
                raise TimeoutError(f"NanoVNA V2: only {len(got)} of {points} points received "
                                   f"(details in vna_debug_log.txt)")
            cnt = min(255, points - len(got))
            self._w(struct.pack("<BBB", 0x18, 0x30, cnt), f"READFIFO {cnt}")
            nbytes = cnt * 32
            d = _read_exact(self.ser, nbytes, cnt * self.SEC_PER_POINT * 2 + 2.0)
            dbg(f"<< {_hx(d)}")
            if len(d) < nbytes:                                 # partial read: resync
                dbg(f"partial read {len(d)}/{nbytes} bytes - resync")
                time.sleep(0.2)
                self.ser.reset_input_buffer()
            idxs = []
            for i in range(len(d) // 32):
                v = struct.unpack_from("<iiiiiih", d, i * 32)
                idxs.append(v[6])
                if 0 <= v[6] < points:
                    got[v[6]] = v
            dbg(f"freq indices in this block: {idxs[:12]}{' ...' if len(idxs) > 12 else ''}; "
                f"have {len(got)}/{points}")
        rec = np.array([got[i][:6] for i in range(hack, points)], dtype=float)
        fwd = rec[:, 0] + 1j * rec[:, 1]
        fwd = np.where(np.abs(fwd) == 0, 1e-12, fwd)
        s11 = (rec[:, 2] + 1j * rec[:, 3]) / fwd
        s21 = (rec[:, 4] + 1j * rec[:, 5]) / fwd
        f = start + step * np.arange(want, dtype=float)
        dbg(f"scan OK: |S21| first/last = {20 * np.log10(abs(s21[0]) + 1e-15):.2f} / "
            f"{20 * np.log10(abs(s21[-1]) + 1e-15):.2f} dB")
        return f, s11, s21

    def resume(self):
        pass                                # the V2 keeps sweeping on its own

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def connect_vna(port: str):
    """Auto-detect: NanoVNA V2 / SAA-2 (binary protocol) first, else NanoVNA-H / H4 (text shell)."""
    try:
        return NanoVNAV2(port)
    except NotNanoVNAV2:
        pass
    return NanoVNA(port)


class SimulatedFreeSpaceVNA(_SweepMixin):
    """Fake NanoVNA + two antennas in free space, for demo mode.

    get_state() must return a dict with dist_a, dist_b (mm), fixture
    ("reference" | "blocked" | "sample") and sample (name). The simulated sample's
    true SE depends on its name ('copper', 'alumin...', 'mesh', anything else =
    composite). Leakage around the finite sample grows with antenna distance, so
    the measured SE falls at larger distances, as in a real free-space setup.
    """

    version = "Simulated NanoVNA-H4 (free space)"
    port = "demo"

    def __init__(self, get_state: Callable[[], dict]):
        self.get_state = get_state
        self.rng = np.random.default_rng()

    @staticmethod
    def _true_se(name: str, f):
        n = name.lower()
        if "copper" in n or re.search(r"\bcu\b", n):
            return np.full_like(f, 110.0)
        if "alumin" in n:
            return np.full_like(f, 95.0)
        if "mesh" in n:
            return np.clip(20 * np.log10(C0 / (2 * f * 3e-3)), 0, 90)
        return 20 + 18 * np.log10(1 + f / 1e8) / np.log10(16)

    def scan(self, start_hz, stop_hz, points):
        time.sleep(0.02)
        st = self.get_state()
        f = np.linspace(start_hz, stop_hz, points)
        da, db = st["dist_a"] / 1000.0, st["dist_b"] / 1000.0
        d = max(da + db, 0.02)
        lam = C0 / f
        gain_db = np.clip(6 + 6 * np.log2(f / 1e9), -12, 10)            # per antenna
        los = (lam / (4 * np.pi * d)) * 10 ** (2 * gain_db / 20) * np.exp(-2j * np.pi * f * d / C0)
        room = 0.0012 * np.exp(-2j * np.pi * f * (d + 1.6) / C0) * 10 ** (gain_db / 20)  # wall reflection
        # native range: floor rises with frequency; above 1.5 GHz the H4 runs on harmonics
        floor_db = np.where(f <= 1.5e9, -100 + 25 * np.minimum(f / 1.5e9, 1) ** 1.3,
                            -68 + 8 * (f / 1e9 - 1.5))
        noise = (self.rng.normal(size=points) + 1j * self.rng.normal(size=points)) / np.sqrt(2)
        noise *= 10 ** (floor_db / 20)
        dm_mm = 1000 * (da + db) / 2
        leak = 10 ** (-(64 - 20 * np.log10(max(dm_mm, 10) / 100.0)) / 20)   # diffraction round the sample
        fx = st.get("fixture", "reference")
        s11 = 0.25 * np.exp(-1j * f * 3e-9)                                  # antenna mismatch
        if fx == "reference":
            s21 = los + room
        elif fx == "blocked":
            s21 = los * leak + room
            s11 = s11 + 0.8 * (lam / (4 * np.pi * 2 * da)) * 10 ** (gain_db / 10) * np.exp(-4j * np.pi * f * da / C0)
        else:
            se = self._true_se(st.get("sample", ""), f)
            s21 = los * (10 ** (-se / 20) + leak) + room
            s11 = s11 + 0.6 * (lam / (4 * np.pi * 2 * da)) * 10 ** (gain_db / 10) * np.exp(-4j * np.pi * f * da / C0)
        s11 = s11 + 0.002 * (self.rng.normal(size=points) + 1j * self.rng.normal(size=points))
        return f, s11, s21 + noise

    def resume(self):
        pass

    def close(self):
        pass
