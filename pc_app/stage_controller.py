"""
Talks to our ESP32 antenna positioner firmware (src/main.cpp) over USB serial,
plus a simulated stage for demo mode. Both have the same methods.

The firmware takes the same plain text commands you type in the Serial Monitor
(move <cm>, home, zero, reset, stop, f <cm>, b <cm>, env ...). The app polls
'status', which the firmware answers with one line:

    STATUS pos=5.00 busy=0 killed=0 t=24.3 rh=56.1
"""
from __future__ import annotations

import threading
import time

try:
    import serial
except ImportError:
    serial = None

MAX_CM = 20.0          # same travel limit as the firmware
NAN = float("nan")


class StageError(Exception):
    pass


class StageAborted(Exception):
    pass


def parse_status(line: str) -> dict:
    """'STATUS pos=5.00 busy=0 killed=0 t=24.3 rh=56.1' -> dict."""
    d = {}
    for part in line.split()[1:]:
        key, _, val = part.partition("=")
        try:
            d[key] = float(val)          # 'nan' (sensor error) becomes NaN
        except ValueError:
            d[key] = NAN
    return {"pos_cm": d.get("pos", NAN), "busy": bool(d.get("busy", 0)),
            "killed": bool(d.get("killed", 0)), "t": d.get("t", NAN), "rh": d.get("rh", NAN)}


class _Common:
    def status(self) -> dict:
        for line in self.send("status"):
            if line.startswith("STATUS"):
                return parse_status(line)
        raise StageError("No status reply from the ESP32")

    def env(self):
        st = self.status()
        return st["t"], st["rh"]

    def move_to(self, cm: float):
        if not 0 <= cm <= MAX_CM:
            raise StageError(f"Distance must be 0 to {MAX_CM:g} cm")
        self.send(f"move {cm:.2f}")

    def home(self):
        self.send("home")

    def stop(self):
        self.send("stop")

    def wait_idle(self, abort=None, timeout: float = 120.0, on_status=None) -> dict:
        """Poll status until the motor has stopped. Raises on abort, kill or timeout."""
        t_end = time.time() + timeout
        while True:
            if abort is not None and abort.is_set():
                self.stop()
                raise StageAborted()
            st = self.status()
            if on_status:
                on_status(st)
            if st["killed"]:
                raise StageError("Kill button pressed - type 'reset' in the command box")
            if not st["busy"]:
                return st
            if time.time() > t_end:
                self.stop()
                raise StageError("Timed out waiting for the antenna to stop")
            time.sleep(0.25)


class StageController(_Common):
    def __init__(self, port: str, on_line=None):
        if serial is None:
            raise RuntimeError("pyserial is not installed (pip install pyserial)")
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        self.port = port
        self.on_line = on_line           # called with every reply line except STATUS
        self.lock = threading.Lock()
        time.sleep(2.5)                  # the ESP32 reboots when the port opens (firmware waits 2 s)
        self.status()                    # check the firmware answers
        self.fw = "Antenna Positioner firmware"

    def send(self, cmd: str, timeout: float = 1.0) -> list[str]:
        """Send one command and return the reply lines."""
        lines = []
        with self.lock:
            self.ser.write((cmd + "\n").encode("ascii"))
            t_end = time.time() + timeout
            while time.time() < t_end:
                raw = self.ser.readline().decode("ascii", errors="replace").strip()
                if raw:
                    lines.append(raw)
                    if raw.startswith("STATUS"):
                        break
                elif lines:              # reply finished (no more lines)
                    break
        for ln in lines:
            if self.on_line and not ln.startswith("STATUS"):
                self.on_line(ln)
        return lines

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class SimulatedStage(_Common):
    """Behaves like the firmware: same commands and replies, 3 cm/s, kill via 'kill'."""

    fw = "Simulated antenna positioner"
    port = "demo"
    SPEED = 3.0                         # cm/s

    def __init__(self, on_line=None):
        self.on_line = on_line
        self.lock = threading.Lock()
        self.p0 = self.p1 = 0.0          # move from p0 to p1 cm
        self.t0 = self.t1 = time.time()
        self.killed = False
        self.t, self.rh = 24.5, 55.0

    def _pos(self) -> float:
        now = time.time()
        if now >= self.t1:
            return self.p1
        return self.p0 + (self.p1 - self.p0) * (now - self.t0) / (self.t1 - self.t0)

    def _go(self, target: float):
        cur = self._pos()
        self.p0, self.p1 = cur, target
        self.t0, self.t1 = time.time(), time.time() + abs(target - cur) / self.SPEED

    def _reply(self, cmd: str) -> str:
        parts = cmd.split()
        name = parts[0] if parts else ""
        try:
            arg = float(parts[1]) if len(parts) > 1 else 0.0
        except ValueError:
            arg = 0.0                     # like the firmware's toFloat()
        if name in ("f", "b", "move", "home") and self.killed:
            return "Killed - type 'reset' first."
        if name in ("f", "b"):
            self._go(self._pos() + (arg if name == "f" else -arg))
            return f"Moving {arg if name == 'f' else -arg:.2f} cm"
        if name == "move":
            if not 0 <= arg <= MAX_CM:
                return f"Error: move must be 0 to {MAX_CM:.0f} cm"
            self._go(arg)
            return f"Moving to {arg:.2f} cm"
        if name == "home":
            self._go(0.0)
            return "Going to 0 cm"
        if name == "stop":
            self._go(self._pos())
            return "Stopping"
        if name == "zero":
            self.p0 = self.p1 = 0.0
            self.t1 = time.time()
            return "Position set to 0 cm"
        if name == "pos":
            return f"Position: {self._pos():.2f} cm"
        if name == "env":
            return f"Temperature: {self.t:.1f} C   Humidity: {self.rh:.1f} %"
        if name == "kill":                # simulates pressing the kill button
            self._go(self._pos())
            self.killed = True
            return "KILL pressed - motor stopped. Type 'reset' to continue."
        if name == "reset":
            self.killed = False
            return "Reset - driver enabled"
        if name == "status":
            busy = int(time.time() < self.t1)
            return (f"STATUS pos={self._pos():.2f} busy={busy} killed={int(self.killed)} "
                    f"t={self.t:.1f} rh={self.rh:.1f}")
        return "Commands: f <cm>, b <cm>, move <cm>, stop, pos, zero, home, env, status, reset"

    def send(self, cmd: str, timeout: float = 1.0) -> list[str]:
        with self.lock:
            line = self._reply(cmd.strip())
        if self.on_line and not line.startswith("STATUS"):
            self.on_line(line)
        return [line]

    def close(self):
        pass
