#!/usr/bin/env python3
"""Real-time magnetometer calibration coverage visualizer.

Reads 'raw: mx my mz' lines from smoke_mag_cal's serial output and
projects each sample onto a unit sphere so coverage gaps are obvious.

  blue dots = prior coverage
  red  dots = last 3 seconds (where you just moved the device)

Rotate until the sphere is fully covered with blue dots.
Stop when each axis span shows ✓ in the stats box.

Usage:
    python scripts/mag_cal_viz.py [-p PORT] [-b BAUD] [--no-reset]

Port/baud default to platformio.ini's monitor_port/monitor_speed.
Requires: pip install pyserial matplotlib numpy
"""
from __future__ import annotations

import argparse
import configparser
import math
import pathlib
import sys
import threading
import time
from collections import deque

try:
    import numpy as np
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  registers 3d projection
except ImportError as exc:
    sys.exit(f"Missing dependency: {exc}\n  pip install matplotlib numpy")

try:
    import serial
except ImportError:
    sys.exit("Missing dependency: pyserial\n  pip install pyserial")

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
PIO_INI   = REPO_ROOT / "platformio.ini"

MAX_POINTS    = 6000   # ring buffer; 10 Hz × 10 min
RECENT_SECS   = 3.0    # dots younger than this show as red
RENDER_PERIOD = 0.5    # seconds between redraws
TARGET_SPAN   = 50.0   # µT — minimum acceptable axis span


def read_pio_serial() -> tuple[str, int]:
    cfg = configparser.ConfigParser()
    cfg.read(PIO_INI)
    candidates = [s for s in cfg.sections() if s.startswith("env:")]
    if cfg.has_section("env"):
        candidates.append("env")
    for section in candidates:
        port = cfg.get(section, "monitor_port", fallback=None)
        if port:
            return port, cfg.getint(section, "monitor_speed", fallback=115200)
    raise SystemExit(f"no monitor_port in {PIO_INI}")


class MagStream:
    """Thread-safe accumulator of raw mag samples with local min/max tracking."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._pts: deque[tuple[float, float, float, float]] = deque(maxlen=MAX_POINTS)
        self._xmin = self._xmax = None
        self._ymin = self._ymax = None
        self._zmin = self._zmax = None
        self.n     = 0
        self.alive = True

    def push(self, mx: float, my: float, mz: float) -> None:
        t = time.monotonic()
        with self._lock:
            self._pts.append((mx, my, mz, t))
            if self._xmin is None:
                self._xmin = self._xmax = mx
                self._ymin = self._ymax = my
                self._zmin = self._zmax = mz
            else:
                if mx < self._xmin: self._xmin = mx
                if mx > self._xmax: self._xmax = mx
                if my < self._ymin: self._ymin = my
                if my > self._ymax: self._ymax = my
                if mz < self._zmin: self._zmin = mz
                if mz > self._zmax: self._zmax = mz
            self.n += 1

    def snapshot(self):
        with self._lock:
            pts  = list(self._pts)
            if self._xmin is not None:
                ox = (self._xmin + self._xmax) / 2
                oy = (self._ymin + self._ymax) / 2
                oz = (self._zmin + self._zmax) / 2
                sx = self._xmax - self._xmin
                sy = self._ymax - self._ymin
                sz = self._zmax - self._zmin
            else:
                ox = oy = oz = sx = sy = sz = 0.0
            return pts, (ox, oy, oz), (sx, sy, sz), self.n


def reader_thread(ser: serial.Serial, stream: MagStream) -> None:
    while stream.alive:
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            if line.startswith("raw:"):
                parts = line[4:].split()
                if len(parts) == 3:
                    try:
                        stream.push(float(parts[0]), float(parts[1]), float(parts[2]))
                    except ValueError:
                        pass
        except serial.SerialException:
            break


def _sphere_wireframe():
    u = np.linspace(0, 2 * np.pi, 25)
    v = np.linspace(0, np.pi, 13)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones_like(u), np.cos(v))
    return x, y, z


_SPHERE_WF = _sphere_wireframe()


def render(ax, stats_text, pts, offset, span, n):
    ax.cla()
    ax.plot_wireframe(*_SPHERE_WF, alpha=0.08, color="steelblue", linewidth=0.5)
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlim(-1.3, 1.3)
    ax.set_ylim(-1.3, 1.3)
    ax.set_zlim(-1.3, 1.3)
    ax.set_xlabel("X", fontsize=8)
    ax.set_ylabel("Y", fontsize=8)
    ax.set_zlabel("Z", fontsize=8)
    ax.tick_params(labelsize=6)

    field_mean = 0.0
    field_std  = 0.0
    if pts:
        now = time.monotonic()
        ox, oy, oz = offset
        old_x, old_y, old_z = [], [], []
        new_x, new_y, new_z = [], [], []
        rs = []

        for mx, my, mz, ts in pts:
            cx, cy, cz = mx - ox, my - oy, mz - oz
            r = math.sqrt(cx * cx + cy * cy + cz * cz)
            if r < 1.0:
                continue
            rs.append(r)
            ux, uy, uz = cx / r, cy / r, cz / r
            if now - ts < RECENT_SECS:
                new_x.append(ux); new_y.append(uy); new_z.append(uz)
            else:
                old_x.append(ux); old_y.append(uy); old_z.append(uz)

        if old_x:
            ax.scatter(old_x, old_y, old_z,
                       c="steelblue", s=4, alpha=0.5, linewidths=0)
        if new_x:
            ax.scatter(new_x, new_y, new_z,
                       c="red", s=14, alpha=0.9, linewidths=0)

        if rs:
            field_mean = sum(rs) / len(rs)
            if len(rs) > 1:
                var = sum((r - field_mean) ** 2 for r in rs) / (len(rs) - 1)
                field_std = math.sqrt(var)

    sx, sy, sz = span
    ox, oy, oz = offset

    def fmt(v: float) -> str:
        if v >= TARGET_SPAN:
            return f"{v:5.1f} µT  ✓"
        return f"{v:5.1f} µT  need +{TARGET_SPAN - v:.0f}"

    # Earth's field at any point on the surface is essentially constant
    # in magnitude (~25-65 µT depending on latitude). After a clean
    # hard-iron calibration, |m_centred| should be near-constant for
    # every sample. A wide stddev means residual offset error or stray
    # local fields — re-cal away from electronics.
    field_mark = "✓" if (field_std < 5.0 and field_mean > 1.0) else " "
    stats_text.set_text(
        f"  samples : {n}\n"
        f"  offset  : X={ox:+.1f}  Y={oy:+.1f}  Z={oz:+.1f}  µT\n"
        f"  span X  : {fmt(sx)}\n"
        f"  span Y  : {fmt(sy)}\n"
        f"  span Z  : {fmt(sz)}\n"
        f"  |m|     : {field_mean:5.1f} ± {field_std:4.1f} µT  {field_mark}\n"
        f"  targets : spans ≥ {TARGET_SPAN:.0f},  |m| stddev < 5 µT"
    )


def main() -> int:
    default_port, default_baud = read_pio_serial()
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("-p", "--port",  default=default_port,
                   help=f"serial port (default: {default_port})")
    p.add_argument("-b", "--baud",  type=int, default=default_baud,
                   help=f"baud rate (default: {default_baud})")
    p.add_argument("--no-reset",    action="store_true",
                   help="skip the DTR/RTS reset pulse on port open")
    args = p.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"[mag_cal_viz] cannot open {args.port}: {e}")

    if not args.no_reset:
        ser.dtr = False
        ser.rts = True
        time.sleep(0.1)
        ser.rts = False
        ser.reset_input_buffer()

    stream = MagStream()
    thr = threading.Thread(target=reader_thread, args=(ser, stream), daemon=True)
    thr.start()

    plt.ion()
    fig = plt.figure(figsize=(9, 7))
    fig.suptitle(
        "Mag cal coverage  —  rotate until the sphere is fully dotted",
        fontsize=10,
    )
    ax = fig.add_subplot(111, projection="3d")
    stats_text = fig.text(
        0.01, 0.01, "",
        family="monospace", fontsize=8,
        verticalalignment="bottom",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.85),
    )

    try:
        while plt.fignum_exists(fig.number):
            pts, offset, span, n = stream.snapshot()
            render(ax, stats_text, pts, offset, span, n)
            plt.pause(RENDER_PERIOD)
    except KeyboardInterrupt:
        pass
    finally:
        stream.alive = False
        ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
