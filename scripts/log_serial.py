#!/usr/bin/env python3
"""Log serial output for a fixed duration to a file (and stdout).

Reads monitor_port / monitor_speed from platformio.ini so it stays in sync
with the build config. Intended for short post-flash captures (boot banner,
sensor smoke tests) that Claude can read back.

By default, pulses the ESP32 auto-reset circuit (RTS=EN, DTR=IO0) on port
open so every capture starts from a fresh boot — otherwise sketches whose
output lives entirely in setup() are silent by the time we connect.

Usage:
    scripts/log_serial.py [seconds] [-o output_file] [-p port] [-b baud]
                          [--no-reset]

Defaults:
    seconds      10
    output_file  logs/serial-YYYYMMDD-HHMMSS.log
"""
from __future__ import annotations

import argparse
import configparser
import datetime as _dt
import pathlib
import sys
import time

import serial

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
PIO_INI = REPO_ROOT / "platformio.ini"
LOG_DIR = REPO_ROOT / "logs"


def read_pio_serial() -> tuple[str, int]:
    """Find monitor_port/speed. Per-env overrides win over the shared [env]."""
    cfg = configparser.ConfigParser()
    cfg.read(PIO_INI)
    candidates = [s for s in cfg.sections() if s.startswith("env:")] + (
        ["env"] if cfg.has_section("env") else []
    )
    for section in candidates:
        port = cfg.get(section, "monitor_port", fallback=None)
        if port:
            speed = cfg.getint(section, "monitor_speed", fallback=115200)
            return port, speed
    raise SystemExit(f"no monitor_port in {PIO_INI}")


def main() -> int:
    default_port, default_baud = read_pio_serial()

    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("seconds", nargs="?", type=float, default=10.0,
                   help="capture duration in seconds (default: 10)")
    p.add_argument("-o", "--output", type=pathlib.Path, default=None,
                   help="output file (default: logs/serial-<timestamp>.log)")
    p.add_argument("-p", "--port", default=default_port,
                   help=f"serial port (default from platformio.ini: {default_port})")
    p.add_argument("-b", "--baud", type=int, default=default_baud,
                   help=f"baud rate (default from platformio.ini: {default_baud})")
    p.add_argument("--no-reset", action="store_true",
                   help="don't pulse DTR/RTS on open; observe whatever the chip is already doing")
    args = p.parse_args()

    if args.output is None:
        LOG_DIR.mkdir(exist_ok=True)
        stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        args.output = LOG_DIR / f"serial-{stamp}.log"
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)

    print(f"[log_serial] {args.port} @ {args.baud} for {args.seconds}s -> {args.output}",
          file=sys.stderr)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"[log_serial] open failed: {e}", file=sys.stderr)
        return 1

    if not args.no_reset:
        # Standard ESP32 dev-board auto-reset: RTS asserted pulls EN low,
        # DTR asserted pulls IO0 low. We want to reset into RUN mode, so
        # hold IO0 high (dtr=False) while pulsing EN (rts True->False).
        ser.dtr = False
        ser.rts = True
        time.sleep(0.1)
        ser.rts = False
        ser.reset_input_buffer()  # discard noise from the line transitions

    deadline = time.monotonic() + args.seconds
    bytes_total = 0
    with ser, open(args.output, "wb") as out:
        while time.monotonic() < deadline:
            chunk = ser.read(4096)
            if chunk:
                out.write(chunk)
                out.flush()
                sys.stdout.buffer.write(chunk)
                sys.stdout.buffer.flush()
                bytes_total += len(chunk)

    print(f"\n[log_serial] captured {bytes_total} bytes -> {args.output}",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
