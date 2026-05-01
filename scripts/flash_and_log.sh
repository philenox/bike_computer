#!/usr/bin/env bash
# Build, upload, and capture serial output. The serial port is held open by
# `pio run -t upload`, so we wait briefly for it to be released before
# starting the logger.
#
# Usage: scripts/flash_and_log.sh [seconds]
set -euo pipefail

SECONDS_TO_LOG="${1:-10}"
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PIO="$HOME/.platformio/penv/bin/pio"
PY="$HOME/.platformio/penv/bin/python"

cd "$REPO_ROOT"
"$PIO" run -t upload
sleep 1
exec "$PY" "$REPO_ROOT/scripts/log_serial.py" "$SECONDS_TO_LOG"
