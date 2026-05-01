#!/usr/bin/env bash
# Build, upload, and capture serial output for a fixed window. The serial
# port is held open by `pio run -t upload`, so we wait briefly for it to
# be released before the logger opens it.
#
# Usage:
#     scripts/flash_and_log.sh                       # default env, 10s
#     scripts/flash_and_log.sh smoke_imu             # named env, 10s
#     scripts/flash_and_log.sh smoke_imu 5           # named env, 5s
#
# When the env is omitted, PlatformIO uses default_envs from platformio.ini.
set -euo pipefail

ENV_NAME="${1:-}"
SECONDS_TO_LOG="${2:-10}"
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PIO="$HOME/.platformio/penv/bin/pio"
PY="$HOME/.platformio/penv/bin/python"

cd "$REPO_ROOT"
if [[ -n "$ENV_NAME" ]]; then
  "$PIO" run -e "$ENV_NAME" -t upload
else
  "$PIO" run -t upload
fi
sleep 1
exec "$PY" "$REPO_ROOT/scripts/log_serial.py" "$SECONDS_TO_LOG"
