#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-7777}"

adb devices
adb reverse "tcp:${PORT}" "tcp:${PORT}"
echo "adb reverse tcp:${PORT} tcp:${PORT} configured"
