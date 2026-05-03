#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-${ROOT_DIR}/../RoboCorpus/.venv/bin/python}"
PORT="${PORT:-7777}"

adb reverse "tcp:${PORT}" "tcp:${PORT}"

cd "${ROOT_DIR}"
exec env \
  PYTHONUNBUFFERED=1 \
  NUMBA_DISABLE_JIT=1 \
  "${PYTHON_BIN}" \
  examples/robomimic_stereo_video.py \
  --host 127.0.0.1 \
  --port "${PORT}" \
  --env-name ToolHang \
  --projection-mode crop \
  --codec h264 \
  --teleop-arm \
  --head-tracking
