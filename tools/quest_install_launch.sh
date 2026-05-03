#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APK="${ROOT_DIR}/quest_client/app/build/outputs/apk/debug/app-debug.apk"
PORT="${1:-7777}"

if [[ ! -f "${APK}" ]]; then
  echo "APK not found: ${APK}" >&2
  echo "Build it first with Android Studio or ./gradlew assembleDebug inside quest_client." >&2
  exit 1
fi

adb devices
adb reverse "tcp:${PORT}" "tcp:${PORT}"
adb install -r "${APK}"
adb shell am start -n com.yuchen.robovr/.VrActivity
echo "launched RoboVR Quest client"
