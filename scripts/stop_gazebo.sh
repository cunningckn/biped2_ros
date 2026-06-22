#!/usr/bin/env bash
set -euo pipefail

echo "Stopping Gazebo processes..."
pkill -x gzserver 2>/dev/null || true
pkill -x gzclient 2>/dev/null || true
sleep 1

if pgrep -x gzserver >/dev/null; then
  echo "Force killing remaining gzserver..."
  pkill -9 -x gzserver 2>/dev/null || true
fi

if pgrep -x gzclient >/dev/null; then
  echo "Force killing remaining gzclient..."
  pkill -9 -x gzclient 2>/dev/null || true
fi

echo "Done."
