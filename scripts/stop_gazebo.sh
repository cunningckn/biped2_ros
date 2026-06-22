#!/usr/bin/env bash
set -euo pipefail

echo "Stopping Gazebo and biped2 demo processes..."
pkill -x gzserver 2>/dev/null || true
pkill -x gzclient 2>/dev/null || true
pkill -f '/biped2_kinematics/foot_lift_demo' 2>/dev/null || true
pkill -f '/biped2_kinematics/foot_fk_publisher' 2>/dev/null || true
sleep 1

if pgrep -x gzserver >/dev/null; then
  echo "Force killing remaining gzserver..."
  pkill -9 -x gzserver 2>/dev/null || true
fi

if pgrep -x gzclient >/dev/null; then
  echo "Force killing remaining gzclient..."
  pkill -9 -x gzclient 2>/dev/null || true
fi

if pgrep -f '/biped2_kinematics/foot_lift_demo' >/dev/null; then
  echo "Force killing remaining foot_lift_demo..."
  pkill -9 -f '/biped2_kinematics/foot_lift_demo' 2>/dev/null || true
fi

if pgrep -f '/biped2_kinematics/foot_fk_publisher' >/dev/null; then
  echo "Force killing remaining foot_fk_publisher..."
  pkill -9 -f '/biped2_kinematics/foot_fk_publisher' 2>/dev/null || true
fi

echo "Done."
