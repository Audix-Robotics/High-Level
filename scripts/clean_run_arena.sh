#!/usr/bin/env bash
set -euo pipefail

echo "[clean_run_arena] Killing known lingering processes..."
# common simulators and GUIs
pkill -f "gz sim" || true
pkill -f "gz_sim" || true
pkill -f "gz" || true
pkill -f rviz2 || true

# ros_gz_bridge parameter_bridge
pkill -f parameter_bridge || true

# installed audix python nodes
pkill -f "/install/audix/lib/audix" || true

# other common node names
pkill -f arena_roamer.py || true
pkill -f arena_obstacle_manager.py || true
pkill -f arena_spawn_panel.py || true
pkill -f mecanum_kinematics.py || true
pkill -f odom_tf_broadcaster.py || true
pkill -f scissor_lift_mapper.py || true

sleep 1

echo "[clean_run_arena] Sourcing workspace..."
if [ -f "install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "install/setup.bash"
else
  echo "[clean_run_arena] Warning: install/setup.bash not found. Make sure you built the workspace." >&2
fi

LAUNCH_PATH="$PWD/src/audix_pkg/launch/arena_experiment.launch.py"
if [ ! -f "$LAUNCH_PATH" ]; then
  echo "[clean_run_arena] Launch file not found: $LAUNCH_PATH" >&2
  exit 1
fi

echo "[clean_run_arena] Starting fresh launch: $LAUNCH_PATH"
exec ros2 launch "$LAUNCH_PATH"
