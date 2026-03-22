#!/usr/bin/env bash
set -euo pipefail

# Wrapper to ensure a single clean Gazebo+ROS2+RViz session.
# Usage: ./scripts/clean_launch_arena.sh

BASE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$BASE_DIR"
# Provide safe defaults for environment variables that install/setup.bash expects
# to avoid "unbound variable" errors when running with `set -u`.
export COLCON_TRACE=${COLCON_TRACE:-0}
export AMENT_TRACE=${AMENT_TRACE:-}
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-}
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-python3}
export COLCON_PYTHON_EXECUTABLE=${COLCON_PYTHON_EXECUTABLE:-/usr/bin/python3}
export COLCON_PREFIX_PATH=${COLCON_PREFIX_PATH:-}
export _CATKIN_SETUP_DIR=${_CATKIN_SETUP_DIR:-}
source install/setup.bash

echo "Killing lingering simulator/ROS processes..."
pkill -f gz || true
pkill -f gz_sim || true
pkill -f gzserver || true
pkill -f gzclient || true
pkill -f rviz2 || true
pkill -f parameter_bridge || true
pkill -f bridge || true
pkill -f arena_spawn_panel.py || true
pkill -f arena_spawn_panel || true
pkill -f arena_obstacle_manager || true
pkill -f arena_roamer || true
pkill -f mecanum_kinematics || true
pkill -f odom_tf_broadcaster || true
pkill -f world_to_odom_publisher || true
pkill -f ros2 || true

sleep 1

# Start the ROS2 launch (Gazebo + nodes)
echo "Starting arena_experiment.launch.py (Gazebo + nodes)..."
# Ensure Gazebo can resolve local `model://` URIs by adding the workspace models
# directory to the resource paths used by gz/ign. This helps when models live in
# the source tree (src/audix_pkg/models) but aren't installed to the package
# share directory.
export GZ_SIM_RESOURCE_PATH="${BASE_DIR}/src/audix_pkg/models:${BASE_DIR}/src/audix_pkg:${GZ_SIM_RESOURCE_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="${BASE_DIR}/src/audix_pkg/models:${BASE_DIR}/src/audix_pkg:${IGN_GAZEBO_RESOURCE_PATH:-}"

ros2 launch src/audix_pkg/launch/full_mission.launch.py &
LAUNCH_PID=$!

# Ensure we clean up child processes on exit or interrupt
cleanup() {
  echo "Cleaning up launcher and rviz..."
  if [ -n "${RVIZ_PID:-}" ]; then
    kill "${RVIZ_PID}" 2>/dev/null || true
  fi
  if [ -n "${GUI_PID:-}" ]; then
    kill "${GUI_PID}" 2>/dev/null || true
  fi
  if [ -n "${LAUNCH_PID:-}" ]; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
  fi
  # Give processes a moment to exit, then force-kill lingering simulator processes
  sleep 1
  pkill -f gz || true
  pkill -f gz_sim || true
  pkill -f gzserver || true
  pkill -f gzclient || true
  pkill -f rviz2 || true
}
trap cleanup EXIT INT TERM

# Give simulator and bridges time to initialize before RViz
sleep 8

# Start the Start/Stop GUI so the robot remains in STOP until user presses START
echo "Starting start_stop_gui..."
python3 src/audix_pkg/scripts/start_stop_gui.py &
GUI_PID=$!

# Start a single RViz instance using the project's config
RVIZ_CONFIG=src/audix_pkg/rviz/full_mission.rviz
if [ ! -f "$RVIZ_CONFIG" ]; then
  echo "RViz config not found: $RVIZ_CONFIG"
else
  echo "Starting rviz2 with $RVIZ_CONFIG"
  rviz2 -d "$RVIZ_CONFIG" &
  RVIZ_PID=$!
fi

echo "Started: launch_pid=${LAUNCH_PID:-none} rviz_pid=${RVIZ_PID:-none}"

echo "Wrapper is running. To stop, Ctrl-C this script or kill the PIDs above." 
wait
