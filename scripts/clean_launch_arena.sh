#!/usr/bin/env bash
set -euo pipefail

# Wrapper to ensure a single clean Gazebo+ROS2+RViz session.
# Usage: ./scripts/clean_launch_arena.sh

BASE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$BASE_DIR"

# Provide safe defaults for environment variables that install/setup.bash expects
# to avoid unbound variable errors when running with `set -u`.
export COLCON_TRACE=${COLCON_TRACE:-}
export AMENT_TRACE=${AMENT_TRACE:-}
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-}
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-python3}
export COLCON_PYTHON_EXECUTABLE=${COLCON_PYTHON_EXECUTABLE:-/usr/bin/python3}
export COLCON_PREFIX_PATH=${COLCON_PREFIX_PATH:-}
export _CATKIN_SETUP_DIR=${_CATKIN_SETUP_DIR:-}

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

wait_for_process_exit() {
  local pattern="$1"
  local attempts="${2:-20}"
  local delay="${3:-0.5}"
  local index

  for ((index=0; index<attempts; index++)); do
    if ! pgrep -f "$pattern" >/dev/null 2>&1; then
      return 0
    fi
    sleep "$delay"
  done

  pkill -9 -f "$pattern" || true
  sleep 1
}

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
pkill -f warehouse_overlay_markers || true
pkill -f warehouse_overlay_markers.py || true
pkill -f ros2 || true
pkill -f start_stop_gui || true
pkill -f start_stop_gui.py || true
pkill -f start_stop_node || true
pkill -f start_stop_node.py || true
pkill -9 -f start_stop || true
pkill -9 -f start_stop_gui || true
pkill -9 -f start_stop_node || true
pkill -f warehouse_environment_viz.py || true
pkill -f warehouse_environment_viz || true

wait_for_process_exit "gz sim -r -s"
wait_for_process_exit "gz sim -g"
wait_for_process_exit "gzserver"
wait_for_process_exit "gzclient"
wait_for_process_exit "rviz2"
wait_for_process_exit "ros2 launch"
wait_for_process_exit "parameter_bridge"
wait_for_process_exit "robot_state_publisher"
wait_for_process_exit "odom_tf_broadcaster"
wait_for_process_exit "warehouse_overlay_markers"
wait_for_process_exit "start_stop_gui"
wait_for_process_exit "/install/audix/lib/audix"

# Start the ROS2 launch (Gazebo + nodes + RViz + overlay)
echo "Starting full_mission.launch.py (Gazebo + RViz + obstacle spawner + overlay)..."
# Ensure Gazebo can resolve local `model://` URIs by adding the workspace models
# directory to the resource paths used by gz/ign. This helps when models live in
# the source tree (src/audix_pkg/models) but aren't installed to the package
# share directory.
export GZ_SIM_RESOURCE_PATH="${BASE_DIR}/src/audix_pkg/models:${BASE_DIR}/src/audix_pkg:${GZ_SIM_RESOURCE_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="${BASE_DIR}/src/audix_pkg/models:${BASE_DIR}/src/audix_pkg:${IGN_GAZEBO_RESOURCE_PATH:-}"

ros2 launch audix full_mission.launch.py use_rviz:=true use_spawn_panel:=true use_gazebo_gui:=true &
LAUNCH_PID=$!

# Ensure we clean up child processes on exit or interrupt
cleanup() {
  if [ "${CLEAN_LAUNCH_CLEANED_UP:-0}" = "1" ]; then
    return 0
  fi
  CLEAN_LAUNCH_CLEANED_UP=1

  echo "Cleaning up launcher..."
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
  pkill -f warehouse_environment_viz.py || true
  pkill -f warehouse_environment_viz || true
  pkill -f warehouse_overlay_markers || true
  pkill -f warehouse_overlay_markers.py || true
  pkill -f start_stop_gui || true
  pkill -f start_stop_gui.py || true
  pkill -f start_stop_node || true
  pkill -f start_stop_node.py || true
  pkill -9 -f start_stop || true
  pkill -9 -f start_stop_gui || true
  pkill -9 -f start_stop_node || true
}
trap cleanup EXIT INT TERM

# Give the ROS graph a moment to settle before starting the manual GUI.
sleep 8

echo "Starting start_stop_gui..."
python3 src/audix_pkg/scripts/start_stop_gui.py &
GUI_PID=$!

echo "Started: launch_pid=${LAUNCH_PID:-none} gui_pid=${GUI_PID:-none}"

echo "Wrapper is running. To stop, Ctrl-C this script or kill the PIDs above." 
wait
