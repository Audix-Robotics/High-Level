#!/usr/bin/env bash
set -euo pipefail

# Wrapper to ensure a single clean Gazebo+ROS2+RViz stress-course session.

BASE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$BASE_DIR"

export COLCON_TRACE=${COLCON_TRACE:-}
export AMENT_TRACE=${AMENT_TRACE:-}
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-}
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-python3}
export COLCON_PYTHON_EXECUTABLE=${COLCON_PYTHON_EXECUTABLE:-/usr/bin/python3}
export COLCON_PREFIX_PATH=${COLCON_PREFIX_PATH:-}
export _CATKIN_SETUP_DIR=${_CATKIN_SETUP_DIR:-}

source /opt/ros/jazzy/setup.bash
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
pkill -f arena_course_spawner || true
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

wait_for_process_exit "gz sim -r -s"
wait_for_process_exit "gz sim -g"
wait_for_process_exit "gzserver"
wait_for_process_exit "gzclient"
wait_for_process_exit "rviz2"
wait_for_process_exit "ros2 launch"
wait_for_process_exit "parameter_bridge"
wait_for_process_exit "robot_state_publisher"
wait_for_process_exit "odom_tf_broadcaster"
wait_for_process_exit "start_stop_gui"
wait_for_process_exit "/install/audix/lib/audix"

echo "Starting arena_stress_course.launch.py (Gazebo + RViz + course spawner)..."
export GZ_SIM_RESOURCE_PATH="${BASE_DIR}/src/audix_pkg/models:${BASE_DIR}/src/audix_pkg:${GZ_SIM_RESOURCE_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="${BASE_DIR}/src/audix_pkg/models:${BASE_DIR}/src/audix_pkg:${IGN_GAZEBO_RESOURCE_PATH:-}"

ros2 launch audix arena_stress_course.launch.py use_rviz:=true use_gazebo_gui:=true use_start_stop_gui:=false auto_start:=false &
LAUNCH_PID=$!

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
  sleep 1
  pkill -f gz || true
  pkill -f gz_sim || true
  pkill -f gzserver || true
  pkill -f gzclient || true
  pkill -f rviz2 || true
  pkill -f start_stop_gui || true
  pkill -f start_stop_gui.py || true
  pkill -f start_stop_node || true
  pkill -f start_stop_node.py || true
  pkill -9 -f start_stop || true
  pkill -9 -f start_stop_gui || true
  pkill -9 -f start_stop_node || true
}
trap cleanup EXIT INT TERM

sleep 8

echo "Starting start_stop_gui..."
python3 src/audix_pkg/scripts/ui/start_stop_gui.py --ros-args -p initial_state:=true &
GUI_PID=$!

echo "Started: launch_pid=${LAUNCH_PID:-none} gui_pid=${GUI_PID:-none}"
echo "Wrapper is running. To stop, Ctrl-C this script or kill the PIDs above."
wait