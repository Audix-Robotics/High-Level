#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$BASE_DIR"

export COLCON_TRACE=${COLCON_TRACE:-}
export AMENT_TRACE=${AMENT_TRACE:-}
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-}
export AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-}
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH:-}
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-python3}
export COLCON_PYTHON_EXECUTABLE=${COLCON_PYTHON_EXECUTABLE:-/usr/bin/python3}
export COLCON_PREFIX_PATH=${COLCON_PREFIX_PATH:-}
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}
export PKG_CONFIG_PATH=${PKG_CONFIG_PATH:-}
export PYTHONPATH=${PYTHONPATH:-}
export _CATKIN_SETUP_DIR=${_CATKIN_SETUP_DIR:-}

set +u
source /opt/ros/jazzy/setup.bash
set -u

if [ "${AUDIX_SKIP_BUILD:-0}" != "1" ]; then
  echo "Building audix with symlink install so source edits are used..."
  colcon build --symlink-install --packages-select audix
else
  echo "AUDIX_SKIP_BUILD=1, using existing install space."
fi

set +u
source install/setup.bash
set -u

export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

SCENARIO_LAUNCH_FILE="${BASE_DIR}/src/audix_pkg/launch/scenarios/arena_simple_cardinal.launch.py"
RVIZ_CONFIG_FILE="${BASE_DIR}/src/audix_pkg/rviz/full_mission.rviz"

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
pkill -f arena_course_spawner || true
pkill -f simple_cardinal_brain || true
pkill -f arena_roamer || true
pkill -f mecanum_kinematics || true
pkill -f odom_tf_broadcaster || true
pkill -f world_to_odom_publisher || true
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
wait_for_process_exit "start_stop_gui"
wait_for_process_exit "/install/audix/lib/audix"

echo "Starting arena_simple_cardinal.launch.py (Gazebo + RViz + course spawner + isolated brain)..."
export GZ_SIM_RESOURCE_PATH="${BASE_DIR}/src/audix_pkg/models:${BASE_DIR}/src/audix_pkg:${GZ_SIM_RESOURCE_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="${BASE_DIR}/src/audix_pkg/models:${BASE_DIR}/src/audix_pkg:${IGN_GAZEBO_RESOURCE_PATH:-}"

ros2 launch "$SCENARIO_LAUNCH_FILE" use_rviz:=true use_gazebo_gui:=true use_slider_gui:=false use_start_stop_gui:=false &
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
  if [ -n "${RVIZ_PID:-}" ]; then
    kill "${RVIZ_PID}" 2>/dev/null || true
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

if ! pgrep -x rviz2 >/dev/null 2>&1; then
  echo "Starting rviz2..."
  rviz2 -d "$RVIZ_CONFIG_FILE" --ros-args -p use_sim_time:=true &
  RVIZ_PID=$!
fi

echo "Starting start_stop_gui..."
python3 src/audix_pkg/scripts/ui/start_stop_gui.py --ros-args -p initial_state:=false &
GUI_PID=$!

echo "Started: launch_pid=${LAUNCH_PID:-none} rviz_pid=${RVIZ_PID:-none} gui_pid=${GUI_PID:-none}"
echo "Wrapper is running. To stop, Ctrl-C this script or kill the PIDs above."
wait
