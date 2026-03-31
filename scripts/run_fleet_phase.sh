#!/usr/bin/env bash

set -euo pipefail

MODE="${1:-gui}"
NUM_ROBOTS="${NUM_ROBOTS:-0}"

cd /home/ahmed/audix_ws

export COLCON_TRACE="${COLCON_TRACE:-0}"
export AMENT_TRACE="${AMENT_TRACE:-}"
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-python3}"
export COLCON_PYTHON_EXECUTABLE="${COLCON_PYTHON_EXECUTABLE:-/usr/bin/python3}"
export COLCON_PREFIX_PATH="${COLCON_PREFIX_PATH:-}"
export _CATKIN_SETUP_DIR="${_CATKIN_SETUP_DIR:-}"

source /opt/ros/jazzy/setup.bash

kill_pattern() {
  local signal="$1"
  local pattern="$2"
  pkill "$signal" -f "$pattern" 2>/dev/null || true
}

echo "[run_fleet_phase] Killing lingering fleet, Gazebo, and ROS processes..."

patterns=(
  'ros2 launch audix full_mission_fleet.launch.py'
  'ros2 launch audix full_mission_multibot.launch.py'
  '/home/ahmed/audix_ws/install/audix/lib/audix/warehouse_fleet_manager.py'
  '/home/ahmed/audix_ws/install/audix/lib/audix/warehouse_mission_manager.py'
  '/home/ahmed/audix_ws/install/audix/lib/audix/arena_spawn_panel_multibot.py'
  '/home/ahmed/audix_ws/install/audix/lib/audix/arena_obstacle_manager.py'
  '/home/ahmed/audix_ws/install/audix/lib/audix/warehouse_waypoint_editor.py'
  '/home/ahmed/audix_ws/install/audix/lib/audix/arena_roamer.py'
  '/home/ahmed/audix_ws/install/audix/lib/audix/fleet_robot_navigator.py'
  '/home/ahmed/audix_ws/install/audix/lib/audix/mecanum_kinematics.py'
  '/home/ahmed/audix_ws/install/audix/lib/audix/scissor_lift_mapper.py'
  '/home/ahmed/audix_ws/install/audix/lib/audix/odom_tf_broadcaster.py'
  'robot_localization ekf_node --ros-args -r __ns:=/robot_'
  '/opt/ros/jazzy/lib/robot_localization/ekf_node --ros-args -r __ns:=/robot_'
  'robot_state_publisher --ros-args -r __ns:=/robot_'
  '/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher --ros-args -r __ns:=/robot_'
  'controller_manager spawner'
  'ros_gz_bridge parameter_bridge /robot_'
  '/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge /robot_'
  'ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock'
  '/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge /clock@rosgraph_msgs/msg/Clock'
  'ros2 run ros_gz_sim create'
  'ruby /opt/ros/.*/bin/gz sim'
  'gz sim -r -s -v 2'
  'gz sim -g -v 2'
  '^gz$'
  'rviz2'
  'static_transform_publisher'
  'world_to_odom_publisher'
)

for pattern in "${patterns[@]}"; do
  kill_pattern -TERM "$pattern"
done

sleep 2

for pattern in "${patterns[@]}"; do
  kill_pattern -KILL "$pattern"
done

rm -f /tmp/audix_fleet_models/robot_*.urdf /tmp/audix_fleet_models/robot_*_controllers.yaml /tmp/audix_fleet_models/robot_*_rsp.yaml 2>/dev/null || true
ros2 daemon stop >/dev/null 2>&1 || true

sleep 1

if [[ -f install/setup.bash ]]; then
  source install/setup.bash
else
  colcon build --symlink-install --packages-select audix
  source install/setup.bash
fi

USE_GAZEBO_GUI=false
USE_RVIZ=false
USE_SPAWN_PANEL=false
USE_OBSTACLE_MANAGER=false

if [[ "$MODE" == "gui" ]]; then
  USE_GAZEBO_GUI=true
  USE_RVIZ=true
  USE_SPAWN_PANEL=true
  USE_OBSTACLE_MANAGER=true
fi

exec ros2 launch audix full_mission_fleet.launch.py \
  num_robots:="$NUM_ROBOTS" \
  use_fleet_manager:=true \
  use_gazebo_gui:="$USE_GAZEBO_GUI" \
  use_rviz:="$USE_RVIZ" \
  use_spawn_panel:="$USE_SPAWN_PANEL" \
  use_obstacle_manager:="$USE_OBSTACLE_MANAGER"