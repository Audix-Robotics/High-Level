#!/usr/bin/env bash

set -eo pipefail

MODE="${1:-gui}"
NUM_ROBOTS="${NUM_ROBOTS:-0}"

cd /home/ahmed/audix_ws
source /opt/ros/jazzy/setup.bash

pkill -f gz || true
pkill -f gz_sim || true
pkill -f gzserver || true
pkill -f gzclient || true
pkill -f rviz2 || true
pkill -f parameter_bridge || true
pkill -f bridge || true
pkill -f 'ros2 launch audix full_mission_fleet.launch.py' || true
pkill -f 'ros2 launch audix full_mission_multibot.launch.py' || true
pkill -f '/home/ahmed/audix_ws/install/audix/lib/audix/warehouse_fleet_manager.py' || true
pkill -f '/home/ahmed/audix_ws/install/audix/lib/audix/warehouse_mission_manager.py' || true
pkill -f '/home/ahmed/audix_ws/install/audix/lib/audix/fleet_robot_navigator.py' || true
pkill -f '/home/ahmed/audix_ws/install/audix/lib/audix/mecanum_kinematics.py' || true
pkill -f '/home/ahmed/audix_ws/install/audix/lib/audix/scissor_lift_mapper.py' || true
pkill -f '/home/ahmed/audix_ws/install/audix/lib/audix/odom_tf_broadcaster.py' || true
pkill -f '/home/ahmed/audix_ws/install/audix/lib/audix/arena_spawn_panel_multibot.py' || true
pkill -f '/home/ahmed/audix_ws/install/audix/lib/audix/warehouse_waypoint_editor.py' || true
pkill -f '/home/ahmed/audix_ws/install/audix/lib/audix/arena_obstacle_manager.py' || true
pkill -f 'robot_state_publisher' || true
pkill -f world_to_odom_publisher || true
pkill -f ros2 || true

sleep 1

if [[ -f install/setup.bash ]]; then
  source install/setup.bash
else
  colcon build --symlink-install --packages-select audix
  source install/setup.bash
fi

set -u

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