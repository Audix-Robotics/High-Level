#!/usr/bin/env python3

import json
import math
import os
import re
import signal
import subprocess
import tempfile
import time
from functools import partial
from pathlib import Path as FilePath

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from audix.msg import FleetStatus, RobotStatus
from audix.srv import AssignMission, DespawnRobot, GetRobotStatus, SpawnRobot


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def quat_to_yaw(x_pos, y_pos, z_pos, w_pos):
    siny_cosp = 2.0 * (w_pos * z_pos + x_pos * y_pos)
    cosy_cosp = 1.0 - 2.0 * (y_pos * y_pos + z_pos * z_pos)
    return math.atan2(siny_cosp, cosy_cosp)


class WarehouseFleetManager(Node):
    def __init__(self):
        super().__init__('warehouse_fleet_manager')

        pkg_share = get_package_share_directory('audix')
        self.declare_parameter(
            'warehouse_config_path',
            os.path.join(pkg_share, 'config', 'warehouse_lanes.yaml'),
        )
        self.declare_parameter(
            'missions_config_path',
            os.path.join(pkg_share, 'config', 'warehouse_missions.yaml'),
        )
        self.declare_parameter(
            'robot_model_path',
            os.path.join(pkg_share, 'urdf', 'audix.urdf'),
        )
        self.declare_parameter('world_name', 'warehouse')
        self.declare_parameter('spawn_z', 0.06)
        self.declare_parameter('auto_spawn_default_fleet', False)
        self.declare_parameter('auto_spawn_num_robots', 3)
        self.declare_parameter('max_robot_slots', 10)
        self.declare_parameter(
            'controllers_yaml_path',
            os.path.join(pkg_share, 'config', 'controllers.yaml'),
        )
        self.declare_parameter('click_topic', '/clicked_point')
        self.declare_parameter('rviz_command_topic', '/fleet/rviz_command')
        self.declare_parameter('path_point_spacing', 0.05)
        self.declare_parameter('ir_half_fov', 0.30543)
        self.declare_parameter('ir_display_range_min', 0.05)
        self.declare_parameter('ir_display_range_max', 0.25)
        self.declare_parameter('robot_center_offset_x', 0.0)
        self.declare_parameter('robot_center_offset_y', 0.0)
        self.declare_parameter('robot_body_frame_flip_180', True)
        self.declare_parameter('obstacle_detect_distance', 0.15)
        self.declare_parameter('obstacle_detect_distance_front', 0.17)
        self.declare_parameter('obstacle_detect_distance_front_center', 0.15)

        self.world_name = str(self.get_parameter('world_name').value)
        self.robot_model_path = str(self.get_parameter('robot_model_path').value)
        self.controllers_yaml_path = str(self.get_parameter('controllers_yaml_path').value)
        self.spawn_z = float(self.get_parameter('spawn_z').value)
        self.auto_spawn_default_fleet = bool(self.get_parameter('auto_spawn_default_fleet').value)
        self.auto_spawn_num_robots = int(self.get_parameter('auto_spawn_num_robots').value)
        self.max_robot_slots = int(self.get_parameter('max_robot_slots').value)
        self.click_topic = str(self.get_parameter('click_topic').value)
        self.rviz_command_topic = str(self.get_parameter('rviz_command_topic').value)
        self.path_point_spacing = float(self.get_parameter('path_point_spacing').value)
        self.ir_half_fov = float(self.get_parameter('ir_half_fov').value)
        self.ir_display_range_min = float(self.get_parameter('ir_display_range_min').value)
        self.ir_display_range_max = float(self.get_parameter('ir_display_range_max').value)
        self.robot_center_offset_x = float(self.get_parameter('robot_center_offset_x').value)
        self.robot_center_offset_y = float(self.get_parameter('robot_center_offset_y').value)
        self.robot_body_frame_flip_180 = bool(self.get_parameter('robot_body_frame_flip_180').value)
        self.obstacle_detect_distance = float(self.get_parameter('obstacle_detect_distance').value)
        self.obstacle_detect_distance_front = float(
            self.get_parameter('obstacle_detect_distance_front').value
        )
        self.obstacle_detect_distance_front_center = float(
            self.get_parameter('obstacle_detect_distance_front_center').value
        )

        self.warehouse_config = self.load_yaml_config('warehouse_config_path')
        self.missions_config = self.load_yaml_config('missions_config_path')

        self.robot_states = {}
        self.robot_odom_subs = {}
        self.robot_scan_subs = {}
        self.robot_mission_pubs = {}
        self.robot_mission_control_pubs = {}
        self.robot_descriptor_pubs = {}
        self.robot_runtime = {}
        self.spawned_robot_ids = set()
        self._default_fleet_spawned = False
        self._next_auto_spawn_id = 0
        self.rviz_mode = 'idle'
        self.selected_robot_id = None
        self.pending_waypoints = {}
        self.pending_spawn_request = None
        self.generated_model_dir = FilePath(tempfile.gettempdir()) / 'audix_fleet_models'
        self.generated_model_dir.mkdir(parents=True, exist_ok=True)
        self.robot_description_xml = FilePath(self.robot_model_path).read_text(encoding='utf-8')
        self.controllers_config = yaml.safe_load(FilePath(self.controllers_yaml_path).read_text(encoding='utf-8')) or {}
        self.mission_params_path = os.path.join(pkg_share, 'config', 'mission_params.yaml')
        self.ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

        self.sensor_positions = {
            'back': (0.16255, -0.00323),
            'left': (0.00273, 0.15504),
            'front_left': (-0.17753, 0.12688),
            'front': (-0.20195, 0.00482),
            'front_right': (-0.18323, -0.11962),
            'right': (-0.00537, -0.15346),
        }
        self.sensor_topics = {
            'front': 'ir_front',
            'front_left': 'ir_front_right',
            'front_right': 'ir_front_left',
            'left': 'ir_right',
            'right': 'ir_left',
            'back': 'ir_back',
        }
        self.sensor_order = ['front', 'front_left', 'front_right', 'left', 'right', 'back']

        self.create_service(SpawnRobot, '/fleet/spawn_robot', self.spawn_robot_cb)
        self.create_service(AssignMission, '/fleet/assign_mission', self.assign_mission_cb)
        self.create_service(GetRobotStatus, '/fleet/get_robot_status', self.get_status_cb)
        self.create_service(DespawnRobot, '/fleet/despawn_robot', self.despawn_robot_cb)

        self.fleet_status_pub = self.create_publisher(FleetStatus, '/fleet/status', 10)
        self.fleet_viz_pub = self.create_publisher(MarkerArray, '/fleet/visualization', 10)
        self.create_subscription(
            RobotStatus,
            '/fleet/robot_status_updates',
            self.on_robot_status_update,
            10,
        )
        self.create_subscription(PointStamped, self.click_topic, self.on_clicked_point, 10)
        self.create_subscription(String, self.rviz_command_topic, self.on_rviz_command, 10)

        self.create_timer(0.5, self.publish_fleet_status)
        self.create_timer(1.0, self._auto_spawn_default_fleet)

    def load_yaml_config(self, parameter_name):
        path = str(self.get_parameter(parameter_name).value)
        with open(path, 'r', encoding='utf-8') as config_file:
            return yaml.safe_load(config_file) or {}

    def _auto_spawn_default_fleet(self):
        if not self.auto_spawn_default_fleet or self._default_fleet_spawned:
            return

        default_fleet = self.warehouse_config.get('default_fleet', {})
        num_robots = min(self.auto_spawn_num_robots, int(default_fleet.get('num_robots', 0)))
        if self._next_auto_spawn_id >= num_robots:
            self._default_fleet_spawned = True
            return

        robot_index = self._next_auto_spawn_id
        request = SpawnRobot.Request()
        request.robot_id = robot_index
        request.lane_id = int(
            default_fleet.get('robots', {}).get(f'robot_{robot_index}', {}).get('lane', robot_index % 6)
        )
        response = SpawnRobot.Response()
        lane_config = self._lane_config(request.lane_id)
        spawn_x, spawn_y, spawn_yaw = self._spawn_pose_for_robot(
            robot_index,
            request.lane_id,
            lane_config,
            use_default_fleet_pose=True,
        )
        success, message = self._spawn_robot_at_pose(
            robot_index,
            request.lane_id,
            spawn_x,
            spawn_y,
            spawn_yaw,
        )
        response.success = success
        response.message = message
        if response.success:
            assignment = self.missions_config.get('robot_assignments', {}).get(f'robot_{robot_index}', {})
            mission_name = assignment.get('initial_mission', '')
            if mission_name:
                mission_request = AssignMission.Request()
                mission_request.robot_id = robot_index
                mission_request.mission_name = mission_name
                mission_response = AssignMission.Response()
                self.assign_mission_cb(mission_request, mission_response)
            self._next_auto_spawn_id += 1
        else:
            self.get_logger().warn(f'Auto-spawn failed for robot_{robot_index}: {response.message}')

    def spawn_robot_cb(self, request, response):
        robot_id = int(request.robot_id)
        lane_id = int(request.lane_id)
        if robot_id in self.spawned_robot_ids:
            if not self._robot_runtime_healthy(robot_id):
                lane_config = self._lane_config(lane_id)
                spawn_x, spawn_y, spawn_yaw = self._spawn_pose_for_robot(robot_id, lane_id, lane_config)
                success, message = self._recover_existing_robot(
                    robot_id,
                    lane_id,
                    spawn_x,
                    spawn_y,
                    spawn_yaw,
                )
                response.success = success
                response.message = message
                if success:
                    self.selected_robot_id = robot_id
                    self.publish_fleet_status()
                return response
            response.success = False
            response.message = f'robot_{robot_id} already exists'
            return response

        lane_config = self._lane_config(lane_id)
        spawn_x, spawn_y, spawn_yaw = self._spawn_pose_for_robot(robot_id, lane_id, lane_config)
        success, message = self._spawn_robot_at_pose(robot_id, lane_id, spawn_x, spawn_y, spawn_yaw)
        if not success:
            if 'already exists' in message.lower() or 'entity named' in message.lower():
                success, message = self._recover_existing_robot(
                    robot_id,
                    lane_id,
                    spawn_x,
                    spawn_y,
                    spawn_yaw,
                )
                response.success = success
                response.message = message
                if success:
                    self.selected_robot_id = robot_id
                    self.publish_fleet_status()
                return response
            response.success = False
            response.message = message
            return response

        self.selected_robot_id = robot_id
        self.pending_waypoints.pop(robot_id, None)
        self.publish_fleet_status()
        response.success = True
        response.message = (
            f'Spawned robot_{robot_id} in {self._lane_label(lane_id, lane_config)} '
            f'at ({spawn_x:.2f}, {spawn_y:.2f})'
        )
        return response

    def assign_mission_cb(self, request, response):
        robot_id = int(request.robot_id)
        if robot_id not in self.robot_states:
            response.success = False
            response.message = f'robot_{robot_id} is not active'
            return response

        if robot_id not in self.robot_mission_pubs:
            self.robot_mission_pubs[robot_id] = self.create_publisher(
                Path,
                f'/robot_{robot_id}/mission_waypoints',
                10,
            )
        if robot_id not in self.robot_descriptor_pubs:
            self.robot_descriptor_pubs[robot_id] = self.create_publisher(
                String,
                f'/robot_{robot_id}/mission_descriptor',
                10,
            )

        mission_name = request.mission_name.strip()
        if request.custom_waypoints:
            waypoints = []
            for pose in request.custom_waypoints:
                waypoint = {
                    'position': [pose.position.x, pose.position.y, pose.position.z],
                    'dwell_time': 0.0,
                }
                has_program_metadata = (
                    abs(float(pose.orientation.x)) > 1e-6
                    or abs(float(pose.orientation.y)) > 1e-6
                    or abs(float(pose.orientation.z)) > 1e-6
                    or abs(float(pose.orientation.w) - 1.0) > 1e-6
                )
                if has_program_metadata:
                    decoded_yaw = math.atan2(
                        2.0 * pose.orientation.w * pose.orientation.z,
                        1.0 - 2.0 * pose.orientation.z * pose.orientation.z,
                    )
                    if float(pose.orientation.x) < -0.5:
                        waypoint['arrival_yaw'] = decoded_yaw
                    else:
                        waypoint['dwell_time'] = max(0.0, float(pose.orientation.y))
                        waypoint['lift_height'] = max(0.0, float(pose.orientation.x))
                        waypoint['scan_yaw'] = decoded_yaw
                waypoints.append(waypoint)
            descriptor = {
                'mission_name': mission_name or f'custom_robot_{robot_id}',
                'source': self._infer_custom_mission_mode(mission_name),
                'mission_mode': self._infer_custom_mission_mode(mission_name),
                'waypoints': waypoints,
            }
        else:
            mission = self.missions_config.get('missions', {}).get(mission_name)
            if mission is None:
                mission = self._descriptor_from_saved_routine(mission_name)
            if mission is None:
                response.success = False
                response.message = f'Unknown mission: {mission_name}'
                return response
            descriptor = {
                'mission_name': mission.get('mission_name', mission_name),
                'source': mission.get('source', 'template'),
                'mission_mode': mission.get('mission_mode', mission.get('mode', 'default_original')),
                'lane_id': mission.get('lane_id'),
                'waypoints': mission.get('waypoints', []),
            }

        descriptor = self._normalize_descriptor(descriptor)

        self._publish_robot_mission(robot_id, descriptor)
        self.publish_fleet_status()

        response.success = True
        response.message = f'Assigned {descriptor["mission_name"]} to robot_{robot_id}'
        return response

    def get_status_cb(self, request, response):
        robot_id = int(request.robot_id)
        if robot_id not in self.robot_states:
            response.success = False
            return response

        response.robot_status = self._build_robot_status_msg(robot_id, self.robot_states[robot_id])
        response.success = True
        return response

    def despawn_robot_cb(self, request, response):
        robot_id = int(request.robot_id)
        robot_name = f'robot_{robot_id}'

        self.pending_waypoints.pop(robot_id, None)
        if robot_id in self.robot_states:
            self._publish_mission_control(robot_id, 'cancel')
        success, message = self._delete_robot_gz(robot_name)
        if not success:
            response.success = False
            response.message = message
            return response

        self._stop_robot_runtime(robot_id)
        self.spawned_robot_ids.discard(robot_id)
        self.robot_states.pop(robot_id, None)
        self.robot_odom_subs.pop(robot_id, None)
        self.robot_scan_subs.pop(robot_id, None)
        self.robot_mission_pubs.pop(robot_id, None)
        self.robot_mission_control_pubs.pop(robot_id, None)
        self.robot_descriptor_pubs.pop(robot_id, None)
        if self.selected_robot_id == robot_id:
            self.selected_robot_id = None
        self.publish_fleet_status()

        response.success = True
        response.message = f'Removed {robot_name}'
        return response

    def on_robot_odom(self, msg, robot_id):
        if robot_id not in self.robot_states:
            return

        position = msg.pose.pose.position
        state = self.robot_states[robot_id]
        orientation = msg.pose.pose.orientation
        if int(state.get('lane_id', 0)) < 0:
            lane_id = int(state.get('lane_id', -1))
            lane_name = state.get('lane_name', self._lane_label(lane_id))
        else:
            lane_id, lane_name = self._lane_from_position(position.x, position.y)
        state['position'] = (position.x, position.y, position.z)
        state['yaw'] = quat_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        state['lane_id'] = lane_id
        state['lane_name'] = lane_name
        trail = state.setdefault('trail', [])
        expected_spawn = state.get('spawn_position')
        if not state.get('trail_initialized', False):
            if expected_spawn is not None:
                spawn_dx = position.x - float(expected_spawn[0])
                spawn_dy = position.y - float(expected_spawn[1])
                if math.hypot(spawn_dx, spawn_dy) > 0.75:
                    return
            trail[:] = [(position.x, position.y, position.z)]
            state['trail_initialized'] = True
            return
        if not trail:
            trail.append((position.x, position.y, position.z))
            return
        last_x, last_y, last_z = trail[-1]
        if math.hypot(position.x - last_x, position.y - last_y) > 1.0:
            trail[:] = [(position.x, position.y, position.z)]
            return
        if math.hypot(position.x - last_x, position.y - last_y) >= self.path_point_spacing:
            trail.append((position.x, position.y, position.z))
            if len(trail) > 800:
                del trail[:-800]

    def on_robot_scan(self, msg, robot_id, sensor_name):
        state = self.robot_states.get(robot_id)
        if state is None:
            return
        sensor_ranges = state.setdefault('sensor_ranges', {})
        sensor_max = state.setdefault('sensor_max_ranges', {})

        valid_ranges = [value for value in msg.ranges if math.isfinite(value) and value >= msg.range_min]
        if valid_ranges:
            sensor_ranges[sensor_name] = max(msg.range_min, min(valid_ranges))
        else:
            sensor_ranges[sensor_name] = float('inf')
        if math.isfinite(msg.range_max) and msg.range_max > 0.0:
            sensor_max[sensor_name] = msg.range_max
        else:
            sensor_max[sensor_name] = self.ir_display_range_max

    def on_robot_status_update(self, msg):
        robot_id = int(msg.robot_id)
        if robot_id not in self.robot_states:
            return

        state = self.robot_states[robot_id]
        state['lane_name'] = msg.lane_name or state['lane_name']
        state['current_mission'] = msg.current_mission
        state['waypoints_completed'] = int(msg.waypoints_completed)
        state['total_waypoints'] = int(msg.total_waypoints)
        state['mission_progress_pct'] = float(msg.mission_progress_pct)
        state['robot_state'] = msg.robot_state or state['robot_state']
        state['position'] = (
            msg.position.x,
            msg.position.y,
            msg.position.z,
        )

    def on_clicked_point(self, msg):
        x_pos = float(msg.point.x)
        y_pos = float(msg.point.y)

        if self.rviz_mode == 'spawn':
            pending = self.pending_spawn_request or {}
            robot_id = pending.get('robot_id', self._next_available_robot_id())
            if robot_id is None:
                self.get_logger().warn('No free robot slots available for RViz spawn')
                return
            lane_id = pending.get('lane_id')
            if lane_id is None:
                lane_id, _ = self._lane_from_position(x_pos, y_pos)
            lane_config = self._lane_config(lane_id)
            spawn_yaw = float(pending.get('spawn_yaw', lane_config.get('home_pad', {}).get('yaw', math.pi)))
            success, message = self._spawn_robot_at_pose(robot_id, lane_id, x_pos, y_pos, spawn_yaw)
            if success:
                self.selected_robot_id = robot_id
                self.pending_waypoints.pop(robot_id, None)
                self.pending_spawn_request = None
                self.rviz_mode = 'idle'
                self.publish_fleet_status()
            self.get_logger().info(message)
            return

        if self.rviz_mode == 'select':
            robot_id = self._nearest_robot_id(x_pos, y_pos)
            if robot_id is None:
                self.get_logger().warn('No active robot near clicked point to select')
                return
            self.selected_robot_id = robot_id
            self.rviz_mode = 'waypoint'
            self.get_logger().info(f'Selected robot_{robot_id} from RViz click and switched to waypoint mode')
            return

        if self.rviz_mode == 'waypoint':
            if self.selected_robot_id is None:
                self.get_logger().warn('Select a robot before adding waypoints from RViz')
                return
            self.pending_waypoints.setdefault(self.selected_robot_id, []).append((x_pos, y_pos, 0.0))
            self.fleet_viz_pub.publish(self._build_visualization_markers())
            self.get_logger().info(
                f'Added waypoint {(x_pos, y_pos)} for robot_{self.selected_robot_id} from RViz click'
            )

    def on_rviz_command(self, msg):
        command = msg.data.strip()
        if not command:
            return

        if command.startswith('{'):
            try:
                payload = json.loads(command)
            except json.JSONDecodeError:
                self.get_logger().warn('Invalid JSON RViz command payload')
                return

            keyword = str(payload.get('command', '')).strip().lower()
            if keyword == 'set_waypoints':
                robot_id = int(payload.get('robot_id', -1))
                if robot_id not in self.robot_states:
                    self.get_logger().warn(f'robot_{robot_id} is not active')
                    return
                waypoints = []
                for point in payload.get('waypoints', []):
                    if len(point) < 2:
                        continue
                    waypoints.append((float(point[0]), float(point[1]), float(point[2]) if len(point) > 2 else 0.0))
                if waypoints:
                    self.pending_waypoints[robot_id] = waypoints
                else:
                    self.pending_waypoints.pop(robot_id, None)
                self.fleet_viz_pub.publish(self._build_visualization_markers())
                return
            if keyword == 'prepare_spawn_capture':
                robot_id = int(payload.get('robot_id', -1))
                if robot_id < 0 or robot_id >= self.max_robot_slots:
                    self.get_logger().warn(f'Invalid robot id for RViz spawn capture: {robot_id}')
                    return
                if robot_id in self.spawned_robot_ids:
                    self.get_logger().warn(f'robot_{robot_id} already exists')
                    return
                self.pending_spawn_request = {
                    'robot_id': robot_id,
                }
                self.selected_robot_id = robot_id
                self.rviz_mode = 'spawn'
                self.get_logger().info(f'RViz spawn capture armed for robot_{robot_id}')
                return
            self.get_logger().warn(f'Unknown JSON RViz command: {keyword}')
            return

        parts = command.split()
        keyword = parts[0].lower()

        if keyword == 'mode' and len(parts) >= 2:
            mode = parts[1].strip().lower()
            if mode in {'idle', 'spawn', 'select', 'waypoint'}:
                self.rviz_mode = mode
                if mode != 'spawn':
                    self.pending_spawn_request = None
                self.get_logger().info(f'RViz mode set to {mode}')
            else:
                self.get_logger().warn(f'Unknown RViz mode: {mode}')
            return

        if keyword == 'select' and len(parts) >= 2:
            try:
                robot_id = int(parts[1])
            except ValueError:
                self.get_logger().warn(f'Invalid robot id in command: {command}')
                return
            if robot_id not in self.robot_states:
                self.get_logger().warn(f'robot_{robot_id} is not active')
                return
            self.selected_robot_id = robot_id
            self.get_logger().info(f'Selected robot_{robot_id}')
            return

        if keyword == 'clear_waypoints':
            robot_id = self._command_robot_target(parts)
            if robot_id is None:
                return
            self.pending_waypoints.pop(robot_id, None)
            self.fleet_viz_pub.publish(self._build_visualization_markers())
            self.get_logger().info(f'Cleared pending RViz waypoints for robot_{robot_id}')
            return

        if keyword == 'commit_waypoints':
            robot_id = self._command_robot_target(parts)
            if robot_id is None:
                return
            waypoints = self.pending_waypoints.get(robot_id, [])
            if not waypoints:
                self.get_logger().warn(f'No pending RViz waypoints for robot_{robot_id}')
                return
            self._publish_pending_waypoint_mission(robot_id)
            self.pending_waypoints.pop(robot_id, None)
            self.publish_fleet_status()
            self.get_logger().info(f'Committed RViz waypoint mission to robot_{robot_id}')
            return

        if keyword in {'despawn_selected', 'reset_selected'}:
            if self.selected_robot_id is None:
                self.get_logger().warn('No selected robot to remove')
                return
            self._remove_robot_by_id(self.selected_robot_id)
            return

        if keyword == 'remove_robot' and len(parts) >= 2:
            try:
                robot_id = int(parts[1])
            except ValueError:
                self.get_logger().warn(f'Invalid robot id in command: {command}')
                return
            self._remove_robot_by_id(robot_id)
            return

        self.get_logger().warn(f'Unknown RViz command: {command}')

    def publish_fleet_status(self):
        fleet_msg = FleetStatus()
        fleet_msg.robots = []
        active_states = []
        for robot_id in sorted(self.robot_states):
            state = self.robot_states[robot_id]
            fleet_msg.robots.append(self._build_robot_status_msg(robot_id, state))
            active_states.append(state.get('robot_state', 'IDLE'))

        fleet_msg.fleet_mode = self._fleet_mode_from_states(active_states)
        fleet_msg.timestamp = self.get_clock().now().nanoseconds / 1e9
        self.fleet_status_pub.publish(fleet_msg)
        self.fleet_viz_pub.publish(self._build_visualization_markers())

    def _ordered_lane_items(self):
        return sorted(
            self.warehouse_config.get('lanes', {}).items(),
            key=lambda item: (
                int(item[1].get('display_order', 999)),
                int(item[0].split('_')[-1]),
            ),
        )

    def _lane_config(self, lane_id):
        if int(lane_id) < 0:
            return self.warehouse_config.get('default_lane', {})
        return self.warehouse_config.get('lanes', {}).get(f'lane_{lane_id}', {})

    def _lane_label(self, lane_id, lane_config=None):
        lane_cfg = lane_config or self._lane_config(lane_id)
        if lane_cfg:
            return lane_cfg.get('operator_name', f'Lane {lane_id + 1}')
        return 'Default' if int(lane_id) < 0 else f'Lane {lane_id + 1}'

    def _robot_body_rgba(self, robot_id):
        palette = [
            (0.89, 0.10, 0.11, 1.0),
            (1.0, 0.50, 0.0, 1.0),
            (0.98, 0.84, 0.12, 1.0),
            (0.20, 0.78, 0.35, 1.0),
            (0.18, 0.42, 0.96, 1.0),
            (0.29, 0.0, 0.51, 1.0),
            (0.56, 0.27, 0.68, 1.0),
            (0.00, 0.74, 0.73, 1.0),
            (0.85, 0.37, 0.01, 1.0),
            (0.58, 0.40, 0.74, 1.0),
        ]
        return palette[int(robot_id) % len(palette)]

    def _apply_base_link_color(self, robot_xml, robot_id):
        red, green, blue, alpha = self._robot_body_rgba(robot_id)
        color_text = f'rgba="{red} {green} {blue} {alpha}"'
        pattern = (
            r'(<link\s+name="base_link">.*?<visual>.*?<material\s+name="">\s*<color\s+)'
            r'rgba="[^"]+"'
            r'(\s*/>.*?</material>)'
        )
        return re.sub(pattern, r'\1' + color_text + r'\2', robot_xml, count=1, flags=re.DOTALL)

    def _spawn_pose_for_robot(self, robot_id, lane_id, lane_config, use_default_fleet_pose=False):
        if use_default_fleet_pose:
            defaults = self.warehouse_config.get('default_fleet', {}).get('robots', {})
            default_robot = defaults.get(f'robot_{robot_id}', {})
            spawn_x = float(default_robot.get('spawn_x', 0.0))
            spawn_y = float(default_robot.get('spawn_y', lane_config.get('center_y', 0.0)))
            spawn_yaw = float(default_robot.get('spawn_yaw', math.pi))
            return spawn_x, spawn_y, spawn_yaw

        home_pad = lane_config.get('home_pad', {})
        spawn_x = float(home_pad.get('x', 0.0))
        spawn_y = float(home_pad.get('y', lane_config.get('center_y', 0.0)))
        spawn_yaw = float(home_pad.get('yaw', math.pi))
        return spawn_x, spawn_y, spawn_yaw

    def _physical_spawn_yaw(self, logical_yaw):
        return logical_yaw

    def _spawn_robot_at_pose(self, robot_id, lane_id, spawn_x, spawn_y, spawn_yaw):
        robot_name = f'robot_{robot_id}'
        lane_config = self._lane_config(lane_id)
        controller_params_path = self._write_namespaced_controller_config(robot_id)
        success, message = self._spawn_robot_gz(
            robot_name,
            spawn_x,
            spawn_y,
            spawn_yaw,
            controller_params_path,
        )
        if success:
            self._register_robot_state(robot_id, lane_id, lane_config, spawn_x, spawn_y)
            runtime_success, runtime_message = self._start_robot_runtime(
                robot_id,
                robot_name,
                controller_params_path,
                spawn_x,
                spawn_y,
                spawn_yaw,
            )
            if not runtime_success:
                self._delete_robot_gz(robot_name)
                self._stop_robot_runtime(robot_id)
                self.spawned_robot_ids.discard(robot_id)
                self.robot_states.pop(robot_id, None)
                return False, runtime_message
            return True, f'Spawned {robot_name} at ({spawn_x:.2f}, {spawn_y:.2f})'
        return False, message

    def _robot_runtime_healthy(self, robot_id):
        runtime = self.robot_runtime.get(robot_id, {})
        required_keys = (
            'robot_state_publisher',
            'bridge',
            'ekf_adapter',
            'ekf',
            'navigator',
            'mecanum',
            'scissor_mapper',
            'odom_tf',
        )
        for key in required_keys:
            process = runtime.get(key)
            if process is None or process.poll() is not None:
                return False
        return True

    def _recover_existing_robot(self, robot_id, lane_id, spawn_x, spawn_y, spawn_yaw):
        robot_name = f'robot_{robot_id}'
        lane_config = self._lane_config(lane_id)
        controller_params_path = self._write_namespaced_controller_config(robot_id)

        self._stop_robot_runtime(robot_id)
        self.spawned_robot_ids.discard(robot_id)
        self.robot_states.pop(robot_id, None)
        self.pending_waypoints.pop(robot_id, None)

        self._register_robot_state(robot_id, lane_id, lane_config, spawn_x, spawn_y)
        runtime_success, runtime_message = self._start_robot_runtime(
            robot_id,
            robot_name,
            controller_params_path,
            spawn_x,
            spawn_y,
            spawn_yaw,
        )
        if runtime_success:
            return True, f'Attached to existing {robot_name} and restored runtime'

        self._stop_robot_runtime(robot_id)
        self.spawned_robot_ids.discard(robot_id)
        self.robot_states.pop(robot_id, None)

        delete_success, delete_message = self._delete_robot_gz(robot_name)
        if not delete_success:
            return False, (
                f'Failed to recover existing {robot_name}: {runtime_message}. '
                f'Cleanup also failed: {delete_message}'
            )

        return self._spawn_robot_at_pose(robot_id, lane_id, spawn_x, spawn_y, spawn_yaw)

    def _build_namespaced_model_xml(self, robot_name, controller_params_path, robot_id):
        with open(self.robot_model_path, 'r', encoding='utf-8') as model_file:
            robot_xml = model_file.read()

        robot_xml = self._apply_base_link_color(robot_xml, robot_id)

        robot_xml = robot_xml.replace(
            '<ros>\n        <remapping>~/robot_description:=robot_description</remapping>\n      </ros>',
            (
                '<ros>\n'
                f'        <namespace>/{robot_name}</namespace>\n'
                '        <remapping>~/robot_description:=robot_description</remapping>\n'
                '      </ros>'
            ),
            1,
        )
        robot_xml = robot_xml.replace('$(find audix)/config/controllers.yaml', str(controller_params_path))

        namespaced_elements = {
            '<topic>/cmd_vel</topic>': f'<topic>/{robot_name}/cmd_vel</topic>',
            '<odom_topic>/odom</odom_topic>': f'<odom_topic>/{robot_name}/odom</odom_topic>',
            '<topic>/imu</topic>': f'<topic>/{robot_name}/imu</topic>',
            '<topic>/ir_front/scan</topic>': f'<topic>/{robot_name}/ir_front/scan</topic>',
            '<topic>/ir_front_left/scan</topic>': f'<topic>/{robot_name}/ir_front_left/scan</topic>',
            '<topic>/ir_front_right/scan</topic>': f'<topic>/{robot_name}/ir_front_right/scan</topic>',
            '<topic>/ir_left/scan</topic>': f'<topic>/{robot_name}/ir_left/scan</topic>',
            '<topic>/ir_right/scan</topic>': f'<topic>/{robot_name}/ir_right/scan</topic>',
            '<topic>/ir_back/scan</topic>': f'<topic>/{robot_name}/ir_back/scan</topic>',
        }
        for original, replacement in namespaced_elements.items():
            robot_xml = robot_xml.replace(original, replacement)

        robot_xml += '\n' + self._lift_stabilization_extensions()
        return robot_xml

    def _lift_stabilization_extensions(self):
        lift_links = [
            'scissors_lift_base',
            'bottom_stud_link',
            'top_stud_link',
            'left_link1',
            'left_link2',
            'left_link3',
            'left_link4',
            'left_link5',
            'left_link6',
            'right_link1',
            'right_link2',
            'right_link3',
            'right_link4',
            'right_link5',
            'right_link6',
        ]
        blocks = []
        for link_name in lift_links:
            blocks.append(
                f'<gazebo reference="{link_name}"><gravity>false</gravity></gazebo>'
            )
        return '\n'.join(blocks)

    def _register_robot_state(self, robot_id, lane_id, lane_config, spawn_x, spawn_y):
        self.spawned_robot_ids.add(robot_id)
        self.robot_states[robot_id] = {
            'position': (spawn_x, spawn_y, self.spawn_z),
            'yaw': float(lane_config.get('home_pad', {}).get('yaw', math.pi)),
            'lane_id': lane_id,
            'lane_name': self._lane_label(lane_id, lane_config),
            'current_mission': '',
            'waypoints_completed': 0,
            'total_waypoints': 0,
            'mission_progress_pct': 0.0,
            'robot_state': 'IDLE',
            'trail': [],
            'trail_initialized': False,
            'spawn_position': (spawn_x, spawn_y, self.spawn_z),
            'sensor_ranges': {sensor_name: float('inf') for sensor_name in self.sensor_order},
            'sensor_max_ranges': {sensor_name: self.ir_display_range_max for sensor_name in self.sensor_order},
        }
        if robot_id not in self.robot_odom_subs:
            self.robot_odom_subs[robot_id] = self.create_subscription(
                Odometry,
                f'/robot_{robot_id}/odometry/filtered',
                partial(self.on_robot_odom, robot_id=robot_id),
                10,
            )
        if robot_id not in self.robot_scan_subs:
            self.robot_scan_subs[robot_id] = []
            for sensor_name in self.sensor_order:
                self.robot_scan_subs[robot_id].append(
                    self.create_subscription(
                        LaserScan,
                        f'/robot_{robot_id}/{self.sensor_topics[sensor_name]}/scan',
                        partial(self.on_robot_scan, robot_id=robot_id, sensor_name=sensor_name),
                        10,
                    )
                )
        if robot_id not in self.robot_mission_pubs:
            self.robot_mission_pubs[robot_id] = self.create_publisher(
                Path,
                f'/robot_{robot_id}/mission_waypoints',
                10,
            )
        if robot_id not in self.robot_descriptor_pubs:
            self.robot_descriptor_pubs[robot_id] = self.create_publisher(
                String,
                f'/robot_{robot_id}/mission_descriptor',
                10,
            )

    def _publish_robot_mission(self, robot_id, descriptor):
        descriptor_msg = String()
        descriptor_msg.data = json.dumps(descriptor)
        self.robot_descriptor_pubs[robot_id].publish(descriptor_msg)
        self.robot_mission_pubs[robot_id].publish(self._build_path_message(descriptor))

        self.robot_states[robot_id]['current_mission'] = descriptor['mission_name']
        self.robot_states[robot_id]['total_waypoints'] = int(descriptor.get('stop_count', len(descriptor['waypoints'])))
        self.robot_states[robot_id]['waypoints_completed'] = 0
        self.robot_states[robot_id]['mission_progress_pct'] = 0.0
        self.robot_states[robot_id]['robot_state'] = 'MOVING'

    def _publish_pending_waypoint_mission(self, robot_id):
        waypoints = self.pending_waypoints.get(robot_id, [])
        if not waypoints or robot_id not in self.robot_states:
            return
        descriptor = {
            'mission_name': f'rviz_robot_{robot_id}',
            'source': 'rviz',
            'waypoints': [
                {'position': [x_pos, y_pos, z_pos], 'dwell_time': 0.0}
                for x_pos, y_pos, z_pos in waypoints
            ],
        }
        self._publish_robot_mission(robot_id, descriptor)

    def _publish_mission_control(self, robot_id, command):
        if robot_id not in self.robot_mission_control_pubs:
            self.robot_mission_control_pubs[robot_id] = self.create_publisher(
                String,
                f'/robot_{robot_id}/mission_control',
                10,
            )
        self.robot_mission_control_pubs[robot_id].publish(String(data=command))

    def _write_robot_state_publisher_params(self, robot_id):
        robot_name = f'robot_{robot_id}'
        params_path = self.generated_model_dir / f'{robot_name}_rsp.yaml'
        robot_description = self._apply_base_link_color(self.robot_description_xml, robot_id)
        indented_description = '\n'.join(
            f'      {line}' for line in robot_description.splitlines()
        )
        params_path.write_text(
            '\n'.join(
                [
                    '/**:',
                    '  ros__parameters:',
                    '    use_sim_time: true',
                    f'    frame_prefix: "{robot_name}/"',
                    '    robot_description: |',
                    indented_description,
                    '',
                ]
            ),
            encoding='utf-8',
        )
        return params_path

    def _write_namespaced_controller_config(self, robot_id):
        robot_name = f'robot_{robot_id}'
        params_path = self.generated_model_dir / f'{robot_name}_controllers.yaml'
        controller_manager_params = (
            self.controllers_config.get('controller_manager', {}).get('ros__parameters', {}).copy()
        )
        joint_state_cfg = dict(controller_manager_params.get('joint_state_broadcaster', {}))
        if 'use_local_topics' in joint_state_cfg:
            joint_state_cfg.pop('use_local_topics')
            controller_manager_params['joint_state_broadcaster'] = joint_state_cfg

        namespaced_config = {
            f'/{robot_name}/controller_manager': {
                'ros__parameters': controller_manager_params,
            },
            f'/{robot_name}/joint_state_broadcaster': {
                'ros__parameters': {
                    'use_local_topics': True,
                },
            },
            f'/{robot_name}/mecanum_velocity_controller': self.controllers_config.get(
                'mecanum_velocity_controller',
                {},
            ),
            f'/{robot_name}/scissor_position_controller': self.controllers_config.get(
                'scissor_position_controller',
                {},
            ),
        }
        params_path.write_text(yaml.safe_dump(namespaced_config, sort_keys=False), encoding='utf-8')
        return params_path

    def _write_ekf_override_params(self, robot_id, spawn_x, spawn_y, spawn_yaw):
        robot_name = f'robot_{robot_id}'
        params_path = self.generated_model_dir / f'{robot_name}_ekf.yaml'
        base_config = yaml.safe_load(FilePath(self.ekf_config_path).read_text(encoding='utf-8')) or {}
        params = dict(base_config.get('ekf_filter_node', {}).get('ros__parameters', {}))
        params.update(
            {
                'use_sim_time': True,
                'publish_tf': False,
                'odom_frame': 'odom',
                'base_link_frame': f'{robot_name}/base_footprint',
                'odom0': 'ekf/odom',
                'imu0': 'ekf/imu',
                'initial_state': [
                    float(spawn_x),
                    float(spawn_y),
                    0.0,
                    0.0,
                    0.0,
                    float(spawn_yaw),
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            }
        )
        override_config = {'/**': {'ros__parameters': params}}
        params_path.write_text(yaml.safe_dump(override_config, sort_keys=False), encoding='utf-8')
        return params_path

    def _write_roamer_override_params(self, robot_id):
        robot_name = f'robot_{robot_id}'
        params_path = self.generated_model_dir / f'{robot_name}_arena_roamer.yaml'
        base_config = yaml.safe_load(FilePath(self.mission_params_path).read_text(encoding='utf-8')) or {}
        params = dict(base_config.get('arena_roamer', {}).get('ros__parameters', {}))
        params.update(
            {
                'use_sim_time': True,
                'debug_frame_id': 'odom',
                'start_enabled': False,
                'enable_dynamic_missions': True,
            }
        )
        override_config = {'/**': {'ros__parameters': params}}
        params_path.write_text(yaml.safe_dump(override_config, sort_keys=False), encoding='utf-8')
        return params_path

    def _spawn_process(self, robot_id, key, command):
        process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        self.robot_runtime.setdefault(robot_id, {})[key] = process
        return process

    def _run_controller_spawner(self, robot_name, controller_name, controller_params_path):
        result = subprocess.run(
            [
                'ros2', 'run', 'controller_manager', 'spawner',
                controller_name,
                '--param-file', str(controller_params_path),
                '--controller-manager', f'/{robot_name}/controller_manager',
                '--controller-manager-timeout', '40',
            ],
            capture_output=True,
            text=True,
            check=False,
            timeout=50.0,
        )
        output = ((result.stdout or '') + (result.stderr or '')).strip()
        if result.returncode != 0:
            return False, output or f'Failed to start {controller_name} for {robot_name}'
        return True, output or f'Started {controller_name} for {robot_name}'

    def _start_robot_runtime(self, robot_id, robot_name, controller_params_path, spawn_x, spawn_y, spawn_yaw):
        self._stop_robot_runtime(robot_id)
        try:
            params_path = self._write_robot_state_publisher_params(robot_id)
            ekf_params_path = self._write_ekf_override_params(robot_id, spawn_x, spawn_y, spawn_yaw)
            roamer_params_path = self._write_roamer_override_params(robot_id)
            self._spawn_process(
                robot_id,
                'robot_state_publisher',
                [
                    'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                    '--ros-args',
                    '-r', f'__ns:=/{robot_name}',
                    '-r', 'joint_states:=joint_state_broadcaster/joint_states',
                    '--params-file', str(params_path),
                ],
            )
            self._spawn_process(
                robot_id,
                'bridge',
                [
                    'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    f'/{robot_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    f'/{robot_name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    f'/{robot_name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    f'/{robot_name}/ir_front/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    f'/{robot_name}/ir_front_left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    f'/{robot_name}/ir_front_right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    f'/{robot_name}/ir_left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    f'/{robot_name}/ir_right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    f'/{robot_name}/ir_back/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                ],
            )
            time.sleep(2.0)

            for controller_name in (
                'joint_state_broadcaster',
                'mecanum_velocity_controller',
                'scissor_position_controller',
            ):
                success, message = self._run_controller_spawner(
                    robot_name,
                    controller_name,
                    controller_params_path,
                )
                if not success:
                    return False, message

            self._spawn_process(
                robot_id,
                'ekf_adapter',
                [
                    'ros2', 'run', 'audix', 'ekf_input_adapter.py',
                    '--ros-args',
                    '-r', f'__ns:=/{robot_name}',
                    '-p', 'use_sim_time:=true',
                    '-p', 'raw_odom_topic:=odom',
                    '-p', 'adapted_odom_topic:=ekf/odom',
                    '-p', 'raw_imu_topic:=imu',
                    '-p', 'adapted_imu_topic:=ekf/imu',
                    '-p', 'odom_frame:=odom',
                    '-p', f'base_frame:={robot_name}/base_footprint',
                    '-p', f'imu_frame:={robot_name}/imu_link',
                ],
            )
            self._spawn_process(
                robot_id,
                'ekf',
                [
                    'ros2', 'run', 'robot_localization', 'ekf_node',
                    '--ros-args',
                    '-r', f'__ns:=/{robot_name}',
                    '--params-file', str(ekf_params_path),
                    '-r', '/odometry/filtered:=odometry/filtered',
                ],
            )
            self._spawn_process(
                robot_id,
                'odom_tf',
                [
                    'ros2', 'run', 'audix', 'odom_tf_broadcaster.py',
                    '--ros-args',
                    '-p', 'use_sim_time:=true',
                    '-p', f'odom_topic:=/{robot_name}/odometry/filtered',
                    '-p', 'odom_frame:=odom',
                    '-p', f'base_frame:={robot_name}/base_footprint',
                ],
            )
            self._spawn_process(
                robot_id,
                'navigator',
                [
                    'ros2', 'run', 'audix', 'arena_roamer.py',
                    '--ros-args',
                    '-r', f'__ns:=/{robot_name}',
                    '--params-file', str(roamer_params_path),
                    '-r', '/odometry/filtered:=odometry/filtered',
                    '-r', '/cmd_vel:=cmd_vel',
                    '-r', '/scissor_lift/slider:=scissor_lift/slider',
                    '-r', '/ir_front/scan:=ir_front/scan',
                    '-r', '/ir_front_right/scan:=ir_front_right/scan',
                    '-r', '/ir_front_left/scan:=ir_front_left/scan',
                    '-r', '/ir_right/scan:=ir_right/scan',
                    '-r', '/ir_left/scan:=ir_left/scan',
                    '-r', '/ir_back/scan:=ir_back/scan',
                    '-r', '/avoid_cmd_vel:=avoid_cmd_vel',
                    '-r', '/debug/planned_path:=debug/planned_path',
                    '-r', '/debug/robot_path:=debug/robot_path',
                    '-r', '/debug/targets:=debug/targets',
                    '-r', '/debug/state:=debug/state',
                    '-r', '/robot_enable:=robot_enable',
                ],
            )
            self._spawn_process(
                robot_id,
                'mecanum',
                [
                    'ros2', 'run', 'audix', 'mecanum_kinematics.py',
                    '--ros-args',
                    '-r', f'__ns:=/{robot_name}',
                    '-r', 'joint_states:=joint_state_broadcaster/joint_states',
                    '-p', 'use_sim_time:=true',
                    '-p', 'publish_odom:=true',
                    '-p', f'initial_x:={spawn_x}',
                    '-p', f'initial_y:={spawn_y}',
                    '-p', f'initial_yaw:={spawn_yaw}',
                ],
            )
            self._spawn_process(
                robot_id,
                'scissor_mapper',
                [
                    'ros2', 'run', 'audix', 'scissor_lift_mapper.py',
                    '--ros-args',
                    '-r', f'__ns:=/{robot_name}',
                    '-r', 'joint_states:=joint_state_broadcaster/joint_states',
                    '-p', 'use_sim_time:=true',
                ],
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self._stop_robot_runtime(robot_id)
            return False, str(exc)
        return True, f'Runtime ready for {robot_name}'

    def _stop_robot_runtime(self, robot_id):
        robot_name = f'robot_{robot_id}'
        runtime = self.robot_runtime.pop(robot_id, {})
        for process in runtime.values():
            if process.poll() is not None:
                continue
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                continue
            except OSError:
                process.terminate()
        time.sleep(0.2)
        for process in runtime.values():
            if process.poll() is not None:
                continue
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                continue
            except OSError:
                process.kill()

        fallback_patterns = [
            f'robot_state_publisher.*__ns:=/{robot_name}',
            f'ekf_input_adapter.py.*__ns:=/{robot_name}',
            f'ekf_node.*__ns:=/{robot_name}',
            f'arena_roamer.py.*__ns:=/{robot_name}',
            f'fleet_robot_navigator.py.*__ns:=/{robot_name}',
            f'mecanum_kinematics.py.*__ns:=/{robot_name}',
            f'scissor_lift_mapper.py.*__ns:=/{robot_name}',
            f'odom_tf_broadcaster.py.*odom_topic:=/{robot_name}/odometry/filtered',
            f'parameter_bridge.*/{robot_name}/cmd_vel@',
        ]
        for pattern in fallback_patterns:
            subprocess.run(['pkill', '-f', pattern], check=False)

    def _spawn_robot_gz(self, robot_name, spawn_x, spawn_y, spawn_yaw, controller_params_path):
        robot_id = int(robot_name.split('_')[-1])
        robot_xml = self._build_namespaced_model_xml(robot_name, controller_params_path, robot_id)
        physical_spawn_yaw = self._physical_spawn_yaw(spawn_yaw)
        try:
            temp_path = self.generated_model_dir / f'{robot_name}.urdf'
            temp_path.write_text(robot_xml, encoding='utf-8')
            result = subprocess.run(
                [
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-world', self.world_name,
                    '-name', robot_name,
                    '-file', str(temp_path),
                    '-x', str(spawn_x),
                    '-y', str(spawn_y),
                    '-z', str(self.spawn_z),
                    '-R', '0.0',
                    '-P', '0.0',
                    '-Y', str(physical_spawn_yaw),
                ],
                capture_output=True,
                text=True,
                check=False,
                timeout=20.0,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            return False, str(exc)

        output = ((result.stdout or '') + (result.stderr or '')).strip()
        if result.returncode != 0:
            return False, output or f'Failed to spawn {robot_name}'
        time.sleep(1.0)
        return True, output or f'Spawned {robot_name}'

    def _delete_robot_gz(self, robot_name):
        try:
            result = subprocess.run(
                [
                    'gz', 'service',
                    '-s', f'/world/{self.world_name}/remove',
                    '--reqtype', 'gz.msgs.Entity',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req', f'name: "{robot_name}" type: MODEL',
                ],
                capture_output=True,
                text=True,
                check=False,
                timeout=10.0,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            return False, str(exc)

        output = ((result.stdout or '') + (result.stderr or '')).strip()
        if result.returncode != 0 or 'data: true' not in output.lower():
            return False, output or f'Failed to remove {robot_name}'
        return True, output


    def _lane_from_position(self, x_position, y_position):
        lanes = self.warehouse_config.get('lanes', {})
        for lane_key, lane_config in lanes.items():
            if (
                float(lane_config.get('min_x', -999.0)) <= x_position <= float(lane_config.get('max_x', 999.0))
                and float(lane_config.get('min_y', -999.0)) <= y_position <= float(lane_config.get('max_y', 999.0))
            ):
                lane_id = int(lane_key.split('_')[-1])
                return lane_id, self._lane_label(lane_id, lane_config)

        nearest_lane = min(
            lanes.items(),
            key=lambda item: math.hypot(
                float(item[1].get('center_x', 0.0)) - x_position,
                float(item[1].get('center_y', 0.0)) - y_position,
            ),
        )
        lane_id = int(nearest_lane[0].split('_')[-1])
        return lane_id, self._lane_label(lane_id, nearest_lane[1])

    def _build_path_message(self, descriptor):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        for index, waypoint in enumerate(descriptor.get('waypoints', [])):
            pose = PoseStamped()
            pose.header = path_msg.header
            position = waypoint.get('position', [0.0, 0.0, 0.0])
            pose.pose.position.x = float(position[0])
            pose.pose.position.y = float(position[1])
            pose.pose.position.z = float(position[2]) if len(position) > 2 else 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        return path_msg

    def _infer_custom_mission_mode(self, mission_name):
        mission_name = (mission_name or '').strip().lower()
        if '_lane_' in mission_name:
            return 'lane_inspection'
        if 'free_roam' in mission_name:
            return 'free_roam'
        return 'custom'

    def _same_position(self, first_position, second_position, tol=1e-4):
        return (
            abs(float(first_position[0]) - float(second_position[0])) <= tol
            and abs(float(first_position[1]) - float(second_position[1])) <= tol
            and abs(float(first_position[2]) - float(second_position[2])) <= tol
        )

    def _normalize_descriptor(self, descriptor):
        source_waypoints = descriptor.get('waypoints', [])
        if not source_waypoints:
            return descriptor

        grouped_waypoints = []
        for waypoint in source_waypoints:
            position = waypoint.get('position', [0.0, 0.0, 0.0])
            normalized_position = [
                float(position[0]),
                float(position[1]),
                float(position[2]) if len(position) > 2 else 0.0,
            ]
            scan_yaw = waypoint.get('scan_yaw')
            lift_height = float(waypoint.get('lift_height', 0.0))
            dwell_time = float(waypoint.get('dwell_time', 0.0))

            if scan_yaw is not None and lift_height > 1e-6:
                if grouped_waypoints:
                    previous = grouped_waypoints[-1]
                    if (
                        previous.get('scan_yaw') is not None
                        and self._same_position(previous.get('position', normalized_position), normalized_position)
                        and abs(float(previous.get('scan_yaw', 0.0)) - float(scan_yaw)) <= 1e-4
                        and not previous.get('return_home', False)
                    ):
                        previous.setdefault('scan_heights', []).append(lift_height)
                        continue

                grouped_waypoints.append(
                    {
                        'position': normalized_position,
                        'scan_yaw': float(scan_yaw),
                        'scan_heights': [lift_height],
                        'dwell_time': dwell_time,
                        'position_tolerance': float(waypoint.get('position_tolerance', 0.05)),
                    }
                )
                continue

            grouped_waypoint = {
                'position': normalized_position,
                'dwell_time': dwell_time,
                'position_tolerance': float(waypoint.get('position_tolerance', 0.05)),
            }
            if 'arrival_yaw' in waypoint:
                grouped_waypoint['arrival_yaw'] = float(waypoint.get('arrival_yaw', 0.0))
            grouped_waypoint['return_home'] = bool(waypoint.get('return_home', False))
            grouped_waypoints.append(grouped_waypoint)

        if descriptor.get('mission_mode') == 'lane_inspection':
            scan_stop_count = 0
            for index, waypoint in enumerate(grouped_waypoints):
                if waypoint.get('scan_heights'):
                    scan_stop_count += 1
                    waypoint['stop_index'] = scan_stop_count
                elif index == len(grouped_waypoints) - 1:
                    waypoint['return_home'] = True
            descriptor['stop_count'] = scan_stop_count

        descriptor['waypoints'] = grouped_waypoints
        return descriptor

    def _descriptor_from_saved_routine(self, routine_name):
        routine = self.missions_config.get('saved_routines', {}).get(routine_name)
        if routine is None:
            return None
        if isinstance(routine.get('descriptor'), dict):
            descriptor = dict(routine['descriptor'])
            descriptor.setdefault('mission_name', routine_name)
            descriptor.setdefault('source', routine.get('mode', 'saved_routine'))
            descriptor.setdefault('mission_mode', routine.get('mode', 'saved_routine'))
            descriptor.setdefault('lane_id', routine.get('lane_id'))
            return descriptor
        if 'waypoints' not in routine:
            return None
        lane_id = int(routine.get('lane_id', 0))
        lane_cfg = self._lane_config(lane_id)
        lane_x = float(lane_cfg.get('center_x', 0.0))
        descriptor = {
            'mission_name': routine_name,
            'source': 'saved_routine',
            'mission_mode': routine.get('mode', 'saved_routine'),
            'lane_id': lane_id,
            'waypoints': [],
        }
        for waypoint in routine.get('waypoints', []):
            direction = str(waypoint.get('direction', 'right')).strip().lower()
            scan_yaw = self._scan_yaw_from_direction(direction, lane_cfg)
            levels = max(1, int(waypoint.get('levels', 1)))
            for level_index in range(levels):
                descriptor['waypoints'].append(
                    {
                        'position': [float(waypoint.get('x', lane_x)), float(waypoint.get('y', 0.0)), 0.0],
                        'lift_height': self._lift_height_for_level(level_index + 1),
                        'scan_yaw': scan_yaw,
                        'dwell_time': float(routine.get('scan_duration', 0.0)),
                        'position_tolerance': 0.05,
                    }
                )
        return descriptor

    def _scan_yaw_from_direction(self, direction, lane_cfg=None):
        if lane_cfg is not None and direction in {'lane', 'shelf', 'auto'}:
            return float(lane_cfg.get('scan_yaw', 0.0))
        lookup = {
            'left': math.pi / 2.0,
            'right': -math.pi / 2.0,
            'forward': 0.0,
            'back': math.pi,
        }
        return lookup.get(direction, float((lane_cfg or {}).get('scan_yaw', 0.0)))

    def _lift_height_for_level(self, level):
        mapping = {1: 0.30, 2: 0.60, 3: 0.90}
        return mapping.get(int(level), 0.90)

    def _build_robot_status_msg(self, robot_id, state):
        msg = RobotStatus()
        msg.robot_id = int(robot_id)
        msg.lane_name = state.get('lane_name', '')
        position = state.get('position', (0.0, 0.0, 0.0))
        msg.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
        msg.current_mission = state.get('current_mission', '')
        msg.waypoints_completed = int(state.get('waypoints_completed', 0))
        msg.total_waypoints = int(state.get('total_waypoints', 0))
        msg.mission_progress_pct = float(state.get('mission_progress_pct', 0.0))
        msg.robot_state = state.get('robot_state', 'IDLE')
        return msg

    def _fleet_mode_from_states(self, states):
        if not states:
            return 'IDLE'
        if any(state == 'ERROR' for state in states):
            return 'HALTED'
        if any(state in ('MOVING', 'SCANNING') for state in states):
            return 'NAVIGATING'
        if any(state == 'SPAWNING' for state in states):
            return 'SPAWNING'
        return 'IDLE'

    def _build_visualization_markers(self):
        markers = MarkerArray()
        timestamp = self.get_clock().now().to_msg()
        clear = Marker()
        clear.header.frame_id = 'odom'
        clear.header.stamp = timestamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        marker_id = 1

        world_markers = self._build_world_space_markers(timestamp, marker_id)
        markers.markers.extend(world_markers)
        marker_id = len(markers.markers)

        lane_markers = self._build_lane_guide_markers(timestamp, marker_id)
        markers.markers.extend(lane_markers)
        marker_id = len(markers.markers)

        for robot_id, state in sorted(self.robot_states.items()):
            x_pos, y_pos, z_pos = state.get('position', (0.0, 0.0, 0.0))
            lane_color = self._lane_config(state.get('lane_id', 0)).get('color', [0.5, 0.5, 0.5])

            sphere = Marker()
            sphere.header.frame_id = 'odom'
            sphere.header.stamp = timestamp
            sphere.ns = 'fleet_positions'
            sphere.id = marker_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x_pos
            sphere.pose.position.y = y_pos
            sphere.pose.position.z = z_pos + 0.18
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.12
            sphere.scale.y = 0.12
            sphere.scale.z = 0.12
            sphere.color.r = float(lane_color[0])
            sphere.color.g = float(lane_color[1])
            sphere.color.b = float(lane_color[2])
            sphere.color.a = 0.8
            markers.markers.append(sphere)
            marker_id += 1

            if self.selected_robot_id == robot_id:
                selection = Marker()
                selection.header.frame_id = 'odom'
                selection.header.stamp = timestamp
                selection.ns = 'fleet_selection'
                selection.id = marker_id
                selection.type = Marker.CYLINDER
                selection.action = Marker.ADD
                selection.pose.position.x = x_pos
                selection.pose.position.y = y_pos
                selection.pose.position.z = z_pos + 0.05
                selection.pose.orientation.w = 1.0
                selection.scale.x = 0.28
                selection.scale.y = 0.28
                selection.scale.z = 0.02
                selection.color.r = 1.0
                selection.color.g = 0.95
                selection.color.b = 0.15
                selection.color.a = 0.75
                markers.markers.append(selection)
                marker_id += 1

            label = Marker()
            label.header.frame_id = 'odom'
            label.header.stamp = timestamp
            label.ns = 'fleet_labels'
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x_pos
            label.pose.position.y = y_pos
            label.pose.position.z = z_pos + 0.34
            label.pose.orientation.w = 1.0
            label.scale.z = 0.1
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.9
            mode_text = ' [selected]' if self.selected_robot_id == robot_id else ''
            label.text = f'robot_{robot_id}: {state.get("robot_state", "IDLE")}{mode_text}'
            markers.markers.append(label)
            marker_id += 1

            trail = state.get('trail', [])
            if len(trail) >= 2:
                trail_marker = Marker()
                trail_marker.header.frame_id = 'odom'
                trail_marker.header.stamp = timestamp
                trail_marker.ns = 'fleet_robot_trails'
                trail_marker.id = marker_id
                trail_marker.type = Marker.LINE_STRIP
                trail_marker.action = Marker.ADD
                trail_marker.pose.orientation.w = 1.0
                trail_marker.scale.x = 0.03
                trail_marker.color.r = 0.0
                trail_marker.color.g = 0.94
                trail_marker.color.b = 1.0
                trail_marker.color.a = 0.8
                for trail_x, trail_y, trail_z in trail:
                    trail_marker.points.append(Point(x=trail_x, y=trail_y, z=trail_z + 0.07))
                markers.markers.append(trail_marker)
                marker_id += 1

            markers.markers.extend(self._build_ir_markers(robot_id, state, timestamp, marker_id))
            marker_id = len(markers.markers)

        markers.markers.extend(self._build_pending_waypoint_markers(timestamp, marker_id))

        return markers

    def _build_world_space_markers(self, timestamp, start_id):
        markers = []
        marker_id = start_id
        grey = (0.55, 0.55, 0.55, 0.32)
        line_grey = (0.62, 0.62, 0.62, 0.55)

        def add_box(name, x_pos, y_pos, z_pos, size_x, size_y, size_z, yaw=0.0):
            nonlocal marker_id
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = timestamp
            marker.ns = 'gazebo_static_space'
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x_pos)
            marker.pose.position.y = float(y_pos)
            marker.pose.position.z = float(z_pos)
            marker.pose.orientation.z = math.sin(0.5 * yaw)
            marker.pose.orientation.w = math.cos(0.5 * yaw)
            marker.scale.x = float(size_x)
            marker.scale.y = float(size_y)
            marker.scale.z = float(size_z)
            marker.color.r = grey[0]
            marker.color.g = grey[1]
            marker.color.b = grey[2]
            marker.color.a = grey[3]
            markers.append(marker)
            marker_id += 1

            label = Marker()
            label.header.frame_id = 'odom'
            label.header.stamp = timestamp
            label.ns = 'gazebo_static_labels'
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(x_pos)
            label.pose.position.y = float(y_pos)
            label.pose.position.z = float(z_pos) + float(size_z) * 0.6
            label.pose.orientation.w = 1.0
            label.scale.z = 0.10
            label.color.r = 0.88
            label.color.g = 0.88
            label.color.b = 0.88
            label.color.a = 0.70
            label.text = name
            markers.append(label)
            marker_id += 1

        def add_cylinder(name, x_pos, y_pos, z_pos, radius, length):
            nonlocal marker_id
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = timestamp
            marker.ns = 'gazebo_static_space'
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(x_pos)
            marker.pose.position.y = float(y_pos)
            marker.pose.position.z = float(z_pos)
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(radius) * 2.0
            marker.scale.y = float(radius) * 2.0
            marker.scale.z = float(length)
            marker.color.r = grey[0]
            marker.color.g = grey[1]
            marker.color.b = grey[2]
            marker.color.a = grey[3]
            markers.append(marker)
            marker_id += 1

            label = Marker()
            label.header.frame_id = 'odom'
            label.header.stamp = timestamp
            label.ns = 'gazebo_static_labels'
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(x_pos)
            label.pose.position.y = float(y_pos)
            label.pose.position.z = float(z_pos) + float(length) * 0.8
            label.pose.orientation.w = 1.0
            label.scale.z = 0.10
            label.color.r = 0.88
            label.color.g = 0.88
            label.color.b = 0.88
            label.color.a = 0.70
            label.text = name
            markers.append(label)
            marker_id += 1

        bounds = self.warehouse_config.get('warehouse', {}).get('bounds', {})
        min_x = float(bounds.get('min_x', -2.9))
        max_x = float(bounds.get('max_x', 5.9))
        min_y = float(bounds.get('min_y', -4.8))
        max_y = float(bounds.get('max_y', 4.8))
        outline = Marker()
        outline.header.frame_id = 'odom'
        outline.header.stamp = timestamp
        outline.ns = 'gazebo_static_outline'
        outline.id = marker_id
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = 0.04
        outline.color.r = line_grey[0]
        outline.color.g = line_grey[1]
        outline.color.b = line_grey[2]
        outline.color.a = line_grey[3]
        for point_x, point_y in [
            (min_x, min_y),
            (max_x, min_y),
            (max_x, max_y),
            (min_x, max_y),
            (min_x, min_y),
        ]:
            outline.points.append(self._make_marker_point(point_x, point_y, 0.05))
        markers.append(outline)
        marker_id += 1

        add_box('north wall', 1.50, 4.80, 0.75, 8.80, 0.12, 1.5)
        add_box('south wall', 1.50, -4.80, 0.75, 8.80, 0.12, 1.5)
        add_box('west wall', -2.90, 0.0, 0.75, 0.12, 9.60, 1.5)
        add_box('east wall', 5.90, 0.0, 0.75, 0.12, 9.60, 1.5)

        shelf_dims = self.warehouse_config.get('shelves', {}).get('shelf_a', {}).get('dimensions', [0.44, 1.31])
        shelf_height = float(self.warehouse_config.get('shelves', {}).get('shelf_a', {}).get('height_m', 1.96))
        for name, x_pos, y_pos, yaw in [
            ('Shelf C', 2.20, -2.50, 1.5707963267948966),
            ('Shelf B', 2.20, 0.0, 1.5707963267948966),
            ('Shelf A', 2.20, 2.50, 1.5707963267948966),
            ('Left Shelf', -2.20, -0.50, 0.0),
        ]:
            add_box(
                name,
                x_pos,
                y_pos,
                shelf_height * 0.5,
                float(shelf_dims[0]),
                float(shelf_dims[1]),
                shelf_height,
                yaw=yaw,
            )

        for name, x_pos, y_pos in [
            ('Scan Rack C', 0.70, -2.40),
            ('Scan Rack B', -0.70, 0.0),
            ('Scan Rack A', 0.70, 2.40),
        ]:
            add_box(name, x_pos, y_pos, 0.16915, 0.82, 0.40, 0.3383, yaw=1.570796)

        add_box('Pallet Jack', 2.20, -4.10, 0.10, 0.85, 0.48, 0.20)
        add_cylinder('Bucket', 2.30, 3.90, 0.18, 0.14, 0.36)
        return markers

    def _build_lane_guide_markers(self, timestamp, start_id):
        markers = []
        marker_id = start_id
        for lane_key, lane_cfg in self._ordered_lane_items():
            lane_id = int(lane_key.split('_')[-1])
            bounds = lane_cfg.get(
                'stop_range_x',
                [
                    self.warehouse_config.get('warehouse', {}).get('bounds', {}).get('min_x', -2.0),
                    self.warehouse_config.get('warehouse', {}).get('bounds', {}).get('max_x', 2.0),
                ],
            )
            line = Marker()
            line.header.frame_id = 'odom'
            line.header.stamp = timestamp
            line.ns = 'fleet_lane_guides'
            line.id = marker_id
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.03
            color = lane_cfg.get('color', [0.6, 0.6, 0.6])
            line.color.r = float(color[0])
            line.color.g = float(color[1])
            line.color.b = float(color[2])
            line.color.a = 0.5
            line.points.append(self._make_marker_point(float(bounds[0]), float(lane_cfg.get('center_y', 0.0)), 0.03))
            line.points.append(self._make_marker_point(float(bounds[1]), float(lane_cfg.get('center_y', 0.0)), 0.03))
            markers.append(line)
            marker_id += 1

            home_pad = lane_cfg.get('home_pad', {})
            pad = Marker()
            pad.header.frame_id = 'odom'
            pad.header.stamp = timestamp
            pad.ns = 'fleet_home_pads'
            pad.id = marker_id
            pad.type = Marker.CYLINDER
            pad.action = Marker.ADD
            pad.pose.position.x = float(home_pad.get('x', 0.0))
            pad.pose.position.y = float(home_pad.get('y', lane_cfg.get('center_y', 0.0)))
            pad.pose.position.z = 0.015
            pad.pose.orientation.w = 1.0
            pad.scale.x = 0.36
            pad.scale.y = 0.36
            pad.scale.z = 0.03
            pad.color.r = float(color[0])
            pad.color.g = float(color[1])
            pad.color.b = float(color[2])
            pad.color.a = 0.32
            markers.append(pad)
            marker_id += 1

            label = Marker()
            label.header.frame_id = 'odom'
            label.header.stamp = timestamp
            label.ns = 'fleet_lane_labels'
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(home_pad.get('x', 0.0)) + 0.45
            label.pose.position.y = float(home_pad.get('y', lane_cfg.get('center_y', 0.0)))
            label.pose.position.z = 0.18
            label.pose.orientation.w = 1.0
            label.scale.z = 0.14
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.95
            label.text = self._lane_label(lane_id, lane_cfg)
            markers.append(label)
            marker_id += 1

        default_cfg = self._lane_config(-1)
        if default_cfg:
            color = default_cfg.get('color', [0.56, 0.27, 0.68])
            guide_points = default_cfg.get('guide_points', [])
            if len(guide_points) >= 2:
                line = Marker()
                line.header.frame_id = 'odom'
                line.header.stamp = timestamp
                line.ns = 'fleet_lane_guides'
                line.id = marker_id
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.pose.orientation.w = 1.0
                line.scale.x = 0.03
                line.color.r = float(color[0])
                line.color.g = float(color[1])
                line.color.b = float(color[2])
                line.color.a = 0.65
                for point in guide_points:
                    z_pos = float(point[2]) if len(point) > 2 else 0.03
                    line.points.append(self._make_marker_point(float(point[0]), float(point[1]), z_pos))
                markers.append(line)
                marker_id += 1

            home_pad = default_cfg.get('home_pad', {})
            pad = Marker()
            pad.header.frame_id = 'odom'
            pad.header.stamp = timestamp
            pad.ns = 'fleet_home_pads'
            pad.id = marker_id
            pad.type = Marker.CYLINDER
            pad.action = Marker.ADD
            pad.pose.position.x = float(home_pad.get('x', 0.0))
            pad.pose.position.y = float(home_pad.get('y', 0.0))
            pad.pose.position.z = 0.015
            pad.pose.orientation.w = 1.0
            pad.scale.x = 0.40
            pad.scale.y = 0.40
            pad.scale.z = 0.03
            pad.color.r = float(color[0])
            pad.color.g = float(color[1])
            pad.color.b = float(color[2])
            pad.color.a = 0.38
            markers.append(pad)
            marker_id += 1

            label = Marker()
            label.header.frame_id = 'odom'
            label.header.stamp = timestamp
            label.ns = 'fleet_lane_labels'
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(home_pad.get('x', 0.0)) + 0.55
            label.pose.position.y = float(home_pad.get('y', 0.0))
            label.pose.position.z = 0.18
            label.pose.orientation.w = 1.0
            label.scale.z = 0.14
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.95
            label.text = self._lane_label(-1, default_cfg)
            markers.append(label)
        return markers

    def _build_pending_waypoint_markers(self, timestamp, start_id):
        markers = []
        marker_id = start_id
        for robot_id, waypoints in sorted(self.pending_waypoints.items()):
            if not waypoints:
                continue
            line = Marker()
            line.header.frame_id = 'odom'
            line.header.stamp = timestamp
            line.ns = 'fleet_pending_waypoints'
            line.id = marker_id
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.05
            line.color.r = 1.0
            line.color.g = 0.55
            line.color.b = 0.0
            line.color.a = 0.95
            if robot_id in self.robot_states:
                x_pos, y_pos, z_pos = self.robot_states[robot_id].get('position', (0.0, 0.0, 0.0))
                line.points.append(Point(x=x_pos, y=y_pos, z=z_pos + 0.08))
            for x_pos, y_pos, z_pos in waypoints:
                line.points.append(Point(x=x_pos, y=y_pos, z=z_pos + 0.08))
            markers.append(line)
            marker_id += 1

            for waypoint_index, (x_pos, y_pos, z_pos) in enumerate(waypoints):
                marker = Marker()
                marker.header.frame_id = 'odom'
                marker.header.stamp = timestamp
                marker.ns = 'fleet_pending_waypoints'
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = x_pos
                marker.pose.position.y = y_pos
                marker.pose.position.z = z_pos + 0.1
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.14
                marker.scale.y = 0.14
                marker.scale.z = 0.14
                marker.color.r = 1.0
                marker.color.g = 0.85
                marker.color.b = 0.1
                marker.color.a = 0.95
                markers.append(marker)
                marker_id += 1

                label = Marker()
                label.header.frame_id = 'odom'
                label.header.stamp = timestamp
                label.ns = 'fleet_pending_waypoint_labels'
                label.id = marker_id
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = x_pos
                label.pose.position.y = y_pos
                label.pose.position.z = z_pos + 0.28
                label.pose.orientation.w = 1.0
                label.scale.z = 0.12
                label.color.r = 1.0
                label.color.g = 1.0
                label.color.b = 1.0
                label.color.a = 0.95
                label.text = f'robot_{robot_id} wp{waypoint_index}'
                markers.append(label)
                marker_id += 1
        return markers

    def _sensor_origin_body(self, sensor_name):
        sensor_x, sensor_y = self.sensor_positions[sensor_name]
        return self.robot_center_offset_x - sensor_x, self.robot_center_offset_y - sensor_y

    def _geometry_body_yaw(self, yaw):
        return yaw + math.pi

    def _sensor_direction_body(self, sensor_name):
        directions = {
            'front': (1.0, 0.0),
            'front_left': (1.0, -1.0),
            'front_right': (1.0, 1.0),
            'left': (0.0, -1.0),
            'right': (0.0, 1.0),
            'back': (-1.0, 0.0),
        }
        dir_x, dir_y = directions[sensor_name]
        norm = math.hypot(dir_x, dir_y)
        return dir_x / norm, dir_y / norm

    def _make_marker_point(self, x_pos, y_pos, z_pos=0.0):
        point = Point()
        point.x = x_pos
        point.y = y_pos
        point.z = z_pos
        return point

    def _body_point_to_world(self, body_x, body_y, center_x, center_y, yaw):
        world_x = body_x * math.cos(yaw) - body_y * math.sin(yaw)
        world_y = body_x * math.sin(yaw) + body_y * math.cos(yaw)
        return center_x + world_x, center_y + world_y

    def _sensor_hit_active(self, sensor_name, sensor_range):
        if not math.isfinite(sensor_range):
            return False
        if sensor_name == 'front':
            return sensor_range <= self.obstacle_detect_distance_front_center
        if sensor_name in {'front_left', 'front_right'}:
            return sensor_range <= self.obstacle_detect_distance_front
        return sensor_range <= self.obstacle_detect_distance

    def _ir_cone_color(self, sensor_name):
        if sensor_name == 'front':
            return 1.0, 0.35, 0.25
        if sensor_name in {'front_left', 'front_right'}:
            return 1.0, 0.75, 0.20
        if sensor_name in {'left', 'right'}:
            return 0.20, 0.85, 1.0
        return 0.55, 0.75, 1.0

    def _build_ir_markers(self, robot_id, state, timestamp, start_id):
        markers = []
        x_pos, y_pos, z_pos = state.get('position', (0.0, 0.0, 0.0))
        yaw = float(state.get('yaw', 0.0))
        sensor_ranges = state.get('sensor_ranges', {})
        sensor_max_ranges = state.get('sensor_max_ranges', {})
        marker_id = start_id

        for sensor_name in self.sensor_order:
            origin_body_x, origin_body_y = self._sensor_origin_body(sensor_name)
            geometry_yaw = self._geometry_body_yaw(yaw)
            origin_x, origin_y = self._body_point_to_world(origin_body_x, origin_body_y, x_pos, y_pos, geometry_yaw)
            dir_body_x, dir_body_y = self._sensor_direction_body(sensor_name)
            sensor_yaw = geometry_yaw + math.atan2(dir_body_y, dir_body_x)
            sensor_range = sensor_ranges.get(sensor_name, float('inf'))
            range_max = sensor_max_ranges.get(sensor_name, self.ir_display_range_max)
            display_range = range_max
            if math.isfinite(sensor_range):
                display_range = max(self.ir_display_range_min, min(sensor_range, range_max))
            display_range = min(display_range, self.ir_display_range_max)
            color_r, color_g, color_b = self._ir_cone_color(sensor_name)

            arc_points = []
            for step in range(13):
                ratio = step / 12.0
                ray_angle = sensor_yaw - self.ir_half_fov + (2.0 * self.ir_half_fov * ratio)
                arc_points.append(
                    self._make_marker_point(
                        origin_x + display_range * math.cos(ray_angle),
                        origin_y + display_range * math.sin(ray_angle),
                        z_pos + 0.08,
                    )
                )

            outline = Marker()
            outline.header.frame_id = 'odom'
            outline.header.stamp = timestamp
            outline.ns = f'fleet_ir_edges_robot_{robot_id}'
            outline.id = marker_id
            outline.type = Marker.LINE_STRIP
            outline.action = Marker.ADD
            outline.pose.orientation.w = 1.0
            outline.scale.x = 0.015
            outline.color.r = color_r
            outline.color.g = color_g
            outline.color.b = color_b
            outline.color.a = 0.9
            outline.points.append(self._make_marker_point(origin_x, origin_y, z_pos + 0.09))
            outline.points.extend(arc_points)
            outline.points.append(self._make_marker_point(origin_x, origin_y, z_pos + 0.09))
            markers.append(outline)
            marker_id += 1

            if self._sensor_hit_active(sensor_name, sensor_range):
                hit = Marker()
                hit.header.frame_id = 'odom'
                hit.header.stamp = timestamp
                hit.ns = f'fleet_ir_hits_robot_{robot_id}'
                hit.id = marker_id
                hit.type = Marker.SPHERE
                hit.action = Marker.ADD
                hit.pose.position.x = origin_x + display_range * math.cos(sensor_yaw)
                hit.pose.position.y = origin_y + display_range * math.sin(sensor_yaw)
                hit.pose.position.z = z_pos + 0.11
                hit.pose.orientation.w = 1.0
                hit.scale.x = 0.06
                hit.scale.y = 0.06
                hit.scale.z = 0.06
                hit.color.r = 1.0
                hit.color.g = 0.12
                hit.color.b = 0.12
                hit.color.a = 0.95
                markers.append(hit)
                marker_id += 1

        return markers

    def _command_robot_target(self, parts):
        if len(parts) >= 2:
            try:
                robot_id = int(parts[1])
            except ValueError:
                self.get_logger().warn(f'Invalid robot id in command: {" ".join(parts)}')
                return None
            if robot_id not in self.robot_states:
                self.get_logger().warn(f'robot_{robot_id} is not active')
                return None
            return robot_id
        if self.selected_robot_id is None:
            self.get_logger().warn('No selected robot')
            return None
        return self.selected_robot_id

    def _nearest_robot_id(self, x_pos, y_pos):
        nearest_id = None
        nearest_distance = None
        for robot_id, state in self.robot_states.items():
            robot_x, robot_y, _ = state.get('position', (0.0, 0.0, 0.0))
            distance = math.hypot(x_pos - robot_x, y_pos - robot_y)
            if nearest_distance is None or distance < nearest_distance:
                nearest_distance = distance
                nearest_id = robot_id
        return nearest_id

    def _next_available_robot_id(self):
        for robot_id in range(self.max_robot_slots):
            if robot_id not in self.spawned_robot_ids:
                return robot_id
        return None

    def _remove_robot_by_id(self, robot_id):
        if robot_id not in self.robot_states:
            self.get_logger().warn(f'robot_{robot_id} is not active')
            return
        request = DespawnRobot.Request()
        request.robot_id = robot_id
        response = DespawnRobot.Response()
        self.despawn_robot_cb(request, response)
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message)


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseFleetManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for robot_id in list(node.robot_runtime):
            node._stop_robot_runtime(robot_id)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()