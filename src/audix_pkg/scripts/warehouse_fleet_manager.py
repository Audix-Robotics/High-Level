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
        self.generated_model_dir = FilePath(tempfile.gettempdir()) / 'audix_fleet_models'
        self.generated_model_dir.mkdir(parents=True, exist_ok=True)
        self.robot_description_xml = FilePath(self.robot_model_path).read_text(encoding='utf-8')
        self.controllers_config = yaml.safe_load(FilePath(self.controllers_yaml_path).read_text(encoding='utf-8')) or {}

        self.sensor_positions = {
            'back': (0.16255, -0.00323),
            'left': (0.00273, 0.15504),
            'front_left': (-0.17753, 0.12688),
            'front': (-0.20195, 0.00482),
            'front_right': (-0.18323, -0.11962),
            'right': (-0.00537, -0.15346),
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
            default_fleet.get('robots', {}).get(f'robot_{robot_index}', {}).get('lane', robot_index % 5)
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
            response.success = False
            response.message = f'robot_{robot_id} already exists'
            return response

        lane_config = self._lane_config(lane_id)
        spawn_x, spawn_y, spawn_yaw = self._spawn_pose_for_robot(robot_id, lane_id, lane_config)
        success, message = self._spawn_robot_at_pose(robot_id, lane_id, spawn_x, spawn_y, spawn_yaw)
        if not success:
            if 'already exists' in message.lower() or 'entity named' in message.lower():
                self._register_robot_state(robot_id, lane_id, lane_config, spawn_x, spawn_y)
                self.selected_robot_id = robot_id
                response.success = True
                response.message = f'Attached to existing robot_{robot_id}'
                return response
            response.success = False
            response.message = message
            return response

        self.selected_robot_id = robot_id
        self.pending_waypoints.pop(robot_id, None)
        self.publish_fleet_status()
        response.success = True
        response.message = (
            f'Spawned robot_{robot_id} in {lane_config.get("name", lane_id)} '
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
                    waypoint['dwell_time'] = max(0.0, float(pose.orientation.y))
                    waypoint['lift_height'] = max(0.0, float(pose.orientation.x))
                    waypoint['scan_yaw'] = math.atan2(
                        2.0 * pose.orientation.w * pose.orientation.z,
                        1.0 - 2.0 * pose.orientation.z * pose.orientation.z,
                    )
                waypoints.append(waypoint)
            descriptor = {
                'mission_name': mission_name or f'custom_robot_{robot_id}',
                'source': 'custom',
                'waypoints': waypoints,
            }
        else:
            mission = self.missions_config.get('missions', {}).get(mission_name)
            if mission is None:
                response.success = False
                response.message = f'Unknown mission: {mission_name}'
                return response
            descriptor = {
                'mission_name': mission_name,
                'source': 'template',
                'waypoints': mission.get('waypoints', []),
            }

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
        lane_id, lane_name = self._lane_from_y(position.y)
        state = self.robot_states[robot_id]
        orientation = msg.pose.pose.orientation
        state['position'] = (position.x, position.y, position.z)
        state['yaw'] = quat_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        state['lane_id'] = lane_id
        state['lane_name'] = lane_name
        trail = state.setdefault('trail', [])
        if not trail:
            trail.append((position.x, position.y, position.z))
            return
        last_x, last_y, last_z = trail[-1]
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
            robot_id = self._next_available_robot_id()
            if robot_id is None:
                self.get_logger().warn('No free robot slots available for RViz spawn')
                return
            lane_id, _ = self._lane_from_y(y_pos)
            success, message = self._spawn_robot_at_pose(robot_id, lane_id, x_pos, y_pos, 0.0)
            if success:
                self.selected_robot_id = robot_id
                self.pending_waypoints.pop(robot_id, None)
                self.rviz_mode = 'waypoint'
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
        parts = command.split()
        keyword = parts[0].lower()

        if keyword == 'mode' and len(parts) >= 2:
            mode = parts[1].strip().lower()
            if mode in {'idle', 'spawn', 'select', 'waypoint'}:
                self.rviz_mode = mode
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

    def _lane_config(self, lane_id):
        return self.warehouse_config.get('lanes', {}).get(f'lane_{lane_id}', {})

    def _spawn_pose_for_robot(self, robot_id, lane_id, lane_config, use_default_fleet_pose=False):
        if use_default_fleet_pose:
            defaults = self.warehouse_config.get('default_fleet', {}).get('robots', {})
            default_robot = defaults.get(f'robot_{robot_id}', {})
            spawn_x = float(default_robot.get('spawn_x', 0.0))
            spawn_y = float(default_robot.get('spawn_y', lane_config.get('center_y', 0.0)))
            spawn_yaw = float(default_robot.get('spawn_yaw', 0.0))
            return spawn_x, spawn_y, spawn_yaw

        spawn_x = 0.0
        spawn_y = float(lane_config.get('center_y', 0.0))
        spawn_yaw = 0.0
        return spawn_x, spawn_y, spawn_yaw

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
            )
            if not runtime_success:
                self._delete_robot_gz(robot_name)
                self._stop_robot_runtime(robot_id)
                self.spawned_robot_ids.discard(robot_id)
                self.robot_states.pop(robot_id, None)
                return False, runtime_message
            return True, f'Spawned {robot_name} at ({spawn_x:.2f}, {spawn_y:.2f})'
        return False, message

    def _build_namespaced_model_xml(self, robot_name, controller_params_path):
        with open(self.robot_model_path, 'r', encoding='utf-8') as model_file:
            robot_xml = model_file.read()

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
            'yaw': 0.0,
            'lane_id': lane_id,
            'lane_name': lane_config.get('name', f'Lane {lane_id}'),
            'current_mission': '',
            'waypoints_completed': 0,
            'total_waypoints': 0,
            'mission_progress_pct': 0.0,
            'robot_state': 'IDLE',
            'trail': [(spawn_x, spawn_y, self.spawn_z)],
            'sensor_ranges': {sensor_name: float('inf') for sensor_name in self.sensor_order},
            'sensor_max_ranges': {sensor_name: self.ir_display_range_max for sensor_name in self.sensor_order},
        }
        if robot_id not in self.robot_odom_subs:
            self.robot_odom_subs[robot_id] = self.create_subscription(
                Odometry,
                f'/robot_{robot_id}/odom',
                partial(self.on_robot_odom, robot_id=robot_id),
                10,
            )
        if robot_id not in self.robot_scan_subs:
            self.robot_scan_subs[robot_id] = []
            for sensor_name in self.sensor_order:
                self.robot_scan_subs[robot_id].append(
                    self.create_subscription(
                        LaserScan,
                        f'/robot_{robot_id}/ir_{sensor_name}/scan',
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
        self.robot_states[robot_id]['total_waypoints'] = len(descriptor['waypoints'])
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
        indented_description = '\n'.join(
            f'      {line}' for line in self.robot_description_xml.splitlines()
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

    def _start_robot_runtime(self, robot_id, robot_name, controller_params_path):
        self._stop_robot_runtime(robot_id)
        try:
            params_path = self._write_robot_state_publisher_params(robot_id)
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
                'navigator',
                [
                    'ros2', 'run', 'audix', 'fleet_robot_navigator.py',
                    '--ros-args',
                    '-r', f'__ns:=/{robot_name}',
                    '-p', 'use_sim_time:=true',
                    '-p', 'position_tolerance:=0.24',
                    '-p', 'max_linear_speed:=0.35',
                    '-p', 'max_lateral_speed:=0.35',
                    '-p', 'max_angular_speed:=1.0',
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
                    '-p', 'publish_odom:=false',
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
            self._spawn_process(
                robot_id,
                'odom_tf',
                [
                    'ros2', 'run', 'audix', 'odom_tf_broadcaster.py',
                    '--ros-args',
                    '-p', 'use_sim_time:=true',
                    '-p', f'odom_topic:=/{robot_name}/odom',
                    '-p', 'odom_frame:=odom',
                    '-p', f'base_frame:={robot_name}/base_footprint',
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
            f'fleet_robot_navigator.py.*__ns:=/{robot_name}',
            f'mecanum_kinematics.py.*__ns:=/{robot_name}',
            f'scissor_lift_mapper.py.*__ns:=/{robot_name}',
            f'odom_tf_broadcaster.py.*odom_topic:=/{robot_name}/odom',
            f'parameter_bridge.*/{robot_name}/cmd_vel@',
        ]
        for pattern in fallback_patterns:
            subprocess.run(['pkill', '-f', pattern], check=False)

    def _spawn_robot_gz(self, robot_name, spawn_x, spawn_y, spawn_yaw, controller_params_path):
        robot_xml = self._build_namespaced_model_xml(robot_name, controller_params_path)
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
                    '-Y', str(spawn_yaw),
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


    def _lane_from_y(self, y_position):
        lanes = self.warehouse_config.get('lanes', {})
        for lane_key, lane_config in lanes.items():
            if lane_config.get('min_y', -999.0) <= y_position <= lane_config.get('max_y', 999.0):
                return int(lane_key.split('_')[-1]), lane_config.get('name', lane_key)

        nearest_lane = min(
            lanes.items(),
            key=lambda item: abs(float(item[1].get('center_y', 0.0)) - y_position),
        )
        return int(nearest_lane[0].split('_')[-1]), nearest_lane[1].get('name', nearest_lane[0])

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
        return -0.16 - sensor_x, -0.15 - sensor_y

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
            origin_x, origin_y = self._body_point_to_world(origin_body_x, origin_body_y, x_pos, y_pos, yaw + math.pi)
            dir_body_x, dir_body_y = self._sensor_direction_body(sensor_name)
            sensor_yaw = yaw + math.pi + math.atan2(dir_body_y, dir_body_x)
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

            cone = Marker()
            cone.header.frame_id = 'odom'
            cone.header.stamp = timestamp
            cone.ns = f'fleet_ir_cones_robot_{robot_id}'
            cone.id = marker_id
            cone.type = Marker.TRIANGLE_LIST
            cone.action = Marker.ADD
            cone.pose.orientation.w = 1.0
            cone.scale.x = 1.0
            cone.scale.y = 1.0
            cone.scale.z = 1.0
            cone.color.r = color_r
            cone.color.g = color_g
            cone.color.b = color_b
            cone.color.a = 0.25
            for step in range(12):
                cone.points.append(self._make_marker_point(origin_x, origin_y, z_pos + 0.08))
                cone.points.append(arc_points[step])
                cone.points.append(arc_points[step + 1])
            markers.append(cone)
            marker_id += 1

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

            if math.isfinite(sensor_range):
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()