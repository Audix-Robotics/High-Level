#!/usr/bin/env python3

import json
import math
import os
from functools import partial

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from audix.msg import FleetStatus
from audix.msg import RobotStatus


class WarehouseMissionManager(Node):
    def __init__(self):
        super().__init__('warehouse_mission_manager')

        pkg_share = get_package_share_directory('audix')
        self.declare_parameter(
            'missions_config_path',
            os.path.join(pkg_share, 'config', 'warehouse_missions.yaml'),
        )
        self.declare_parameter(
            'warehouse_config_path',
            os.path.join(pkg_share, 'config', 'warehouse_lanes.yaml'),
        )
        self.declare_parameter(
            'robot_params_path',
            os.path.join(pkg_share, 'config', 'warehouse_robot_params.yaml'),
        )
        self.declare_parameter('max_robots', 10)

        self.missions_config = self._load_yaml('missions_config_path')
        self.warehouse_config = self._load_yaml('warehouse_config_path')
        self.robot_params = self._load_yaml('robot_params_path')
        self.max_robots = int(self.get_parameter('max_robots').value)
        self.waypoint_tolerance = float(
            self.robot_params.get('fleet', {}).get('waypoint_tolerance', 0.24)
        )
        self.collision_buffer = float(
            self.robot_params.get('fleet', {}).get('collision_buffer', 0.5)
        )

        self.robot_positions = {}
        self.robot_missions = {}
        self.descriptors = {}
        self.active_robot_ids = set()

        self.status_update_pub = self.create_publisher(
            RobotStatus,
            '/fleet/robot_status_updates',
            10,
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/fleet/mission_markers', 10)
        self.create_subscription(FleetStatus, '/fleet/status', self.on_fleet_status, 10)

        for robot_id in range(self.max_robots):
            self.create_subscription(
                Path,
                f'/robot_{robot_id}/mission_waypoints',
                partial(self.on_mission_waypoints, robot_id=robot_id),
                10,
            )
            self.create_subscription(
                String,
                f'/robot_{robot_id}/mission_descriptor',
                partial(self.on_mission_descriptor, robot_id=robot_id),
                10,
            )
            self.create_subscription(
                String,
                f'/robot_{robot_id}/mission_control',
                partial(self.on_mission_control, robot_id=robot_id),
                10,
            )
            self.create_subscription(
                Odometry,
                f'/robot_{robot_id}/odom',
                partial(self.on_robot_odom, robot_id=robot_id),
                10,
            )

        self.create_timer(0.1, self.check_mission_progress)

    def _load_yaml(self, parameter_name):
        path = str(self.get_parameter(parameter_name).value)
        with open(path, 'r', encoding='utf-8') as config_file:
            return yaml.safe_load(config_file) or {}

    def on_mission_descriptor(self, msg, robot_id):
        try:
            self.descriptors[robot_id] = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Invalid mission descriptor for robot_{robot_id}')

    def on_mission_waypoints(self, msg, robot_id):
        descriptor = self.descriptors.get(robot_id, {})
        mission_name = descriptor.get('mission_name', f'robot_{robot_id}_mission')
        waypoints = [
            {
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
                'z': pose.pose.position.z,
                'dwell_time': 0.0,
            }
            for pose in msg.poses
        ]
        if descriptor.get('waypoints') and len(descriptor['waypoints']) == len(waypoints):
            for waypoint, source in zip(waypoints, descriptor['waypoints']):
                waypoint['dwell_time'] = float(source.get('dwell_time', 0.0))

        is_valid, warning = self.validate_mission_path(robot_id, waypoints)
        self.robot_missions[robot_id] = {
            'current_mission': mission_name,
            'waypoints': waypoints,
            'waypoint_index': 0,
            'progress_pct': 0.0,
            'state': 'MOVING' if is_valid and waypoints else 'IDLE',
            'paused': False,
            'warning': warning,
            'dwell_started_sec': None,
        }
        if warning:
            self.get_logger().warn(f'robot_{robot_id}: {warning}')
        self._publish_robot_status(robot_id)
        self.marker_pub.publish(self._build_mission_markers())

    def on_fleet_status(self, msg):
        self.active_robot_ids = {int(robot.robot_id) for robot in msg.robots}
        inactive_ids = [robot_id for robot_id in self.robot_missions if robot_id not in self.active_robot_ids]
        removed_any = False
        for robot_id in inactive_ids:
            self.robot_missions.pop(robot_id, None)
            self.robot_positions.pop(robot_id, None)
            self.descriptors.pop(robot_id, None)
            removed_any = True
        if removed_any:
            self.marker_pub.publish(self._build_mission_markers())

    def validate_mission_path(self, robot_id, waypoints):
        bounds = self.warehouse_config.get('warehouse', {}).get('bounds', {})
        lanes = self.warehouse_config.get('lanes', {})
        lane_margin = float(self.robot_params.get('fleet', {}).get('lane_margin', 0.3))
        assigned_lane = self.robot_params.get('robots', {}).get(f'robot_{robot_id}', {}).get('assigned_lane')
        lane_config = lanes.get(f'lane_{assigned_lane}', {}) if assigned_lane is not None else {}
        warnings = []

        for waypoint in waypoints:
            if waypoint['x'] < float(bounds.get('min_x', -999.0)) or waypoint['x'] > float(bounds.get('max_x', 999.0)):
                return False, 'Waypoint outside warehouse X bounds'
            if waypoint['y'] < float(bounds.get('min_y', -999.0)) or waypoint['y'] > float(bounds.get('max_y', 999.0)):
                return False, 'Waypoint outside warehouse Y bounds'
            if lane_config:
                if waypoint['y'] < float(lane_config.get('min_y', -999.0)) - lane_margin:
                    warnings.append('Mission leaves assigned lane')
                if waypoint['y'] > float(lane_config.get('max_y', 999.0)) + lane_margin:
                    warnings.append('Mission leaves assigned lane')

        shelves = self.warehouse_config.get('scan_shelves', {})
        for waypoint in waypoints:
            for shelf in shelves.values():
                sx, sy, _ = shelf.get('position', [0.0, 0.0, 0.0])
                distance = math.hypot(waypoint['x'] - sx, waypoint['y'] - sy)
                if distance < 0.15:
                    continue
                if distance < 0.35:
                    warnings.append('Mission passes close to a scan shelf')

        for other_robot, mission in self.robot_missions.items():
            if other_robot == robot_id:
                continue
            for waypoint in waypoints:
                for other_waypoint in mission.get('waypoints', []):
                    if math.hypot(
                        waypoint['x'] - other_waypoint['x'],
                        waypoint['y'] - other_waypoint['y'],
                    ) < self.collision_buffer:
                        warnings.append('Mission is close to another robot route')

        warning_message = '; '.join(sorted(set(warnings)))
        return True, warning_message

    def on_mission_control(self, msg, robot_id):
        mission = self.robot_missions.get(robot_id)
        if mission is None:
            return

        command = msg.data.strip().lower()
        if command == 'pause':
            mission['paused'] = True
            mission['state'] = 'WAITING'
        elif command == 'resume':
            mission['paused'] = False
            mission['state'] = 'MOVING'
        elif command == 'cancel':
            mission['paused'] = False
            mission['state'] = 'IDLE'
            mission['waypoints'] = []
            mission['waypoint_index'] = 0
            mission['progress_pct'] = 0.0

        self._publish_robot_status(robot_id)
        self.marker_pub.publish(self._build_mission_markers())

    def on_robot_odom(self, msg, robot_id):
        self.robot_positions[robot_id] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )

    def check_mission_progress(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        for robot_id, mission in list(self.robot_missions.items()):
            waypoints = mission.get('waypoints', [])
            if not waypoints:
                continue
            if mission.get('paused') or mission.get('state') == 'ERROR':
                self._publish_robot_status(robot_id)
                continue

            current_index = mission.get('waypoint_index', 0)
            if current_index >= len(waypoints):
                mission['state'] = 'IDLE'
                mission['progress_pct'] = 100.0
                self._publish_robot_status(robot_id)
                continue

            if robot_id not in self.robot_positions:
                continue

            position = self.robot_positions[robot_id]
            target = waypoints[current_index]
            distance = math.hypot(position[0] - target['x'], position[1] - target['y'])
            if distance <= self.waypoint_tolerance:
                dwell_time = float(target.get('dwell_time', 0.0))
                if dwell_time > 0.0:
                    if mission['dwell_started_sec'] is None:
                        mission['dwell_started_sec'] = now_sec
                        mission['state'] = 'SCANNING'
                    elif now_sec - mission['dwell_started_sec'] >= dwell_time:
                        mission['waypoint_index'] = current_index + 1
                        mission['dwell_started_sec'] = None
                        mission['state'] = 'MOVING'
                else:
                    mission['waypoint_index'] = current_index + 1
                    mission['state'] = 'MOVING'

            total_waypoints = max(1, len(waypoints))
            completed = min(mission.get('waypoint_index', 0), len(waypoints))
            mission['progress_pct'] = 100.0 * float(completed) / float(total_waypoints)
            self._publish_robot_status(robot_id)

        self.marker_pub.publish(self._build_mission_markers())

    def _publish_robot_status(self, robot_id):
        mission = self.robot_missions.get(robot_id, {})
        position = self.robot_positions.get(robot_id, (0.0, 0.0, 0.0))
        lane_name = self._lane_name_from_position(position[1])

        status = RobotStatus()
        status.robot_id = robot_id
        status.lane_name = lane_name
        status.position = Point(x=position[0], y=position[1], z=position[2])
        status.current_mission = mission.get('current_mission', '')
        status.waypoints_completed = int(min(mission.get('waypoint_index', 0), len(mission.get('waypoints', []))))
        status.total_waypoints = int(len(mission.get('waypoints', [])))
        status.mission_progress_pct = float(mission.get('progress_pct', 0.0))
        status.robot_state = mission.get('state', 'IDLE')
        self.status_update_pub.publish(status)

    def _lane_name_from_position(self, y_position):
        lanes = self.warehouse_config.get('lanes', {})
        for lane in lanes.values():
            if float(lane.get('min_y', -999.0)) <= y_position <= float(lane.get('max_y', 999.0)):
                return lane.get('name', 'Unknown Lane')
        return 'Unknown Lane'

    def _build_mission_markers(self):
        markers = MarkerArray()
        timestamp = self.get_clock().now().to_msg()
        clear = Marker()
        clear.header.frame_id = 'odom'
        clear.header.stamp = timestamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        marker_id = 1

        for robot_id, mission in sorted(self.robot_missions.items()):
            waypoints = mission.get('waypoints', [])
            if not waypoints:
                continue

            line = Marker()
            line.header.frame_id = 'odom'
            line.header.stamp = timestamp
            line.ns = 'mission_paths'
            line.id = marker_id
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.04
            line.color.r = 0.2
            line.color.g = 0.8
            line.color.b = 1.0
            line.color.a = 0.9
            for waypoint in waypoints:
                line.points.append(Point(x=waypoint['x'], y=waypoint['y'], z=0.08))
            markers.markers.append(line)
            marker_id += 1

            for index, waypoint in enumerate(waypoints):
                marker = Marker()
                marker.header.frame_id = 'odom'
                marker.header.stamp = timestamp
                marker.ns = 'mission_waypoints'
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = waypoint['x']
                marker.pose.position.y = waypoint['y']
                marker.pose.position.z = 0.12
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.12
                marker.scale.y = 0.12
                marker.scale.z = 0.12
                marker.color.r = 1.0 if index == mission.get('waypoint_index', 0) else 0.0
                marker.color.g = 0.8
                marker.color.b = 0.2
                marker.color.a = 0.95
                markers.markers.append(marker)
                marker_id += 1

        return markers


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseMissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()