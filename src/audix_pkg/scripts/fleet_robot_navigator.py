#!/usr/bin/env python3

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def quat_to_yaw(x_pos, y_pos, z_pos, w_pos):
    siny_cosp = 2.0 * (w_pos * z_pos + x_pos * y_pos)
    cosy_cosp = 1.0 - 2.0 * (y_pos * y_pos + z_pos * z_pos)
    return math.atan2(siny_cosp, cosy_cosp)


class FleetRobotNavigator(Node):
    def __init__(self):
        super().__init__('fleet_robot_navigator')

        self.declare_parameter('control_period_sec', 0.05)
        self.declare_parameter('position_tolerance', 0.24)
        self.declare_parameter('heading_gain', 1.6)
        self.declare_parameter('linear_gain', 0.9)
        self.declare_parameter('lateral_gain', 0.9)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_lateral_speed', 0.35)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('cmd_forward_sign', -1.0)
        self.declare_parameter('cmd_lateral_sign', 1.0)
        self.declare_parameter('lift_dwell_time', 2.0)
        self.declare_parameter('lowering_dwell_time', 1.5)
        self.declare_parameter('obstacle_stop_distance', 0.18)
        self.declare_parameter('obstacle_escape_speed', 0.16)

        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.lateral_gain = float(self.get_parameter('lateral_gain').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_lateral_speed = float(self.get_parameter('max_lateral_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.cmd_forward_sign = float(self.get_parameter('cmd_forward_sign').value)
        self.cmd_lateral_sign = float(self.get_parameter('cmd_lateral_sign').value)
        self.lift_dwell_time = float(self.get_parameter('lift_dwell_time').value)
        self.lowering_dwell_time = float(self.get_parameter('lowering_dwell_time').value)
        self.obstacle_stop_distance = float(self.get_parameter('obstacle_stop_distance').value)
        self.obstacle_escape_speed = float(self.get_parameter('obstacle_escape_speed').value)
        control_period = float(self.get_parameter('control_period_sec').value)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lift_pub = self.create_publisher(Float64, 'scissor_lift/slider', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Path, 'mission_waypoints', self.on_path, 10)
        self.create_subscription(String, 'mission_descriptor', self.on_descriptor, 10)
        self.create_subscription(String, 'mission_control', self.on_control, 10)
        for key, topic in {
            'front': 'ir_front/scan',
            'front_left': 'ir_front_right/scan',
            'front_right': 'ir_front_left/scan',
            'left': 'ir_right/scan',
            'right': 'ir_left/scan',
            'back': 'ir_back/scan',
        }.items():
            self.create_subscription(LaserScan, topic, lambda msg, name=key: self.on_scan(name, msg), 10)

        self.create_timer(control_period, self.control_loop)

        self.position = None
        self.yaw = 0.0
        self.path_points = []
        self.dwell_times = []
        self.route_scan_yaws = []
        self.route_lift_heights = []
        self.current_index = 0
        self.paused = False
        self.cancelled = False
        self.settle_state = None
        self.sensor_ranges = {
            'front': float('inf'),
            'front_left': float('inf'),
            'front_right': float('inf'),
            'left': float('inf'),
            'right': float('inf'),
            'back': float('inf'),
        }

    def on_odom(self, msg):
        self.position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.yaw = quat_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)

    def on_path(self, msg):
        self.path_points = [pose.pose.position for pose in msg.poses]
        self.current_index = 0
        self.cancelled = False
        self.paused = False
        self.settle_state = None

    def on_descriptor(self, msg):
        try:
            descriptor = json.loads(msg.data)
        except json.JSONDecodeError:
            self.dwell_times = []
            self.route_scan_yaws = []
            self.route_lift_heights = []
            return
        self.dwell_times = [float(waypoint.get('dwell_time', 0.0)) for waypoint in descriptor.get('waypoints', [])]
        self.route_scan_yaws = [waypoint.get('scan_yaw') for waypoint in descriptor.get('waypoints', [])]
        self.route_lift_heights = [float(waypoint.get('lift_height', 0.0)) for waypoint in descriptor.get('waypoints', [])]

    def on_scan(self, sensor_name, msg):
        valid_ranges = [
            sample
            for sample in msg.ranges
            if not math.isnan(sample)
            and sample >= max(msg.range_min, 0.01)
            and sample < msg.range_max
        ]
        self.sensor_ranges[sensor_name] = min(valid_ranges) if valid_ranges else float('inf')

    def on_control(self, msg):
        command = msg.data.strip().lower()
        if command == 'pause':
            self.paused = True
            self.publish_stop()
        elif command == 'resume':
            self.paused = False
        elif command == 'cancel':
            self.cancelled = True
            self.current_index = len(self.path_points)
            self.settle_state = None
            self.publish_lift(0.0)
            self.publish_stop()

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def normalize(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def publish_lift(self, value):
        msg = Float64()
        msg.data = max(0.0, min(1.0, float(value)))
        self.lift_pub.publish(msg)

    def _needs_waypoint_settle(self, index):
        dwell = self.dwell_times[index] if index < len(self.dwell_times) else 0.0
        lift = self.route_lift_heights[index] if index < len(self.route_lift_heights) else 0.0
        scan_yaw = self.route_scan_yaws[index] if index < len(self.route_scan_yaws) else None
        return abs(dwell) > 1e-6 or abs(lift) > 1e-6 or scan_yaw is not None

    def _advance_waypoint(self):
        self.publish_lift(0.0)
        self.current_index += 1
        self.settle_state = None

    def _begin_waypoint_settle(self):
        target_yaw = None
        if self.current_index < len(self.route_scan_yaws):
            target_yaw = self.route_scan_yaws[self.current_index]
        self.settle_state = {
            'state': 'idle',
            'origin_yaw': self.yaw,
            'scan_target': self.normalize(float(target_yaw)) if target_yaw is not None else None,
            'start_sec': self.now_sec(),
        }

    def _handle_waypoint_settle(self):
        if self.settle_state is None:
            return False
        state = self.settle_state['state']
        now = self.now_sec()
        dwell_time = self.dwell_times[self.current_index] if self.current_index < len(self.dwell_times) else 0.0
        dwell_time = max(dwell_time, self.lift_dwell_time)
        lift_height = self.route_lift_heights[self.current_index] if self.current_index < len(self.route_lift_heights) else 0.0

        if state == 'idle':
            self.publish_stop()
            if self.settle_state['scan_target'] is None and lift_height <= 0.0 and dwell_time <= 0.0:
                self._advance_waypoint()
                return True
            self.settle_state['state'] = 'rotating_to_scan'
            self.settle_state['start_sec'] = now
            return True

        if state == 'rotating_to_scan':
            target = self.settle_state['scan_target']
            if target is None:
                self.settle_state['state'] = 'lifting'
                self.settle_state['start_sec'] = now
                self.publish_stop()
                return True
            err = self.normalize(target - self.yaw)
            elapsed = now - self.settle_state['start_sec']
            wz = math.copysign(max(0.15, min(abs(err) * 2.0, self.max_angular_speed)), err)
            cmd = Twist()
            cmd.angular.z = wz
            self.cmd_pub.publish(cmd)
            if abs(err) < math.radians(3.0) or elapsed > 5.0:
                self.publish_stop()
                self.settle_state['state'] = 'lifting'
                self.settle_state['start_sec'] = now
            return True

        if state == 'lifting':
            self.publish_stop()
            self.publish_lift(lift_height)
            if now - self.settle_state['start_sec'] >= dwell_time:
                self.settle_state['state'] = 'lowering'
                self.settle_state['start_sec'] = now
            return True

        if state == 'lowering':
            self.publish_stop()
            self.publish_lift(0.0)
            if now - self.settle_state['start_sec'] >= self.lowering_dwell_time:
                self.settle_state['state'] = 'rotating_back'
                self.settle_state['start_sec'] = now
            return True

        if state == 'rotating_back':
            if self.settle_state['scan_target'] is None:
                self.settle_state['state'] = 'done'
                return True
            err = self.normalize(self.settle_state['origin_yaw'] - self.yaw)
            elapsed = now - self.settle_state['start_sec']
            wz = math.copysign(max(0.15, min(abs(err) * 2.0, self.max_angular_speed)), err)
            cmd = Twist()
            cmd.angular.z = wz
            self.cmd_pub.publish(cmd)
            if abs(err) < math.radians(3.0) or elapsed > 5.0:
                self.publish_stop()
                self.settle_state['state'] = 'done'
            return True

        if state == 'done':
            self._advance_waypoint()
            return True

        return False

    def _publish_obstacle_escape(self, heading_error):
        left_clearance = min(self.sensor_ranges['left'], self.sensor_ranges['front_left'])
        right_clearance = min(self.sensor_ranges['right'], self.sensor_ranges['front_right'])
        cmd = Twist()
        if left_clearance > right_clearance + 0.03:
            cmd.linear.y = self.cmd_lateral_sign * self.obstacle_escape_speed
        elif right_clearance > left_clearance + 0.03:
            cmd.linear.y = -self.cmd_lateral_sign * self.obstacle_escape_speed
        else:
            cmd.angular.z = clamp(self.heading_gain * heading_error, -self.max_angular_speed, self.max_angular_speed)
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        if self.position is None or not self.path_points or self.paused or self.cancelled:
            self.publish_stop()
            return

        if self.current_index >= len(self.path_points):
            self.publish_stop()
            return

        if self._handle_waypoint_settle():
            return

        target = self.path_points[self.current_index]
        dx_world = target.x - self.position.x
        dy_world = target.y - self.position.y
        distance = math.hypot(dx_world, dy_world)

        if distance <= self.position_tolerance:
            if self._needs_waypoint_settle(self.current_index):
                self._begin_waypoint_settle()
            else:
                self._advance_waypoint()
            self.publish_stop()
            return

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        dx_body = cos_yaw * dx_world + sin_yaw * dy_world
        dy_body = -sin_yaw * dx_world + cos_yaw * dy_world
        target_heading = math.atan2(dy_world, dx_world)
        heading_error = math.atan2(math.sin(target_heading - self.yaw), math.cos(target_heading - self.yaw))

        closest_front = min(
            self.sensor_ranges['front'],
            self.sensor_ranges['front_left'],
            self.sensor_ranges['front_right'],
        )
        if closest_front <= self.obstacle_stop_distance:
            self._publish_obstacle_escape(heading_error)
            return

        cmd = Twist()
        cmd.linear.x = clamp(
            self.cmd_forward_sign * self.linear_gain * dx_body,
            -self.max_linear_speed,
            self.max_linear_speed,
        )
        cmd.linear.y = clamp(
            self.cmd_lateral_sign * self.lateral_gain * dy_body,
            -self.max_lateral_speed,
            self.max_lateral_speed,
        )
        cmd.angular.z = clamp(self.heading_gain * heading_error, -self.max_angular_speed, self.max_angular_speed)
        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = FleetRobotNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()