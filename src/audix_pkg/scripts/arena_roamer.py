#!/usr/bin/env python3

import math
import random

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def clamp(value, low, high):
    return max(low, min(high, value))


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ArenaRoamer(Node):
    def __init__(self):
        super().__init__('arena_roamer')

        self.sensor_names = ['front', 'front_left', 'front_right', 'left', 'right', 'back']

        self.declare_parameter('arena_min_x', -4.35)
        self.declare_parameter('arena_max_x', 4.35)
        self.declare_parameter('arena_min_y', -4.35)
        self.declare_parameter('arena_max_y', 4.35)
        self.declare_parameter('goal_margin', 0.80)
        self.declare_parameter('min_goal_separation', 1.20)
        self.declare_parameter('goal_reached_tolerance', 0.32)
        self.declare_parameter('goal_timeout_sec', 18.0)
        self.declare_parameter('sensor_warmup_sec', 2.0)
        self.declare_parameter('sensor_timeout_sec', 0.35)
        self.declare_parameter('odom_timeout_sec', 0.35)
        self.declare_parameter('control_period_sec', 0.05)
        self.declare_parameter('max_linear_speed', 0.42)
        self.declare_parameter('max_lateral_speed', 0.42)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('goal_gain', 0.95)
        self.declare_parameter('wall_gain', 1.15)
        self.declare_parameter('repulsion_gain', 1.75)
        self.declare_parameter('escape_gain', 1.25)
        self.declare_parameter('angular_kp', 1.6)
        self.declare_parameter('obstacle_detect_distance', 0.42)
        self.declare_parameter('obstacle_danger_distance', 0.20)
        self.declare_parameter('obstacle_clear_distance', 0.48)
        self.declare_parameter('startup_clearance_distance', 0.24)
        self.declare_parameter('hard_escape_distance', 0.12)
        self.declare_parameter('hard_escape_speed', 0.10)
        self.declare_parameter('caution_speed', 0.18)
        self.declare_parameter('wall_influence_distance', 0.90)
        self.declare_parameter('obstacle_side_min', 0.14)
        self.declare_parameter('robot_center_offset_x', -0.16)
        self.declare_parameter('robot_center_offset_y', -0.15)
        self.declare_parameter('robot_body_frame_flip_180', True)
        self.declare_parameter('debug_frame_id', 'arena10')
        self.declare_parameter('ir_filter_alpha_rise', 0.16)
        self.declare_parameter('ir_filter_alpha_fall', 0.72)
        self.declare_parameter('path_point_spacing', 0.05)
        self.declare_parameter('random_seed', 7)
        self.declare_parameter('startup_escape_delay_sec', 2.0)
        self.declare_parameter('reorient_heading_threshold_deg', 115.0)
        self.declare_parameter('forward_heading_allowance_deg', 70.0)
        self.declare_parameter('rotate_clearance_bias', 0.35)
        self.declare_parameter('forward_clearance_distance', 0.55)
        self.declare_parameter('lateral_clearance_distance', 0.20)
        self.declare_parameter('escape_hold_sec', 0.9)
        self.declare_parameter('stuck_timeout_sec', 1.0)
        self.declare_parameter('stuck_distance_epsilon', 0.04)
        self.declare_parameter('cmd_forward_sign', -1.0)
        self.declare_parameter('cmd_lateral_sign', 1.0)
        self.declare_parameter('goal_keepout_radius', 0.95)
        self.declare_parameter('ir_low_sample_count', 3)
        self.declare_parameter('cmd_linear_accel_limit', 0.70)
        self.declare_parameter('cmd_linear_decel_limit', 1.20)
        self.declare_parameter('cmd_lateral_accel_limit', 0.85)
        self.declare_parameter('cmd_lateral_decel_limit', 1.40)
        self.declare_parameter('cmd_angular_accel_limit', 2.20)
        self.declare_parameter('cmd_angular_decel_limit', 3.50)
        self.declare_parameter('cmd_deadband_linear', 0.015)
        self.declare_parameter('cmd_deadband_lateral', 0.015)
        self.declare_parameter('cmd_deadband_angular', 0.035)
        self.declare_parameter('control_mode', 'acceptance_path')
        self.declare_parameter('route_name', 'double_pinch_figure8')
        self.declare_parameter('route_loop', False)
        self.declare_parameter('route_waypoint_tolerance', 0.24)
        self.declare_parameter('route_waypoint_dwell_sec', 0.35)
        self.declare_parameter('route_fail_on_timeout', True)

        self.arena_min_x = float(self.get_parameter('arena_min_x').value)
        self.arena_max_x = float(self.get_parameter('arena_max_x').value)
        self.arena_min_y = float(self.get_parameter('arena_min_y').value)
        self.arena_max_y = float(self.get_parameter('arena_max_y').value)
        self.goal_margin = float(self.get_parameter('goal_margin').value)
        self.min_goal_separation = float(self.get_parameter('min_goal_separation').value)
        self.goal_reached_tolerance = float(self.get_parameter('goal_reached_tolerance').value)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.sensor_warmup_sec = float(self.get_parameter('sensor_warmup_sec').value)
        self.sensor_timeout_sec = float(self.get_parameter('sensor_timeout_sec').value)
        self.odom_timeout_sec = float(self.get_parameter('odom_timeout_sec').value)
        self.control_period = float(self.get_parameter('control_period_sec').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_lateral_speed = float(self.get_parameter('max_lateral_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.goal_gain = float(self.get_parameter('goal_gain').value)
        self.wall_gain = float(self.get_parameter('wall_gain').value)
        self.repulsion_gain = float(self.get_parameter('repulsion_gain').value)
        self.escape_gain = float(self.get_parameter('escape_gain').value)
        self.angular_kp = float(self.get_parameter('angular_kp').value)
        self.obstacle_detect_distance = float(self.get_parameter('obstacle_detect_distance').value)
        self.obstacle_danger_distance = float(self.get_parameter('obstacle_danger_distance').value)
        self.obstacle_clear_distance = float(self.get_parameter('obstacle_clear_distance').value)
        self.startup_clearance_distance = float(self.get_parameter('startup_clearance_distance').value)
        self.hard_escape_distance = float(self.get_parameter('hard_escape_distance').value)
        self.hard_escape_speed = float(self.get_parameter('hard_escape_speed').value)
        self.caution_speed = float(self.get_parameter('caution_speed').value)
        self.wall_influence_distance = float(self.get_parameter('wall_influence_distance').value)
        self.obstacle_side_min = float(self.get_parameter('obstacle_side_min').value)
        self.robot_center_offset_x = float(self.get_parameter('robot_center_offset_x').value)
        self.robot_center_offset_y = float(self.get_parameter('robot_center_offset_y').value)
        self.robot_body_frame_flip_180 = bool(self.get_parameter('robot_body_frame_flip_180').value)
        self.debug_frame_id = str(self.get_parameter('debug_frame_id').value)
        self.ir_filter_alpha_rise = float(self.get_parameter('ir_filter_alpha_rise').value)
        self.ir_filter_alpha_fall = float(self.get_parameter('ir_filter_alpha_fall').value)
        self.path_point_spacing = float(self.get_parameter('path_point_spacing').value)
        self.random_seed = int(self.get_parameter('random_seed').value)
        self.startup_escape_delay_sec = float(self.get_parameter('startup_escape_delay_sec').value)
        self.reorient_heading_threshold = math.radians(
            float(self.get_parameter('reorient_heading_threshold_deg').value)
        )
        self.forward_heading_allowance = math.radians(
            float(self.get_parameter('forward_heading_allowance_deg').value)
        )
        self.rotate_clearance_bias = float(self.get_parameter('rotate_clearance_bias').value)
        self.forward_clearance_distance = float(self.get_parameter('forward_clearance_distance').value)
        self.lateral_clearance_distance = float(self.get_parameter('lateral_clearance_distance').value)
        self.escape_hold_sec = float(self.get_parameter('escape_hold_sec').value)
        self.stuck_timeout_sec = float(self.get_parameter('stuck_timeout_sec').value)
        self.stuck_distance_epsilon = float(self.get_parameter('stuck_distance_epsilon').value)
        self.cmd_forward_sign = float(self.get_parameter('cmd_forward_sign').value)
        self.cmd_lateral_sign = float(self.get_parameter('cmd_lateral_sign').value)
        self.goal_keepout_radius = float(self.get_parameter('goal_keepout_radius').value)
        self.ir_low_sample_count = max(1, int(self.get_parameter('ir_low_sample_count').value))
        self.cmd_linear_accel_limit = float(self.get_parameter('cmd_linear_accel_limit').value)
        self.cmd_linear_decel_limit = float(self.get_parameter('cmd_linear_decel_limit').value)
        self.cmd_lateral_accel_limit = float(self.get_parameter('cmd_lateral_accel_limit').value)
        self.cmd_lateral_decel_limit = float(self.get_parameter('cmd_lateral_decel_limit').value)
        self.cmd_angular_accel_limit = float(self.get_parameter('cmd_angular_accel_limit').value)
        self.cmd_angular_decel_limit = float(self.get_parameter('cmd_angular_decel_limit').value)
        self.cmd_deadband_linear = float(self.get_parameter('cmd_deadband_linear').value)
        self.cmd_deadband_lateral = float(self.get_parameter('cmd_deadband_lateral').value)
        self.cmd_deadband_angular = float(self.get_parameter('cmd_deadband_angular').value)
        self.control_mode = str(self.get_parameter('control_mode').value).strip().lower()
        self.route_name = str(self.get_parameter('route_name').value).strip().lower()
        self.route_loop = bool(self.get_parameter('route_loop').value)
        self.route_waypoint_tolerance = float(self.get_parameter('route_waypoint_tolerance').value)
        self.route_waypoint_dwell_sec = float(self.get_parameter('route_waypoint_dwell_sec').value)
        self.route_fail_on_timeout = bool(self.get_parameter('route_fail_on_timeout').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/debug/planned_path', 10)
        self.trail_pub = self.create_publisher(Path, '/debug/robot_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/debug/targets', 10)
        self.state_pub = self.create_publisher(String, '/debug/state', 10)

        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        for key, topic in {
            'front': '/ir_front/scan',
            'front_left': '/ir_front_left/scan',
            'front_right': '/ir_front_right/scan',
            'left': '/ir_left/scan',
            'right': '/ir_right/scan',
            'back': '/ir_back/scan',
        }.items():
            self.create_subscription(LaserScan, topic, lambda msg, k=key: self._ir_cb(k, msg), 10)

        self.ir = {name: float('inf') for name in self.sensor_names}
        self.ir_raw = dict(self.ir)
        self.ir_range_max = {key: 0.30 for key in self.ir}
        self.sensor_last_update_sec = {key: None for key in self.ir}
        self.ir_default_range_min = 0.05
        self.ir_default_range_max = 0.30
        self.ir_half_fov = 0.30543

        self.sensor_positions = {
            'back': (0.00389, -0.15402),
            'right': (-0.15593, 0.00425),
            'front_right': (-0.33619, -0.02391),
            'front': (-0.36061, -0.14597),
            'front_left': (-0.34189, -0.27041),
            'left': (-0.16403, -0.30425),
        }

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.center_x = 0.0
        self.center_y = 0.0
        self.goal_x = None
        self.goal_y = None
        self.goal_started_sec = 0.0
        self.node_started_sec = self.get_clock().now().nanoseconds / 1e9
        self.last_odom_sec = None
        self.last_path_point = None
        self.robot_path = []
        self.randomizer = random.Random(self.random_seed)
        self.state_name = 'WARMUP'
        self.motion_name = 'STOP'
        self.startup_clear_cycles = 0
        self.blocked_started_sec = None
        self.escape_hold_until_sec = 0.0
        self.stuck_motion_name = None
        self.stuck_motion_started_sec = None
        self.stuck_motion_start_xy = None
        self.last_cmd_time_sec = None
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_wz = 0.0
        self.route_waypoints = self._build_route_waypoints(self.route_name) if self.control_mode == 'acceptance_path' else []
        self.route_waypoint_index = 0
        self.route_goal_hold_until_sec = 0.0
        self.route_complete = False
        self.route_failed = False

        self.create_timer(self.control_period, self._control_loop)
        self.create_timer(0.2, self._publish_debug)

        self.goal_keepouts = [
            (-0.6, 3.1, self.goal_keepout_radius),
            (2.4, 2.5, self.goal_keepout_radius),
            (-0.6, -3.1, self.goal_keepout_radius),
            (2.4, -2.5, self.goal_keepout_radius),
            (0.0, 0.0, 1.35),
        ]

        if self.control_mode == 'acceptance_path':
            self.get_logger().info(
                'Arena acceptance controller ready. Route=%s with %d waypoints.' % (
                    self.route_name,
                    len(self.route_waypoints),
                )
            )
        else:
            self.get_logger().info(
                'Arena roamer ready. Click-spawn obstacles in the sandbox and the robot will keep roaming.'
            )

    def _build_route_waypoints(self, route_name):
        routes = {
            'double_pinch_figure8': [
                (-3.60, 0.00),
                (-2.20, 1.80),
                (1.35, 1.45),
                (3.20, 2.20),
                (2.90, 0.00),
                (1.35, -1.45),
                (3.20, -2.20),
                (-2.20, -1.80),
                (-3.60, 0.00),
            ],
        }
        return routes.get(route_name, routes['double_pinch_figure8'])

    def _representative_scan_range(self, valid_samples):
        if not valid_samples:
            return float('inf')
        samples = sorted(valid_samples)
        if len(samples) < self.ir_low_sample_count:
            return samples[0]
        lowest = samples[:self.ir_low_sample_count]
        return sum(lowest) / len(lowest)

    def _sensor_control_value(self, sensor_name):
        filtered_range = self.ir[sensor_name]
        raw_range = self.ir_raw[sensor_name]
        if math.isfinite(filtered_range) and math.isfinite(raw_range):
            return min(filtered_range, raw_range)
        if math.isfinite(filtered_range):
            return filtered_range
        return raw_range

    def _sensor_min_control(self, *sensor_names):
        values = []
        for sensor_name in sensor_names:
            sensor_range = self._sensor_control_value(sensor_name)
            if math.isfinite(sensor_range):
                values.append(sensor_range)
        if not values:
            return float('inf')
        return min(values)

    def _sensors_fresh(self):
        now_sec = self._now_sec()
        return all(
            self.sensor_last_update_sec[name] is not None
            and now_sec - self.sensor_last_update_sec[name] <= self.sensor_timeout_sec
            for name in self.sensor_names
        )

    def _odom_fresh(self):
        return self.last_odom_sec is not None and self._now_sec() - self.last_odom_sec <= self.odom_timeout_sec

    def _apply_deadband(self, value, deadband):
        if abs(value) < deadband:
            return 0.0
        return value

    def _limit_axis(self, target, current, accel_limit, decel_limit, dt):
        if dt <= 0.0:
            return target
        limit = accel_limit if abs(target) > abs(current) else decel_limit
        max_delta = max(0.0, limit) * dt
        delta = clamp(target - current, -max_delta, max_delta)
        return current + delta

    def _activate_route_waypoint(self, waypoint_index, reason):
        self.route_waypoint_index = waypoint_index
        self.goal_x, self.goal_y = self.route_waypoints[waypoint_index]
        self.goal_started_sec = self._now_sec()
        self.route_goal_hold_until_sec = 0.0
        self.get_logger().info('%s Target waypoint %d -> (%.2f, %.2f)' % (
            reason,
            waypoint_index,
            self.goal_x,
            self.goal_y,
        ))

    def _advance_route_waypoint(self, reason):
        next_index = self.route_waypoint_index + 1
        if next_index < len(self.route_waypoints):
            self._activate_route_waypoint(next_index, reason)
            return
        if self.route_loop and self.route_waypoints:
            self._activate_route_waypoint(0, reason)
            return
        self.route_complete = True
        self.goal_x = None
        self.goal_y = None
        self.get_logger().info('Acceptance route complete.')

    def _ir_cb(self, key, msg):
        rmin = msg.range_min if msg.range_min > 0.0 else 0.01
        rmax = msg.range_max if msg.range_max > 0.0 else float('inf')
        valid = [
            sample for sample in msg.ranges
            if not math.isnan(sample) and sample >= rmin and sample < rmax
        ]
        raw_value = self._representative_scan_range(valid)
        self.ir_raw[key] = raw_value
        self.ir_range_max[key] = rmax
        self.sensor_last_update_sec[key] = self._now_sec()

        prev = self.ir[key]
        if not math.isfinite(prev):
            self.ir[key] = raw_value
            return

        raw_proxy = rmax if not math.isfinite(raw_value) else raw_value
        prev_proxy = rmax if not math.isfinite(prev) else prev
        alpha = self.ir_filter_alpha_fall if raw_proxy < prev_proxy else self.ir_filter_alpha_rise
        filtered = alpha * raw_proxy + (1.0 - alpha) * prev_proxy
        self.ir[key] = float('inf') if filtered >= rmax * 0.98 else filtered

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.center_x, self.center_y = self._center_xy(self.x, self.y, self.yaw)
        self.last_odom_sec = self._now_sec()

        if self.last_path_point is None:
            self.last_path_point = (self.center_x, self.center_y)
            self.robot_path.append(self._make_pose(self.center_x, self.center_y, self._geometry_body_yaw()))
            return

        dx = self.center_x - self.last_path_point[0]
        dy = self.center_y - self.last_path_point[1]
        if math.hypot(dx, dy) >= self.path_point_spacing:
            self.last_path_point = (self.center_x, self.center_y)
            self.robot_path.append(self._make_pose(self.center_x, self.center_y, self._geometry_body_yaw()))
            self.robot_path = self.robot_path[-3000:]

    def _center_xy(self, base_x, base_y, yaw):
        return (
            base_x + math.cos(yaw) * self.robot_center_offset_x - math.sin(yaw) * self.robot_center_offset_y,
            base_y + math.sin(yaw) * self.robot_center_offset_x + math.cos(yaw) * self.robot_center_offset_y,
        )

    def _geometry_body_yaw(self):
        if self.robot_body_frame_flip_180:
            return self._normalize(self.yaw + math.pi)
        return self.yaw

    def _sensor_origin_body(self, sensor_name):
        sensor_x, sensor_y = self.sensor_positions[sensor_name]
        return (
            self.robot_center_offset_x - sensor_x,
            self.robot_center_offset_y - sensor_y,
        )

    def _sensor_direction_body(self, sensor_name):
        directions = {
            'front': (1.0, 0.0),
            'front_left': (1.0, 1.0),
            'front_right': (1.0, -1.0),
            'left': (0.0, 1.0),
            'right': (0.0, -1.0),
            'back': (-1.0, 0.0),
        }
        dir_x, dir_y = directions[sensor_name]
        norm = math.hypot(dir_x, dir_y)
        return dir_x / norm, dir_y / norm

    def _body_point_to_world(self, body_x, body_y):
        world_x, world_y = self._body_to_world_vector(body_x, body_y)
        return self.center_x + world_x, self.center_y + world_y

    def _make_point(self, x, y, z=0.0):
        point = Marker().pose.position
        point.x = x
        point.y = y
        point.z = z
        return point

    def _sensor_display_range(self, sensor_name):
        range_max = self.ir_range_max.get(sensor_name, self.ir_default_range_max)
        if not math.isfinite(range_max):
            range_max = self.ir_default_range_max
        raw_range = self.ir_raw.get(sensor_name, float('inf'))
        if math.isfinite(raw_range):
            return max(self.ir_default_range_min, min(raw_range, range_max))
        return range_max

    def _sensor_hit_visible(self, sensor_name):
        raw_range = self.ir_raw.get(sensor_name, float('inf'))
        range_max = self.ir_range_max.get(sensor_name, self.ir_default_range_max)
        return math.isfinite(raw_range) and raw_range < range_max

    def _ir_cone_color(self, sensor_name):
        if sensor_name == 'front':
            return (1.0, 0.35, 0.25)
        if sensor_name in ('front_left', 'front_right'):
            return (1.0, 0.75, 0.20)
        if sensor_name in ('left', 'right'):
            return (0.20, 0.85, 1.0)
        return (0.55, 0.75, 1.0)

    def _build_ir_markers(self, now):
        markers = []
        sensor_names = ['front', 'front_left', 'front_right', 'left', 'right', 'back']
        arc_segments = 12

        for index, sensor_name in enumerate(sensor_names):
            origin_body_x, origin_body_y = self._sensor_origin_body(sensor_name)
            origin_x, origin_y = self._body_point_to_world(origin_body_x, origin_body_y)
            dir_body_x, dir_body_y = self._sensor_direction_body(sensor_name)
            sensor_yaw = self._geometry_body_yaw() + math.atan2(dir_body_y, dir_body_x)
            display_range = self._sensor_display_range(sensor_name)
            color_r, color_g, color_b = self._ir_cone_color(sensor_name)

            arc_points = []
            for step in range(arc_segments + 1):
                ratio = step / arc_segments
                ray_angle = sensor_yaw - self.ir_half_fov + (2.0 * self.ir_half_fov * ratio)
                arc_points.append(
                    self._make_point(
                        origin_x + display_range * math.cos(ray_angle),
                        origin_y + display_range * math.sin(ray_angle),
                        0.08,
                    )
                )

            cone = Marker()
            cone.header.frame_id = self.debug_frame_id
            cone.header.stamp = now
            cone.ns = 'arena_ir_cones'
            cone.id = 1000 + index
            cone.type = Marker.TRIANGLE_LIST
            cone.action = Marker.ADD
            cone.pose.orientation.w = 1.0
            cone.scale.x = 1.0
            cone.scale.y = 1.0
            cone.scale.z = 1.0
            cone.color.r = color_r
            cone.color.g = color_g
            cone.color.b = color_b
            cone.color.a = 0.34
            for step in range(arc_segments):
                cone.points.append(self._make_point(origin_x, origin_y, 0.08))
                cone.points.append(arc_points[step])
                cone.points.append(arc_points[step + 1])
            markers.append(cone)

            outline = Marker()
            outline.header.frame_id = self.debug_frame_id
            outline.header.stamp = now
            outline.ns = 'arena_ir_edges'
            outline.id = 1100 + index
            outline.type = Marker.LINE_STRIP
            outline.action = Marker.ADD
            outline.pose.orientation.w = 1.0
            outline.scale.x = 0.02
            outline.color.r = color_r
            outline.color.g = color_g
            outline.color.b = color_b
            outline.color.a = 0.92
            outline.points.append(self._make_point(origin_x, origin_y, 0.09))
            outline.points.extend(arc_points)
            outline.points.append(self._make_point(origin_x, origin_y, 0.09))
            markers.append(outline)

            origin = Marker()
            origin.header.frame_id = self.debug_frame_id
            origin.header.stamp = now
            origin.ns = 'arena_ir_origins'
            origin.id = 1150 + index
            origin.type = Marker.SPHERE
            origin.action = Marker.ADD
            origin.pose.position.x = origin_x
            origin.pose.position.y = origin_y
            origin.pose.position.z = 0.10
            origin.pose.orientation.w = 1.0
            origin.scale.x = 0.05
            origin.scale.y = 0.05
            origin.scale.z = 0.05
            origin.color.r = color_r
            origin.color.g = color_g
            origin.color.b = color_b
            origin.color.a = 0.95
            markers.append(origin)

            hit = Marker()
            hit.header.frame_id = self.debug_frame_id
            hit.header.stamp = now
            hit.ns = 'arena_ir_hits'
            hit.id = 1200 + index
            hit.type = Marker.SPHERE
            hit.action = Marker.ADD
            hit.pose.orientation.w = 1.0
            hit.scale.x = 0.08
            hit.scale.y = 0.08
            hit.scale.z = 0.08
            if self._sensor_hit_visible(sensor_name):
                hit.pose.position.x = origin_x + display_range * math.cos(sensor_yaw)
                hit.pose.position.y = origin_y + display_range * math.sin(sensor_yaw)
                hit.pose.position.z = 0.11
                hit.color.r = 1.0
                hit.color.g = 0.12
                hit.color.b = 0.12
                hit.color.a = 0.95
            else:
                hit.pose.position.z = -10.0
                hit.color.a = 0.0
            markers.append(hit)

        return markers

    @staticmethod
    def _normalize(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _sensor_warm(self):
        return self._now_sec() - self.node_started_sec >= self.sensor_warmup_sec

    def _all_sensors_clear(self, threshold):
        for sensor_range in self.ir.values():
            if math.isfinite(sensor_range) and sensor_range <= threshold:
                return False
        return True

    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.debug_frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _choose_new_goal(self, reason):
        if self.control_mode == 'acceptance_path':
            if self.route_complete or self.route_failed or not self.route_waypoints:
                self.goal_x = None
                self.goal_y = None
                return
            self._activate_route_waypoint(self.route_waypoint_index, reason)
            return

        min_x = self.arena_min_x + self.goal_margin
        max_x = self.arena_max_x - self.goal_margin
        min_y = self.arena_min_y + self.goal_margin
        max_y = self.arena_max_y - self.goal_margin
        for _ in range(40):
            candidate_x = self.randomizer.uniform(min_x, max_x)
            candidate_y = self.randomizer.uniform(min_y, max_y)
            if math.hypot(candidate_x - self.center_x, candidate_y - self.center_y) < self.min_goal_separation:
                continue
            if self._goal_in_keepout(candidate_x, candidate_y):
                continue
            self.goal_x = candidate_x
            self.goal_y = candidate_y
            self.goal_started_sec = self._now_sec()
            self.get_logger().info('%s New roam target: (%.2f, %.2f)' % (reason, self.goal_x, self.goal_y))
            return

        self.goal_x = clamp(self.center_x + 1.5, min_x, max_x)
        self.goal_y = clamp(self.center_y + 1.5, min_y, max_y)
        self.goal_started_sec = self._now_sec()

    def _goal_in_keepout(self, goal_x, goal_y):
        for keepout_x, keepout_y, keepout_radius in self.goal_keepouts:
            if math.hypot(goal_x - keepout_x, goal_y - keepout_y) <= keepout_radius:
                return True
        return False

    def _sensor_repulsion_body_vector(self):
        directions = {
            'front': (1.0, 0.0),
            'front_left': (1.0, 0.9),
            'front_right': (1.0, -0.9),
            'left': (0.0, 1.0),
            'right': (0.0, -1.0),
            'back': (-1.0, 0.0),
        }
        repulse_x = 0.0
        repulse_y = 0.0
        hottest_sensor = None
        hottest_range = float('inf')
        for sensor_name in self.sensor_names:
            sensor_range = self._sensor_control_value(sensor_name)
            if not math.isfinite(sensor_range):
                continue
            threshold = self.obstacle_clear_distance if sensor_name == 'front' else self.obstacle_detect_distance
            if sensor_range >= threshold:
                continue
            urgency = (threshold - sensor_range) / max(threshold, 1e-6)
            dir_x, dir_y = directions[sensor_name]
            repulse_x += dir_x * urgency
            repulse_y += dir_y * urgency
            if sensor_range < hottest_range:
                hottest_range = sensor_range
                hottest_sensor = sensor_name
        return repulse_x, repulse_y, hottest_sensor, hottest_range

    def _focused_escape_body_vector(self, sensor_name):
        if sensor_name == 'front_left':
            return 0.25, 1.0
        if sensor_name == 'front_right':
            return 0.25, -1.0
        if sensor_name == 'left':
            return 0.05, -1.0
        if sensor_name == 'right':
            return 0.05, 1.0
        if sensor_name == 'back':
            return 1.0, 0.0
        left_min = self._sensor_min_control('front_left', 'left')
        right_min = self._sensor_min_control('front_right', 'right')
        return (0.2, -1.0) if left_min < right_min else (0.2, 1.0)

    def _wall_repulsion_world_vector(self):
        repulse_x = 0.0
        repulse_y = 0.0
        if self.center_x - self.arena_min_x < self.wall_influence_distance:
            repulse_x += (self.wall_influence_distance - (self.center_x - self.arena_min_x)) / self.wall_influence_distance
        if self.arena_max_x - self.center_x < self.wall_influence_distance:
            repulse_x -= (self.wall_influence_distance - (self.arena_max_x - self.center_x)) / self.wall_influence_distance
        if self.center_y - self.arena_min_y < self.wall_influence_distance:
            repulse_y += (self.wall_influence_distance - (self.center_y - self.arena_min_y)) / self.wall_influence_distance
        if self.arena_max_y - self.center_y < self.wall_influence_distance:
            repulse_y -= (self.wall_influence_distance - (self.arena_max_y - self.center_y)) / self.wall_influence_distance
        return repulse_x, repulse_y

    def _world_to_body_vector(self, world_x, world_y):
        body_yaw = self._geometry_body_yaw()
        cos_y = math.cos(body_yaw)
        sin_y = math.sin(body_yaw)
        return (
            world_x * cos_y + world_y * sin_y,
            -world_x * sin_y + world_y * cos_y,
        )

    def _body_to_world_vector(self, body_x, body_y):
        body_yaw = self._geometry_body_yaw()
        cos_y = math.cos(body_yaw)
        sin_y = math.sin(body_yaw)
        return (
            body_x * cos_y - body_y * sin_y,
            body_x * sin_y + body_y * cos_y,
        )

    def _sensor_min(self, *sensor_names):
        return self._sensor_min_control(*sensor_names)

    def _movement_candidate_map(self):
        diagonal = math.sqrt(0.5)
        side_threshold = max(self.lateral_clearance_distance, self.obstacle_side_min, self.obstacle_danger_distance)
        back_threshold = max(self.obstacle_side_min, self.hard_escape_distance + 0.02)
        diagonal_threshold = max(self.forward_clearance_distance, side_threshold)
        return {
            'forward': {
                'vx': 1.0,
                'vy': 0.0,
                'speed_scale': 1.0,
                'sensors': ('front', 'front_left', 'front_right'),
                'threshold': self.forward_clearance_distance,
            },
            'left': {
                'vx': 0.0,
                'vy': 1.0,
                'speed_scale': 0.78,
                'sensors': ('left', 'front_left'),
                'threshold': side_threshold,
            },
            'right': {
                'vx': 0.0,
                'vy': -1.0,
                'speed_scale': 0.78,
                'sensors': ('right', 'front_right'),
                'threshold': side_threshold,
            },
            'backward': {
                'vx': -1.0,
                'vy': 0.0,
                'speed_scale': 0.52,
                'sensors': ('back',),
                'threshold': back_threshold,
            },
            'diag_left': {
                'vx': diagonal,
                'vy': diagonal,
                'speed_scale': 0.45,
                'sensors': ('front', 'front_left', 'left'),
                'threshold': diagonal_threshold,
            },
            'diag_right': {
                'vx': diagonal,
                'vy': -diagonal,
                'speed_scale': 0.45,
                'sensors': ('front', 'front_right', 'right'),
                'threshold': diagonal_threshold,
            },
        }

    def _choose_motion_command(self, desired_body_x, desired_body_y, goal_heading_error, hottest_sensor, hottest_range):
        candidates = self._movement_candidate_map()
        danger_mode = hottest_sensor is not None and hottest_range <= self.obstacle_danger_distance
        desired_norm = math.hypot(desired_body_x, desired_body_y)
        if desired_norm < 1e-6:
            desired_body_x = 1.0
            desired_body_y = 0.0
            desired_norm = 1.0

        if danger_mode and hottest_sensor is not None:
            escape_x, escape_y = self._focused_escape_body_vector(hottest_sensor)
            escape_norm = max(1e-6, math.hypot(escape_x, escape_y))
            escape_x /= escape_norm
            escape_y /= escape_norm
        else:
            escape_x = 0.0
            escape_y = 0.0

        scored_candidates = []
        for motion_name, candidate in candidates.items():
            clearance = self._sensor_min(*candidate['sensors'])
            if math.isfinite(clearance) and clearance <= candidate['threshold']:
                continue

            if motion_name == 'forward' and abs(goal_heading_error) > self.forward_heading_allowance and desired_body_x >= 0.0:
                continue

            cand_norm = max(1e-6, math.hypot(candidate['vx'], candidate['vy']))
            direction_score = (
                candidate['vx'] * desired_body_x + candidate['vy'] * desired_body_y
            ) / (cand_norm * desired_norm)
            clearance_score = 0.0
            if math.isfinite(clearance):
                clearance_score = clamp(
                    (clearance - candidate['threshold']) / max(candidate['threshold'], 1e-6),
                    0.0,
                    2.0,
                )
            escape_score = 0.0
            if danger_mode:
                escape_score = (
                    candidate['vx'] * escape_x + candidate['vy'] * escape_y
                ) / cand_norm

            score = 2.2 * direction_score + 0.35 * clearance_score + 0.9 * escape_score
            if motion_name == 'backward':
                score -= 0.18
            if motion_name.startswith('diag'):
                score -= 0.04
            scored_candidates.append((score, motion_name, candidate, clearance))

        if scored_candidates:
            scored_candidates.sort(key=lambda item: item[0], reverse=True)
            _, motion_name, candidate, clearance = scored_candidates[0]
            return motion_name, candidate, clearance, danger_mode

        return None, None, float('inf'), danger_mode

    def _rotation_command(self, goal_heading_error, hottest_sensor, translational_motion):
        cmd_wz = self.angular_kp * goal_heading_error
        if hottest_sensor == 'front':
            left_clearance = self._sensor_min('left', 'front_left')
            right_clearance = self._sensor_min('right', 'front_right')
            cmd_wz += self.rotate_clearance_bias if left_clearance >= right_clearance else -self.rotate_clearance_bias
        elif hottest_sensor in ('left', 'front_left'):
            cmd_wz -= self.rotate_clearance_bias
        elif hottest_sensor in ('right', 'front_right'):
            cmd_wz += self.rotate_clearance_bias

        if translational_motion in ('backward', 'diag_left', 'diag_right'):
            cmd_wz *= 0.6

        return clamp(cmd_wz, -self.max_angular_speed, self.max_angular_speed)

    def _update_stuck_state(self):
        now_sec = self._now_sec()
        if self.motion_name != self.stuck_motion_name:
            self.stuck_motion_name = self.motion_name
            self.stuck_motion_started_sec = now_sec
            self.stuck_motion_start_xy = (self.center_x, self.center_y)
            return False

        if self.stuck_motion_name in ('STOP', 'ROTATE', 'ESCAPE'):
            self.stuck_motion_started_sec = now_sec
            self.stuck_motion_start_xy = (self.center_x, self.center_y)
            return False

        if self.stuck_motion_started_sec is None or self.stuck_motion_start_xy is None:
            self.stuck_motion_started_sec = now_sec
            self.stuck_motion_start_xy = (self.center_x, self.center_y)
            return False

        if now_sec - self.stuck_motion_started_sec < self.stuck_timeout_sec:
            return False

        moved_distance = math.hypot(
            self.center_x - self.stuck_motion_start_xy[0],
            self.center_y - self.stuck_motion_start_xy[1],
        )
        return moved_distance < self.stuck_distance_epsilon

    def _begin_escape_hold(self):
        self.escape_hold_until_sec = self._now_sec() + self.escape_hold_sec

    def _escape_hold_active(self):
        return self._now_sec() < self.escape_hold_until_sec

    def _publish_cmd(self, vx, vy, wz):
        if vx > 0.0 and self._sensor_min_control('front', 'front_left', 'front_right') < self.obstacle_detect_distance:
            vx = 0.0
        if vy > 0.0 and self._sensor_min_control('left', 'front_left') < self.obstacle_side_min:
            vy = 0.0
        if vy < 0.0 and self._sensor_min_control('right', 'front_right') < self.obstacle_side_min:
            vy = 0.0
        if vx < 0.0 and self._sensor_min_control('back') < self.obstacle_side_min:
            vx = 0.0

        now_sec = self._now_sec()
        if self.last_cmd_time_sec is None:
            dt = self.control_period
        else:
            dt = max(1e-3, now_sec - self.last_cmd_time_sec)
        self.last_cmd_time_sec = now_sec

        vx = self._limit_axis(vx, self.last_cmd_vx, self.cmd_linear_accel_limit, self.cmd_linear_decel_limit, dt)
        vy = self._limit_axis(vy, self.last_cmd_vy, self.cmd_lateral_accel_limit, self.cmd_lateral_decel_limit, dt)
        wz = self._limit_axis(wz, self.last_cmd_wz, self.cmd_angular_accel_limit, self.cmd_angular_decel_limit, dt)

        vx = self._apply_deadband(vx, self.cmd_deadband_linear)
        vy = self._apply_deadband(vy, self.cmd_deadband_lateral)
        wz = self._apply_deadband(wz, self.cmd_deadband_angular)

        cmd = Twist()
        cmd.linear.x = clamp(self.cmd_forward_sign * vx, -self.max_linear_speed, self.max_linear_speed)
        cmd.linear.y = clamp(self.cmd_lateral_sign * vy, -self.max_lateral_speed, self.max_lateral_speed)
        cmd.angular.z = clamp(wz, -self.max_angular_speed, self.max_angular_speed)
        self.cmd_pub.publish(cmd)
        self.last_cmd_vx = vx
        self.last_cmd_vy = vy
        self.last_cmd_wz = wz

    def _control_loop(self):
        if self.last_path_point is None or not self._sensor_warm():
            self.state_name = 'WARMUP'
            self.motion_name = 'STOP'
            self._publish_cmd(0.0, 0.0, 0.0)
            return

        if not self._odom_fresh():
            self.state_name = 'STALE_ODOM'
            self.motion_name = 'STOP'
            self._publish_cmd(0.0, 0.0, 0.0)
            return

        if not self._sensors_fresh():
            self.state_name = 'STALE_IR'
            self.motion_name = 'STOP'
            self._publish_cmd(0.0, 0.0, 0.0)
            return

        if self._all_sensors_clear(self.startup_clearance_distance):
            self.startup_clear_cycles += 1
            self.blocked_started_sec = None
        else:
            self.startup_clear_cycles = 0
            if self.blocked_started_sec is None:
                self.blocked_started_sec = self._now_sec()

        if self.startup_clear_cycles < 3:
            if self.blocked_started_sec is not None and self._now_sec() - self.blocked_started_sec >= self.startup_escape_delay_sec:
                _, _, hottest_sensor, _ = self._sensor_repulsion_body_vector()
                if hottest_sensor is not None:
                    escape_x, escape_y = self._focused_escape_body_vector(hottest_sensor)
                    self.state_name = 'STARTUP_ESCAPE'
                    self.motion_name = 'ESCAPE'
                    self._publish_cmd(
                        escape_x * self.hard_escape_speed,
                        escape_y * self.hard_escape_speed,
                        0.0,
                    )
                    return
            self.state_name = 'WAIT_CLEAR'
            self.motion_name = 'STOP'
            self._publish_cmd(0.0, 0.0, 0.0)
            return

        if self.control_mode == 'acceptance_path':
            if self.route_failed:
                self.state_name = 'ROUTE_FAILED'
                self.motion_name = 'STOP'
                self._publish_cmd(0.0, 0.0, 0.0)
                return
            if self.route_complete:
                self.state_name = 'ROUTE_COMPLETE'
                self.motion_name = 'STOP'
                self._publish_cmd(0.0, 0.0, 0.0)
                return

        if self.goal_x is None or self.goal_y is None:
            self._choose_new_goal('Startup target selected.')
            if self.goal_x is None or self.goal_y is None:
                self.state_name = 'NO_TARGET'
                self.motion_name = 'STOP'
                self._publish_cmd(0.0, 0.0, 0.0)
                return

        goal_dx = self.goal_x - self.center_x
        goal_dy = self.goal_y - self.center_y
        goal_distance = math.hypot(goal_dx, goal_dy)
        goal_tolerance = self.route_waypoint_tolerance if self.control_mode == 'acceptance_path' else self.goal_reached_tolerance
        if goal_distance <= goal_tolerance:
            if self.control_mode == 'acceptance_path':
                if self.route_goal_hold_until_sec <= 0.0:
                    self.route_goal_hold_until_sec = self._now_sec() + self.route_waypoint_dwell_sec
                if self._now_sec() < self.route_goal_hold_until_sec:
                    self.state_name = 'WAYPOINT_SETTLE'
                    self.motion_name = 'STOP'
                    self._publish_cmd(0.0, 0.0, 0.0)
                    return
                self._advance_route_waypoint('Waypoint reached.')
                if self.route_complete:
                    self.state_name = 'ROUTE_COMPLETE'
                    self.motion_name = 'STOP'
                    self._publish_cmd(0.0, 0.0, 0.0)
                    return
            else:
                self._choose_new_goal('Goal reached.')
            goal_dx = self.goal_x - self.center_x
            goal_dy = self.goal_y - self.center_y
            goal_distance = math.hypot(goal_dx, goal_dy)

        if self._now_sec() - self.goal_started_sec >= self.goal_timeout_sec:
            if self.control_mode == 'acceptance_path' and self.route_fail_on_timeout:
                self.route_failed = True
                self.state_name = 'WAYPOINT_TIMEOUT'
                self.motion_name = 'STOP'
                self._publish_cmd(0.0, 0.0, 0.0)
                return
            self._choose_new_goal('Goal timed out.')
            goal_dx = self.goal_x - self.center_x
            goal_dy = self.goal_y - self.center_y
            goal_distance = math.hypot(goal_dx, goal_dy)

        goal_body_x, goal_body_y = self._world_to_body_vector(goal_dx, goal_dy)
        goal_norm = max(goal_distance, 1e-6)
        goal_body_x /= goal_norm
        goal_body_y /= goal_norm
        goal_heading = math.atan2(goal_dy, goal_dx)
        goal_heading_error = self._normalize(goal_heading - self._geometry_body_yaw())

        repulse_x, repulse_y, hottest_sensor, hottest_range = self._sensor_repulsion_body_vector()
        wall_world_x, wall_world_y = self._wall_repulsion_world_vector()
        wall_body_x, wall_body_y = self._world_to_body_vector(wall_world_x, wall_world_y)
        desired_body_x = self.goal_gain * goal_body_x + self.wall_gain * wall_body_x - self.repulsion_gain * repulse_x
        desired_body_y = self.goal_gain * goal_body_y + self.wall_gain * wall_body_y - self.repulsion_gain * repulse_y
        if hottest_sensor is not None and hottest_range <= self.obstacle_danger_distance:
            escape_x, escape_y = self._focused_escape_body_vector(hottest_sensor)
            desired_body_x += self.escape_gain * escape_x
            desired_body_y += self.escape_gain * escape_y

        closest_front = self._sensor_min('front', 'front_left', 'front_right')
        closest_any = min(self._sensor_control_value(name) for name in self.sensor_names)

        if self._escape_hold_active() and hottest_sensor is not None:
            escape_x, escape_y = self._focused_escape_body_vector(hottest_sensor)
            self.state_name = 'ESCAPE_HOLD'
            self.motion_name = 'ESCAPE'
            self._publish_cmd(
                escape_x * self.hard_escape_speed,
                escape_y * self.hard_escape_speed,
                self._rotation_command(goal_heading_error, hottest_sensor, None),
            )
            return

        if hottest_sensor is None and abs(goal_heading_error) >= self.reorient_heading_threshold:
            self.state_name = 'REORIENT'
            self.motion_name = 'ROTATE'
            self._publish_cmd(0.0, 0.0, self._rotation_command(goal_heading_error, None, None))
            return

        if hottest_sensor is None and abs(goal_heading_error) > self.forward_heading_allowance:
            self.state_name = 'ALIGN_FORWARD'
            self.motion_name = 'ROTATE'
            self._publish_cmd(0.0, 0.0, self._rotation_command(goal_heading_error, None, None))
            return

        motion_name, candidate, motion_clearance, danger_mode = self._choose_motion_command(
            desired_body_x,
            desired_body_y,
            goal_heading_error,
            hottest_sensor,
            hottest_range,
        )

        if motion_name is None:
            rotate_sensor = hottest_sensor
            if rotate_sensor is None and math.isfinite(closest_front) and closest_front <= self.obstacle_detect_distance:
                rotate_sensor = 'front'
            self.state_name = 'HOLD_ROTATE'
            self.motion_name = 'ROTATE'
            self._publish_cmd(0.0, 0.0, self._rotation_command(goal_heading_error, rotate_sensor, None))
            return

        if motion_name == 'forward' and math.isfinite(closest_front) and closest_front <= self.forward_clearance_distance:
            side_choice = 'left' if self._sensor_min('left', 'front_left') >= self._sensor_min('right', 'front_right') else 'right'
            candidate = self._movement_candidate_map()[side_choice]
            side_clearance = self._sensor_min(*candidate['sensors'])
            if not math.isfinite(side_clearance) or side_clearance > candidate['threshold']:
                motion_name = side_choice
                motion_clearance = side_clearance
            else:
                self.state_name = 'FRONT_BLOCKED_ROTATE'
                self.motion_name = 'ROTATE'
                self._publish_cmd(0.0, 0.0, self._rotation_command(goal_heading_error, 'front', None))
                return

        speed_cap = min(self.max_linear_speed, self.max_lateral_speed) * candidate['speed_scale']
        if math.isfinite(closest_any):
            if closest_any <= self.hard_escape_distance:
                speed_cap = min(speed_cap, self.hard_escape_speed)
            elif closest_any <= self.obstacle_danger_distance:
                speed_cap *= 0.28
            elif closest_any <= self.obstacle_detect_distance:
                ratio = (closest_any - self.obstacle_danger_distance) / max(
                    self.obstacle_detect_distance - self.obstacle_danger_distance,
                    1e-6,
                )
                speed_cap *= 0.28 + 0.52 * clamp(ratio, 0.0, 1.0)

        if hottest_sensor is not None:
            speed_cap = min(speed_cap, self.caution_speed)

        if math.isfinite(motion_clearance):
            safety_ratio = clamp(
                (motion_clearance - candidate['threshold']) / max(candidate['threshold'], 1e-6),
                0.0,
                1.0,
            )
            speed_cap *= 0.35 + 0.65 * safety_ratio

        if goal_distance < 1.0:
            speed_cap *= max(0.35, goal_distance)

        if hottest_sensor is not None and hottest_range <= self.hard_escape_distance:
            speed_cap = min(speed_cap, self.hard_escape_speed)

        cmd_vx = candidate['vx'] * speed_cap
        cmd_vy = candidate['vy'] * speed_cap
        cmd_wz = self._rotation_command(goal_heading_error, hottest_sensor, motion_name)

        if danger_mode:
            self.state_name = 'ESCAPE_%s' % motion_name.upper()
        else:
            self.state_name = 'ROAM_%s' % motion_name.upper()
        self.motion_name = motion_name.upper()

        if self._update_stuck_state() and hottest_sensor is not None:
            self._begin_escape_hold()
            escape_x, escape_y = self._focused_escape_body_vector(hottest_sensor)
            self.state_name = 'STUCK_ESCAPE'
            self.motion_name = 'ESCAPE'
            self._publish_cmd(
                escape_x * self.hard_escape_speed,
                escape_y * self.hard_escape_speed,
                self._rotation_command(goal_heading_error, hottest_sensor, None),
            )
            return

        self._publish_cmd(cmd_vx, cmd_vy, cmd_wz)

    def _build_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.debug_frame_id
        poses = [self._make_pose(self.center_x, self.center_y, self._geometry_body_yaw())]
        if self.control_mode == 'acceptance_path' and self.route_waypoints:
            for waypoint_index in range(self.route_waypoint_index, len(self.route_waypoints)):
                wx, wy = self.route_waypoints[waypoint_index]
                poses.append(self._make_pose(wx, wy, 0.0))
        elif self.goal_x is not None and self.goal_y is not None:
            poses.append(self._make_pose(self.goal_x, self.goal_y, 0.0))
        path.poses = poses
        return path

    def _build_trail(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.debug_frame_id
        path.poses = self.robot_path
        return path

    def _build_markers(self):
        now = self.get_clock().now().to_msg()
        markers = []

        clear = Marker()
        clear.header.frame_id = self.debug_frame_id
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.append(clear)

        bounds = Marker()
        bounds.header.frame_id = self.debug_frame_id
        bounds.header.stamp = now
        bounds.ns = 'arena'
        bounds.id = 1
        bounds.type = Marker.LINE_STRIP
        bounds.action = Marker.ADD
        bounds.scale.x = 0.04
        bounds.color.r = 0.25
        bounds.color.g = 0.85
        bounds.color.b = 0.95
        bounds.color.a = 0.85
        for x, y in [
            (self.arena_min_x, self.arena_min_y),
            (self.arena_max_x, self.arena_min_y),
            (self.arena_max_x, self.arena_max_y),
            (self.arena_min_x, self.arena_max_y),
            (self.arena_min_x, self.arena_min_y),
        ]:
            point = PoseStamped().pose.position
            point.x = x
            point.y = y
            point.z = 0.05
            bounds.points.append(point)
        markers.append(bounds)

        if self.goal_x is not None and self.goal_y is not None:
            goal = Marker()
            goal.header.frame_id = self.debug_frame_id
            goal.header.stamp = now
            goal.ns = 'arena'
            goal.id = 2
            goal.type = Marker.SPHERE
            goal.action = Marker.ADD
            goal.pose.position.x = self.goal_x
            goal.pose.position.y = self.goal_y
            goal.pose.position.z = 0.14
            goal.pose.orientation.w = 1.0
            goal.scale.x = 0.24
            goal.scale.y = 0.24
            goal.scale.z = 0.24
            goal.color.r = 1.0
            goal.color.g = 0.85
            goal.color.b = 0.18
            goal.color.a = 0.95
            markers.append(goal)

        if self.control_mode == 'acceptance_path':
            for waypoint_index, (wx, wy) in enumerate(self.route_waypoints):
                marker = Marker()
                marker.header.frame_id = self.debug_frame_id
                marker.header.stamp = now
                marker.ns = 'acceptance_route'
                marker.id = waypoint_index
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = wx
                marker.pose.position.y = wy
                marker.pose.position.z = 0.12
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.18
                marker.scale.y = 0.18
                marker.scale.z = 0.18
                if waypoint_index < self.route_waypoint_index:
                    marker.color.r = 0.35
                    marker.color.g = 0.35
                    marker.color.b = 0.35
                    marker.color.a = 0.90
                elif waypoint_index == self.route_waypoint_index and not self.route_complete:
                    marker.color.r = 1.0
                    marker.color.g = 0.90
                    marker.color.b = 0.15
                    marker.color.a = 1.0
                else:
                    marker.color.r = 0.15
                    marker.color.g = 0.78
                    marker.color.b = 1.0
                    marker.color.a = 0.92
                markers.append(marker)

                label = Marker()
                label.header.frame_id = self.debug_frame_id
                label.header.stamp = now
                label.ns = 'acceptance_route_labels'
                label.id = 100 + waypoint_index
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = wx
                label.pose.position.y = wy
                label.pose.position.z = 0.32
                label.pose.orientation.w = 1.0
                label.scale.z = 0.12
                label.color.r = 1.0
                label.color.g = 1.0
                label.color.b = 1.0
                label.color.a = 0.95
                label.text = 'P%d' % waypoint_index
                markers.append(label)

        markers.extend(self._build_ir_markers(now))

        return MarkerArray(markers=markers)

    def _publish_debug(self):
        self.path_pub.publish(self._build_path())
        self.trail_pub.publish(self._build_trail())
        self.marker_pub.publish(self._build_markers())

        state = String()
        goal_text = 'none' if self.goal_x is None else '(%.2f, %.2f)' % (self.goal_x, self.goal_y)
        route_text = ''
        if self.control_mode == 'acceptance_path':
            route_text = ' route=%d/%d complete=%s failed=%s' % (
                self.route_waypoint_index,
                len(self.route_waypoints),
                self.route_complete,
                self.route_failed,
            )
        state.data = (
            'state=%s motion=%s goal=%s%s pos=(%.2f, %.2f) ir[f=%.2f fl=%.2f fr=%.2f l=%.2f r=%.2f b=%.2f]' % (
                self.state_name,
                self.motion_name,
                goal_text,
                route_text,
                self.center_x,
                self.center_y,
                self.ir_raw['front'],
                self.ir_raw['front_left'],
                self.ir_raw['front_right'],
                self.ir_raw['left'],
                self.ir_raw['right'],
                self.ir_raw['back'],
            )
        )
        self.state_pub.publish(state)


def main():
    rclpy.init()
    node = ArenaRoamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()