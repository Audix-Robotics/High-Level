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
        self.declare_parameter('rotate_forward_speed', 0.08)
        # Reroute (3-point turn) parameters
        self.declare_parameter('reroute_back_time', 0.45)
        self.declare_parameter('reroute_stop_time', 0.12)
        self.declare_parameter('reroute_rotate_time', 0.60)
        self.declare_parameter('reroute_rotate_speed', 0.85)
        self.declare_parameter('reroute_post_clear_pause', 0.40)
        self.declare_parameter('reroute_correct_heading_tol_deg', 8.0)
        # Use single 15 cm detection/clear distance across the project
        self.declare_parameter('obstacle_detect_distance', 0.15)
        self.declare_parameter('obstacle_danger_distance', 0.15)
        self.declare_parameter('obstacle_clear_distance', 0.15)
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
        self.declare_parameter('repulse_decay_sec', 1.50)
        self.declare_parameter('control_mode', 'acceptance_path')
        self.declare_parameter('route_name', 'double_pinch_figure8')
        self.declare_parameter('route_loop', False)
        self.declare_parameter('route_waypoint_tolerance', 0.24)
        self.declare_parameter('route_waypoint_dwell_sec', 0.35)
        self.declare_parameter('route_fail_on_timeout', True)
        self.declare_parameter('motion_hold_sec', 0.30)
        self.declare_parameter('motion_switch_score_margin', 0.22)
        self.declare_parameter('motion_clearance_hysteresis', 0.03)
        self.declare_parameter('ir_emergency_raw_ratio', 0.82)
        self.declare_parameter('blocked_side_release_cycles', 6)
        self.declare_parameter('blocked_side_release_clearance', 0.24)
        self.declare_parameter('blocked_side_forward_bias', 0.35)
        self.declare_parameter('blocked_clearance_margin', 0.10)
        self.declare_parameter('blocked_goal_lateral_min', 0.18)
        self.declare_parameter('blocked_goal_forward_bonus', 1.20)
        self.declare_parameter('avoid_commit_cycles', 5)
        self.declare_parameter('avoid_forward_cycles', 8)
        # avoidance override parameters
        self.declare_parameter('avoid_override_enable', True)
        self.declare_parameter('avoid_override_timeout_sec', 0.8)

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
        self.rotate_forward_speed = float(self.get_parameter('rotate_forward_speed').value)
        self.reroute_back_time = float(self.get_parameter('reroute_back_time').value)
        self.reroute_stop_time = float(self.get_parameter('reroute_stop_time').value)
        self.reroute_rotate_time = float(self.get_parameter('reroute_rotate_time').value)
        self.reroute_rotate_speed = float(self.get_parameter('reroute_rotate_speed').value)
        self.reroute_post_clear_pause = float(self.get_parameter('reroute_post_clear_pause').value)
        self.reroute_correct_heading_tol = math.radians(float(self.get_parameter('reroute_correct_heading_tol_deg').value))
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
        self.repulse_decay_sec = float(self.get_parameter('repulse_decay_sec').value)
        self.control_mode = str(self.get_parameter('control_mode').value).strip().lower()
        self.route_name = str(self.get_parameter('route_name').value).strip().lower()
        self.route_loop = bool(self.get_parameter('route_loop').value)
        self.route_waypoint_tolerance = float(self.get_parameter('route_waypoint_tolerance').value)
        self.route_waypoint_dwell_sec = float(self.get_parameter('route_waypoint_dwell_sec').value)
        self.route_fail_on_timeout = bool(self.get_parameter('route_fail_on_timeout').value)
        self.motion_hold_sec = float(self.get_parameter('motion_hold_sec').value)
        self.motion_switch_score_margin = float(self.get_parameter('motion_switch_score_margin').value)
        self.motion_clearance_hysteresis = float(self.get_parameter('motion_clearance_hysteresis').value)
        self.ir_emergency_raw_ratio = float(self.get_parameter('ir_emergency_raw_ratio').value)
        self.blocked_side_release_cycles = max(1, int(self.get_parameter('blocked_side_release_cycles').value))
        self.blocked_side_release_clearance = float(self.get_parameter('blocked_side_release_clearance').value)
        self.blocked_side_forward_bias = float(self.get_parameter('blocked_side_forward_bias').value)
        self.blocked_goal_lateral_min = float(self.get_parameter('blocked_goal_lateral_min').value)
        self.blocked_goal_forward_bonus = float(self.get_parameter('blocked_goal_forward_bonus').value)
        self.avoid_commit_cycles = max(1, int(self.get_parameter('avoid_commit_cycles').value))
        self.avoid_forward_cycles = max(1, int(self.get_parameter('avoid_forward_cycles').value))
        self.blocked_clearance_margin = float(self.get_parameter('blocked_clearance_margin').value)
        self.avoid_override_enable = bool(self.get_parameter('avoid_override_enable').value)
        self.avoid_override_timeout = float(self.get_parameter('avoid_override_timeout_sec').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/debug/planned_path', 10)
        self.trail_pub = self.create_publisher(Path, '/debug/robot_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/debug/targets', 10)
        self.state_pub = self.create_publisher(String, '/debug/state', 10)

        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        # SWAPPED mapping to correct URDF labeling (physical left/right reversed)
        for key, topic in {
            'front': '/ir_front/scan',
            'front_left': '/ir_front_right/scan',
            'front_right': '/ir_front_left/scan',
            'left': '/ir_right/scan',
            'right': '/ir_left/scan',
            'back': '/ir_back/scan',
        }.items():
            self.create_subscription(LaserScan, topic, lambda msg, k=key: self._ir_cb(k, msg), 10)

        # Optional avoidance override topic: short-lived reflexive commands
        self.last_avoid_cmd = None
        self.last_avoid_time = 0.0
        self.create_subscription(Twist, '/avoid_cmd_vel', self._avoid_cb, 10)

        # timers, debug publishers, and goal keepouts
        self.create_timer(self.control_period, self._control_loop)
        self.create_timer(0.2, self._publish_debug)

        self.ir = {name: float('inf') for name in self.sensor_names}
        self.ir_raw = dict(self.ir)
        # Increase IR detection range to 20cm as requested
        self.ir_range_max = {key: 0.20 for key in self.ir}
        self.sensor_last_update_sec = {key: None for key in self.ir}
        self.ir_default_range_min = 0.05
        self.ir_default_range_max = 0.20
        # For time-sequenced binary trigger handling (front sensors)
        self._last_ir_trigger = {name: 0.0 for name in ('front', 'front_left', 'front_right')}
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
        # Reroute state: None or dict with keys: phase, start_sec, sensor
        self.reroute_state = None
        self.randomizer = random.Random(self.random_seed)
        self.state_name = 'WARMUP'
        self.motion_name = 'STOP'
        self.startup_clear_cycles = 0
        self.blocked_started_sec = None
        self.startup_gate_complete = False
        self.escape_hold_until_sec = 0.0
        self.stuck_motion_name = None
        self.stuck_motion_started_sec = None
        self.stuck_motion_start_xy = None
        self.last_cmd_time_sec = None
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_wz = 0.0
        self.current_planar_speed = 0.0
        self.route_waypoints = self._build_route_waypoints(self.route_name) if self.control_mode == 'acceptance_path' else []
        self.route_waypoint_index = 0
        self.route_goal_hold_until_sec = 0.0
        self.route_complete = False
        self.route_failed = False
        self.active_motion_selection = None
        self.active_motion_hold_until_sec = 0.0
        self.blocked_side = None
        self.blocked_side_clear_count = 0
        self.avoid_memory = None
        # Odometry-based blind-spot clearance tracking
        self.blocked_clearance_active = False
        self.blocked_clearance_origin = None  # (x, y, yaw)
        self.blocked_clearance_target = 0.0
        # Keepout zones to remember obstacle locations (x, y, radius)
        self.goal_keepouts = []
        # Last obstacle world position and timestamp (x,y), and flag when
        # a new waypoint was just activated so we can bias the first rotation.
        self.last_obstacle_world = None
        self.last_obstacle_time = 0.0
        self.just_activated_waypoint = False

    def _avoid_cb(self, msg):
        # store the latest avoidance suggestion and timestamp
        self.last_avoid_cmd = msg
        self.last_avoid_time = self._now_sec()

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
        emergency_threshold = max(0.0, self.hard_escape_distance * self.ir_emergency_raw_ratio)
        if math.isfinite(raw_range) and raw_range <= emergency_threshold:
            return raw_range
        if math.isfinite(filtered_range):
            return filtered_range
        if math.isfinite(raw_range):
            return raw_range
        if math.isfinite(filtered_range) and math.isfinite(raw_range):
            return min(filtered_range, raw_range)
        if math.isfinite(filtered_range):
            return filtered_range
        return raw_range

    def _reset_motion_selection(self):
        self.active_motion_selection = None
        self.active_motion_hold_until_sec = 0.0

    def _reset_blocked_side(self):
        self.blocked_side = None
        self.blocked_side_clear_count = 0
        self.avoid_memory = None

    def _blocked_side_from_sensor(self, sensor_name):
        if sensor_name in ('left', 'front_left'):
            return 'left'
        if sensor_name in ('right', 'front_right'):
            return 'right'
        return None

    def _blocked_side_clearance(self, side_name):
        # For binary IR sensors, return 0.0 when either relevant sensor is blocked,
        # otherwise +inf to indicate clear.
        if side_name == 'left':
            return 0.0 if self._sensors_blocked_any('left', 'front_left') else float('inf')
        if side_name == 'right':
            return 0.0 if self._sensors_blocked_any('right', 'front_right') else float('inf')
        return float('inf')

    def _avoidance_rejoin_allowed(self):
        if self.blocked_side is None:
            return True
        if self.blocked_clearance_active:
            return False
        return self.blocked_side_clear_count >= self.blocked_side_release_cycles

    def _start_avoidance_memory(self, blocked_side, bypass_side):
        if blocked_side not in ('left', 'right') or bypass_side not in ('left', 'right'):
            return
        self.avoid_memory = {
            'blocked_side': blocked_side,
            'bypass_side': bypass_side,
            'phase': 'commit_bypass',
            'cycles_remaining': self.avoid_commit_cycles,
        }

    def _set_forward_until_clear_memory(self, blocked_side):
        if blocked_side not in ('left', 'right'):
            return

        self.avoid_memory = {
            'blocked_side': blocked_side,
            'bypass_side': 'right' if blocked_side == 'left' else 'left',
            'phase': 'forward_until_clear',
            'cycles_remaining': self.avoid_forward_cycles,
        }

    def _goal_side_preference(self, goal_body_x, goal_body_y):
        if goal_body_x <= 0.0:
            return None
        if goal_body_y >= self.blocked_goal_lateral_min:
            return 'left'
        if goal_body_y <= -self.blocked_goal_lateral_min:
            return 'right'
        return None

    def _should_prefer_forward_until_clear(self, goal_body_x, goal_body_y, forward_blocked):
        if self.blocked_side is None:
            return False
        if self._goal_side_preference(goal_body_x, goal_body_y) != self.blocked_side:
            return False
        # prefer forward-until-clear only when forward sensors are currently clear
        if forward_blocked:
            return False
        return True

    def _update_blocked_side(self, hottest_sensor):
        sensed_side = self._blocked_side_from_sensor(hottest_sensor)

        if self.blocked_side is None:
            if sensed_side is not None:
                self.blocked_side = sensed_side
                self.blocked_side_clear_count = 0
                # entering a blocked side — cancel any prior clearance tracking
                self.blocked_clearance_active = False
            return
        # Binary sensors: count clear cycles when the blocked side's sensors are all clear
        if self.blocked_side == 'left':
            side_blocked = self._sensors_blocked_any('left', 'front_left')
        elif self.blocked_side == 'right':
            side_blocked = self._sensors_blocked_any('right', 'front_right')
        else:
            side_blocked = False

        if not side_blocked:
            # start counting clear cycles; when cleared, begin odometry-based clearance
            self.blocked_side_clear_count += 1
            if self.blocked_side_clear_count >= self.blocked_side_release_cycles and not self.blocked_clearance_active:
                # begin clearance hold: record odom origin and required forward distance
                left_x = self.sensor_positions.get('left', (0.0, 0.0))[0]
                back_x = self.sensor_positions.get('back', (0.0, 0.0))[0]
                # distance from the sensor to the rear-most chassis edge (approx)
                back_chassis = back_x - left_x if self.blocked_side == 'left' else back_x - self.sensor_positions.get('right', (0.0, 0.0))[0]
                # safety margin param
                margin = self.blocked_clearance_margin
                self.blocked_clearance_target = max(0.0, back_chassis + margin)
                self.blocked_clearance_origin = (self.center_x, self.center_y, self.yaw)
                self.blocked_clearance_active = True
                self.blocked_side_clear_count = 0
                return
        else:
            self.blocked_side_clear_count = 0

        if sensed_side == self.blocked_side:
            self.blocked_side_clear_count = 0

        # If we are tracking odometry-based clearance, check for completion
        if self.blocked_clearance_active and self._blocked_clearance_reached():
            self.blocked_side = None
            self.blocked_clearance_active = False
            self.blocked_side_clear_count = 0
            self.blocked_clearance_origin = None
            self.blocked_clearance_target = 0.0

    def _blocked_clearance_reached(self):
        if not self.blocked_clearance_active or self.blocked_clearance_origin is None:
            return False
        ox, oy, oyaw = self.blocked_clearance_origin
        dx = self.center_x - ox
        dy = self.center_y - oy
        forward = math.cos(oyaw) * dx + math.sin(oyaw) * dy
        return forward >= self.blocked_clearance_target

    def _update_avoidance_memory(self, hottest_sensor):
        sensed_side = self._blocked_side_from_sensor(hottest_sensor)
        self._update_blocked_side(hottest_sensor)

        if self.avoid_memory is None:
            return

        blocked_side = self.avoid_memory['blocked_side']
        if self.blocked_side not in (None, blocked_side):
            self.avoid_memory = None
            return

        if sensed_side == blocked_side:
            self.avoid_memory = {
                'blocked_side': blocked_side,
                'bypass_side': 'right' if blocked_side == 'left' else 'left',
                'phase': 'commit_bypass',
                'cycles_remaining': self.avoid_commit_cycles,
            }
            return

        if not self._avoidance_rejoin_allowed():
            if self.avoid_memory['phase'] == 'forward_until_clear':
                self.avoid_memory['cycles_remaining'] = max(
                    self.avoid_memory['cycles_remaining'],
                    self.avoid_forward_cycles,
                )
            return

        if self.avoid_memory['phase'] == 'commit_bypass':
            self.avoid_memory['phase'] = 'forward_until_clear'
            self.avoid_memory['cycles_remaining'] = self.avoid_forward_cycles
            return

        if self.avoid_memory['phase'] == 'forward_until_clear':
            self.avoid_memory['cycles_remaining'] -= 1
            if self.avoid_memory['cycles_remaining'] <= 0:
                self.avoid_memory = None

    def _motion_blocked_by_memory(self, motion_name):
        if self.blocked_side == 'left' and motion_name in ('left', 'diag_left'):
            return True
        if self.blocked_side == 'right' and motion_name in ('right', 'diag_right'):
            return True

        if self.avoid_memory is None:
            return False

        blocked_side = self.avoid_memory['blocked_side']
        bypass_side = self.avoid_memory['bypass_side']
        phase = self.avoid_memory['phase']

        if blocked_side == 'left' and motion_name in ('left', 'diag_left'):
            return True
        if blocked_side == 'right' and motion_name in ('right', 'diag_right'):
            return True

        if phase == 'commit_bypass':
            allowed = {'forward', 'backward', bypass_side, 'diag_' + bypass_side}
            return motion_name not in allowed

        if phase == 'forward_until_clear':
            allowed = {'forward', 'backward', bypass_side, 'diag_' + bypass_side}
            return motion_name not in allowed

        return False

    def _candidate_is_safe(self, candidate, clearance):
        return not math.isfinite(clearance) or clearance > candidate['threshold']

    def _select_committed_motion(self, scored_candidates, danger_mode):
        if not scored_candidates:
            self._reset_motion_selection()
            return None

        scored_candidates.sort(key=lambda item: item[0], reverse=True)
        best_score, best_motion_name, best_candidate, best_clearance = scored_candidates[0]
        now_sec = self._now_sec()

        if self.active_motion_selection is None:
            self.active_motion_selection = best_motion_name
            self.active_motion_hold_until_sec = now_sec + self.motion_hold_sec
            return best_score, best_motion_name, best_candidate, best_clearance

        active_entry = None
        for entry in scored_candidates:
            if entry[1] == self.active_motion_selection:
                active_entry = entry
                break

        if active_entry is None:
            self.active_motion_selection = best_motion_name
            self.active_motion_hold_until_sec = now_sec + self.motion_hold_sec
            return best_score, best_motion_name, best_candidate, best_clearance

        active_score, active_motion_name, active_candidate, active_clearance = active_entry
        if not self._candidate_is_safe(
            active_candidate,
            active_clearance - self.motion_clearance_hysteresis,
        ):
            self.active_motion_selection = best_motion_name
            self.active_motion_hold_until_sec = now_sec + self.motion_hold_sec
            return best_score, best_motion_name, best_candidate, best_clearance

        if danger_mode and active_motion_name.startswith('diag') and best_motion_name in ('left', 'right', 'backward'):
            self.active_motion_selection = best_motion_name
            self.active_motion_hold_until_sec = now_sec + self.motion_hold_sec
            return best_score, best_motion_name, best_candidate, best_clearance

        if now_sec < self.active_motion_hold_until_sec:
            return active_entry

        if best_motion_name != active_motion_name and best_score > active_score + self.motion_switch_score_margin:
            self.active_motion_selection = best_motion_name
            self.active_motion_hold_until_sec = now_sec + self.motion_hold_sec
            return best_score, best_motion_name, best_candidate, best_clearance

        self.active_motion_hold_until_sec = now_sec + self.motion_hold_sec
        return active_entry

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
        # mark that we just activated a waypoint so the next rotation can
        # perform a rotate-away maneuver if an obstacle was recently seen
        self.just_activated_waypoint = True
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
        self.current_planar_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

        if self.last_path_point is None:
            self.last_path_point = (self.center_x, self.center_y)
            self.robot_path.append(self._make_pose(self.center_x, self.center_y, self._geometry_body_yaw()))
            return

        dx = self.center_x - self.last_path_point[0]
        dy = self.center_y - self.last_path_point[1]
        if math.hypot(dx, dy) >= self.path_point_spacing:
            self.last_path_point = (self.center_x, self.center_y)
            self.robot_path.append(self._make_pose(self.center_x, self.center_y, self._geometry_body_yaw()))
            # Reduce trail history to avoid heavy RViz rendering while preserving visibility
            self.robot_path = self.robot_path[-800:]

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

    def _sensor_blocked(self, sensor_name):
        # Binary interpretation of IR: blocked if sensor reports a hit
        return self._sensor_hit_visible(sensor_name)

    def _sensors_blocked_any(self, *sensor_names):
        for n in sensor_names:
            if self._sensor_blocked(n):
                return True
        return False

    def _all_sensors_clear_binary(self):
        return not any(self._sensor_blocked(n) for n in self.sensor_names)

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

            # Draw a threshold arc at the configured detect distance so RViz
            # clearly indicates the IR trigger radius (e.g., 0.15 m)
            range_max = self.ir_range_max.get(sensor_name, self.ir_default_range_max)
            if not math.isfinite(range_max):
                range_max = self.ir_default_range_max
            detect_range = min(self.obstacle_detect_distance, range_max)
            threshold_arc = []
            for step in range(arc_segments + 1):
                ratio = step / arc_segments
                ray_angle = sensor_yaw - self.ir_half_fov + (2.0 * self.ir_half_fov * ratio)
                threshold_arc.append(
                    self._make_point(
                        origin_x + detect_range * math.cos(ray_angle),
                        origin_y + detect_range * math.sin(ray_angle),
                        0.092,
                    )
                )
            thr = Marker()
            thr.header.frame_id = self.debug_frame_id
            thr.header.stamp = now
            thr.ns = 'arena_ir_threshold'
            thr.id = 1300 + index
            thr.type = Marker.LINE_STRIP
            thr.action = Marker.ADD
            thr.pose.orientation.w = 1.0
            thr.scale.x = 0.02
            thr.color.r = 1.0
            thr.color.g = 1.0
            thr.color.b = 0.2
            thr.color.a = 0.7
            thr.points.append(self._make_point(origin_x, origin_y, 0.092))
            thr.points.extend(threshold_arc)
            thr.points.append(self._make_point(origin_x, origin_y, 0.092))
            markers.append(thr)

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
        # TIME-SEQUENCED BINARY 20cm EVASION
        # This honors a binary IR trigger at 0.20 m and runs a timed maneuver
        # using the front sensors. We return a repulsion vector (body-frame)
        # and the hottest sensor plus a synthetic range (0.0 when active).
        repulse_x = 0.0
        repulse_y = 0.0
        hottest_sensor = None
        hottest_range = float('inf')

        now = self._now_sec()

        # Track triggers and active threats from forward-facing sensors
        active_threats = []
        time_since_trigger = float('inf')
        for name in ('front', 'front_left', 'front_right'):
            dist = self._sensor_control_value(name)
            if math.isfinite(dist) and dist <= self.obstacle_detect_distance:
                # register trigger time
                self._last_ir_trigger[name] = now
                active_threats.append(name)
                time_since_trigger = 0.0
            else:
                last = self._last_ir_trigger.get(name, 0.0)
                elapsed = now - last
                if elapsed <= self.repulse_decay_sec:
                    active_threats.append(name)
                    if elapsed < time_since_trigger:
                        time_since_trigger = elapsed

        if active_threats:
            # Phase 1: initial backing phase (0.0 - 0.4s)
            if time_since_trigger <= 0.40:
                repulse_x = -0.50
                repulse_y = 0.0
                hottest_sensor = active_threats[0]
                hottest_range = 0.0
                return repulse_x, repulse_y, hottest_sensor, hottest_range

            # Phase 2: rotate-in-place phase (0.4s - decay)
            # Provide a reduced backward repulsion (to cancel forward drive)
            repulse_x = -0.30
            repulse_y = 0.0
            # determine rotation side based on which front sensors triggered
            if 'front_left' in active_threats and 'front_right' not in active_threats:
                hottest_sensor = 'front_left'
            elif 'front_right' in active_threats and 'front_left' not in active_threats:
                hottest_sensor = 'front_right'
            else:
                # center or ambiguous: choose front_left as default bias (turn right)
                hottest_sensor = 'front'
            hottest_range = 0.0
            return repulse_x, repulse_y, hottest_sensor, hottest_range

        # Fallback: no front binary threats — compute simple repulsion from any blocked sensor
        directions = {
            'front': (1.0, 0.0),
            'front_left': (1.0, 0.9),
            'front_right': (1.0, -0.9),
            'left': (0.0, 1.0),
            'right': (0.0, -1.0),
            'back': (-1.0, 0.0),
        }
        for sensor_name in self.sensor_names:
            if self._sensor_blocked(sensor_name):
                dir_x, dir_y = directions[sensor_name]
                repulse_x += dir_x * 1.0
                repulse_y += dir_y * 1.0
                if hottest_sensor is None:
                    hottest_sensor = sensor_name
        if hottest_sensor is not None:
            hottest_range = 0.0
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
            # Reduced backward magnitude and speed_scale to limit how far the
            # robot retreats during reroute/escape maneuvers.
            'backward': {
                'vx': -0.6,
                'vy': 0.0,
                'speed_scale': 0.35,
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

    def _choose_motion_command(self, desired_body_x, desired_body_y, goal_body_x, goal_body_y, goal_heading_error, hottest_sensor, hottest_range):
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

        forward_candidate = candidates['forward']
        forward_blocked = self._sensors_blocked_any(*forward_candidate['sensors'])
        prefer_forward_until_clear = self._should_prefer_forward_until_clear(
            goal_body_x,
            goal_body_y,
            forward_blocked,
        )

        if prefer_forward_until_clear and (
            self.avoid_memory is None or self.avoid_memory.get('phase') != 'forward_until_clear'
        ):
            self._set_forward_until_clear_memory(self.blocked_side)

        scored_candidates = []
        for motion_name, candidate in candidates.items():
            if self._motion_blocked_by_memory(motion_name):
                continue

            # Binary blocked check: if any of the candidate sensors report blocked,
            # consider the candidate unsafe.
            blocked = self._sensors_blocked_any(*candidate['sensors'])
            if blocked:
                continue

            if motion_name == 'forward' and abs(goal_heading_error) > self.forward_heading_allowance and desired_body_x >= 0.0:
                continue

            cand_norm = max(1e-6, math.hypot(candidate['vx'], candidate['vy']))
            direction_score = (
                candidate['vx'] * desired_body_x + candidate['vy'] * desired_body_y
            ) / (cand_norm * desired_norm)
            # Binary clearance score: clear -> best score contribution, blocked -> excluded above
            clearance_score = 1.0
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
            if self.blocked_side is not None and motion_name == 'forward':
                score += self.blocked_side_forward_bias
            if prefer_forward_until_clear and motion_name == 'forward':
                score += self.blocked_goal_forward_bonus
            if self.avoid_memory is not None:
                if self.avoid_memory['phase'] == 'commit_bypass':
                    if motion_name == self.avoid_memory['bypass_side']:
                        score += 0.70
                    elif motion_name == ('diag_' + self.avoid_memory['bypass_side']):
                        score += 0.45
                    elif motion_name == 'forward':
                        score += 0.12
                elif self.avoid_memory['phase'] == 'forward_until_clear' and motion_name == 'forward':
                    score += 0.85
            # motion_clearance: +inf when clear (for later scaling), 0.0 when blocked (but blocked already filtered)
            motion_clearance = float('inf')
            scored_candidates.append((score, motion_name, candidate, motion_clearance))

        selected = self._select_committed_motion(scored_candidates, danger_mode)
        if selected is not None:
            _, motion_name, candidate, clearance = selected
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
        # Reflexive avoidance override: if a recent /avoid_cmd_vel exists, use it
        now = self._now_sec()
        if self.avoid_override_enable and self.last_avoid_cmd is not None and (now - self.last_avoid_time) <= self.avoid_override_timeout:
            cmd = Twist()
            # copy suggested command but enforce sensor gating and speed limits
            cmd.linear.x = float(self.last_avoid_cmd.linear.x)
            cmd.linear.y = float(self.last_avoid_cmd.linear.y)
            cmd.angular.z = float(self.last_avoid_cmd.angular.z)

            # Binary sensor gating: stop motion along an axis if relevant sensors are blocked
            if cmd.linear.x > 0.0 and self._sensors_blocked_any('front', 'front_left', 'front_right'):
                cmd.linear.x = 0.0
            if cmd.linear.y > 0.0 and self._sensors_blocked_any('left', 'front_left'):
                cmd.linear.y = 0.0
            if cmd.linear.y < 0.0 and self._sensors_blocked_any('right', 'front_right'):
                cmd.linear.y = 0.0
            if cmd.linear.x < 0.0 and self._sensors_blocked_any('back'):
                cmd.linear.x = 0.0

            # enforce max speeds
            cmd.linear.x = clamp(cmd.linear.x, -self.max_linear_speed, self.max_linear_speed)
            cmd.linear.y = clamp(cmd.linear.y, -self.max_lateral_speed, self.max_lateral_speed)
            cmd.angular.z = clamp(cmd.angular.z, -self.max_angular_speed, self.max_angular_speed)

            # publish and update last command state
            self.cmd_pub.publish(cmd)
            # update last_cmd_* for continuity
            # Note: store internal representation (vx, vy, wz) consistent with prior usage
            self.last_cmd_vx = cmd.linear.x / (self.cmd_forward_sign if self.cmd_forward_sign != 0 else 1.0)
            self.last_cmd_vy = cmd.linear.y
            self.last_cmd_wz = cmd.angular.z
            self.last_cmd_time_sec = now
            return

        # Binary sensor gating: stop motion along an axis if relevant sensors are blocked
        if vx > 0.0 and self._sensors_blocked_any('front', 'front_left', 'front_right'):
            vx = 0.0
        if vy > 0.0 and self._sensors_blocked_any('left', 'front_left'):
            vy = 0.0
        if vy < 0.0 and self._sensors_blocked_any('right', 'front_right'):
            vy = 0.0
        if vx < 0.0 and self._sensors_blocked_any('back'):
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
            self._reset_motion_selection()
            self._reset_blocked_side()
            self._publish_cmd(0.0, 0.0, 0.0)
            return

        if not self._odom_fresh():
            self.state_name = 'STALE_ODOM'
            self.motion_name = 'STOP'
            self._reset_motion_selection()
            self._reset_blocked_side()
            self._publish_cmd(0.0, 0.0, 0.0)
            return

        if not self._sensors_fresh():
            self.state_name = 'STALE_IR'
            self.motion_name = 'STOP'
            self._reset_motion_selection()
            self._reset_blocked_side()
            self._publish_cmd(0.0, 0.0, 0.0)
            return

        if not self.startup_gate_complete:
            if self._all_sensors_clear_binary():
                self.startup_clear_cycles += 1
                self.blocked_started_sec = None
            else:
                self.startup_clear_cycles = 0
                if self.blocked_started_sec is None:
                    self.blocked_started_sec = self._now_sec()

            if self.startup_clear_cycles >= 3:
                self.startup_gate_complete = True
                self.blocked_started_sec = None
            else:
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
                self._reset_motion_selection()
                self._reset_blocked_side()
                self._publish_cmd(0.0, 0.0, 0.0)
                return

        if self.control_mode == 'acceptance_path':
            if self.route_failed:
                self.state_name = 'ROUTE_FAILED'
                self.motion_name = 'STOP'
                self._reset_motion_selection()
                self._reset_blocked_side()
                self._publish_cmd(0.0, 0.0, 0.0)
                return
            if self.route_complete:
                self.state_name = 'ROUTE_COMPLETE'
                self.motion_name = 'STOP'
                self._reset_motion_selection()
                self._reset_blocked_side()
                self._publish_cmd(0.0, 0.0, 0.0)
                return

        if self.goal_x is None or self.goal_y is None:
            self._choose_new_goal('Startup target selected.')
            if self.goal_x is None or self.goal_y is None:
                self.state_name = 'NO_TARGET'
                self.motion_name = 'STOP'
                self._reset_motion_selection()
                self._reset_blocked_side()
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
                    self._reset_motion_selection()
                    self._reset_blocked_side()
                    self._publish_cmd(0.0, 0.0, 0.0)
                    return
                self._advance_route_waypoint('Waypoint reached.')
                if self.route_complete:
                    self.state_name = 'ROUTE_COMPLETE'
                    self.motion_name = 'STOP'
                    self._reset_motion_selection()
                    self._reset_blocked_side()
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
                self._reset_motion_selection()
                self._reset_blocked_side()
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
        # record recent obstacle world position when a forward-facing sensor sees one
        if hottest_sensor is not None:
            sensor_range = self._sensor_control_value(hottest_sensor)
            if math.isfinite(sensor_range) and sensor_range <= self.obstacle_detect_distance:
                origin_bx, origin_by = self._sensor_origin_body(hottest_sensor)
                dir_bx, dir_by = self._sensor_direction_body(hottest_sensor)
                obs_body_x = origin_bx + dir_bx * sensor_range
                obs_body_y = origin_by + dir_by * sensor_range
                obs_world_x, obs_world_y = self._body_point_to_world(obs_body_x, obs_body_y)
                self.last_obstacle_world = (obs_world_x, obs_world_y)
                self.last_obstacle_time = self._now_sec()
        self._update_avoidance_memory(hottest_sensor)
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
            self._reset_motion_selection()
            self._publish_cmd(
                escape_x * self.hard_escape_speed,
                escape_y * self.hard_escape_speed,
                self._rotation_command(goal_heading_error, hottest_sensor, None),
            )
            return

        # --- REROUTE STATE MACHINE (3-point turn style) ---
        # Trigger reroute when a forward-facing sensor detects an obstacle.
        # The reroute will: back-diagonal away from the detected side, stop,
        # rotate in place away from obstacle, drive forward, then pause and
        # correct heading toward the path if clear. Repeat until cleared.
        if hottest_sensor in ('front_left', 'front_right') or (hottest_sensor == 'front' and not math.isfinite(self._sensor_min('front_left', 'front_right'))):
            if self.reroute_state is None:
                # Trigger reroute — do NOT skip or advance the current waypoint.
                # Keep waypoint so robot will continue attempting to reach it
                # after the reroute completes.
                self.reroute_state = {'phase': 'back_diag', 'start_sec': self._now_sec(), 'sensor': hottest_sensor}

        if self.reroute_state is not None:
            state = self.reroute_state
            elapsed = self._now_sec() - state['start_sec']
            sensor = state.get('sensor')

            # Compute a normalized escape vector for the detected sensor and use
            # the opposite direction for backing away.
            esc_x, esc_y = self._focused_escape_body_vector(sensor)
            norm = max(1e-6, math.hypot(esc_x, esc_y))
            # desired back/away vector is opposite of escape vector
            back_dir_x = -esc_x / norm
            back_dir_y = -esc_y / norm

            # Phase: back_diag
            if state['phase'] == 'back_diag':
                if elapsed < self.reroute_back_time:
                    # Back further/stronger to move away from detected obstacle
                    back_speed = 0.35
                    vx = back_dir_x * back_speed
                    vy = back_dir_y * (back_speed * 0.65)

                    # Use side/back IRs as observers: if intended lateral/backward
                    # component is blocked, zero that axis to avoid driving into it.
                    if vy > 0 and self._sensors_blocked_any('left', 'front_left'):
                        vy = 0.0
                    if vy < 0 and self._sensors_blocked_any('right', 'front_right'):
                        vy = 0.0
                    if vx < 0 and self._sensors_blocked_any('back'):
                        vx = 0.0

                    self._publish_cmd(vx, vy, 0.0)
                    return
                state['phase'] = 'stop'
                state['start_sec'] = self._now_sec()
                return

            # Phase: brief stop
            if state['phase'] == 'stop':
                if elapsed < self.reroute_stop_time:
                    self._publish_cmd(0.0, 0.0, 0.0)
                    return
                state['phase'] = 'rotate'
                state['start_sec'] = self._now_sec()
                return

            # Phase: rotate in place away from obstacle (no strafing)
            if state['phase'] == 'rotate':
                if elapsed < self.reroute_rotate_time:
                    # stronger in-place rotation away from obstacle
                    rotate_sign = math.copysign(1.0, esc_y)
                    wz = rotate_sign * self.reroute_rotate_speed
                    self._publish_cmd(0.0, 0.0, clamp(wz, -self.max_angular_speed, self.max_angular_speed))
                    return
                state['phase'] = 'forward'
                state['start_sec'] = self._now_sec()
                return

            # Phase: drive forward a short burst to attempt rejoin
            if state['phase'] == 'forward':
                if elapsed < (self.reroute_back_time * 0.9 + self.reroute_rotate_time):
                    # drive forward only (no lateral). Use stronger forward burst
                    fwd = self.rotate_forward_speed
                    # require front clearance greater than obstacle_clear_distance + margin
                    fc = self._sensor_min('front', 'front_left', 'front_right')
                    required_clear = self.obstacle_clear_distance + 0.06
                    if not math.isfinite(fc) or fc < required_clear or self._sensors_blocked_any('front', 'front_left', 'front_right'):
                        fwd = 0.0
                    self._publish_cmd(fwd, 0.0, 0.0)
                    return
                # after forward burst, if still blocked restart, otherwise pause
                if self._sensors_blocked_any('front', 'front_left', 'front_right'):
                    self.reroute_state = {'phase': 'back_diag', 'start_sec': self._now_sec(), 'sensor': sensor}
                    return
                state['phase'] = 'post_clear'
                state['start_sec'] = self._now_sec()
                return

            # Phase: short pause to 'take a breath' then finish reroute
            # NOTE: per user request, we do NOT try to face the path here —
            # complete the 3-point reroute and resume normal behavior.
            if state['phase'] == 'post_clear':
                if elapsed < self.reroute_post_clear_pause:
                    self._publish_cmd(0.0, 0.0, 0.0)
                    return
                # end reroute and allow normal motion selection to resume
                self.reroute_state = None
                return

        if hottest_sensor is None and abs(goal_heading_error) >= self.reorient_heading_threshold:
            self.state_name = 'REORIENT'
            self.motion_name = 'ROTATE'
            self._reset_motion_selection()
            # If we just activated a waypoint and recently saw an obstacle,
            # perform a one-time rotate-away + lateral/back-off maneuver to
            # move away from that obstacle before committing the heading.
            if self.just_activated_waypoint and self.last_obstacle_world is not None and (self._now_sec() - self.last_obstacle_time) < 8.0:
                lx, ly = self.last_obstacle_world
                # obstacle in body frame
                rel_bx, rel_by = self._world_to_body_vector(lx - self.center_x, ly - self.center_y)
                bearing = math.atan2(rel_by, rel_bx)
                # move laterally away and rotate away (stronger than normal)
                lateral_mag = 0.28
                vy = -math.copysign(lateral_mag, bearing) if abs(bearing) > 1e-3 else 0.0
                # rotate away from obstacle: rotate sign opposite obstacle bearing
                if abs(bearing) > 1e-3:
                    rotate_sign = -math.copysign(1.0, bearing)
                else:
                    # fallback: pick the side with more clearance
                    left_clear = self._sensor_min('left', 'front_left')
                    right_clear = self._sensor_min('right', 'front_right')
                    rotate_sign = 1.0 if left_clear > right_clear else -1.0
                wz = clamp(rotate_sign * max(self.reroute_rotate_speed, 0.6), -self.max_angular_speed, self.max_angular_speed)
                # a small forward to keep momentum into the waypoint direction
                vx = min(self.rotate_forward_speed * 1.1, self.max_linear_speed)
                self._publish_cmd(vx, vy, wz)
                self.just_activated_waypoint = False
                return
            # Drive forward slightly while rotating, do not strafe.
            self._publish_cmd(self.rotate_forward_speed, 0.0, self._rotation_command(goal_heading_error, None, None))
            return

        if hottest_sensor is None and abs(goal_heading_error) > self.forward_heading_allowance:
            self.state_name = 'ALIGN_FORWARD'
            self.motion_name = 'ROTATE'
            self._reset_motion_selection()
            # If we just activated a waypoint, bias away from last obstacle first
            if self.just_activated_waypoint and self.last_obstacle_world is not None and (self._now_sec() - self.last_obstacle_time) < 8.0:
                lx, ly = self.last_obstacle_world
                rel_bx, rel_by = self._world_to_body_vector(lx - self.center_x, ly - self.center_y)
                bearing = math.atan2(rel_by, rel_bx)
                lateral_mag = 0.22
                vy = -math.copysign(lateral_mag, bearing) if abs(bearing) > 1e-3 else 0.0
                if abs(bearing) > 1e-3:
                    rotate_sign = -math.copysign(1.0, bearing)
                else:
                    left_clear = self._sensor_min('left', 'front_left')
                    right_clear = self._sensor_min('right', 'front_right')
                    rotate_sign = 1.0 if left_clear > right_clear else -1.0
                wz = clamp(rotate_sign * max(self.reroute_rotate_speed * 0.85, 0.5), -self.max_angular_speed, self.max_angular_speed)
                vx = min(self.rotate_forward_speed, self.max_linear_speed)
                self._publish_cmd(vx, vy, wz)
                self.just_activated_waypoint = False
                return
            # Drive forward slightly while correcting heading; no strafing.
            self._publish_cmd(self.rotate_forward_speed, 0.0, self._rotation_command(goal_heading_error, None, None))
            return

        motion_name, candidate, motion_clearance, danger_mode = self._choose_motion_command(
            desired_body_x,
            desired_body_y,
            goal_body_x,
            goal_body_y,
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
            self._reset_motion_selection()
            # If just activated waypoint, bias away from last obstacle
            if self.just_activated_waypoint and self.last_obstacle_world is not None and (self._now_sec() - self.last_obstacle_time) < 8.0:
                lx, ly = self.last_obstacle_world
                rel_bx, rel_by = self._world_to_body_vector(lx - self.center_x, ly - self.center_y)
                bearing = math.atan2(rel_by, rel_bx)
                lateral_mag = 0.20
                vy = -math.copysign(lateral_mag, bearing) if abs(bearing) > 1e-3 else 0.0
                if abs(bearing) > 1e-3:
                    rotate_sign = -math.copysign(1.0, bearing)
                else:
                    left_clear = self._sensor_min('left', 'front_left')
                    right_clear = self._sensor_min('right', 'front_right')
                    rotate_sign = 1.0 if left_clear > right_clear else -1.0
                wz = clamp(rotate_sign * max(self.reroute_rotate_speed * 0.7, 0.45), -self.max_angular_speed, self.max_angular_speed)
                vx = min(self.rotate_forward_speed, self.max_linear_speed)
                self._publish_cmd(vx, vy, wz)
                self.just_activated_waypoint = False
                return
            # Rotate while moving forward a bit to help rejoin path; no lateral motion.
            self._publish_cmd(self.rotate_forward_speed, 0.0, self._rotation_command(goal_heading_error, rotate_sensor, None))
            return

        if motion_name == 'forward' and self._sensors_blocked_any('front', 'front_left', 'front_right'):
            # Prefer a side that is clear (binary). If both clear, choose by current clearance.
            left_blocked = self._sensors_blocked_any('left', 'front_left')
            right_blocked = self._sensors_blocked_any('right', 'front_right')
            if not left_blocked and right_blocked:
                side_choice = 'left'
            elif not right_blocked and left_blocked:
                side_choice = 'right'
            else:
                side_choice = 'left' if self._sensor_min('left', 'front_left') >= self._sensor_min('right', 'front_right') else 'right'

            candidate = self._movement_candidate_map()[side_choice]
            side_clear_ok = not self._sensors_blocked_any(*candidate['sensors'])
            if side_clear_ok:
                motion_name = side_choice
                motion_clearance = float('inf')
                blocked_side = 'right' if side_choice == 'left' else 'left'
                self._start_avoidance_memory(blocked_side, side_choice)
            else:
                self.state_name = 'FRONT_BLOCKED_ROTATE'
                self.motion_name = 'ROTATE'
                self._reset_motion_selection()
                # When front is blocked, rotate in place without strafing or forward drive.
                self._publish_cmd(0.0, 0.0, self._rotation_command(goal_heading_error, 'front', None))
                return

        if self.avoid_memory is None and hottest_sensor in ('front_left', 'left') and motion_name in ('right', 'diag_right'):
            self._start_avoidance_memory('left', 'right')
        elif self.avoid_memory is None and hottest_sensor in ('front_right', 'right') and motion_name in ('left', 'diag_left'):
            self._start_avoidance_memory('right', 'left')

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
            self._reset_motion_selection()
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
            'state=%s motion=%s goal=%s%s blocked_side=%s pos=(%.2f, %.2f) ir[f=%.2f fl=%.2f fr=%.2f l=%.2f r=%.2f b=%.2f]' % (
                self.state_name,
                self.motion_name,
                goal_text,
                route_text,
                self.blocked_side,
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