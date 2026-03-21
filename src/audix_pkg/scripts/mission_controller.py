#!/usr/bin/env python3
"""
Unified mission controller for Audix warehouse robot.

State machine:
    IDLE -> ROTATE_TO_TARGET -> MOVE_FORWARD -> ROTATE_TO_SCAN
    -> LIFTING_UP -> SCANNING -> ROTATE_BACK_TO_PATH -> LIFTING_DOWN
    -> (next waypoint or COMPLETE)

Interrupt states: OBSTACLE_HALT, OBSTACLE_REROUTE

Subscribes to 6 individual IR sensor topics, IMU, filtered odom, enable flag.
Publishes /cmd_vel (Twist) and high-level scissor lift slider commands.
Receives mission goals via /send_mission service.
"""

import math
import subprocess
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


def _quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def _euler_from_quat(x, y, z, w):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(t2)
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


class State(Enum):
    IDLE = auto()
    ROTATE_TO_TARGET = auto()
    MOVE_FORWARD = auto()
    ROTATE_TO_SCAN = auto()
    LIFTING_UP = auto()
    SCANNING = auto()
    LIFTING_DOWN = auto()
    ROTATE_BACK_TO_PATH = auto()
    OBSTACLE_HALT = auto()
    OBSTACLE_REROUTE = auto()
    COMPLETE = auto()


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # --- Declare ALL parameters ---
        self.declare_parameter('waypoints', [0.0])
        self.declare_parameter('obstacle_speed_full', 0.30)
        self.declare_parameter('obstacle_speed_slow', 0.18)
        self.declare_parameter('obstacle_speed_creep', 0.08)
        self.declare_parameter('obstacle_detect_distance', 0.15)
        self.declare_parameter('obstacle_clear_distance', 0.15)
        self.declare_parameter('obstacle_danger_distance', 0.15)
        self.declare_parameter('obstacle_side_min', 0.15)
        self.declare_parameter('front_blocked_threshold', 3)
        self.declare_parameter('clear_threshold', 5)
        self.declare_parameter('reroute_strafe_speed', 0.16)
        self.declare_parameter('reroute_creep_speed', 0.08)
        self.declare_parameter('reroute_lateral_distance', 0.42)
        self.declare_parameter('reroute_advance_distance', 0.45)
        self.declare_parameter('reroute_return_lookahead', 0.18)
        self.declare_parameter('reroute_return_tolerance', 0.03)
        self.declare_parameter('reroute_rejoin_extra_distance', 0.30)
        self.declare_parameter('reroute_side_clearance', 0.20)
        self.declare_parameter('reroute_side_clear_hysteresis', 0.03)
        self.declare_parameter('reroute_side_clear_cycles', 6)
        self.declare_parameter('obstacle_memory_clear_cycles', 6)
        self.declare_parameter('obstacle_halt_timeout', 2.5)
        self.declare_parameter('reroute_timeout', 12.0)
        self.declare_parameter('reroute_wall_follow_distance', 0.70)
        self.declare_parameter('reroute_rotation_search_angle_deg', 55.0)
        self.declare_parameter('reroute_rotation_clear_distance', 0.28)
        self.declare_parameter('reroute_rotation_max_attempts', 2)
        self.declare_parameter('obstacle_sensor_warmup_sec', 2.0)
        self.declare_parameter('ir_filter_alpha_rise', 0.18)
        self.declare_parameter('ir_filter_alpha_fall', 0.70)
        self.declare_parameter('obstacle_safety_margin', 0.06)
        self.declare_parameter('obstacle_path_corridor_margin', 0.03)
        self.declare_parameter('obstacle_return_tolerance', 0.02)
        self.declare_parameter('obstacle_envelope_cone_gain', 0.60)
        self.declare_parameter('optimized_rejoin_min_lookahead', 0.04)
        self.declare_parameter('optimized_rejoin_lateral_gain', 0.35)
        self.declare_parameter('lift_dwell_time', 3.0)
        self.declare_parameter('lift_position_tolerance', 0.003)
        self.declare_parameter('lift_motion_time', 2.0)
        self.declare_parameter('linear_kp', 0.5)
        self.declare_parameter('angular_kp', 1.5)
        self.declare_parameter('scan_angle_tolerance', 0.08)
        self.declare_parameter('scan_rotation_kp_scale', 1.6)
        self.declare_parameter('scan_rotation_min_speed', 0.12)
        self.declare_parameter('scan_rotation_max_speed', 1.2)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('angle_tolerance', 0.05)
        self.declare_parameter('scan_turn_degrees', 90.0)
        self.declare_parameter('scan_turn_direction', 'left')
        self.declare_parameter('scripted_skid_scan_mode', False)
        self.declare_parameter('skid_lateral_distance', 0.6)
        self.declare_parameter('end_goal_distance', 4.0)
        self.declare_parameter('scan_use_lift', False)
        self.declare_parameter('straight_line_only', True)
        self.declare_parameter('straight_line_distance', 3.0)
        self.declare_parameter('enable_obstacle_avoidance', False)
        self.declare_parameter('enable_stop_scan', False)
        self.declare_parameter('status_log_to_terminal', True)
        self.declare_parameter('status_log_period_sec', 0.5)
        self.declare_parameter('debug_visualization', True)
        self.declare_parameter('debug_path_point_spacing', 0.03)
        self.declare_parameter('debug_path_max_points', 2500)
        self.declare_parameter('robot_center_offset_x', -0.16)
        self.declare_parameter('robot_center_offset_y', -0.15)
        self.declare_parameter('robot_body_frame_flip_180', True)
        self.declare_parameter('straight_line_lift_height', 0.0)
        self.declare_parameter('cmd_linear_accel_limit', 0.9)
        self.declare_parameter('cmd_linear_decel_limit', 1.8)
        self.declare_parameter('cmd_lateral_accel_limit', 1.1)
        self.declare_parameter('cmd_lateral_decel_limit', 2.2)
        self.declare_parameter('cmd_angular_accel_limit', 2.8)
        self.declare_parameter('cmd_angular_decel_limit', 4.5)

        # Load parameters
        self.obstacle_speed_full = float(self.get_parameter('obstacle_speed_full').value)
        self.obstacle_speed_slow = float(self.get_parameter('obstacle_speed_slow').value)
        self.obstacle_speed_creep = float(self.get_parameter('obstacle_speed_creep').value)
        self.obstacle_detect_distance = float(self.get_parameter('obstacle_detect_distance').value)
        self.obstacle_clear_distance = float(self.get_parameter('obstacle_clear_distance').value)
        self.obstacle_danger_distance = float(self.get_parameter('obstacle_danger_distance').value)
        self.obstacle_side_min = float(self.get_parameter('obstacle_side_min').value)
        self.front_blocked_threshold = int(self.get_parameter('front_blocked_threshold').value)
        self.clear_threshold = int(self.get_parameter('clear_threshold').value)
        self.reroute_strafe_speed = float(self.get_parameter('reroute_strafe_speed').value)
        self.reroute_creep_speed = float(self.get_parameter('reroute_creep_speed').value)
        self.reroute_lateral_distance = float(self.get_parameter('reroute_lateral_distance').value)
        self.reroute_advance_distance = float(self.get_parameter('reroute_advance_distance').value)
        self.reroute_return_lookahead = float(self.get_parameter('reroute_return_lookahead').value)
        self.reroute_return_tolerance = float(self.get_parameter('reroute_return_tolerance').value)
        self.reroute_rejoin_extra_distance = float(self.get_parameter('reroute_rejoin_extra_distance').value)
        self.reroute_side_clearance = float(self.get_parameter('reroute_side_clearance').value)
        self.reroute_side_clear_hysteresis = float(self.get_parameter('reroute_side_clear_hysteresis').value)
        self.reroute_side_clear_cycles = int(self.get_parameter('reroute_side_clear_cycles').value)
        self.obstacle_memory_clear_cycles = int(self.get_parameter('obstacle_memory_clear_cycles').value)
        self.obstacle_halt_timeout = float(self.get_parameter('obstacle_halt_timeout').value)
        self.reroute_timeout = float(self.get_parameter('reroute_timeout').value)
        self.reroute_wall_follow_distance = float(self.get_parameter('reroute_wall_follow_distance').value)
        self.reroute_rotation_search_angle = math.radians(
            float(self.get_parameter('reroute_rotation_search_angle_deg').value)
        )
        self.reroute_rotation_clear_distance = float(self.get_parameter('reroute_rotation_clear_distance').value)
        self.reroute_rotation_max_attempts = int(self.get_parameter('reroute_rotation_max_attempts').value)
        self.obstacle_warmup = self.get_parameter('obstacle_sensor_warmup_sec').value
        self.ir_filter_alpha_rise = float(self.get_parameter('ir_filter_alpha_rise').value)
        self.ir_filter_alpha_fall = float(self.get_parameter('ir_filter_alpha_fall').value)
        self.obstacle_safety_margin = float(self.get_parameter('obstacle_safety_margin').value)
        self.obstacle_path_corridor_margin = float(self.get_parameter('obstacle_path_corridor_margin').value)
        self.obstacle_return_tolerance = float(self.get_parameter('obstacle_return_tolerance').value)
        self.obstacle_envelope_cone_gain = float(self.get_parameter('obstacle_envelope_cone_gain').value)
        self.optimized_rejoin_min_lookahead = float(self.get_parameter('optimized_rejoin_min_lookahead').value)
        self.optimized_rejoin_lateral_gain = float(self.get_parameter('optimized_rejoin_lateral_gain').value)
        self.lift_dwell = self.get_parameter('lift_dwell_time').value
        self.lift_tol = self.get_parameter('lift_position_tolerance').value
        self.lift_motion_time = float(self.get_parameter('lift_motion_time').value)
        self.lin_kp = self.get_parameter('linear_kp').value
        self.ang_kp = self.get_parameter('angular_kp').value
        self.scan_ang_tol = float(self.get_parameter('scan_angle_tolerance').value)
        self.scan_rotation_kp_scale = float(self.get_parameter('scan_rotation_kp_scale').value)
        self.scan_rotation_min_speed = float(self.get_parameter('scan_rotation_min_speed').value)
        self.scan_rotation_max_speed = float(self.get_parameter('scan_rotation_max_speed').value)
        self.max_lin = self.get_parameter('max_linear_speed').value
        self.max_ang = self.get_parameter('max_angular_speed').value
        self.pos_tol = self.get_parameter('position_tolerance').value
        self.ang_tol = self.get_parameter('angle_tolerance').value
        self.scan_turn_rad = math.radians(self.get_parameter('scan_turn_degrees').value)
        self.scan_turn_direction = str(self.get_parameter('scan_turn_direction').value).lower()
        self.scripted_skid_scan_mode = bool(self.get_parameter('scripted_skid_scan_mode').value)
        self.skid_lateral_distance = float(self.get_parameter('skid_lateral_distance').value)
        self.end_goal_distance = float(self.get_parameter('end_goal_distance').value)
        self.scan_use_lift = bool(self.get_parameter('scan_use_lift').value)
        self.straight_line_only = bool(self.get_parameter('straight_line_only').value)
        self.straight_line_distance = float(self.get_parameter('straight_line_distance').value)
        self.enable_obstacle_avoidance = bool(self.get_parameter('enable_obstacle_avoidance').value)
        self.enable_stop_scan = bool(self.get_parameter('enable_stop_scan').value)
        self.status_log_to_terminal = bool(self.get_parameter('status_log_to_terminal').value)
        self.status_log_period = float(self.get_parameter('status_log_period_sec').value)
        self.debug_viz = self.get_parameter('debug_visualization').value
        self.debug_path_spacing = self.get_parameter('debug_path_point_spacing').value
        self.debug_path_max_points = self.get_parameter('debug_path_max_points').value
        self.robot_center_offset_x = float(self.get_parameter('robot_center_offset_x').value)
        self.robot_center_offset_y = float(self.get_parameter('robot_center_offset_y').value)
        self.robot_body_frame_flip_180 = bool(self.get_parameter('robot_body_frame_flip_180').value)
        self.straight_line_lift_height = float(self.get_parameter('straight_line_lift_height').value)
        self.cmd_linear_accel_limit = float(self.get_parameter('cmd_linear_accel_limit').value)
        self.cmd_linear_decel_limit = float(self.get_parameter('cmd_linear_decel_limit').value)
        self.cmd_lateral_accel_limit = float(self.get_parameter('cmd_lateral_accel_limit').value)
        self.cmd_lateral_decel_limit = float(self.get_parameter('cmd_lateral_decel_limit').value)
        self.cmd_angular_accel_limit = float(self.get_parameter('cmd_angular_accel_limit').value)
        self.cmd_angular_decel_limit = float(self.get_parameter('cmd_angular_decel_limit').value)

        # Parse waypoints from YAML: flat list [x,y,yaw,h, x,y,yaw,h, ...]
        wp_flat = self.get_parameter('waypoints').value
        self.waypoints = []
        if wp_flat and len(wp_flat) >= 4:
            # Could be list-of-lists or flat
            if isinstance(wp_flat[0], (list, tuple)):
                for wp in wp_flat:
                    self.waypoints.append((float(wp[0]), float(wp[1]),
                                           float(wp[2]), float(wp[3])))
            else:
                for i in range(0, len(wp_flat), 4):
                    self.waypoints.append((float(wp_flat[i]), float(wp_flat[i+1]),
                                           float(wp_flat[i+2]), float(wp_flat[i+3])))

        # For waypoint-driven missions, use YAML waypoint lift targets directly.
        self.waypoint_lift_targets = [wp[3] for wp in self.waypoints]

        # --- State ---
        self.state = State.IDLE
        self.enabled = False
        self.mission_loaded = False
        self.current_wp_idx = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.center_x = 0.0
        self.center_y = 0.0

        # IR sensor readings
        self.ir = {
            'front': float('inf'),
            'front_left': float('inf'),
            'front_right': float('inf'),
            'left': float('inf'),
            'right': float('inf'),
            'back': float('inf'),
        }
        self.ir_raw = dict(self.ir)
        self.ir_range_max = {key: float('inf') for key in self.ir}

        # Obstacle tracking
        self.front_blocked_count = 0
        self.clear_count = 0
        self.raw_clear_count = 0
        self.pre_obstacle_state = None
        self.reroute_waypoints = []
        self.reroute_step = 0
        self.reroute_direction = 0.0
        self.reroute_start_x = 0.0
        self.reroute_start_y = 0.0
        self.reroute_start_progress = 0.0
        self.reroute_rejoin_min_progress = 0.0
        self.reroute_side_clear_count = 0
        self.reroute_entry_time = None
        self.path_line_start = None
        self.path_line_end = None
        self.node_start_time = self.get_clock().now()
        self.mission_start_center = (0.0, 0.0)
        self.mission_start_yaw = 0.0
        self.active_scan_direction = self.scan_turn_direction
        self.active_scan_dwell = self.lift_dwell
        self.waypoint_scan_directions = []
        self.waypoint_scan_dwell_times = []
        if self.straight_line_only:
            self.waypoint_lift_targets = []
        self.waypoint_parking_headings = {}
        self.leg_profiles = []
        self.active_leg_profile = None
        self.pending_obstacle_removals = []
        self.parking_mode = False
        self.active_leg_spawned = False
        self.active_leg_present = False
        self.active_leg_trigger_progress = None
        self.post_lift_state = None
        self.use_resume_heading_for_target_rotate = False
        self.reroute_plan_optimized = False
        self.reroute_target_offset = 0.0
        self.reroute_required_pass_distance = 0.0
        self.reroute_obstacle_side = None
        self.reroute_trigger_sensor = None
        self.obstacle_memory_trigger_sensor = None
        self.obstacle_memory_blocked_side = None
        self.obstacle_memory_bypass_side = None
        self.obstacle_memory_clear_count = 0
        self.reroute_rotation_target_yaw = 0.0
        self.reroute_rotation_direction = 0.0
        self.reroute_rotation_attempts = 0

        # Lift state
        self.current_lift_pos = 0.0
        self.target_lift_pos = 0.0
        self.lift_motion_start_time = None
        self.lift_motion_start_pos = 0.0
        self.current_planar_speed = 0.0
        self.scan_start_time = None
        self.scan_heading_target = 0.0
        self.resume_heading_target = 0.0
        self.scan_waypoint_indices = set()
        self.forced_turn_direction = 0  # 0: normal, 1: force CCW (Left), -1: force CW (Right)

        # Sensor positions used for RViz visualization (body-frame x,y)
        # Note: left/right and front_left/front_right placement below
        # are ordered so RViz markers align visually with URDF sensors.
        self.sensor_positions = {
            'back':        ( 0.16255, -0.00323),
            'right':       ( 0.00273,  0.15504),
            'front_right': (-0.17753,  0.12688),
            'front':       (-0.20195,  0.00482),
            'front_left':  (-0.18323, -0.11962),
            'left':        (-0.00537, -0.15346),
        }
        self.ir_half_fov = 0.30543
        self.ir_default_range_min = 0.05
        self.ir_default_range_max = 0.25
        self.front_extent = abs(self.sensor_positions['front'][0] - self.robot_center_offset_x)
        self.back_extent = abs(self.sensor_positions['back'][0] - self.robot_center_offset_x)
        self.left_extent = abs(self.sensor_positions['left'][1] - self.robot_center_offset_y)
        self.right_extent = abs(self.sensor_positions['right'][1] - self.robot_center_offset_y)
        self.robot_length = self.front_extent + self.back_extent
        self.robot_width = self.left_extent + self.right_extent

        # Vector repulsion params (integrated avoidance to ensure single-writer to /cmd_vel)
        self.declare_parameter('repulse_M_front', 0.30)
        self.declare_parameter('repulse_M_front_corner', 0.30)
        self.declare_parameter('repulse_M_side', 0.25)
        self.declare_parameter('repulse_M_back', 0.12)
        self.declare_parameter('repulse_decay_sec', 0.5)
        self.declare_parameter('repulse_angular_gain', 1.2)

        self.repulse_M_front = float(self.get_parameter('repulse_M_front').value)
        self.repulse_M_front_corner = float(self.get_parameter('repulse_M_front_corner').value)
        self.repulse_M_side = float(self.get_parameter('repulse_M_side').value)
        self.repulse_M_back = float(self.get_parameter('repulse_M_back').value)
        self.repulse_decay_sec = float(self.get_parameter('repulse_decay_sec').value)
        self.repulse_angular_gain = float(self.get_parameter('repulse_angular_gain').value)

        # Timestamp of last trigger per sensor for phantom-tail decay
        self._last_ir_trigger = {k: 0.0 for k in self.ir}

        # Debug visualization state
        self.robot_path_points = []
        self.last_path_point = None
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_wz = 0.0
        self.last_cmd_time = None

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(
            Float64,
            '/scissor_lift/slider',
            10
        )
        self.planned_path_pub = self.create_publisher(
            Path,
            '/debug/planned_path',
            10,
        )
        self.robot_trail_pub = self.create_publisher(
            Path,
            '/debug/robot_path',
            10,
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/debug/targets', 10)
        self.state_pub = self.create_publisher(String, '/debug/state', 10)

        # --- Subscribers ---
        # Normal 1-to-1 mapping. Fixes RViz visualization.
        ir_topics = {
            'front': '/ir_front/scan',
            'front_left': '/ir_front_left/scan',
            'front_right': '/ir_front_right/scan',
            'left': '/ir_left/scan',
            'right': '/ir_right/scan',
            'back': '/ir_back/scan',
        }
        for key, topic in ir_topics.items():
            self.create_subscription(
                LaserScan, topic,
                lambda msg, k=key: self._ir_cb(k, msg), 10
            )

        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, 10)
        self.create_subscription(Bool, '/robot_enable', self._enable_cb, 10)

        # --- Service ---
        self.create_service(Trigger, '/send_mission', self._mission_srv_cb)

        # --- Control timer at 20 Hz ---
        self.create_timer(0.05, self._control_loop)
        if self.status_log_to_terminal:
            self.create_timer(self.status_log_period, self._status_log_cb)
        if self.debug_viz:
            self.create_timer(0.2, self._publish_debug_viz)

        self.get_logger().info(
            f'Mission controller ready. {len(self.waypoints)} waypoints loaded.'
        )

    # ================================================================
    # Callbacks
    # ================================================================
    def _ir_cb(self, key: str, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=float)
        # Mask readings outside [range_min, range_max) — Gazebo returns range_max for "no hit"
        rmin = msg.range_min if msg.range_min > 0 else 0.01
        rmax = msg.range_max if msg.range_max > 0 else float('inf')
        ranges[(ranges < rmin) | (ranges >= rmax) | np.isnan(ranges)] = np.inf
        valid = ranges[np.isfinite(ranges)]
        raw_value = float(np.min(valid)) if len(valid) > 0 else float('inf')
        self.ir_raw[key] = raw_value
        self.ir_range_max[key] = rmax

        if not math.isfinite(raw_value):
            raw_proxy = rmax
        else:
            raw_proxy = raw_value

        prev_filtered = self.ir[key]
        if not math.isfinite(prev_filtered):
            if math.isfinite(raw_value):
                self.ir[key] = raw_value
            else:
                self.ir[key] = float('inf')
            return

        prev_proxy = prev_filtered
        alpha = self.ir_filter_alpha_fall if raw_proxy < prev_proxy else self.ir_filter_alpha_rise
        filtered_proxy = alpha * raw_proxy + (1.0 - alpha) * prev_proxy
        if not math.isfinite(raw_value) and filtered_proxy >= rmax * 0.98:
            self.ir[key] = float('inf')
        else:
            self.ir[key] = filtered_proxy
        # update last trigger timestamp for binary-style repulsion logic
        try:
            now_s = self.get_clock().now().nanoseconds / 1e9
            if math.isfinite(raw_value) and raw_value <= self.obstacle_detect_distance:
                self._last_ir_trigger[key] = now_s
        except Exception:
            # be defensive if called early during init
            pass

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = _euler_from_quat(q.x, q.y, q.z, q.w)
        self.yaw = yaw
        self.center_x, self.center_y = self._center_xy(self.x, self.y, self.yaw)
        self.current_planar_speed = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
        )

    def _imu_cb(self, msg: Imu):
        pass  # EKF handles IMU fusion; kept for potential direct use

    def _enable_cb(self, msg: Bool):
        prev = self.enabled
        self.enabled = msg.data
        if self.enabled and not prev:
            self.get_logger().info('Robot ENABLED — start signal received')
        elif not self.enabled and prev:
            self.get_logger().warn('Robot DISABLED — emergency stop')
            self._publish_stop()
            self.state = State.IDLE

    def _mission_srv_cb(self, request, response):
        if self.scripted_skid_scan_mode or self.straight_line_only or self.waypoints:
            self.mission_loaded = True
            response.success = True
            if self.scripted_skid_scan_mode:
                response.message = 'Mission loaded: scripted skid-scan mode'
            elif self.straight_line_only:
                response.message = 'Mission loaded: straight-line debug mode'
            else:
                response.message = f'Mission loaded: {len(self.waypoints)} waypoints'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = 'No waypoints in parameters'
            self.get_logger().error(response.message)
        return response

    def _service_pending_obstacle_removals(self):
        if not self.pending_obstacle_removals:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        remaining = []
        for item in self.pending_obstacle_removals:
            if now_sec >= item['time']:
                self._remove_obstacle(item['name'])
            else:
                remaining.append(item)
        self.pending_obstacle_removals = remaining

    def _schedule_obstacle_removal(self, name, delay_sec):
        if not name:
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        for item in self.pending_obstacle_removals:
            if item['name'] == name:
                return
        self.pending_obstacle_removals.append({'name': name, 'time': now_sec + delay_sec})

    def _spawn_box_obstacle(self, profile):
        name = profile['name']
        x, y = profile['pose']
        sx, sy, sz = profile.get('size', (0.14, 0.14, 0.24))
        rgba = profile.get('color', (0.15, 0.75, 0.95, 1.0))
        sdf = (
            f'<sdf version="1.8">'
            f'<model name="{name}">'
            f'<static>true</static>'
            f'<pose>0 0 0 0 0 0</pose>'
            f'<link name="link">'
            f'<collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>'
            f'<visual name="visual"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
            f'<material><ambient>{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}</ambient>'
            f'<diffuse>{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}</diffuse></material></visual>'
            f'</link></model></sdf>'
        )
        result = subprocess.run(
            [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', 'warehouse',
                '-name', name,
                '-string', sdf,
                '-x', str(x), '-y', str(y), '-z', str(sz * 0.5),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode == 0:
            if self.active_leg_profile and self.active_leg_profile.get('name') == name:
                self.active_leg_present = True
            self.get_logger().info(f'Spawned obstacle {name} at ({x:.2f}, {y:.2f})')
        else:
            self.get_logger().warn(
                f'Failed to spawn obstacle {name}: {(result.stderr or result.stdout).strip()}'
            )

    def _remove_obstacle(self, name):
        result = subprocess.run(
            [
                'gz', 'service',
                '-s', '/world/warehouse/remove',
                '--reqtype', 'gz.msgs.Entity',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '3000',
                '--req', f'name: "{name}" type: MODEL',
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode == 0 and 'data: true' in result.stdout:
            if self.active_leg_profile and self.active_leg_profile.get('name') == name:
                self.active_leg_present = False
            self.get_logger().info(f'Removed obstacle {name}')
            if (
                self.active_leg_profile
                and self.active_leg_profile.get('name') == name
                and self.state == State.OBSTACLE_REROUTE
            ):
                self._optimize_reroute_plan_from_current_pose(
                    'Obstacle vanished during reroute — optimizing back onto the original path'
                )
        elif result.returncode != 0:
            self.get_logger().warn(
                f'Failed removing obstacle {name}: {(result.stderr or result.stdout).strip()}'
            )

    def _clear_dynamic_obstacles(self):
        for name in ('dynamic_obstacle_seg2', 'dynamic_obstacle_seg3'):
            self._remove_obstacle(name)
        self.pending_obstacle_removals = []

    def _scan_enabled_for_waypoint(self, idx):
        return idx in self.scan_waypoint_indices and idx < len(self.waypoint_lift_targets)

    def _lift_target_for_waypoint(self, idx):
        if idx < len(self.waypoint_lift_targets):
            return self.waypoint_lift_targets[idx]
        return 0.0

    def _scan_direction_for_waypoint(self, idx):
        if idx < len(self.waypoint_scan_directions):
            return self.waypoint_scan_directions[idx]
        return self.scan_turn_direction

    def _scan_dwell_for_waypoint(self, idx):
        if idx < len(self.waypoint_scan_dwell_times):
            return self.waypoint_scan_dwell_times[idx]
        return self.lift_dwell

    def _parking_heading_for_waypoint(self, idx):
        return self.waypoint_parking_headings.get(idx)

    def _activate_leg(self, idx):
        self.active_leg_profile = self.leg_profiles[idx] if idx < len(self.leg_profiles) else None
        self.active_leg_spawned = False
        self.active_leg_present = False
        self.active_leg_trigger_progress = None
        profile = self.active_leg_profile
        if profile and profile.get('surprise_spawn'):
            leg_length = math.hypot(
                self.path_line_end[0] - self.path_line_start[0],
                self.path_line_end[1] - self.path_line_start[1],
            )
            self.active_leg_trigger_progress = max(
                0.65,
                min(1.20, 0.35 * leg_length),
            )
            return
        if profile and profile.get('spawn_on_activation'):
            if profile.get('spawn_distance_ahead') is not None:
                tangent, _ = self._path_unit_vectors()
                distance = float(profile['spawn_distance_ahead'])
                profile['pose'] = (
                    self.center_x + tangent[0] * distance,
                    self.center_y + tangent[1] * distance,
                )
            self._spawn_box_obstacle(profile)
            self.active_leg_spawned = True

    def _start_waypoint_motion(self, idx):
        self.current_wp_idx = idx
        self.path_line_start = (self.center_x, self.center_y)
        self.path_line_end = self.waypoints[idx][:2]
        self.parking_mode = False
        self._activate_leg(idx)
        if idx == 3:
            self.resume_heading_target = self._front_facing_heading_to(*self.waypoints[idx][:2])
            self.use_resume_heading_for_target_rotate = True
            self.state = State.ROTATE_TO_TARGET
            self.get_logger().info('Starting return leg — rotating onto the planned path')
            return
        if idx >= 3:
            self.resume_heading_target = self._front_facing_heading_to(*self.waypoints[idx][:2])
            self.use_resume_heading_for_target_rotate = True
            self.state = State.ROTATE_TO_TARGET
            return
        self.use_resume_heading_for_target_rotate = False
        self.state = State.MOVE_FORWARD

    def _advance_from_waypoint(self):
        next_idx = self.current_wp_idx + 1
        if next_idx < len(self.waypoints):
            self._start_waypoint_motion(next_idx)
            self.get_logger().info(f'Moving to waypoint {next_idx}')
        else:
            self.state = State.COMPLETE
            self.get_logger().info('ALL WAYPOINTS COMPLETE!')

    def _maybe_spawn_active_leg_obstacle(self):
        profile = self.active_leg_profile
        if not profile or self.active_leg_spawned or not profile.get('surprise_spawn'):
            return
        if self.state != State.MOVE_FORWARD:
            return

        current_progress = self._path_progress(self.center_x, self.center_y)
        if self.active_leg_trigger_progress is None or current_progress < self.active_leg_trigger_progress:
            return

        size_x, size_y, _ = profile.get('size', (0.14, 0.14, 0.24))
        obstacle_half_depth = 0.5 * max(size_x, size_y)
        reaction_time = self.front_blocked_threshold * 0.05 + 0.20
        current_speed = min(max(self.current_planar_speed, self.obstacle_speed_creep), self.max_lin)

        shell_clearance = self.front_extent + obstacle_half_depth + self.obstacle_safety_margin
        stopping_margin = self.obstacle_detect_distance + current_speed * reaction_time
        surprise_distance = max(
            0.85,
            shell_clearance + stopping_margin + profile.get('surprise_buffer', 0.18),
        )

        target_progress = self._path_progress(*self.path_line_end)
        remaining_progress = max(0.0, target_progress - current_progress)
        if remaining_progress <= surprise_distance + self.pos_tol:
            self.get_logger().warn(
                'Skipping dynamic obstacle spawn because the remaining path is shorter than the safe spawn distance'
            )
            self.active_leg_spawned = True
            return

        tangent, lateral = self._path_unit_vectors()
        lateral_offset = float(profile.get('spawn_lateral_offset', 0.0))
        profile['pose'] = (
            self.center_x + tangent[0] * surprise_distance + lateral[0] * lateral_offset,
            self.center_y + tangent[1] * surprise_distance + lateral[1] * lateral_offset,
        )
        self._spawn_box_obstacle(profile)
        self.active_leg_spawned = True

    # ================================================================
    # Helpers
    # ================================================================
    @staticmethod
    def _normalize(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _latch_safe_rotation_direction(self):
        """Determine which way to spin based on the last seen object."""
        now_s = self.get_clock().now().nanoseconds / 1e9
        left_time = max(self._last_ir_trigger.get('left', 0.0), self._last_ir_trigger.get('front_left', 0.0))
        right_time = max(self._last_ir_trigger.get('right', 0.0), self._last_ir_trigger.get('front_right', 0.0))

        self.forced_turn_direction = 0
        recent_threshold = 4.0  # Only care if an object was seen in the last 4 seconds

        if (now_s - left_time) < recent_threshold or (now_s - right_time) < recent_threshold:
            if left_time > right_time:
                self.forced_turn_direction = -1  # Object on Left -> Force Right Turn (CW)
                self.get_logger().info("Safe Pivot: Box recently on Left. Forcing RIGHT rotation.")
            elif right_time > left_time:
                self.forced_turn_direction = 1   # Object on Right -> Force Left Turn (CCW)
                self.get_logger().info("Safe Pivot: Box recently on Right. Forcing LEFT rotation.")


    @staticmethod
    def _slew_axis(current, target, accel_limit, decel_limit, dt):
        if dt <= 0.0:
            return target
        limit = decel_limit if current * target < 0.0 or abs(target) < abs(current) else accel_limit
        max_delta = max(0.0, limit) * dt
        if target > current:
            return min(target, current + max_delta)
        return max(target, current - max_delta)

    def _publish_stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.last_cmd_vx = 0.0
        self.last_cmd_vy = 0.0
        self.last_cmd_wz = 0.0
        self.last_cmd_time = self.get_clock().now()

    def _publish_cmd(self, vx=0.0, vy=0.0, wz=0.0):
        now = self.get_clock().now()
        # Apply integrated vector-repulsion avoidance before clamping/slewing
        if self.enable_obstacle_avoidance and self.state in (
            State.MOVE_FORWARD,
            State.ROTATE_TO_TARGET,
            State.ROTATE_TO_SCAN,
            State.ROTATE_BACK_TO_PATH,
        ):
            try:
                rx, ry, rw = self._compute_vector_repulsion()
                vx = vx + rx
                vy = vy + ry
                wz = wz + rw
            except Exception:
                # defensive: if repulsion fails, fall back to original command
                pass

            # allow negative vx so repulsion can command a safe reverse maneuver

        target_vx = max(-self.max_lin, min(self.max_lin, vx))
        target_vy = max(-self.max_lin, min(self.max_lin, vy))
        target_wz = max(-self.max_ang, min(self.max_ang, wz))

        if self.last_cmd_time is None:
            cmd_vx = target_vx
            cmd_vy = target_vy
            cmd_wz = target_wz
        else:
            dt = (now - self.last_cmd_time).nanoseconds / 1e9
            cmd_vx = self._slew_axis(
                self.last_cmd_vx,
                target_vx,
                self.cmd_linear_accel_limit,
                self.cmd_linear_decel_limit,
                dt,
            )
            cmd_vy = self._slew_axis(
                self.last_cmd_vy,
                target_vy,
                self.cmd_lateral_accel_limit,
                self.cmd_lateral_decel_limit,
                dt,
            )
            cmd_wz = self._slew_axis(
                self.last_cmd_wz,
                target_wz,
                self.cmd_angular_accel_limit,
                self.cmd_angular_decel_limit,
                dt,
            )

        cmd = Twist()
        if self.robot_body_frame_flip_180:
            cmd.linear.x = -cmd_vx
            cmd.linear.y = -cmd_vy
        else:
            cmd.linear.x = cmd_vx
            cmd.linear.y = cmd_vy
        cmd.angular.z = cmd_wz
        self.cmd_pub.publish(cmd)
        self.last_cmd_vx = cmd_vx
        self.last_cmd_vy = cmd_vy
        self.last_cmd_wz = cmd_wz
        self.last_cmd_time = now

    def _center_xy(self, base_x, base_y, yaw):
        # robot_center_offset is moved to zeroed YAML; center is the base origin
        return base_x, base_y

    def _compute_vector_repulsion(self):
        """
        SEQUENTIAL 3-POINT TURN (front sensors only):
        < 0.20m : Backup Straight (stronger but limited)
        0.20m - 0.35m : Rotate In Place (small back, stronger rotation)
        Side/Back sensors are handled by parking-camera limits elsewhere.
        """
        now_s = self.get_clock().now().nanoseconds / 1e9
        rx, ry, rw = 0.0, 0.0, 0.0

        backup_threats = []
        rotate_threats = []

        for name in ('front', 'front_left', 'front_right'):
            dist = self._sensor_range_for_detection(name)

            if math.isfinite(dist) and dist <= 0.20:
                backup_threats.append(name)
                self._last_ir_trigger[name] = now_s
            elif math.isfinite(dist) and dist <= 0.35:
                rotate_threats.append(name)
                self._last_ir_trigger[name] = now_s
            else:
                last = self._last_ir_trigger.get(name, 0.0)
                if (now_s - last) <= self.repulse_decay_sec:
                    rotate_threats.append(name)

        if backup_threats:
            # PHASE 1: BACKUP STRAIGHT (slightly reduced magnitude)
            rx = -0.40
            rw = 0.0

        elif rotate_threats:
            # PHASE 2: ROTATE IN PLACE (Toned down to prevent over-rotation)
            rx = -0.30
            if 'front_left' in rotate_threats and 'front_right' not in rotate_threats:
                rw = -0.80
            elif 'front_right' in rotate_threats and 'front_left' not in rotate_threats:
                rw = 0.80
            else:
                rw = -0.80

        return rx, ry, rw

    def _closest_front_obstacle(self):
        return min(self.ir['front'], self.ir['front_left'], self.ir['front_right'])

    def _effective_detect_distance(self):
        reaction_time = self.front_blocked_threshold * 0.05 + 0.10
        speed_margin = min(0.05, max(0.0, self.current_planar_speed) * reaction_time)
        sensor_cap = max(self.ir_default_range_min, self.ir_default_range_max - 0.01)
        return min(sensor_cap, self.obstacle_detect_distance + speed_margin)

    def _sensor_range_for_detection(self, sensor_name):
        filtered_range = self.ir[sensor_name]
        raw_range = self.ir_raw[sensor_name]
        if math.isfinite(filtered_range) and math.isfinite(raw_range):
            return min(filtered_range, raw_range)
        if math.isfinite(filtered_range):
            return filtered_range
        return raw_range

    def _front_sensor_names(self):
        return ['front', 'front_left', 'front_right']

    def _clear_obstacle_memory(self):
        self.obstacle_memory_trigger_sensor = None
        self.obstacle_memory_blocked_side = None
        self.obstacle_memory_bypass_side = None
        self.obstacle_memory_clear_count = 0

    def _side_sensor_names(self, side):
        if side == 'left':
            return ['left', 'front_left']
        if side == 'right':
            return ['right', 'front_right']
        return []

    def _latch_obstacle_memory(self, trigger_sensor=None, bypass_side=None):
        if trigger_sensor is None:
            trigger_sensor = self._choose_trigger_sensor(self._effective_detect_distance())
        if trigger_sensor is None:
            return

        blocked_side = None
        if trigger_sensor in ('front_left', 'left'):
            blocked_side = 'left'
                'spawn_lateral_offset': 0.30,
        elif trigger_sensor in ('front_right', 'right'):
            blocked_side = 'right'
        else:
            left_clearance = min(self.ir_raw['left'], self.ir_raw['front_left'])
            right_clearance = min(self.ir_raw['right'], self.ir_raw['front_right'])
            if left_clearance + self.reroute_side_clear_hysteresis < right_clearance:
                blocked_side = 'left'
            elif right_clearance + self.reroute_side_clear_hysteresis < left_clearance:
                blocked_side = 'right'
                'spawn_lateral_offset': -0.28,

        if bypass_side not in ('left', 'right'):
            if blocked_side == 'left':
                bypass_side = 'right'
            elif blocked_side == 'right':
                bypass_side = 'left'

        self.obstacle_memory_trigger_sensor = trigger_sensor
        self.obstacle_memory_blocked_side = blocked_side
        self.obstacle_memory_bypass_side = bypass_side
        self.obstacle_memory_clear_count = 0

    def _update_obstacle_memory(self):
        trigger_sensor = self._choose_trigger_sensor(self._effective_detect_distance())
        if trigger_sensor is not None:
            self._latch_obstacle_memory(trigger_sensor)
            return

        if self.obstacle_memory_trigger_sensor is None:
            return

        self.obstacle_memory_clear_count += 1
        if self.obstacle_memory_clear_count >= self.obstacle_memory_clear_cycles:
            self._clear_obstacle_memory()

    def _choose_trigger_sensor(self, threshold=None):
        if not self._obstacle_detection_active():
            return None
        threshold = self._effective_detect_distance() if threshold is None else threshold
        candidates = []
        for sensor_name in ('front', 'front_left', 'front_right', 'left', 'right'):
            candidate = self._sensor_blocks_path(sensor_name, threshold)
            if candidate is not None:
                candidates.append(candidate)

        if not candidates:
            return None
        candidates.sort(
            key=lambda item: (
                max(0.0, item['envelope']['min_progress'] - self._path_progress(self.center_x, self.center_y)),
                item['range'],
            )
        )
        return candidates[0]['sensor']

    def _obstacle_side_clear(self):
        if self.reroute_obstacle_side is None:
            return True
        side_clear_threshold = self.reroute_side_clearance + self.reroute_side_clear_hysteresis
        return all(self.ir[name] > side_clear_threshold for name in self._side_sensor_names(self.reroute_obstacle_side))

    def _front_path_clear(self):
        return all(self.ir[name] > self.obstacle_clear_distance for name in self._front_sensor_names())

    def _all_ir_clear(self):
        threshold = min(
            self.reroute_side_clearance + self.reroute_side_clear_hysteresis,
            self.obstacle_clear_distance,
        )
        return all(value > threshold for value in self.ir.values())

    def _obstacle_detection_active(self):
        elapsed = (self.get_clock().now() - self.node_start_time).nanoseconds / 1e9
        return elapsed >= self.obstacle_warmup

    def _front_obstacle_detected(self):
        return self._choose_trigger_sensor(self._effective_detect_distance()) is not None

    def _reset_obstacle_counters(self):
        self.front_blocked_count = 0
        self.clear_count = 0
        self.raw_clear_count = 0

    def _update_obstacle_counters(self):
        blocked = self._choose_trigger_sensor(self._effective_detect_distance()) is not None
        clear = self._choose_trigger_sensor(self.obstacle_clear_distance) is None
        if blocked:
            self.front_blocked_count += 1
            self.clear_count = 0
        elif clear:
            self.clear_count += 1
            self.front_blocked_count = 0

    def _max_allowed_speed(self):
        closest = min(self.ir_raw.values())
        if closest <= self.obstacle_danger_distance:
            return self.obstacle_speed_creep
        if closest <= self._effective_detect_distance():
            return self.obstacle_speed_slow
        return self.obstacle_speed_full

    def _apply_side_collision_prevention(self, vx, vy, wz):
        # PARKING CAMERA: Absolute limits to prevent moving INTO a wall.
        limit = 0.15

        left_dist = self.ir_raw.get('left', float('inf'))
        right_dist = self.ir_raw.get('right', float('inf'))
        back_dist = self.ir_raw.get('back', float('inf'))

        # Prevent strafing into walls
        if vy > 0.0 and left_dist < limit:
            vy = 0.0
        if vy < 0.0 and right_dist < limit:
            vy = 0.0

        # Prevent steering into walls
        if wz > 0.0 and left_dist < limit:
            wz = 0.0
        if wz < 0.0 and right_dist < limit:
            wz = 0.0

        # Prevent reversing into back wall
        if vx < 0.0 and back_dist < limit:
            vx = 0.0

        return vx, vy, wz

    def _publish_motion(self, vx=0.0, vy=0.0, wz=0.0, apply_speed_zone=True):
        wz = max(-self.max_ang, min(self.max_ang, wz))
        planar_speed = math.hypot(vx, vy)
        if apply_speed_zone and planar_speed > 1e-6:
            speed_cap = min(self._max_allowed_speed(), self.max_lin)
            if planar_speed > speed_cap:
                scale = speed_cap / planar_speed
                vx *= scale
                vy *= scale

        if abs(wz) > 1e-6:
            vx += wz * self.robot_center_offset_y
            vy -= wz * self.robot_center_offset_x

        # PASS wz INTO THE SAFETY CAMERA FUNCTION
        vx, vy, wz = self._apply_side_collision_prevention(vx, vy, wz)

        self._publish_cmd(vx=vx, vy=vy, wz=wz)

    def _publish_lift(self, slider_value):
        """Publish a normalized slider command so scissor_lift_mapper applies the agreed joint mapping."""
        msg = Float64()
        msg.data = max(0.0, min(1.0, slider_value))
        self.current_lift_pos = msg.data
        self.lift_pub.publish(msg)

    def _begin_lift_motion(self, target_slider):
        self.target_lift_pos = max(0.0, min(1.0, target_slider))
        self.lift_motion_start_pos = self.current_lift_pos
        self.lift_motion_start_time = self.get_clock().now()

    def _step_lift_motion(self):
        if self.lift_motion_start_time is None:
            self._publish_lift(self.target_lift_pos)
            return True

        elapsed = (self.get_clock().now() - self.lift_motion_start_time).nanoseconds / 1e9
        progress = 1.0 if self.lift_motion_time <= 1e-6 else max(0.0, min(1.0, elapsed / self.lift_motion_time))
        commanded = self.lift_motion_start_pos + (self.target_lift_pos - self.lift_motion_start_pos) * progress
        self._publish_lift(commanded)
        if progress >= 1.0:
            self.lift_motion_start_time = None
            return True
        return False

    def _return_leg_heading_target(self, tx, ty):
        return self._front_facing_heading_to(tx, ty)

    def _front_heading_from_vector(self, dx, dy):
        heading = math.atan2(dy, dx)
        if self.robot_body_frame_flip_180:
            heading += math.pi
        return self._normalize(heading)

    def _front_facing_heading_to(self, tx, ty):
        return self._front_heading_from_vector(tx - self.center_x, ty - self.center_y)

    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _make_point(self, x, y, z=0.0):
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point

    def _body_point_to_world(self, body_x, body_y):
        world_x, world_y = self._body_to_world_vector(body_x, body_y)
        return self.center_x + world_x, self.center_y + world_y

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
                        0.03,
                    )
                )

            cone = Marker()
            cone.header.frame_id = 'odom'
            cone.header.stamp = now
            cone.ns = 'ir_cones'
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
            cone.color.a = 0.16
            for step in range(arc_segments):
                cone.points.append(self._make_point(origin_x, origin_y, 0.03))
                cone.points.append(arc_points[step])
                cone.points.append(arc_points[step + 1])
            markers.append(cone)

            outline = Marker()
            outline.header.frame_id = 'odom'
            outline.header.stamp = now
            outline.ns = 'ir_cone_edges'
            outline.id = 1100 + index
            outline.type = Marker.LINE_STRIP
            outline.action = Marker.ADD
            outline.pose.orientation.w = 1.0
            outline.scale.x = 0.01
            outline.color.r = color_r
            outline.color.g = color_g
            outline.color.b = color_b
            outline.color.a = 0.9
            outline.points.append(self._make_point(origin_x, origin_y, 0.035))
            outline.points.extend(arc_points)
            outline.points.append(self._make_point(origin_x, origin_y, 0.035))
            markers.append(outline)

            hit = Marker()
            hit.header.frame_id = 'odom'
            hit.header.stamp = now
            hit.ns = 'ir_hits'
            hit.id = 1200 + index
            hit.type = Marker.SPHERE
            hit.action = Marker.ADD
            hit.pose.orientation.w = 1.0
            hit.scale.x = 0.035
            hit.scale.y = 0.035
            hit.scale.z = 0.035
            if self._sensor_hit_visible(sensor_name):
                hit_range = self.ir_raw[sensor_name]
                hit.pose.position.x = origin_x + hit_range * math.cos(sensor_yaw)
                hit.pose.position.y = origin_y + hit_range * math.sin(sensor_yaw)
                hit.pose.position.z = 0.04
                hit.color.r = 1.0
                hit.color.g = 0.1
                hit.color.b = 0.1
                hit.color.a = 0.95
            else:
                hit.pose.position.x = origin_x
                hit.pose.position.y = origin_y
                hit.pose.position.z = -10.0
                hit.color.a = 0.0
            markers.append(hit)

        return markers

    def _append_robot_path(self):
        if self.last_path_point is None:
            self.last_path_point = (self.center_x, self.center_y)
            self.robot_path_points.append(self._make_pose(self.center_x, self.center_y, self.yaw))
            return
        dx = self.center_x - self.last_path_point[0]
        dy = self.center_y - self.last_path_point[1]
        if math.hypot(dx, dy) < self.debug_path_spacing:
            return
        self.last_path_point = (self.center_x, self.center_y)
        self.robot_path_points.append(self._make_pose(self.center_x, self.center_y, self.yaw))
        if len(self.robot_path_points) > self.debug_path_max_points:
            self.robot_path_points = self.robot_path_points[-self.debug_path_max_points:]

    def _build_planned_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'

        poses = [self._make_pose(self.center_x, self.center_y, self.yaw)]
        if self.state == State.OBSTACLE_REROUTE and self.reroute_waypoints:
            for i in range(len(self.reroute_waypoints)):
                wx, wy = self.reroute_waypoints[i]
                poses.append(self._make_pose(wx, wy, 0.0))

        for i in range(self.current_wp_idx, len(self.waypoints)):
            wx, wy, wyaw, _ = self.waypoints[i]
            poses.append(self._make_pose(wx, wy, wyaw))

        path.poses = poses
        return path

    def _build_obstacle_markers(self, now):
        markers = []

        if self.active_leg_profile and self.active_leg_present and self.active_leg_profile.get('pose'):
            ox, oy = self.active_leg_profile['pose']
            sx, sy, sz = self.active_leg_profile.get('size', (0.14, 0.14, 0.24))
            rgba = self.active_leg_profile.get('color', (0.9, 0.2, 0.8, 1.0))

            obstacle = Marker()
            obstacle.header.frame_id = 'odom'
            obstacle.header.stamp = now
            obstacle.ns = 'dynamic_obstacles'
            obstacle.id = 300
            obstacle.type = Marker.CUBE
            obstacle.action = Marker.ADD
            obstacle.pose.position.x = ox
            obstacle.pose.position.y = oy
            obstacle.pose.position.z = 0.5 * sz
            obstacle.pose.orientation.w = 1.0
            obstacle.scale.x = sx
            obstacle.scale.y = sy
            obstacle.scale.z = sz
            obstacle.color.r = rgba[0]
            obstacle.color.g = rgba[1]
            obstacle.color.b = rgba[2]
            obstacle.color.a = 0.55
            markers.append(obstacle)

        envelope = self._estimate_obstacle_envelope_path_frame()
        if envelope is not None and self.path_line_start is not None:
            center_progress = 0.5 * (envelope['min_progress'] + envelope['max_progress'])
            center_lateral = 0.5 * (envelope['min_lateral'] + envelope['max_lateral'])
            ex, ey = self._point_on_path(center_progress, center_lateral)
            tangent, _ = self._path_unit_vectors()
            yaw = math.atan2(tangent[1], tangent[0])
            qx, qy, qz, qw = _quat_from_yaw(yaw)

            estimate = Marker()
            estimate.header.frame_id = 'odom'
            estimate.header.stamp = now
            estimate.ns = 'detected_obstacles'
            estimate.id = 320
            estimate.type = Marker.CUBE
            estimate.action = Marker.ADD
            estimate.pose.position.x = ex
            estimate.pose.position.y = ey
            estimate.pose.position.z = 0.09
            estimate.pose.orientation.x = qx
            estimate.pose.orientation.y = qy
            estimate.pose.orientation.z = qz
            estimate.pose.orientation.w = qw
            estimate.scale.x = max(0.08, envelope['max_progress'] - envelope['min_progress'])
            estimate.scale.y = max(0.08, envelope['max_lateral'] - envelope['min_lateral'])
            estimate.scale.z = 0.18
            estimate.color.r = 1.0
            estimate.color.g = 0.18
            estimate.color.b = 0.18
            estimate.color.a = 0.28
            markers.append(estimate)

        return markers

    def _build_target_markers(self):
        now = self.get_clock().now().to_msg()
        markers = []

        clear = Marker()
        clear.header.frame_id = 'odom'
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.append(clear)
        markers.extend(self._build_ir_markers(now))
        markers.extend(self._build_obstacle_markers(now))

        for i, wp in enumerate(self.waypoints):
            wx, wy, _, _ = wp

            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = now
            marker.ns = 'mission_targets'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = wx
            marker.pose.position.y = wy
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.14
            marker.scale.y = 0.14
            marker.scale.z = 0.14

            if i < self.current_wp_idx:
                marker.color.r = 0.3
                marker.color.g = 0.3
                marker.color.b = 0.3
                marker.color.a = 0.9
            elif i == self.current_wp_idx and self.state != State.COMPLETE:
                marker.color.r = 1.0
                marker.color.g = 0.85
                marker.color.b = 0.1
                marker.color.a = 1.0
            else:
                marker.color.r = 0.1
                marker.color.g = 0.75
                marker.color.b = 1.0
                marker.color.a = 0.9

            markers.append(marker)

            label = Marker()
            label.header.frame_id = 'odom'
            label.header.stamp = now
            label.ns = 'mission_labels'
            label.id = 100 + i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = wx
            label.pose.position.y = wy
            label.pose.position.z = 0.22
            label.pose.orientation.w = 1.0
            label.scale.z = 0.12
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'WP{i}'
            markers.append(label)

        if self.state == State.OBSTACLE_REROUTE:
            for i in range(len(self.reroute_waypoints)):
                rx, ry = self.reroute_waypoints[i]
                reroute = Marker()
                reroute.header.frame_id = 'odom'
                reroute.header.stamp = now
                reroute.ns = 'reroute_targets'
                reroute.id = 200 + i
                reroute.type = Marker.CUBE
                reroute.action = Marker.ADD
                reroute.pose.position.x = rx
                reroute.pose.position.y = ry
                reroute.pose.orientation.w = 1.0
                reroute.scale.x = 0.12
                reroute.scale.y = 0.12
                reroute.scale.z = 0.12
                reroute.color.r = 1.0
                reroute.color.g = 0.2
                reroute.color.b = 0.9
                reroute.color.a = 1.0
                markers.append(reroute)

        state_marker = Marker()
        state_marker.header.frame_id = 'odom'
        state_marker.header.stamp = now
        state_marker.ns = 'mission_state'
        state_marker.id = 900
        state_marker.type = Marker.TEXT_VIEW_FACING
        state_marker.action = Marker.ADD
        state_marker.pose.position.x = self.center_x
        state_marker.pose.position.y = self.center_y
        state_marker.pose.position.z = 0.45
        state_marker.pose.orientation.w = 1.0
        state_marker.scale.z = 0.10
        state_marker.color.r = 0.95
        state_marker.color.g = 0.95
        state_marker.color.b = 0.95
        state_marker.color.a = 1.0
        state_marker.text = (
            f'{self.state.name} | wp={self.current_wp_idx} '
            f'| f={self.ir["front"]:.2f} fl={self.ir["front_left"]:.2f} '
            f'fr={self.ir["front_right"]:.2f}'
        )
        markers.append(state_marker)

        arr = MarkerArray()
        arr.markers = markers
        return arr

    def _publish_debug_viz(self):
        if not self.debug_viz:
            return

        trail = Path()
        trail.header.stamp = self.get_clock().now().to_msg()
        trail.header.frame_id = 'odom'

        self._append_robot_path()
        trail.poses = self.robot_path_points
        self.robot_trail_pub.publish(trail)

        self.planned_path_pub.publish(self._build_planned_path())
        self.marker_pub.publish(self._build_target_markers())

        s = String()
        s.data = (
            f'state={self.state.name}, wp={self.current_wp_idx}, '
            f'front={self.ir["front"]:.3f}, front_left={self.ir["front_left"]:.3f}, '
            f'front_right={self.ir["front_right"]:.3f}, cx={self.center_x:.3f}, cy={self.center_y:.3f}'
        )
        self.state_pub.publish(s)

    def _height_to_slider(self, height_m):
        """Convert desired platform height to the normalized slider used by scissor_lift_mapper."""
        max_height = 0.3383
        clamped_height = max(0.0, min(max_height, height_m))
        return clamped_height / max_height if max_height > 1e-6 else 0.0

    def _scan_direction_sign(self):
        return -1.0 if self.active_scan_direction == 'right' else 1.0

    def _compute_resume_heading(self, tx, ty):
        if self.current_wp_idx + 1 < len(self.waypoints):
            nx, ny, _, _ = self.waypoints[self.current_wp_idx + 1]
            return self._front_heading_from_vector(nx - tx, ny - ty)
        if self.current_wp_idx > 0:
            px, py, _, _ = self.waypoints[self.current_wp_idx - 1]
            return self._front_heading_from_vector(tx - px, ty - py)
        return self.yaw

    def _start_skid_scan_mission(self):
        base_yaw = self.yaw
        self.resume_heading_target = base_yaw
        fx = math.cos(base_yaw)
        fy = math.sin(base_yaw)
        rx = math.cos(base_yaw - math.pi / 2.0)
        ry = math.sin(base_yaw - math.pi / 2.0)

        lateral_distance = self.skid_lateral_distance
        x0, y0 = self.center_x, self.center_y
        p1 = (x0 + lateral_distance * rx, y0 + lateral_distance * ry)
        p2 = (p1[0] - 2.0 * lateral_distance * rx, p1[1] - 2.0 * lateral_distance * ry)
        p3 = (p2[0] + lateral_distance * rx, p2[1] + lateral_distance * ry)
        p4 = (
            p3[0] + self.end_goal_distance * fx,
            p3[1] + self.end_goal_distance * fy,
        )

        self.waypoints = [
            (p1[0], p1[1], base_yaw, 0.0),
            (p2[0], p2[1], base_yaw, 0.0),
            (p3[0], p3[1], base_yaw, 0.0),
            (p4[0], p4[1], base_yaw, 0.0),
        ]
        self.scan_waypoint_indices = {0, 1}
        self.current_wp_idx = 0
        self.path_line_start = (self.center_x, self.center_y)
        self.path_line_end = self.waypoints[0][:2]
        self.state = State.MOVE_FORWARD
        self.get_logger().info(
            'Mission started! Skid sequence generated: RIGHT -> LEFT -> CENTER -> GOAL'
        )

    def _start_straight_line_mission(self):
        self._clear_dynamic_obstacles()
        self.mission_start_center = (self.center_x, self.center_y)
        self.mission_start_yaw = self.yaw
        tangent = (-math.cos(self.yaw), -math.sin(self.yaw))
        right = (tangent[1], -tangent[0])
        first_segment_length = 1.95
        remaining_length = max(1.0, self.straight_line_distance - first_segment_length)
        later_segment_length = remaining_length / 2.0

        def point(progress, lateral=0.0):
            return (
                self.mission_start_center[0] + tangent[0] * progress + right[0] * lateral,
                self.mission_start_center[1] + tangent[1] * progress + right[1] * lateral,
            )

        seg1_end = first_segment_length
        seg2_end = first_segment_length + later_segment_length
        seg3_end = first_segment_length + 2.0 * later_segment_length

        wp0 = point(seg1_end)
        wp1 = point(seg2_end)
        wp2 = point(seg3_end)

        self.waypoints = [
            (wp0[0], wp0[1], self.yaw, 0.0),
            (wp1[0], wp1[1], self.yaw, 0.0),
            (wp2[0], wp2[1], self.yaw, 0.0),
        ]
        self.scan_waypoint_indices = {0, 1, 2}
        self.waypoint_scan_directions = ['left', 'right', 'left']
        self.waypoint_scan_dwell_times = [2.0, 2.0, 2.0]
        self.waypoint_lift_targets = [0.25, 0.50, 0.75]
        self.waypoint_parking_headings = {}
        self.leg_profiles = [
            None,
            {
                'name': 'dynamic_obstacle_seg2',
                'surprise_spawn': True,
                'remove_after_detect': 1.1,
                'surprise_buffer': 0.16,
                'size': (0.14, 0.14, 0.24),
                'color': (0.10, 0.75, 0.95, 1.0),
            },
            {
                'name': 'dynamic_obstacle_seg3',
                'surprise_spawn': True,
                'remove_after_reroute': 2.3,
                'surprise_buffer': 0.16,
                'size': (0.14, 0.14, 0.24),
                'color': (0.95, 0.70, 0.10, 1.0),
            },
        ]
        self.resume_heading_target = self.yaw
        self._start_waypoint_motion(0)
        self.get_logger().info(
            'Mission started! Three straight segments with three inspection stops are active'
        )

    def _status_log_cb(self):
        self.get_logger().info(
            'state=%s wp=%d pos=(%.2f, %.2f) yaw=%.2f | '
            'ir[f=%.2f fl=%.2f fr=%.2f l=%.2f r=%.2f b=%.2f]' % (
                self.state.name,
                self.current_wp_idx,
                self.center_x,
                self.center_y,
                self.yaw,
                self.ir['front'],
                self.ir['front_left'],
                self.ir['front_right'],
                self.ir['left'],
                self.ir['right'],
                self.ir['back'],
            )
        )

    def _reset_reroute_state(self):
        self.reroute_waypoints = []
        self.reroute_step = 0
        self.reroute_direction = 0.0
        self.reroute_start_x = self.center_x
        self.reroute_start_y = self.center_y
        self.reroute_start_progress = self._path_progress(self.center_x, self.center_y)
        self.reroute_rejoin_min_progress = self.reroute_start_progress
        self.reroute_side_clear_count = 0
        self.reroute_entry_time = None
        self.reroute_plan_optimized = False
        self.reroute_target_offset = 0.0
        self.reroute_required_pass_distance = 0.0
        self.reroute_obstacle_side = None
        self.reroute_trigger_sensor = None
        self.reroute_rotation_target_yaw = self.yaw
        self.reroute_rotation_direction = 0.0
        self.reroute_rotation_attempts = 0
        self._clear_obstacle_memory()

    def _point_to_line_distance(self, px, py):
        if self.path_line_start is None or self.path_line_end is None:
            return 0.0

        _, lateral = self._path_unit_vectors()
        dx = px - self.path_line_start[0]
        dy = py - self.path_line_start[1]
        return dx * lateral[0] + dy * lateral[1]

    def _path_unit_vectors(self):
        if self.path_line_start is None or self.path_line_end is None:
            return (math.cos(self.yaw), math.sin(self.yaw)), (-math.sin(self.yaw), math.cos(self.yaw))

        dx = self.path_line_end[0] - self.path_line_start[0]
        dy = self.path_line_end[1] - self.path_line_start[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return (math.cos(self.yaw), math.sin(self.yaw)), (-math.sin(self.yaw), math.cos(self.yaw))

        tangent = (dx / norm, dy / norm)
        lateral = (-tangent[1], tangent[0])
        return tangent, lateral

    def _path_progress(self, px, py):
        if self.path_line_start is None:
            return 0.0
        tangent, _ = self._path_unit_vectors()
        dx = px - self.path_line_start[0]
        dy = py - self.path_line_start[1]
        return dx * tangent[0] + dy * tangent[1]

    def _point_on_path(self, progress, lateral_offset=0.0):
        if self.path_line_start is None:
            return self.center_x, self.center_y
        tangent, lateral = self._path_unit_vectors()
        return (
            self.path_line_start[0] + progress * tangent[0] + lateral_offset * lateral[0],
            self.path_line_start[1] + progress * tangent[1] + lateral_offset * lateral[1],
        )

    def _sensor_origin_body(self, sensor_name):
        sensor_x, sensor_y = self.sensor_positions[sensor_name]
        return (
            self.robot_center_offset_x - sensor_x,
            self.robot_center_offset_y - sensor_y,
        )

    def _geometry_body_yaw(self):
        if self.robot_body_frame_flip_180:
            return self._normalize(self.yaw + math.pi)
        return self.yaw

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

    def _sensor_hit_points_world(self, sensor_name, hit_range):
        origin_body_x, origin_body_y = self._sensor_origin_body(sensor_name)
        dir_body_x, dir_body_y = self._sensor_direction_body(sensor_name)
        sensor_body_yaw = math.atan2(dir_body_y, dir_body_x)
        points = []
        for angle_offset in (-self.ir_half_fov, 0.0, self.ir_half_fov):
            ray_body_yaw = sensor_body_yaw + angle_offset
            hit_body_x = origin_body_x + hit_range * math.cos(ray_body_yaw)
            hit_body_y = origin_body_y + hit_range * math.sin(ray_body_yaw)
            hit_world_x, hit_world_y = self._body_to_world_vector(hit_body_x, hit_body_y)
            points.append((self.center_x + hit_world_x, self.center_y + hit_world_y))
        return points

    def _sensor_obstacle_envelope_path_frame(self, sensor_name, hit_range):
        if not math.isfinite(hit_range):
            return None

        progress_samples = []
        lateral_samples = []
        for hit_world_x, hit_world_y in self._sensor_hit_points_world(sensor_name, hit_range):
            progress_samples.append(self._path_progress(hit_world_x, hit_world_y))
            lateral_samples.append(self._point_to_line_distance(hit_world_x, hit_world_y))

        if not progress_samples:
            return None

        cone_half_width = max(0.0, hit_range * math.sin(self.ir_half_fov))
        lateral_padding = self.obstacle_safety_margin + self.obstacle_envelope_cone_gain * cone_half_width
        progress_padding = self.obstacle_safety_margin + 0.25 * cone_half_width
        return {
            'sensor': sensor_name,
            'min_progress': min(progress_samples) - progress_padding,
            'max_progress': max(progress_samples) + progress_padding,
            'min_lateral': min(lateral_samples) - lateral_padding,
            'max_lateral': max(lateral_samples) + lateral_padding,
        }

    def _sensor_blocks_path(self, sensor_name, threshold):
        sensor_range = self._sensor_range_for_detection(sensor_name)
        if not math.isfinite(sensor_range):
            return None

        sensor_horizon = threshold + self.robot_length + self.obstacle_safety_margin
        if sensor_range > sensor_horizon:
            return None

        envelope = self._sensor_obstacle_envelope_path_frame(sensor_name, sensor_range)
        if envelope is None:
            return None

        current_progress = self._path_progress(self.center_x, self.center_y)
        forward_limit = current_progress + threshold + self.front_extent + self.obstacle_safety_margin
        rear_limit = current_progress - 0.25 * self.back_extent
        path_half_width = 0.5 * self.robot_width + self.obstacle_path_corridor_margin

        if envelope['max_progress'] < rear_limit or envelope['min_progress'] > forward_limit:
            return None
        if envelope['min_lateral'] > path_half_width or envelope['max_lateral'] < -path_half_width:
            return None

        return {
            'sensor': sensor_name,
            'range': sensor_range,
            'envelope': envelope,
        }

    def _estimate_obstacle_envelope_path_frame(self):
        sensor_horizon = max(self.obstacle_clear_distance, self.reroute_side_clearance) + self.robot_length
        progress_samples = []
        lateral_samples = []
        sensors = set()

        for sensor_name, raw_range in self.ir_raw.items():
            if not math.isfinite(raw_range) or raw_range > sensor_horizon:
                continue

            envelope = self._sensor_obstacle_envelope_path_frame(sensor_name, raw_range)
            if envelope is None:
                continue

            progress_samples.extend((envelope['min_progress'], envelope['max_progress']))
            lateral_samples.extend((envelope['min_lateral'], envelope['max_lateral']))
            sensors.add(sensor_name)

        if not progress_samples:
            return None

        return {
            'min_progress': min(progress_samples),
            'max_progress': max(progress_samples),
            'min_lateral': min(lateral_samples),
            'max_lateral': max(lateral_samples),
            'sensors': sensors,
        }

    def _candidate_bypass_offsets(self, envelope):
        fallback = max(
            self.reroute_lateral_distance,
            self.left_extent + self.obstacle_safety_margin,
            self.right_extent + self.obstacle_safety_margin,
        )
        if envelope is None:
            return {
                'left': fallback,
                'right': -fallback,
            }
        return {
            'left': envelope['max_lateral'] + self.right_extent + self.obstacle_safety_margin,
            'right': envelope['min_lateral'] - self.left_extent - self.obstacle_safety_margin,
        }

    def _select_reroute_plan(self, envelope, trigger_sensor):
        offsets = self._candidate_bypass_offsets(envelope)
        bypass_side = None

        if trigger_sensor in ('front_left', 'left'):
            bypass_side = 'right'
        elif trigger_sensor in ('front_right', 'right'):
            bypass_side = 'left'
        elif envelope is not None:
            if envelope['max_lateral'] <= 0.0:
                bypass_side = 'left'
            elif envelope['min_lateral'] >= 0.0:
                bypass_side = 'right'

        if bypass_side is None:
            left_clearance = min(self.ir_raw['left'], self.ir_raw['front_left'])
            right_clearance = min(self.ir_raw['right'], self.ir_raw['front_right'])
            if left_clearance > right_clearance + self.reroute_side_clear_hysteresis:
                bypass_side = 'left'
            elif right_clearance > left_clearance + self.reroute_side_clear_hysteresis:
                bypass_side = 'right'
            else:
                bypass_side = min(offsets.items(), key=lambda item: abs(item[1]))[0]

        reroute_direction = 1.0 if bypass_side == 'left' else -1.0
        obstacle_side = 'right' if bypass_side == 'left' else 'left'
        return bypass_side, obstacle_side, reroute_direction, offsets[bypass_side]

    def _geometry_based_rejoin_progress(self, envelope):
        base_progress = self.reroute_start_progress + self.robot_length + self.reroute_rejoin_extra_distance
        target_progress = self._path_progress(*self.path_line_end) if self.path_line_end is not None else None
        if envelope is None:
            if target_progress is None:
                return base_progress
            return min(base_progress, target_progress)

        sensors = envelope['sensors']
        tail_clearance = self.back_extent + self.obstacle_safety_margin
        if sensors & {'left', 'right'}:
            tail_clearance = max(
                tail_clearance,
                0.5 * self.robot_length + self.obstacle_safety_margin,
            )

        rejoin_progress = max(
            base_progress,
            envelope['max_progress'] + tail_clearance + self.reroute_rejoin_extra_distance,
        )
        if target_progress is None:
            return rejoin_progress
        return min(rejoin_progress, target_progress)

    def _rejoin_lookahead(self, line_error):
        if not self.reroute_plan_optimized:
            return self.reroute_return_lookahead
        return min(
            self.reroute_return_lookahead,
            max(
                self.optimized_rejoin_min_lookahead,
                abs(line_error) * self.optimized_rejoin_lateral_gain,
            ),
        )

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

    def _sensor_clearance_threshold(self, sensor_name):
        if sensor_name == 'front':
            return self.obstacle_detect_distance + self.obstacle_safety_margin
        if sensor_name == 'back':
            return self.reroute_side_clearance + self.obstacle_safety_margin
        return self.reroute_side_clearance + self.obstacle_safety_margin

    def _any_sensor_inside_shell(self):
        for sensor_name, sensor_range in self.ir.items():
            if math.isfinite(sensor_range) and sensor_range < self._sensor_clearance_threshold(sensor_name):
                return True
        return False

    def _all_sensors_clear_for_motion(self):
        for sensor_name, sensor_range in self.ir.items():
            if math.isfinite(sensor_range) and sensor_range < self._sensor_clearance_threshold(sensor_name):
                return False
        return True

    def _all_sensors_raw_clear_for_motion(self):
        for sensor_name, sensor_range in self.ir_raw.items():
            if math.isfinite(sensor_range) and sensor_range < self._sensor_clearance_threshold(sensor_name):
                return False
        return True

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
        active_sensors = []
        primary_sensor = None
        primary_range = float('inf')
        for sensor_name, sensor_range in self.ir.items():
            if not math.isfinite(sensor_range):
                continue
            threshold = self._sensor_clearance_threshold(sensor_name)
            if sensor_range >= threshold:
                continue
            urgency = (threshold - sensor_range) / max(threshold, 1e-6)
            direction_x, direction_y = directions[sensor_name]
            repulse_x += direction_x * urgency
            repulse_y += direction_y * urgency
            active_sensors.append(sensor_name)
            if sensor_range < primary_range:
                primary_range = sensor_range
                primary_sensor = sensor_name
        return repulse_x, repulse_y, active_sensors, primary_sensor

    def _focused_escape_body_vector(self, sensor_name):
        if sensor_name == 'front_left':
            return 0.4, 1.0
        if sensor_name == 'front_right':
            return 0.4, -1.0
        if sensor_name == 'left':
            return 0.0, 1.0
        if sensor_name == 'right':
            return 0.0, -1.0
        if sensor_name == 'back':
            return -1.0, 0.0
        left_min = min(self.ir['front_left'], self.ir['left'])
        right_min = min(self.ir['front_right'], self.ir['right'])
        return (0.4, 1.0) if left_min < right_min else (0.4, -1.0)

    def _update_reactive_reroute_preview(self, body_vx, body_vy):
        world_vx, world_vy = self._body_to_world_vector(body_vx, body_vy)
        lookahead = 0.45
        mid_x = self.center_x + world_vx * lookahead
        mid_y = self.center_y + world_vy * lookahead
        final_x, final_y = self.waypoints[self.current_wp_idx][:2]
        self.reroute_waypoints = [
            (self.center_x, self.center_y),
            (mid_x, mid_y),
            (final_x, final_y),
        ]

    def _front_arc_clear(self):
        return min(self.ir['front'], self.ir['front_left'], self.ir['front_right']) > self.obstacle_detect_distance

    def _reroute_obstacle_side_distance(self):
        if self.reroute_direction > 0.0:
            return min(self.ir_raw['right'], self.ir_raw['front_right'])
        return min(self.ir_raw['left'], self.ir_raw['front_left'])

    def _reroute_distance_from_start(self):
        return math.hypot(
            self.center_x - self.reroute_start_x,
            self.center_y - self.reroute_start_y,
        )

    def _choose_rotation_search_direction(self):
        left_clearance = min(self.ir_raw['left'], self.ir_raw['front_left'])
        right_clearance = min(self.ir_raw['right'], self.ir_raw['front_right'])
        if left_clearance > right_clearance + self.reroute_side_clear_hysteresis:
            return 1.0
        if right_clearance > left_clearance + self.reroute_side_clear_hysteresis:
            return -1.0
        if abs(self.reroute_direction) > 1e-6:
            return self.reroute_direction
        return 1.0 if left_clearance >= right_clearance else -1.0

    def _reroute_wall_detected(self, envelope):
        if envelope is None:
            return False
        if self.reroute_rotation_attempts >= self.reroute_rotation_max_attempts:
            return False
        if self._reroute_distance_from_start() < self.reroute_wall_follow_distance:
            return False
        return (
            self._reroute_obstacle_side_distance() < self.reroute_side_clearance
            or not self._front_path_clear()
        )

    def _start_reroute_rotation_search(self, reason):
        self.reroute_step = 4
        self.reroute_rotation_direction = self._choose_rotation_search_direction()
        self.reroute_rotation_target_yaw = self._normalize(
            self.yaw + self.reroute_rotation_direction * self.reroute_rotation_search_angle
        )
        self.reroute_rotation_attempts += 1
        self.clear_count = 0
        self.reroute_side_clear_count = 0
        self.get_logger().warn(reason)

    def _replan_reroute_after_rotation_search(self, envelope):
        self.reroute_start_x = self.center_x
        self.reroute_start_y = self.center_y
        self.reroute_start_progress = self._path_progress(self.center_x, self.center_y)
        self.reroute_plan_optimized = False
        trigger_sensor = self._choose_trigger_sensor(self.obstacle_clear_distance)
        if trigger_sensor is None:
            trigger_sensor = self.reroute_trigger_sensor or self._choose_trigger_sensor() or 'front'
        _, obstacle_side, self.reroute_direction, self.reroute_target_offset = self._select_reroute_plan(
            envelope,
            trigger_sensor,
        )
        self.reroute_obstacle_side = obstacle_side
        self.reroute_rejoin_min_progress = self._geometry_based_rejoin_progress(envelope)
        self.reroute_required_pass_distance = max(
            self.reroute_rejoin_min_progress - self.reroute_start_progress,
            0.5 * self.robot_length + self.obstacle_safety_margin,
        )
        self.reroute_step = 1
        self.clear_count = 0
        self.reroute_side_clear_count = 0
        self._update_reroute_debug_points()
        self.get_logger().info('Rotation search found a clearer sector — replanning the reroute lane')

    def _update_reroute_side_clear_count(self):
        if self._reroute_obstacle_side_distance() >= self.reroute_side_clearance + self.reroute_side_clear_hysteresis:
            self.reroute_side_clear_count += 1
        else:
            self.reroute_side_clear_count = 0

    def _reroute_ready_to_rejoin(self, current_progress):
        self._update_reroute_side_clear_count()
        return (
            current_progress >= self.reroute_rejoin_min_progress
            and self.reroute_side_clear_count >= self.reroute_side_clear_cycles
        )

    def _drive_toward_point(self, target_x, target_y, max_speed, heading_target=None, apply_speed_zone=False):
        dx = target_x - self.center_x
        dy = target_y - self.center_y
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        body_x = dx * cos_y + dy * sin_y
        body_y = -dx * sin_y + dy * cos_y
        norm = math.hypot(body_x, body_y)
        if norm < 1e-6:
            self._publish_motion(wz=0.0, apply_speed_zone=apply_speed_zone)
            return

        vx = max_speed * body_x / norm
        vy = max_speed * body_y / norm
        wz = 0.0
        if heading_target is not None:
            heading_error = self._normalize(heading_target - self.yaw)
            wz = self.ang_kp * heading_error * 0.25
        self._publish_motion(vx=vx, vy=vy, wz=wz, apply_speed_zone=apply_speed_zone)

    def _drive_world_vector(self, world_vx, world_vy, heading_target=None, apply_speed_zone=False):
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        body_x = world_vx * cos_y + world_vy * sin_y
        body_y = -world_vx * sin_y + world_vy * cos_y
        wz = 0.0
        if heading_target is not None:
            heading_error = self._normalize(heading_target - self.yaw)
            wz = self.ang_kp * heading_error * 0.25
        self._publish_motion(vx=body_x, vy=body_y, wz=wz, apply_speed_zone=apply_speed_zone)

    def _enter_obstacle_halt(self):
        self.pre_obstacle_state = self.state
        self.state = State.OBSTACLE_HALT
        self.reroute_entry_time = self.get_clock().now()
        self.clear_count = 0
        self.raw_clear_count = 0
        self.reroute_trigger_sensor = self._choose_trigger_sensor()
        self._latch_obstacle_memory(self.reroute_trigger_sensor)
        self._publish_stop()
        if self.active_leg_profile and self.active_leg_profile.get('remove_after_detect'):
            self._schedule_obstacle_removal(
                self.active_leg_profile['name'],
                self.active_leg_profile['remove_after_detect'],
            )
        self.get_logger().warn('Obstacle ahead — entering halt state')

    def _enter_reroute(self):
        self.state = State.OBSTACLE_REROUTE
        self.reroute_step = 1
        self.reroute_plan_optimized = False
        self.reroute_rotation_attempts = 0
        self.reroute_rotation_direction = 0.0
        self.reroute_rotation_target_yaw = self.yaw
        self.reroute_start_x = self.center_x
        self.reroute_start_y = self.center_y
        self.reroute_start_progress = self._path_progress(self.center_x, self.center_y)
        self.reroute_side_clear_count = 0
        self.reroute_entry_time = self.get_clock().now()
        envelope = self._estimate_obstacle_envelope_path_frame()

        trigger_sensor = (
            self.obstacle_memory_trigger_sensor
            or self.reroute_trigger_sensor
            or self._choose_trigger_sensor()
            or 'front'
        )
        if self.obstacle_memory_bypass_side in ('left', 'right'):
            offsets = self._candidate_bypass_offsets(envelope)
            bypass_side = self.obstacle_memory_bypass_side
            obstacle_side = 'right' if bypass_side == 'left' else 'left'
            self.reroute_direction = 1.0 if bypass_side == 'left' else -1.0
            self.reroute_target_offset = offsets[bypass_side]
        else:
            bypass_side, obstacle_side, self.reroute_direction, self.reroute_target_offset = self._select_reroute_plan(
                envelope,
                trigger_sensor,
            )
        self._latch_obstacle_memory(trigger_sensor, bypass_side)
        self.reroute_obstacle_side = obstacle_side
        self.reroute_rejoin_min_progress = self._geometry_based_rejoin_progress(envelope)

        self.reroute_required_pass_distance = max(
            self.reroute_rejoin_min_progress - self.reroute_start_progress,
            0.5 * self.robot_length + self.obstacle_safety_margin,
        )
        self._update_reroute_debug_points()
        self._reset_obstacle_counters()
        if self.active_leg_profile and self.active_leg_profile.get('remove_after_reroute'):
            self._schedule_obstacle_removal(
                self.active_leg_profile['name'],
                self.active_leg_profile['remove_after_reroute'],
            )
        left_clearance = min(self.ir_raw['left'], self.ir_raw['front_left'])
        right_clearance = min(self.ir_raw['right'], self.ir_raw['front_right'])
        if max(left_clearance, right_clearance) < self.reroute_side_clearance:
            self._start_reroute_rotation_search(
                'Obstacle blocks both lateral lanes — rotating first to find a clearer bypass'
            )
            return
        self.get_logger().warn(
            'Obstacle remained blocked — shifting %s with footprint clearance' %
            ('left' if self.reroute_direction > 0.0 else 'right')
        )

    def _optimize_reroute_plan_from_current_pose(self, reason):
        current_progress = self._path_progress(self.center_x, self.center_y)
        line_error = self._point_to_line_distance(self.center_x, self.center_y)
        optimized_lookahead = self._rejoin_lookahead(line_error)
        desired_progress = current_progress + optimized_lookahead
        if self.path_line_end is not None:
            target_progress = self._path_progress(*self.path_line_end)
            self.reroute_rejoin_min_progress = min(desired_progress, target_progress)
        else:
            self.reroute_rejoin_min_progress = desired_progress
        self.reroute_required_pass_distance = max(
            0.0,
            self.reroute_rejoin_min_progress - self.reroute_start_progress,
        )
        self.reroute_step = 3
        self.reroute_plan_optimized = True
        self.reroute_target_offset = line_error
        self._update_reroute_debug_points()
        self.get_logger().info(reason)

    def _update_reroute_debug_points(self):
        current_progress = self._path_progress(self.center_x, self.center_y)
        target_progress = self._path_progress(*self.path_line_end) if self.path_line_end is not None else None
        shift_progress = max(current_progress, self.reroute_start_progress)
        pass_progress = max(shift_progress, self.reroute_start_progress + self.reroute_required_pass_distance)
        settle_progress = pass_progress + self.reroute_return_lookahead
        if target_progress is not None:
            shift_progress = min(shift_progress, target_progress)
            pass_progress = min(pass_progress, target_progress)
            settle_progress = min(settle_progress, target_progress)
        x1, y1 = self._point_on_path(shift_progress, self.reroute_target_offset)
        x2, y2 = self._point_on_path(pass_progress, self.reroute_target_offset)
        x3, y3 = self._point_on_path(settle_progress, 0.0)
        self.reroute_waypoints = [(x1, y1), (x2, y2), (x3, y3)]

    def _reroute_timed_out(self):
        if self.reroute_entry_time is None:
            return False
        elapsed = (self.get_clock().now() - self.reroute_entry_time).nanoseconds / 1e9
        return elapsed >= self.reroute_timeout

    def _halt_timed_out(self):
        if self.reroute_entry_time is None:
            return False
        elapsed = (self.get_clock().now() - self.reroute_entry_time).nanoseconds / 1e9
        return elapsed >= self.obstacle_halt_timeout

    def _rotation_command(self, yaw_error, speed_scale=1.0, max_speed=None, min_speed=0.0):
        limit = self.max_ang if max_speed is None else min(self.max_ang, max_speed)
        command = max(-limit, min(limit, self.ang_kp * speed_scale * yaw_error))
        if min_speed > 0.0 and abs(yaw_error) > 1e-6:
            magnitude = max(min_speed, abs(command))
            command = math.copysign(min(limit, magnitude), yaw_error)

        return command

    def _handle_reroute(self):
        if self._reroute_timed_out():
            self._publish_stop()
            self.state = State.COMPLETE
            self.get_logger().error('Reroute timed out — stopping mission for safety')
            return

        current_progress = self._path_progress(self.center_x, self.center_y)
        target_progress = self._path_progress(*self.path_line_end) if self.path_line_end is not None else None
        line_error = self._point_to_line_distance(self.center_x, self.center_y)
        offset_error = self.reroute_target_offset - line_error
        shift_tolerance = max(self.obstacle_return_tolerance, 0.04)
        rejoin_tolerance = max(self.obstacle_return_tolerance, self.reroute_return_tolerance)
        heading_wz = self.ang_kp * self._normalize(self.resume_heading_target - self.yaw) * 0.2
        current_envelope = self._estimate_obstacle_envelope_path_frame()

        if (
            self.reroute_step in (1, 2)
            and current_envelope is None
            and self._all_sensors_raw_clear_for_motion()
            and not self.reroute_plan_optimized
        ):
            self._optimize_reroute_plan_from_current_pose(
                'Obstacle vanished during reroute — shortening rejoin from current pose'
            )
            return

        if self.reroute_step in (1, 2) and self._reroute_wall_detected(current_envelope):
            self._start_reroute_rotation_search(
                'Obstacle keeps extending along the bypass lane — rotating to reacquire a clearer opening'
            )
            return

        if self.reroute_step <= 1:
            shift_progress = max(
                current_progress,
                self.reroute_start_progress,
            )
            if target_progress is not None:
                shift_progress = min(shift_progress, target_progress)
            shift_x, shift_y = self._point_on_path(shift_progress, self.reroute_target_offset)
            self.reroute_waypoints = [
                (shift_x, shift_y),
                self._point_on_path(max(shift_progress, self.reroute_rejoin_min_progress), self.reroute_target_offset),
                self._point_on_path(max(shift_progress, self.reroute_rejoin_min_progress) + self.reroute_return_lookahead, 0.0),
            ]
            self._drive_toward_point(
                shift_x,
                shift_y,
                self.reroute_strafe_speed,
                heading_target=self.resume_heading_target,
                apply_speed_zone=False,
            )
            if abs(offset_error) <= shift_tolerance:
                self.reroute_step = 2
                self.clear_count = 0
                self.get_logger().info('Reroute shift complete — holding offset lane past the obstacle')
            return

        if self.reroute_step == 2:
            envelope = current_envelope
            if envelope is not None:
                offsets = self._candidate_bypass_offsets(envelope)
                move_side = 'left' if self.reroute_direction > 0.0 else 'right'
                updated_offset = offsets[move_side]
                if self.reroute_direction > 0.0:
                    self.reroute_target_offset = max(self.reroute_target_offset, updated_offset)
                else:
                    self.reroute_target_offset = min(self.reroute_target_offset, updated_offset)
                self.reroute_rejoin_min_progress = max(
                    self.reroute_rejoin_min_progress,
                    self._geometry_based_rejoin_progress(envelope),
                )

            lane_progress = max(
                current_progress + self.reroute_return_lookahead,
                self.reroute_start_progress + self.reroute_return_lookahead,
            )
            if target_progress is not None:
                lane_progress = min(lane_progress, target_progress)
            lane_x, lane_y = self._point_on_path(lane_progress, self.reroute_target_offset)
            self.reroute_waypoints = [
                (lane_x, lane_y),
                self._point_on_path(max(lane_progress, self.reroute_rejoin_min_progress), self.reroute_target_offset),
                self._point_on_path(max(lane_progress, self.reroute_rejoin_min_progress) + self.reroute_return_lookahead, 0.0),
            ]
            lane_speed = self.reroute_creep_speed if not self._all_sensors_clear_for_motion() else min(self.obstacle_speed_slow, self.max_lin)
            self._drive_toward_point(
                lane_x,
                lane_y,
                lane_speed,
                heading_target=self.resume_heading_target,
                apply_speed_zone=False,
            )
            if envelope is None and self._all_sensors_raw_clear_for_motion() and current_progress >= self.reroute_rejoin_min_progress:
                self.reroute_step = 3
                self.clear_count = 0
                self.get_logger().info('Offset lane is clear — rejoining center path')
            return

        if self.reroute_step == 4:
            rotation_error = self._normalize(self.reroute_rotation_target_yaw - self.yaw)
            front_clear = min(
                self.ir_raw['front'],
                self.ir_raw['front_left'],
                self.ir_raw['front_right'],
            ) >= self.reroute_rotation_clear_distance
            if not front_clear and abs(rotation_error) > self.scan_ang_tol:
                self._publish_motion(
                    wz=self._rotation_command(
                        rotation_error,
                        speed_scale=self.scan_rotation_kp_scale,
                        min_speed=self.scan_rotation_min_speed,
                        max_speed=self.scan_rotation_max_speed,
                    ),
                    apply_speed_zone=False,
                )
                return

            self._replan_reroute_after_rotation_search(current_envelope)
            return

        rejoin_lookahead = self._rejoin_lookahead(line_error)
        rejoin_progress = max(current_progress + rejoin_lookahead, self.reroute_rejoin_min_progress)
        if target_progress is not None:
            rejoin_progress = min(rejoin_progress, target_progress)
        rejoin_x, rejoin_y = self._point_on_path(rejoin_progress, 0.0)
        self.reroute_waypoints = [
            self._point_on_path(current_progress, self.reroute_target_offset),
            (rejoin_x, rejoin_y),
            self._point_on_path(rejoin_progress + self.reroute_return_lookahead, 0.0),
        ]
        self._drive_toward_point(
            rejoin_x,
            rejoin_y,
            min(self.obstacle_speed_full, self.max_lin),
            heading_target=self.resume_heading_target,
            apply_speed_zone=False,
        )

        if not self._all_sensors_raw_clear_for_motion():
            self.reroute_step = 2
            self.clear_count = 0
            return

        if abs(line_error) <= rejoin_tolerance and self._all_sensors_clear_for_motion():
            self.clear_count += 1
        else:
            self.clear_count = 0

        if self.clear_count >= self.clear_threshold:
            self._reset_reroute_state()
            self.state = self.pre_obstacle_state or State.MOVE_FORWARD
            self.get_logger().info('Reroute complete — resuming waypoint tracking')

    # ================================================================
    # Main control loop
    # ================================================================
    def _control_loop(self):
        self._service_pending_obstacle_removals()
        self._maybe_spawn_active_leg_obstacle()

        # Emergency stop check
        if not self.enabled and self.state != State.IDLE:
            self._publish_stop()
            self.state = State.IDLE
            return

        # ---- IDLE ----
        if self.state == State.IDLE:
            self._publish_stop()
            if self.enabled and self.mission_loaded:
                if self.scripted_skid_scan_mode:
                    self._start_skid_scan_mission()
                elif self.straight_line_only:
                    self._start_straight_line_mission()
                else:
                    self.scan_waypoint_indices = set(range(len(self.waypoints)))
                    self.current_wp_idx = 0
                    if self.waypoints:
                        self.path_line_start = (self.center_x, self.center_y)
                        self.path_line_end = self.waypoints[0][:2]
                        self.state = State.ROTATE_TO_TARGET
                        self.get_logger().info('Mission started! Heading to waypoint 0')
            return

        # Get current target
        if self.current_wp_idx < len(self.waypoints):
            tx, ty, tyaw, tlift = self.waypoints[self.current_wp_idx]
        else:
            self.state = State.COMPLETE
            self._publish_stop()
            self.get_logger().info('ALL WAYPOINTS COMPLETE — mission done!')
            return

        dx = tx - self.center_x
        dy = ty - self.center_y
        dist = math.sqrt(dx**2 + dy**2)
        angle_to_target = self._front_facing_heading_to(tx, ty)
        rotate_target = self.resume_heading_target if self.use_resume_heading_for_target_rotate else angle_to_target
        angle_error = self._normalize(rotate_target - self.yaw)
        scan_yaw_error = self._normalize(self.scan_heading_target - self.yaw)
        resume_yaw_error = self._normalize(self.resume_heading_target - self.yaw)

        # ---- Check obstacles during movement states ----
        obstacle_sensitive_states = (
            State.MOVE_FORWARD,
            State.ROTATE_TO_TARGET,
            State.ROTATE_TO_SCAN,
            State.ROTATE_BACK_TO_PATH,
        )
        self._update_obstacle_memory()
        if self.enable_obstacle_avoidance and self.state in obstacle_sensitive_states:
            self._update_obstacle_counters()
            if self.front_blocked_count >= self.front_blocked_threshold:
                self._enter_obstacle_halt()
                return
        elif self.state not in (State.OBSTACLE_HALT, State.OBSTACLE_REROUTE):
            self._reset_obstacle_counters()

        # ---- OBSTACLE_HALT ----
        if self.state == State.OBSTACLE_HALT:
            self._publish_stop()
            self._update_obstacle_counters()
            self._update_obstacle_memory()
            if self._all_sensors_raw_clear_for_motion():
                self.raw_clear_count += 1
            else:
                self.raw_clear_count = 0

            if (
                self.active_leg_profile
                and self.active_leg_profile.get('remove_after_detect')
                and self.raw_clear_count >= 2
            ):
                self._reset_obstacle_counters()
                self.state = self.pre_obstacle_state or State.MOVE_FORWARD
                self.reroute_entry_time = None
                self.get_logger().info('Transient obstacle despawned during halt — resuming without reroute')
                return

            if self.clear_count >= self.clear_threshold:
                self._reset_obstacle_counters()
                self.state = self.pre_obstacle_state or State.MOVE_FORWARD
                self.reroute_entry_time = None
                self.get_logger().info('Obstacle cleared — resuming mission')
                return
            if self._halt_timed_out():
                self._enter_reroute()
            return

        # ---- OBSTACLE_REROUTE ----
        if self.state == State.OBSTACLE_REROUTE:
            self._handle_reroute()
            return

        # ---- ROTATE_TO_TARGET ----
        if self.state == State.ROTATE_TO_TARGET:
            if abs(angle_error) > self.ang_tol:
                self._publish_motion(wz=self._rotation_command(angle_error))
            else:
                # Unlock forced turn when rotation is complete
                self.forced_turn_direction = 0
                self.use_resume_heading_for_target_rotate = False
                self.state = State.MOVE_FORWARD
                self.path_line_start = (self.center_x, self.center_y)
                self.path_line_end = (tx, ty)
                self.get_logger().info(
                    f'Aligned to waypoint {self.current_wp_idx} — moving forward'
                )
            return

        # ---- MOVE_FORWARD ----
        if self.state == State.MOVE_FORWARD:
            if dist > self.pos_tol:
                target_speed = 0.3

                # THE CARVING ARC FIX:
                # If recovering from a dodge (>20 degrees off target), disable lateral
                # sliding (vy) so the robot drives forward and carves a smooth arc back.
                # If aligned (<20 degrees), allow mecanum sliding for perfect centering.
                if abs(angle_error) > math.radians(20.0):
                    vx = target_speed
                    vy = 0.0
                else:
                    vx = target_speed * math.cos(angle_error)
                    vy = target_speed * math.sin(angle_error)

                # Clamp the recovery steering speed so it doesn't snap violently
                wz = self.ang_kp * angle_error
                wz = max(-0.7, min(0.7, wz))

                # Route through safety pipeline (Repulsion + Parking Cameras)
                self._publish_motion(vx=vx, vy=vy, wz=wz, apply_speed_zone=False)
            else:
                if not self.enable_stop_scan:
                    self._publish_stop()
                    self._advance_from_waypoint()
                    return
                if not self._scan_enabled_for_waypoint(self.current_wp_idx):
                    self._publish_stop()
                    park_heading = self._parking_heading_for_waypoint(self.current_wp_idx)
                    if park_heading is not None:
                        self.resume_heading_target = park_heading
                        self.parking_mode = True
                        self.state = State.ROTATE_BACK_TO_PATH
                        self.get_logger().info('At return waypoint — rotating into original parking pose')
                    else:
                        self._advance_from_waypoint()
                    return
                self.active_scan_direction = self._scan_direction_for_waypoint(self.current_wp_idx)
                self.active_scan_dwell = self._scan_dwell_for_waypoint(self.current_wp_idx)
                if self.straight_line_only and self.current_wp_idx < 3:
                    self.resume_heading_target = self.mission_start_yaw
                else:
                    self.resume_heading_target = self._compute_resume_heading(tx, ty)
                if not self.straight_line_only:
                    # In waypoint mode, waypoint yaw is the explicit shelf-facing heading.
                    shelf_heading = tyaw
                    if self.robot_body_frame_flip_180:
                        shelf_heading += math.pi
                    self.scan_heading_target = self._normalize(shelf_heading)
                else:
                    self.scan_heading_target = self._normalize(
                        self.resume_heading_target +
                        self._scan_direction_sign() * self.scan_turn_rad
                    )
                self.state = State.ROTATE_TO_SCAN
                self.get_logger().info('At stop point — rotating to shelf-facing heading')
            return

        # ---- ROTATE_TO_SCAN ----
        if self.state == State.ROTATE_TO_SCAN:
            if abs(scan_yaw_error) > self.scan_ang_tol:
                self._publish_motion(
                    wz=self._rotation_command(
                        scan_yaw_error,
                        speed_scale=self.scan_rotation_kp_scale,
                        min_speed=self.scan_rotation_min_speed,
                        max_speed=self.scan_rotation_max_speed,
                    )
                )
            else:
                # Unlock forced turn when rotation is complete
                self.forced_turn_direction = 0
                self._publish_stop()
                desired_lift = self._lift_target_for_waypoint(self.current_wp_idx)
                if self.scan_use_lift and desired_lift > 0.0:
                    self._begin_lift_motion(desired_lift)
                    self.state = State.LIFTING_UP
                    self.get_logger().info(
                        f'Waypoint {self.current_wp_idx} reached — rotating done, lifting with slider={self.target_lift_pos:.2f}'
                    )
                else:
                    self.scan_start_time = None
                    self.state = State.SCANNING
                    self.get_logger().info('Waypoint reached — rotating done, scanning in place')
            return

        # ---- LIFTING_UP ----
        if self.state == State.LIFTING_UP:
            self._publish_stop()
            if self._step_lift_motion():
                self.state = State.SCANNING
                self.get_logger().info('Lift at height — scanning shelf')
            return

        # ---- SCANNING ----
        if self.state == State.SCANNING:
            self._publish_stop()
            if self.scan_start_time is None:
                self.scan_start_time = self.get_clock().now()
            elapsed = (self.get_clock().now() - self.scan_start_time).nanoseconds / 1e9
            if elapsed >= self.active_scan_dwell:
                self.scan_start_time = None
                if self.scan_use_lift and self.target_lift_pos > 0.0:
                    self.post_lift_state = State.ROTATE_BACK_TO_PATH
                    self.state = State.LIFTING_DOWN
                    self.get_logger().info('Scan complete — lowering lift before rotating back to path heading')
                else:
                    self.state = State.ROTATE_BACK_TO_PATH
                    self.get_logger().info('Scan complete — rotating back to path heading')
            return

        # ---- LIFTING_DOWN ----
        if self.state == State.LIFTING_DOWN:
            self._publish_stop()
            if self.lift_motion_start_time is None and self.target_lift_pos != 0.0:
                self._begin_lift_motion(0.0)
            if self._step_lift_motion():
                self.target_lift_pos = 0.0
                if self.post_lift_state is not None:
                    self.state = self.post_lift_state
                    self.post_lift_state = None
                    self.get_logger().info('Lift lowered — rotating back to travel heading')
                else:
                    self._advance_from_waypoint()
            return

        # ---- ROTATE_BACK_TO_PATH ----
        if self.state == State.ROTATE_BACK_TO_PATH:
            if abs(resume_yaw_error) > self.scan_ang_tol:
                self._publish_motion(
                    wz=self._rotation_command(
                        resume_yaw_error,
                        speed_scale=self.scan_rotation_kp_scale,
                        min_speed=self.scan_rotation_min_speed,
                        max_speed=self.scan_rotation_max_speed,
                    )
                )
            else:
                # Unlock forced turn when rotation is complete
                self.forced_turn_direction = 0
                self._publish_stop()
                if self.parking_mode:
                    self.parking_mode = False
                    self.state = State.COMPLETE
                    self.get_logger().info('Returned to start and parked in original pose')
                    return
                if self.scan_use_lift and self.target_lift_pos > 0.0:
                    self.scan_start_time = None
                    self.state = State.LIFTING_DOWN
                    self.get_logger().info('Resumed heading — lowering lift')
                else:
                    self._advance_from_waypoint()
            return

        # ---- COMPLETE ----
        if self.state == State.COMPLETE:
            self._publish_stop()


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
