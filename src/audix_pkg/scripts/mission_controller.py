#!/usr/bin/env python3
"""
Unified mission controller for Audix warehouse robot.

State machine:
    IDLE -> ROTATE_TO_TARGET -> MOVE_FORWARD -> ROTATE_TO_SCAN
    -> LIFTING_UP -> SCANNING -> LIFTING_DOWN -> ROTATE_BACK_TO_PATH
    -> (next waypoint or COMPLETE)

Interrupt states: OBSTACLE_STOP (dynamic), OBSTACLE_REROUTE (static)

Subscribes to 6 individual IR sensor topics, IMU, filtered odom, enable flag.
Publishes /cmd_vel (Twist) and scissor lift position commands.
Receives mission goals via /send_mission service.
"""

import math
from collections import deque
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float64MultiArray, String
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
    OBSTACLE_STOP = auto()
    OBSTACLE_REROUTE = auto()
    COMPLETE = auto()


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # --- Declare ALL parameters ---
        self.declare_parameter('waypoints', [0.0])
        self.declare_parameter('safe_distance', 0.25)
        self.declare_parameter('side_safe_distance', 0.15)
        self.declare_parameter('static_persistence_cycles', 12)
        self.declare_parameter('reroute_lateral_offset', 0.35)
        self.declare_parameter('dynamic_clear_cycles', 5)
        self.declare_parameter('obstacle_sensor_warmup_sec', 2.0)
        self.declare_parameter('obstacle_stop_timeout_sec', 4.0)
        self.declare_parameter('lift_dwell_time', 3.0)
        self.declare_parameter('lift_position_tolerance', 0.003)
        self.declare_parameter('linear_kp', 0.5)
        self.declare_parameter('angular_kp', 1.5)
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

        # Load parameters
        self.safe_dist = self.get_parameter('safe_distance').value
        self.side_safe_dist = self.get_parameter('side_safe_distance').value
        self.static_persist = self.get_parameter('static_persistence_cycles').value
        self.reroute_offset = self.get_parameter('reroute_lateral_offset').value
        self.clear_cycles = self.get_parameter('dynamic_clear_cycles').value
        self.obstacle_warmup = self.get_parameter('obstacle_sensor_warmup_sec').value
        self.obstacle_stop_timeout = self.get_parameter('obstacle_stop_timeout_sec').value
        self.lift_dwell = self.get_parameter('lift_dwell_time').value
        self.lift_tol = self.get_parameter('lift_position_tolerance').value
        self.lin_kp = self.get_parameter('linear_kp').value
        self.ang_kp = self.get_parameter('angular_kp').value
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

        # --- State ---
        self.state = State.IDLE
        self.enabled = False
        self.mission_loaded = False
        self.current_wp_idx = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # IR sensor readings
        self.ir = {
            'front': float('inf'),
            'front_left': float('inf'),
            'front_right': float('inf'),
            'left': float('inf'),
            'right': float('inf'),
            'back': float('inf'),
        }

        # Obstacle tracking
        self.front_obstacle_history = deque(maxlen=30)
        self.consecutive_clear = 0
        self.pre_reroute_state = None
        self.reroute_waypoints = []
        self.reroute_wp_idx = 0
        self.obstacle_stop_start_time = None
        self.node_start_time = self.get_clock().now()

        # Lift state
        self.current_lift_pos = 0.0
        self.target_lift_pos = 0.0
        self.scan_start_time = None
        self.scan_heading_target = 0.0
        self.resume_heading_target = 0.0
        self.scan_waypoint_indices = set()

        # Debug visualization state
        self.robot_path_points = []
        self.last_path_point = None

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(
            Float64MultiArray,
            '/scissor_position_controller/commands',
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
        self.ir[key] = float(np.min(valid)) if len(valid) > 0 else float('inf')

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = _euler_from_quat(q.x, q.y, q.z, q.w)
        self.yaw = yaw

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

    # ================================================================
    # Helpers
    # ================================================================
    @staticmethod
    def _normalize(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _publish_stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _publish_cmd(self, vx=0.0, vy=0.0, wz=0.0):
        cmd = Twist()
        cmd.linear.x = max(-self.max_lin, min(self.max_lin, vx))
        cmd.linear.y = max(-self.max_lin, min(self.max_lin, vy))
        cmd.angular.z = max(-self.max_ang, min(self.max_ang, wz))
        self.cmd_pub.publish(cmd)

    def _publish_lift(self, carriage_pos):
        """Publish all scissor joint positions for a given carriage displacement."""
        # Scissor kinematics: given carriage_x, compute link angles
        L = 0.120  # link length in meters
        theta_min = math.radians(10)
        cos_min = L * math.cos(theta_min)

        clamped = max(0.0, min(0.0772, carriage_pos))
        cos_theta = (cos_min - clamped) / L
        cos_theta = max(-1.0, min(1.0, cos_theta))
        theta = math.acos(cos_theta)

        # Joint angles (simplified — all links rotate by same angle offset from initial)
        angle = theta - theta_min

        # Order must match controllers.yaml:
        # bottom, top, L1, R1, L2, R2, L3, R3, L4, R4, L5, R5, L6, R6, camera
        msg = Float64MultiArray()
        msg.data = [
            clamped,     # bottom_stud_joint
            0.0,         # top_stud_joint
            angle,       # left_joint1
            angle,       # right_joint1
            angle,       # left_joint2
            angle,       # right_joint2
            -angle,      # left_joint3
            -angle,      # right_joint3
            angle,       # left_joint4
            angle,       # right_joint4
            -angle,      # left_joint5
            -angle,      # right_joint5
            -angle,      # left_joint6
            -angle,      # right_joint6
            -angle,      # camera_joint
        ]
        self.lift_pub.publish(msg)

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

    def _append_robot_path(self):
        if self.last_path_point is None:
            self.last_path_point = (self.x, self.y)
            self.robot_path_points.append(self._make_pose(self.x, self.y, self.yaw))
            return
        dx = self.x - self.last_path_point[0]
        dy = self.y - self.last_path_point[1]
        if math.hypot(dx, dy) < self.debug_path_spacing:
            return
        self.last_path_point = (self.x, self.y)
        self.robot_path_points.append(self._make_pose(self.x, self.y, self.yaw))
        if len(self.robot_path_points) > self.debug_path_max_points:
            self.robot_path_points = self.robot_path_points[-self.debug_path_max_points:]

    def _build_planned_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'

        poses = [self._make_pose(self.x, self.y, self.yaw)]
        if self.state == State.OBSTACLE_REROUTE and self.reroute_wp_idx < len(self.reroute_waypoints):
            for i in range(self.reroute_wp_idx, len(self.reroute_waypoints)):
                wx, wy = self.reroute_waypoints[i]
                poses.append(self._make_pose(wx, wy, 0.0))

        for i in range(self.current_wp_idx, len(self.waypoints)):
            wx, wy, wyaw, _ = self.waypoints[i]
            poses.append(self._make_pose(wx, wy, wyaw))

        path.poses = poses
        return path

    def _build_target_markers(self):
        now = self.get_clock().now().to_msg()
        markers = []

        clear = Marker()
        clear.header.frame_id = 'odom'
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.append(clear)

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
            for i in range(self.reroute_wp_idx, len(self.reroute_waypoints)):
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
        state_marker.pose.position.x = self.x
        state_marker.pose.position.y = self.y
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
            f'front_right={self.ir["front_right"]:.3f}, x={self.x:.3f}, y={self.y:.3f}'
        )
        self.state_pub.publish(s)

    def _height_to_carriage(self, height_m):
        """Convert desired platform height to carriage X displacement."""
        L = 0.120
        n_stages = 3
        theta_min = math.radians(10)
        # H = n * L * sin(theta) => theta = asin(H / (n*L))
        h = max(0.0625, min(0.3383, height_m))
        theta = math.asin(h / (n_stages * L))
        carriage_x = L * (math.cos(theta_min) - math.cos(theta))
        return max(0.0, min(0.0772, carriage_x))

    def _scan_direction_sign(self):
        return -1.0 if self.scan_turn_direction == 'right' else 1.0

    def _compute_resume_heading(self, tx, ty):
        if self.current_wp_idx + 1 < len(self.waypoints):
            nx, ny, _, _ = self.waypoints[self.current_wp_idx + 1]
            return math.atan2(ny - ty, nx - tx)
        if self.current_wp_idx > 0:
            px, py, _, _ = self.waypoints[self.current_wp_idx - 1]
            return math.atan2(ty - py, tx - px)
        return self.yaw

    def _status_log_cb(self):
        self.get_logger().info(
            'state=%s wp=%d pos=(%.2f, %.2f) yaw=%.2f | '
            'ir[f=%.2f fl=%.2f fr=%.2f l=%.2f r=%.2f b=%.2f]' % (
                self.state.name,
                self.current_wp_idx,
                self.x,
                self.y,
                self.yaw,
                self.ir['front'],
                self.ir['front_left'],
                self.ir['front_right'],
                self.ir['left'],
                self.ir['right'],
                self.ir['back'],
            )
        )

    def _front_obstacle_detected(self):
        # Ignore early sensor transients right after startup/spawn.
        elapsed = (self.get_clock().now() - self.node_start_time).nanoseconds / 1e9
        if elapsed < self.obstacle_warmup:
            return False

        front_hit = self.ir['front'] < self.safe_dist
        side_pair_hit = (
            self.ir['front_left'] < self.safe_dist and
            self.ir['front_right'] < self.safe_dist
        )
        return front_hit or side_pair_hit

    def _enter_reroute(self):
        self.get_logger().warn('Static obstacle detected — REROUTING')
        offset = self.reroute_offset
        perp_angle = self.yaw + math.pi / 2  # left of heading
        bx1 = self.x + offset * math.cos(perp_angle)
        by1 = self.y + offset * math.sin(perp_angle)
        bx2 = bx1 + 0.4 * math.cos(self.yaw)  # advance past obstacle
        by2 = by1 + 0.4 * math.sin(self.yaw)
        bx3 = bx2 - offset * math.cos(perp_angle)  # return to original line
        by3 = by2 - offset * math.sin(perp_angle)
        self.reroute_waypoints = [(bx1, by1), (bx2, by2), (bx3, by3)]
        self.reroute_wp_idx = 0
        self.state = State.OBSTACLE_REROUTE
        self.front_obstacle_history.clear()
        self.obstacle_stop_start_time = None

    def _classify_obstacle(self):
        """Returns 'dynamic', 'static', or 'none'."""
        detected = self._front_obstacle_detected()
        self.front_obstacle_history.append(detected)

        if not detected:
            return 'none'

        recent = list(self.front_obstacle_history)

        # Static: obstacle present in 80% of last static_persist cycles
        if len(recent) >= self.static_persist:
            persistence = sum(recent[-self.static_persist:])
            if persistence >= self.static_persist * 0.8:
                return 'static'

        # Dynamic: require at least 3 consecutive detections to avoid
        # triggering on sensor init artifacts or single-sample noise
        min_confirm = 3
        if len(recent) < min_confirm:
            return 'none'
        if sum(recent[-min_confirm:]) >= min_confirm:
            return 'dynamic'

        return 'none'

    # ================================================================
    # Main control loop
    # ================================================================
    def _control_loop(self):
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
                    base_yaw = self.yaw
                    self.resume_heading_target = base_yaw
                    fx = math.cos(base_yaw)
                    fy = math.sin(base_yaw)
                    rx = math.cos(base_yaw - math.pi / 2.0)
                    ry = math.sin(base_yaw - math.pi / 2.0)

                    s = self.skid_lateral_distance
                    x0, y0 = self.x, self.y
                    p1 = (x0 + s * rx, y0 + s * ry)
                    p2 = (p1[0] - 2.0 * s * rx, p1[1] - 2.0 * s * ry)
                    p3 = (p2[0] + s * rx, p2[1] + s * ry)
                    p4 = (p3[0] + self.end_goal_distance * fx,
                          p3[1] + self.end_goal_distance * fy)

                    self.waypoints = [
                        (p1[0], p1[1], base_yaw, 0.0),
                        (p2[0], p2[1], base_yaw, 0.0),
                        (p3[0], p3[1], base_yaw, 0.0),
                        (p4[0], p4[1], base_yaw, 0.0),
                    ]
                    self.scan_waypoint_indices = {0, 1}
                    self.current_wp_idx = 0
                    self.state = State.MOVE_FORWARD
                    self.get_logger().info(
                        'Mission started! Skid sequence generated: RIGHT -> LEFT -> CENTER -> GOAL'
                    )
                elif self.straight_line_only:
                    tx = self.x + self.straight_line_distance * math.cos(self.yaw)
                    ty = self.y + self.straight_line_distance * math.sin(self.yaw)
                    self.waypoints = [(tx, ty, self.yaw, 0.0)]
                    self.scan_waypoint_indices = set()
                    self.current_wp_idx = 0
                    self.state = State.MOVE_FORWARD
                    self.get_logger().info(
                        'Mission started! Straight-line debug target set to '
                        f'({tx:.2f}, {ty:.2f})'
                    )
                else:
                    self.scan_waypoint_indices = set(range(len(self.waypoints)))
                    self.current_wp_idx = 0
                    self.state = State.ROTATE_TO_TARGET
                    self.get_logger().info('Mission started! Heading to waypoint 0')
            return

        # Get current target
        if self.state in (State.OBSTACLE_REROUTE,):
            if self.reroute_wp_idx < len(self.reroute_waypoints):
                tx, ty = self.reroute_waypoints[self.reroute_wp_idx]
                tyaw = math.atan2(ty - self.y, tx - self.x)
                tlift = 0.0
            else:
                self.state = State.ROTATE_TO_TARGET
                self.front_obstacle_history.clear()
                return
        elif self.current_wp_idx < len(self.waypoints):
            tx, ty, tyaw, tlift = self.waypoints[self.current_wp_idx]
        else:
            self.state = State.COMPLETE
            self._publish_stop()
            self.get_logger().info('ALL WAYPOINTS COMPLETE — mission done!')
            return

        dx = tx - self.x
        dy = ty - self.y
        dist = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx)
        angle_error = self._normalize(angle_to_target - self.yaw)
        scan_yaw_error = self._normalize(self.scan_heading_target - self.yaw)
        resume_yaw_error = self._normalize(self.resume_heading_target - self.yaw)

        # ---- Check obstacles during movement states ----
        if self.enable_obstacle_avoidance and self.state in (State.MOVE_FORWARD,):
            obs_type = self._classify_obstacle()
            if obs_type == 'dynamic':
                self.pre_reroute_state = self.state
                self.state = State.OBSTACLE_STOP
                self._publish_stop()
                self.consecutive_clear = 0
                self.obstacle_stop_start_time = self.get_clock().now()
                self.get_logger().warn('Dynamic obstacle detected — STOPPING')
                return
            elif obs_type == 'static':
                self._enter_reroute()
                return

        # ---- OBSTACLE_STOP (dynamic) ----
        if self.state == State.OBSTACLE_STOP:
            self._publish_stop()
            if not self._front_obstacle_detected():
                self.consecutive_clear += 1
            else:
                self.consecutive_clear = 0

            if self.obstacle_stop_start_time is not None:
                stop_elapsed = (
                    self.get_clock().now() - self.obstacle_stop_start_time
                ).nanoseconds / 1e9
                if stop_elapsed >= self.obstacle_stop_timeout:
                    self.get_logger().warn(
                        'Obstacle stop timeout reached — escalating to reroute'
                    )
                    self._enter_reroute()
                    return

            if self.consecutive_clear >= self.clear_cycles:
                self.state = self.pre_reroute_state or State.MOVE_FORWARD
                self.front_obstacle_history.clear()
                self.obstacle_stop_start_time = None
                self.get_logger().info('Dynamic obstacle cleared — resuming')
            return

        # ---- OBSTACLE_REROUTE ----
        if self.state == State.OBSTACLE_REROUTE:
            if self.reroute_wp_idx >= len(self.reroute_waypoints):
                self.state = State.ROTATE_TO_TARGET
                self.get_logger().info('Reroute complete — resuming original path')
                return
            rx, ry = self.reroute_waypoints[self.reroute_wp_idx]
            rdx = rx - self.x
            rdy = ry - self.y
            rdist = math.sqrt(rdx**2 + rdy**2)
            if rdist < self.pos_tol:
                self.reroute_wp_idx += 1
                return
            # Use mecanum strafing for reroute
            # Transform target vector into robot body frame
            cos_y = math.cos(self.yaw)
            sin_y = math.sin(self.yaw)
            body_x = rdx * cos_y + rdy * sin_y
            body_y = -rdx * sin_y + rdy * cos_y
            speed = min(0.15, 0.5 * rdist)
            norm = math.sqrt(body_x**2 + body_y**2)
            if norm > 0.001:
                self._publish_cmd(
                    vx=speed * body_x / norm,
                    vy=speed * body_y / norm,
                )
            return

        # ---- ROTATE_TO_TARGET ----
        if self.state == State.ROTATE_TO_TARGET:
            if abs(angle_error) > self.ang_tol:
                self._publish_cmd(wz=self.ang_kp * angle_error)
            else:
                self.state = State.MOVE_FORWARD
                self.get_logger().info(
                    f'Aligned to waypoint {self.current_wp_idx} — moving forward'
                )
            return

        # ---- MOVE_FORWARD ----
        if self.state == State.MOVE_FORWARD:
            if dist > self.pos_tol:
                # Holonomic tracking: move toward waypoint in robot frame.
                cos_y = math.cos(self.yaw)
                sin_y = math.sin(self.yaw)
                body_x = dx * cos_y + dy * sin_y
                body_y = -dx * sin_y + dy * cos_y
                norm = math.hypot(body_x, body_y)
                speed = min(self.lin_kp * dist, self.max_lin)
                if norm > 1e-6:
                    vx = speed * body_x / norm
                    vy = speed * body_y / norm
                else:
                    vx = 0.0
                    vy = 0.0
                hold_heading_error = self._normalize(self.resume_heading_target - self.yaw)
                self._publish_cmd(
                    vx=vx,
                    vy=vy,
                    wz=self.ang_kp * hold_heading_error * 0.25,
                )
            else:
                if not self.enable_stop_scan:
                    self._publish_stop()
                    self.current_wp_idx += 1
                    if self.current_wp_idx < len(self.waypoints):
                        self.state = State.MOVE_FORWARD
                    else:
                        self.state = State.COMPLETE
                        self.get_logger().info('Straight-line run complete')
                    return
                if self.current_wp_idx not in self.scan_waypoint_indices:
                    self._publish_stop()
                    self.current_wp_idx += 1
                    if self.current_wp_idx < len(self.waypoints):
                        self.state = State.MOVE_FORWARD
                    else:
                        self.state = State.COMPLETE
                        self.get_logger().info('Skid sequence complete')
                    return
                self.resume_heading_target = self._compute_resume_heading(tx, ty)
                self.scan_heading_target = self._normalize(
                    self.resume_heading_target +
                    self._scan_direction_sign() * self.scan_turn_rad
                )
                self.state = State.ROTATE_TO_SCAN
                self.get_logger().info('At stop point — rotating 90deg toward shelf')
            return

        # ---- ROTATE_TO_SCAN ----
        if self.state == State.ROTATE_TO_SCAN:
            if abs(scan_yaw_error) > self.ang_tol:
                self._publish_cmd(wz=self.ang_kp * scan_yaw_error)
            else:
                self._publish_stop()
                if self.scan_use_lift and tlift > 0.0:
                    self.target_lift_pos = self._height_to_carriage(tlift)
                    self.state = State.LIFTING_UP
                    self.get_logger().info(
                        f'Waypoint {self.current_wp_idx} reached — rotating done, lifting to {tlift:.3f}m'
                    )
                else:
                    self.scan_start_time = None
                    self.state = State.SCANNING
                    self.get_logger().info('Waypoint reached — rotating done, scanning in place')
            return

        # ---- LIFTING_UP ----
        if self.state == State.LIFTING_UP:
            self._publish_stop()
            self._publish_lift(self.target_lift_pos)
            # For now, wait a fixed time for the lift to reach position
            if self.scan_start_time is None:
                self.scan_start_time = self.get_clock().now()
            elapsed = (self.get_clock().now() - self.scan_start_time).nanoseconds / 1e9
            if elapsed > 2.0:  # give 2s for lift to reach
                self.scan_start_time = None
                self.state = State.SCANNING
                self.get_logger().info('Lift at height — scanning shelf')
            return

        # ---- SCANNING ----
        if self.state == State.SCANNING:
            self._publish_stop()
            if self.scan_start_time is None:
                self.scan_start_time = self.get_clock().now()
            elapsed = (self.get_clock().now() - self.scan_start_time).nanoseconds / 1e9
            if elapsed >= self.lift_dwell:
                self.scan_start_time = None
                if self.scan_use_lift and self.target_lift_pos > 0.0:
                    self.state = State.LIFTING_DOWN
                    self.get_logger().info('Scan complete — retracting lift')
                else:
                    self.state = State.ROTATE_BACK_TO_PATH
                    self.get_logger().info('Scan complete — rotating back to path heading')
            return

        # ---- LIFTING_DOWN ----
        if self.state == State.LIFTING_DOWN:
            self._publish_stop()
            self._publish_lift(0.0)  # fully retracted
            if self.scan_start_time is None:
                self.scan_start_time = self.get_clock().now()
            elapsed = (self.get_clock().now() - self.scan_start_time).nanoseconds / 1e9
            if elapsed > 2.0:
                self.scan_start_time = None
                self.state = State.ROTATE_BACK_TO_PATH
                self.get_logger().info('Lift retracted — rotating back to path heading')
            return

        # ---- ROTATE_BACK_TO_PATH ----
        if self.state == State.ROTATE_BACK_TO_PATH:
            if abs(resume_yaw_error) > self.ang_tol:
                self._publish_cmd(wz=self.ang_kp * resume_yaw_error)
            else:
                self._publish_stop()
                self.current_wp_idx += 1
                if self.current_wp_idx < len(self.waypoints):
                    self.state = State.MOVE_FORWARD
                    self.get_logger().info(
                        f'Resumed heading — moving to waypoint {self.current_wp_idx}'
                    )
                else:
                    self.state = State.COMPLETE
                    self.get_logger().info('ALL WAYPOINTS COMPLETE!')
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
