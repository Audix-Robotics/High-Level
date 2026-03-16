#!/usr/bin/env python3
"""
Unified mission controller for Audix warehouse robot.

State machine:
  IDLE -> ROTATE_TO_TARGET -> MOVE_FORWARD -> ROTATE_TO_FINAL
  -> LIFTING_UP -> SCANNING -> LIFTING_DOWN -> (next waypoint or COMPLETE)

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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger
import tf_transformations


class State(Enum):
    IDLE = auto()
    ROTATE_TO_TARGET = auto()
    MOVE_FORWARD = auto()
    ROTATE_TO_FINAL = auto()
    LIFTING_UP = auto()
    SCANNING = auto()
    LIFTING_DOWN = auto()
    OBSTACLE_STOP = auto()
    OBSTACLE_REROUTE = auto()
    COMPLETE = auto()


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        # --- Declare ALL parameters ---
        self.declare_parameter('waypoints', [])
        self.declare_parameter('safe_distance', 0.25)
        self.declare_parameter('side_safe_distance', 0.15)
        self.declare_parameter('static_persistence_cycles', 12)
        self.declare_parameter('reroute_lateral_offset', 0.35)
        self.declare_parameter('dynamic_clear_cycles', 5)
        self.declare_parameter('lift_dwell_time', 3.0)
        self.declare_parameter('lift_position_tolerance', 0.003)
        self.declare_parameter('linear_kp', 0.5)
        self.declare_parameter('angular_kp', 1.5)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('angle_tolerance', 0.05)

        # Load parameters
        self.safe_dist = self.get_parameter('safe_distance').value
        self.side_safe_dist = self.get_parameter('side_safe_distance').value
        self.static_persist = self.get_parameter('static_persistence_cycles').value
        self.reroute_offset = self.get_parameter('reroute_lateral_offset').value
        self.clear_cycles = self.get_parameter('dynamic_clear_cycles').value
        self.lift_dwell = self.get_parameter('lift_dwell_time').value
        self.lift_tol = self.get_parameter('lift_position_tolerance').value
        self.lin_kp = self.get_parameter('linear_kp').value
        self.ang_kp = self.get_parameter('angular_kp').value
        self.max_lin = self.get_parameter('max_linear_speed').value
        self.max_ang = self.get_parameter('max_angular_speed').value
        self.pos_tol = self.get_parameter('position_tolerance').value
        self.ang_tol = self.get_parameter('angle_tolerance').value

        # Parse waypoints from YAML: flat list [x,y,yaw,h, x,y,yaw,h, ...]
        wp_flat = self.get_parameter('waypoints').value
        self.waypoints = []
        if wp_flat:
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

        # Lift state
        self.current_lift_pos = 0.0
        self.target_lift_pos = 0.0
        self.scan_start_time = None

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(
            Float64MultiArray,
            '/scissor_position_controller/commands',
            10
        )

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

        self.get_logger().info(
            f'Mission controller ready. {len(self.waypoints)} waypoints loaded.'
        )

    # ================================================================
    # Callbacks
    # ================================================================
    def _ir_cb(self, key: str, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=float('inf'), posinf=float('inf'))
        if len(ranges) > 0:
            self.ir[key] = float(np.min(ranges))
        else:
            self.ir[key] = float('inf')

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
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
        if self.waypoints:
            self.mission_loaded = True
            response.success = True
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

        # Order matches scissor_position_controller joints config:
        # bottom_stud, R1, R2, R3, R4, R5, R6, L1, L2, L3, L4, L5, L6, cam, top_stud
        msg = Float64MultiArray()
        msg.data = [
            clamped,     # bottom_stud_joint
            angle,       # right_joint1
            angle,       # right_joint2
            -angle,      # right_joint3
            -angle,      # right_joint4
            angle,       # right_joint5
            angle,       # right_joint6
            angle,       # left_joint1
            angle,       # left_joint2
            -angle,      # left_joint3
            -angle,      # left_joint4
            angle,       # left_joint5
            angle,       # left_joint6
            0.0,         # camera_joint (keep level)
            0.0,         # top_stud_joint
        ]
        self.lift_pub.publish(msg)

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

    def _front_obstacle_detected(self):
        return (self.ir['front'] < self.safe_dist or
                self.ir['front_left'] < self.safe_dist or
                self.ir['front_right'] < self.safe_dist)

    def _classify_obstacle(self):
        """Returns 'dynamic', 'static', or 'none'."""
        detected = self._front_obstacle_detected()
        self.front_obstacle_history.append(detected)

        if not detected:
            return 'none'

        # Count how many of the last N cycles had an obstacle
        recent = list(self.front_obstacle_history)
        if len(recent) >= self.static_persist:
            persistence = sum(recent[-self.static_persist:])
            if persistence >= self.static_persist * 0.8:
                return 'static'
        return 'dynamic'

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
        final_yaw_error = self._normalize(tyaw - self.yaw)

        # ---- Check obstacles during movement states ----
        if self.state in (State.MOVE_FORWARD, State.ROTATE_TO_TARGET):
            obs_type = self._classify_obstacle()
            if obs_type == 'dynamic':
                self.pre_reroute_state = self.state
                self.state = State.OBSTACLE_STOP
                self._publish_stop()
                self.consecutive_clear = 0
                self.get_logger().warn('Dynamic obstacle detected — STOPPING')
                return
            elif obs_type == 'static':
                self.get_logger().warn('Static obstacle detected — REROUTING')
                # Compute bypass waypoints: strafe left, advance, strafe back
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
                return

        # ---- OBSTACLE_STOP (dynamic) ----
        if self.state == State.OBSTACLE_STOP:
            self._publish_stop()
            if not self._front_obstacle_detected():
                self.consecutive_clear += 1
            else:
                self.consecutive_clear = 0
            if self.consecutive_clear >= self.clear_cycles:
                self.state = self.pre_reroute_state or State.MOVE_FORWARD
                self.front_obstacle_history.clear()
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
                self._publish_cmd(wz=-self.ang_kp * angle_error)
            else:
                self.state = State.MOVE_FORWARD
                self.get_logger().info(
                    f'Aligned to waypoint {self.current_wp_idx} — moving forward'
                )
            return

        # ---- MOVE_FORWARD ----
        if self.state == State.MOVE_FORWARD:
            if dist > self.pos_tol:
                speed = min(self.lin_kp * dist, self.max_lin)
                # Small angular correction while moving
                self._publish_cmd(
                    vx=speed,
                    wz=-self.ang_kp * angle_error * 0.3,
                )
            else:
                self.state = State.ROTATE_TO_FINAL
                self.get_logger().info('At position — rotating to final orientation')
            return

        # ---- ROTATE_TO_FINAL ----
        if self.state == State.ROTATE_TO_FINAL:
            if abs(final_yaw_error) > self.ang_tol:
                self._publish_cmd(wz=-self.ang_kp * final_yaw_error)
            else:
                self._publish_stop()
                self.target_lift_pos = self._height_to_carriage(tlift)
                self.state = State.LIFTING_UP
                self.get_logger().info(
                    f'Waypoint {self.current_wp_idx} reached — lifting to {tlift:.3f}m'
                )
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
                self.state = State.LIFTING_DOWN
                self.get_logger().info('Scan complete — retracting lift')
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
                self.current_wp_idx += 1
                if self.current_wp_idx < len(self.waypoints):
                    self.state = State.ROTATE_TO_TARGET
                    self.get_logger().info(
                        f'Moving to waypoint {self.current_wp_idx}'
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
