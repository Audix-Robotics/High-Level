#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64, String
from visualization_msgs.msg import Marker, MarkerArray


LEFT = 1
RIGHT = -1


def clamp(value, low, high):
    return max(low, min(high, value))


def opposite_direction(direction):
    return LEFT if direction == RIGHT else RIGHT


def quat_to_yaw(x_val, y_val, z_val, w_val):
    siny_cosp = 2.0 * (w_val * z_val + x_val * y_val)
    cosy_cosp = 1.0 - 2.0 * (y_val * y_val + z_val * z_val)
    return math.atan2(siny_cosp, cosy_cosp)


class SimpleCardinalBrain(Node):
    def __init__(self):
        super().__init__('simple_cardinal_brain')

        self.declare_parameter('odom_topic', '/mecanum_odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('debug_frame_id', 'arena10')
        self.declare_parameter('debug_origin_x', 0.0)
        self.declare_parameter('debug_origin_y', 0.0)
        self.declare_parameter('goal_distance', 3.55)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('forward_speed', 0.08)
        self.declare_parameter('lateral_speed', 0.10)
        self.declare_parameter('backoff_speed', 0.06)
        self.declare_parameter('line_kp', 1.6)
        self.declare_parameter('max_line_correction_speed', 0.05)
        self.declare_parameter('backoff_distance', 0.05)
        self.declare_parameter('diagonal_shift_distance', 0.10)
        self.declare_parameter('lateral_step_distance', 0.2555)
        self.declare_parameter('lateral_clearance_distance', 0.23)
        self.declare_parameter('post_front_clear_lateral_distance', 0.25)
        self.declare_parameter('lateral_recovery_step', 0.05)
        self.declare_parameter('max_lateral_clearance_distance', 0.42)
        self.declare_parameter('forward_clear_distance', 0.4884)
        self.declare_parameter('post_side_clear_forward_distance', 0.25)
        self.declare_parameter('shift_tolerance', 0.015)
        self.declare_parameter('rejoin_tolerance', 0.02)
        self.declare_parameter('avoidance_buffer', 0.05)
        self.declare_parameter('sensor_timeout_sec', 0.5)
        self.declare_parameter('path_point_spacing', 0.02)
        self.declare_parameter('sensor_visual_range', 0.08)
        self.declare_parameter('sensor_fov_deg', 35.0)
        self.declare_parameter('preferred_first_direction', 'right')
        self.declare_parameter('max_shift_cycles', 12)

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.debug_frame_id = str(self.get_parameter('debug_frame_id').value)
        self.debug_origin_x = float(self.get_parameter('debug_origin_x').value)
        self.debug_origin_y = float(self.get_parameter('debug_origin_y').value)
        self.goal_distance = float(self.get_parameter('goal_distance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.lateral_speed = float(self.get_parameter('lateral_speed').value)
        self.backoff_speed = float(self.get_parameter('backoff_speed').value)
        self.line_kp = float(self.get_parameter('line_kp').value)
        self.max_line_correction_speed = float(self.get_parameter('max_line_correction_speed').value)
        self.backoff_distance = float(self.get_parameter('backoff_distance').value)
        self.diagonal_shift_distance = float(self.get_parameter('diagonal_shift_distance').value)
        self.lateral_step_distance = float(self.get_parameter('lateral_step_distance').value)
        self.lateral_clearance_distance = float(self.get_parameter('lateral_clearance_distance').value)
        self.post_front_clear_lateral_distance = float(self.get_parameter('post_front_clear_lateral_distance').value)
        self.lateral_recovery_step = float(self.get_parameter('lateral_recovery_step').value)
        self.max_lateral_clearance_distance = float(self.get_parameter('max_lateral_clearance_distance').value)
        self.forward_clear_distance = float(self.get_parameter('forward_clear_distance').value)
        self.post_side_clear_forward_distance = float(self.get_parameter('post_side_clear_forward_distance').value)
        self.shift_tolerance = float(self.get_parameter('shift_tolerance').value)
        self.rejoin_tolerance = float(self.get_parameter('rejoin_tolerance').value)
        self.avoidance_buffer = float(self.get_parameter('avoidance_buffer').value)
        self.sensor_timeout_sec = float(self.get_parameter('sensor_timeout_sec').value)
        self.path_point_spacing = float(self.get_parameter('path_point_spacing').value)
        self.sensor_visual_range = float(self.get_parameter('sensor_visual_range').value)
        self.sensor_fov_rad = math.radians(float(self.get_parameter('sensor_fov_deg').value))
        self.max_shift_cycles = max(1, int(self.get_parameter('max_shift_cycles').value))

        preferred_direction = str(self.get_parameter('preferred_first_direction').value).strip().lower()
        self.preferred_direction = RIGHT if preferred_direction != 'left' else LEFT

        self.sensor_positions = {
            'front': (-0.20195, 0.00482),
            'front_left': (-0.18323, -0.11962),
            'front_right': (-0.17753, 0.12688),
            'left': (-0.00537, -0.15346),
            'right': (0.00273, 0.15504),
            'back': (0.16255, -0.00323),
        }
        self.robot_half_width = max(
            abs(self.sensor_positions['left'][1]),
            abs(self.sensor_positions['right'][1]),
        )
        self.robot_half_length = max(
            abs(self.sensor_positions['front'][0]),
            abs(self.sensor_positions['back'][0]),
        )
        if self.lateral_clearance_distance <= 0.0:
            self.lateral_clearance_distance = self.robot_half_width + self.avoidance_buffer
        if self.forward_clear_distance <= 0.0:
            self.forward_clear_distance = self.robot_half_length + self.avoidance_buffer
        inv_sqrt2 = 1.0 / math.sqrt(2.0)
        self.sensor_directions = {
            'front': (-1.0, 0.0),
            'front_left': (-inv_sqrt2, -inv_sqrt2),
            'front_right': (-inv_sqrt2, inv_sqrt2),
            'left': (0.0, -1.0),
            'right': (0.0, 1.0),
            'back': (1.0, 0.0),
        }
        self.sensor_hits = {name: False for name in self.sensor_positions}
        self.sensor_update_sec = {name: None for name in self.sensor_positions}

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(Float64, '/scissor_lift/slider', 10)
        self.path_pub = self.create_publisher(Path, '/debug/planned_path', 10)
        self.trail_pub = self.create_publisher(Path, '/debug/robot_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/debug/targets', 10)
        self.state_pub = self.create_publisher(String, '/debug/state', 10)

        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self._imu_cb, 10)

        ir_topics = {
            'front': '/ir/front/blocked',
            'front_left': '/ir/front_left/blocked',
            'front_right': '/ir/front_right/blocked',
            'left': '/ir/left/blocked',
            'right': '/ir/right/blocked',
            'back': '/ir/back/blocked',
        }
        for key, topic in ir_topics.items():
            self.create_subscription(Bool, topic, lambda msg, sensor_name=key: self._ir_cb(sensor_name, msg), 10)

        enable_qos = QoSProfile(depth=1)
        enable_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        enable_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(Bool, '/robot_enable', self._enable_cb, enable_qos)

        self.current_x = None
        self.current_y = None
        self.odom_yaw = None
        self.current_yaw = None
        self.start_pose = None
        self.goal_pose = None
        self.forward_unit = None
        self.left_unit = None
        self.last_trail_xy = None
        self.robot_trail = []
        self.enabled = False
        self.state = 'WAIT_ENABLE'
        self.state_started_at = 0.0
        self.current_offset_target = 0.0
        self.current_shift_direction = self.preferred_direction
        self.shift_start_cross = 0.0
        self.obstacle_reference_side = opposite_direction(self.current_shift_direction)
        self.backoff_start_progress = 0.0
        self.advance_start_progress = 0.0
        self.front_clear_cross = None
        self.side_seen_during_clear = False
        self.side_clear_progress = None
        self.shift_cycle_count = 0
        self.last_trigger = 'none'

        self.create_timer(0.05, self._control_loop)
        self.create_timer(0.2, self._publish_debug)

        self.get_logger().info(
            'Simple cardinal brain ready. Inputs: encoder odom, IMU, binary IR, /robot_enable. '
            'Outputs: /cmd_vel, /debug/*, lift hold at zero.'
        )

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _odom_cb(self, msg):
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)
        orientation = msg.pose.pose.orientation
        self.odom_yaw = quat_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        if self.current_yaw is None:
            self.current_yaw = self.odom_yaw
        self._capture_start_pose_if_ready()
        self._append_trail_pose()

    def _imu_cb(self, msg):
        orientation = msg.orientation
        quat_norm = (
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        if quat_norm > 1e-6:
            self.current_yaw = quat_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
            self._capture_start_pose_if_ready()

    def _ir_cb(self, sensor_name, msg):
        self.sensor_hits[sensor_name] = bool(msg.data)
        self.sensor_update_sec[sensor_name] = self._now_sec()

    def _enable_cb(self, msg):
        self.enabled = bool(msg.data)
        if not self.enabled:
            self.state = 'WAIT_ENABLE'
            self.current_offset_target = self.cross_track_error() if self.start_pose is not None else 0.0
            self.front_clear_cross = None

    def _capture_start_pose_if_ready(self):
        if self.start_pose is not None or self.current_x is None or self.current_y is None:
            return

        heading = self.pose_yaw()
        self.start_pose = (self.current_x, self.current_y)
        self.forward_unit = (-math.cos(heading), -math.sin(heading))
        self.left_unit = (math.sin(heading), -math.cos(heading))
        self.goal_pose = self._progress_to_debug(self.goal_distance, 0.0)
        self.state_started_at = self._now_sec()

    def _append_trail_pose(self):
        if self.start_pose is None or self.current_x is None or self.current_y is None:
            return

        if self.last_trail_xy is None:
            self.last_trail_xy = (self.current_x, self.current_y)
            debug_x, debug_y = self._local_to_debug(self.current_x, self.current_y)
            self.robot_trail.append(self._make_pose(debug_x, debug_y, self.debug_heading()))
            return

        dx = self.current_x - self.last_trail_xy[0]
        dy = self.current_y - self.last_trail_xy[1]
        if math.hypot(dx, dy) >= self.path_point_spacing:
            self.last_trail_xy = (self.current_x, self.current_y)
            debug_x, debug_y = self._local_to_debug(self.current_x, self.current_y)
            self.robot_trail.append(self._make_pose(debug_x, debug_y, self.debug_heading()))

    def _make_pose(self, x_pos, y_pos, yaw):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.debug_frame_id
        pose.pose.position.x = float(x_pos)
        pose.pose.position.y = float(y_pos)
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _progress_to_debug(self, along_track, cross_track):
        return (
            self.debug_origin_x - float(cross_track),
            self.debug_origin_y + float(along_track),
        )

    def _odom_vector_to_debug(self, delta_x, delta_y):
        if self.forward_unit is None or self.left_unit is None:
            return float(delta_x), float(delta_y)

        along_track = delta_x * self.forward_unit[0] + delta_y * self.forward_unit[1]
        cross_track = delta_x * self.left_unit[0] + delta_y * self.left_unit[1]
        return (
            -cross_track,
            along_track,
        )

    def _local_to_debug(self, local_x, local_y):
        if self.start_pose is None:
            return float(local_x), float(local_y)

        delta_x, delta_y = self._odom_vector_to_debug(
            local_x - self.start_pose[0],
            local_y - self.start_pose[1],
        )
        return self.debug_origin_x + delta_x, self.debug_origin_y + delta_y

    def current_heading(self):
        return self.pose_yaw()

    def pose_yaw(self):
        if self.odom_yaw is not None:
            return self.odom_yaw
        if self.current_yaw is not None:
            return self.current_yaw
        return 0.0

    def debug_heading(self):
        heading = self.current_heading()
        dir_x = math.cos(heading)
        dir_y = math.sin(heading)
        debug_dir_x, debug_dir_y = self._odom_vector_to_debug(dir_x, dir_y)
        return math.atan2(debug_dir_y, debug_dir_x)

    def sensor_active(self, sensor_name):
        stamp = self.sensor_update_sec[sensor_name]
        if stamp is None:
            return False
        if self._now_sec() - stamp > self.sensor_timeout_sec:
            return False
        return self.sensor_hits[sensor_name]

    def forward_blocked(self):
        return self.sensor_active('front')

    def front_left_blocked(self):
        return self.sensor_active('front_left')

    def front_right_blocked(self):
        return self.sensor_active('front_right')

    def front_blocked_any(self):
        return (
            self.sensor_active('front')
            or self.sensor_active('front_left')
            or self.sensor_active('front_right')
        )

    def side_blocked(self, direction):
        if direction == LEFT:
            return self.sensor_active('left') or self.sensor_active('front_left')
        return self.sensor_active('right') or self.sensor_active('front_right')

    def side_sensor_name(self, direction):
        return 'left' if direction == LEFT else 'right'

    def front_corner_sensor_name(self, direction):
        return 'front_left' if direction == LEFT else 'front_right'

    def obstacle_side_active(self):
        return self.sensor_active(self.side_sensor_name(self.obstacle_reference_side))

    def _reset_clearance_tracking(self):
        self.side_seen_during_clear = False
        self.side_clear_progress = None

    def _reset_shift_tracking(self):
        self.front_clear_cross = None

    def _expected_clearance_trigger(self, trigger):
        return trigger == self.front_corner_sensor_name(self.obstacle_reference_side)

    def _trigger_needs_backoff(self, trigger):
        return False

    def along_track_progress(self):
        if self.start_pose is None or self.current_x is None or self.current_y is None:
            return 0.0
        dx = self.current_x - self.start_pose[0]
        dy = self.current_y - self.start_pose[1]
        return dx * self.forward_unit[0] + dy * self.forward_unit[1]

    def cross_track_error(self):
        if self.start_pose is None or self.current_x is None or self.current_y is None:
            return 0.0
        dx = self.current_x - self.start_pose[0]
        dy = self.current_y - self.start_pose[1]
        return dx * self.left_unit[0] + dy * self.left_unit[1]

    def publish_twist(self, vx=0.0, vy=0.0, wz=0.0):
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        self.cmd_pub.publish(cmd)

    def publish_lift(self, slider):
        msg = Float64()
        msg.data = clamp(float(slider), 0.0, 1.0)
        self.lift_pub.publish(msg)

    def choose_trigger(self):
        if self.forward_blocked():
            return 'front'
        if self.front_left_blocked():
            return 'front_left'
        if self.front_right_blocked():
            return 'front_right'
        return None

    def choose_direction_for_trigger(self, trigger):
        if trigger == 'front_left':
            return RIGHT
        if trigger == 'front_right':
            return LEFT
        if self.front_left_blocked() and not self.front_right_blocked():
            return RIGHT
        if self.front_right_blocked() and not self.front_left_blocked():
            return LEFT
        return self.preferred_direction

    def begin_shift(self, trigger, with_backoff):
        direction = self.choose_direction_for_trigger(trigger)
        if self.side_blocked(direction) and not self.side_blocked(opposite_direction(direction)):
            direction = opposite_direction(direction)

        self.shift_start_cross = self.cross_track_error()
        self.current_offset_target = self.shift_start_cross
        self.current_shift_direction = direction
        self.obstacle_reference_side = opposite_direction(direction)
        self._reset_shift_tracking()
        self._reset_clearance_tracking()
        self.shift_cycle_count += 1
        self.last_trigger = trigger
        if self.shift_cycle_count > self.max_shift_cycles:
            self.get_logger().warn('Max shift cycles reached; stopping to avoid oscillation.')
            self.state = 'DONE'
            return

        self.state = 'SHIFT_OUT'

    def switch_direction(self):
        self.current_shift_direction *= -1
        self.preferred_direction = self.current_shift_direction
        self.obstacle_reference_side = opposite_direction(self.current_shift_direction)
        self.current_offset_target = self.cross_track_error()
        self.shift_start_cross = self.current_offset_target
        self._reset_shift_tracking()
        self._reset_clearance_tracking()
        self.state = 'SHIFT_OUT'

    def extend_shift_same_direction(self):
        next_target = self.current_offset_target + self.current_shift_direction * self.lateral_recovery_step
        shifted_distance = abs(next_target - self.shift_start_cross)
        if shifted_distance > self.max_lateral_clearance_distance:
            self.switch_direction()
            return

        self.current_offset_target = next_target
        self._reset_shift_tracking()
        self._reset_clearance_tracking()
        self.state = 'SHIFT_OUT'

    def run_move_to_goal(self):
        remaining = self.goal_distance - self.along_track_progress()
        if remaining <= self.goal_tolerance:
            self.state = 'DONE'
            return

        trigger = self.choose_trigger()
        if trigger is not None:
            self.begin_shift(trigger, with_backoff=self._trigger_needs_backoff(trigger))
            return

        line_error = self.cross_track_error()
        vy = clamp(-self.line_kp * line_error, -self.max_line_correction_speed, self.max_line_correction_speed)
        self.publish_twist(-self.forward_speed, vy, 0.0)

    def run_backoff(self):
        backed_off = self.backoff_start_progress - self.along_track_progress()
        if backed_off >= self.backoff_distance:
            self.state = 'SHIFT_OUT'
            self.publish_twist(0.0, 0.0, 0.0)
            return
        self.publish_twist(self.backoff_speed, 0.0, 0.0)

    def run_shift_out(self):
        line_error = self.cross_track_error()
        if self.side_blocked(self.current_shift_direction) and self.front_blocked_any():
            self.switch_direction()
            return

        shifted_distance = abs(line_error - self.shift_start_cross)
        if shifted_distance >= self.max_lateral_clearance_distance:
            self.switch_direction()
            return

        if self.front_blocked_any():
            self.front_clear_cross = None
            self.current_offset_target = line_error
            vy = float(self.current_shift_direction) * self.lateral_speed
            self.publish_twist(0.0, vy, 0.0)
            return

        if self.front_clear_cross is None:
            self.front_clear_cross = line_error
            self.current_offset_target = (
                self.front_clear_cross
                + self.current_shift_direction * self.post_front_clear_lateral_distance
            )

        offset_error = self.current_offset_target - line_error
        vy = clamp(offset_error * 1.8, -self.lateral_speed, self.lateral_speed)
        self.publish_twist(0.0, vy, 0.0)

        if abs(offset_error) <= self.shift_tolerance:
            self.advance_start_progress = self.along_track_progress()
            self._reset_clearance_tracking()
            self.state = 'ADVANCE_CLEAR'

    def run_advance_clear(self):
        if self.forward_blocked():
            self.shift_start_cross = self.cross_track_error()
            self.current_offset_target = self.shift_start_cross
            self._reset_shift_tracking()
            self._reset_clearance_tracking()
            self.state = 'SHIFT_OUT'
            return

        progress = self.along_track_progress()
        if self.obstacle_side_active():
            self.side_seen_during_clear = True
            self.side_clear_progress = None
        elif self.side_seen_during_clear and self.side_clear_progress is None:
            self.side_clear_progress = progress

        self.publish_twist(-self.forward_speed, 0.0, 0.0)

        if self.side_clear_progress is not None:
            if progress - self.side_clear_progress >= self.forward_clear_distance:
                self.state = 'RETURN_TO_PATH'
            return

        if (not self.side_seen_during_clear) and (progress - self.advance_start_progress >= self.post_side_clear_forward_distance):
            self.side_clear_progress = progress

    def run_return_to_path(self):
        line_error = self.cross_track_error()
        if abs(line_error) <= self.rejoin_tolerance:
            self.current_offset_target = 0.0
            self.state = 'MOVE_TO_GOAL'
            return

        return_direction = RIGHT if line_error > 0.0 else LEFT
        if self.side_blocked(return_direction):
            self.advance_start_progress = self.along_track_progress()
            self.side_clear_progress = self.along_track_progress()
            self.state = 'ADVANCE_CLEAR'
            return

        vy = clamp(-self.line_kp * line_error, -self.lateral_speed, self.lateral_speed)
        self.publish_twist(0.0, vy, 0.0)

    def _planned_points(self):
        if self.start_pose is None or self.goal_pose is None:
            return []

        x0, y0 = self.debug_origin_x, self.debug_origin_y
        x1, y1 = self.goal_pose
        total = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(math.ceil(total / self.path_point_spacing)))
        return [
            (x0 + (x1 - x0) * (index / steps), y0 + (y1 - y0) * (index / steps))
            for index in range(steps + 1)
        ]

    def _sensor_world_pose(self, sensor_name):
        sensor_x, sensor_y = self.sensor_positions[sensor_name]
        heading = self.current_heading()
        base_x, base_y = self._local_to_debug(self.current_x, self.current_y)

        sensor_world_x = math.cos(heading) * sensor_x - math.sin(heading) * sensor_y
        sensor_world_y = math.sin(heading) * sensor_x + math.cos(heading) * sensor_y
        sensor_debug_x, sensor_debug_y = self._odom_vector_to_debug(sensor_world_x, sensor_world_y)
        world_x = base_x + sensor_debug_x
        world_y = base_y + sensor_debug_y

        dir_x, dir_y = self.sensor_directions[sensor_name]
        world_dir_x = math.cos(heading) * dir_x - math.sin(heading) * dir_y
        world_dir_y = math.sin(heading) * dir_x + math.cos(heading) * dir_y
        world_dir_x, world_dir_y = self._odom_vector_to_debug(world_dir_x, world_dir_y)
        return world_x, world_y, world_dir_x, world_dir_y

    def _point(self, x_pos, y_pos, z_pos):
        point = Point()
        point.x = float(x_pos)
        point.y = float(y_pos)
        point.z = float(z_pos)
        return point

    def _build_markers(self):
        now = self.get_clock().now().to_msg()
        markers = []

        clear = Marker()
        clear.header.frame_id = self.debug_frame_id
        clear.header.stamp = now
        clear.action = Marker.DELETEALL
        markers.append(clear)

        if self.goal_pose is not None:
            goal = Marker()
            goal.header.frame_id = self.debug_frame_id
            goal.header.stamp = now
            goal.ns = 'simple_cardinal_goal'
            goal.id = 1
            goal.type = Marker.SPHERE
            goal.action = Marker.ADD
            goal.pose.position.x = self.goal_pose[0]
            goal.pose.position.y = self.goal_pose[1]
            goal.pose.position.z = 0.04
            goal.pose.orientation.w = 1.0
            goal.scale.x = 0.14
            goal.scale.y = 0.14
            goal.scale.z = 0.14
            goal.color.r = 0.18
            goal.color.g = 0.78
            goal.color.b = 0.36
            goal.color.a = 0.95
            markers.append(goal)

        if self.current_x is None or self.current_y is None:
            return MarkerArray(markers=markers)

        sensor_names = ['front', 'front_left', 'front_right', 'left', 'right', 'back']
        for index, sensor_name in enumerate(sensor_names):
            world_x, world_y, dir_x, dir_y = self._sensor_world_pose(sensor_name)
            base_id = 10 + index * 5
            blocked = self.sensor_active(sensor_name)
            half_fov = 0.5 * self.sensor_fov_rad
            cos_half = math.cos(half_fov)
            sin_half = math.sin(half_fov)
            left_bound_x = dir_x * cos_half - dir_y * sin_half
            left_bound_y = dir_x * sin_half + dir_y * cos_half
            right_bound_x = dir_x * cos_half + dir_y * sin_half
            right_bound_y = -dir_x * sin_half + dir_y * cos_half
            range_x = world_x + dir_x * self.sensor_visual_range
            range_y = world_y + dir_y * self.sensor_visual_range

            origin = Marker()
            origin.header.frame_id = self.debug_frame_id
            origin.header.stamp = now
            origin.ns = 'arena_ir_origins'
            origin.id = base_id
            origin.type = Marker.SPHERE
            origin.action = Marker.ADD
            origin.pose.position.x = world_x
            origin.pose.position.y = world_y
            origin.pose.position.z = 0.02
            origin.pose.orientation.w = 1.0
            origin.scale.x = 0.03
            origin.scale.y = 0.03
            origin.scale.z = 0.03
            origin.color.r = 0.9
            origin.color.g = 0.9
            origin.color.b = 0.9
            origin.color.a = 0.85
            markers.append(origin)

            cone = Marker()
            cone.header.frame_id = self.debug_frame_id
            cone.header.stamp = now
            cone.ns = 'arena_ir_cones'
            cone.id = base_id + 1
            cone.type = Marker.TRIANGLE_LIST
            cone.action = Marker.ADD
            cone.pose.orientation.w = 1.0
            cone.scale.x = 1.0
            cone.scale.y = 1.0
            cone.scale.z = 1.0
            cone.color.r = 0.95 if blocked else 0.20
            cone.color.g = 0.22 if blocked else 0.72
            cone.color.b = 0.18 if blocked else 0.95
            cone.color.a = 0.24 if blocked else 0.10
            cone.points.append(self._point(world_x, world_y, 0.01))
            cone.points.append(
                self._point(
                    world_x + left_bound_x * self.sensor_visual_range,
                    world_y + left_bound_y * self.sensor_visual_range,
                    0.01,
                )
            )
            cone.points.append(
                self._point(
                    world_x + right_bound_x * self.sensor_visual_range,
                    world_y + right_bound_y * self.sensor_visual_range,
                    0.01,
                )
            )
            markers.append(cone)

            edges = Marker()
            edges.header.frame_id = self.debug_frame_id
            edges.header.stamp = now
            edges.ns = 'arena_ir_edges'
            edges.id = base_id + 2
            edges.type = Marker.LINE_LIST
            edges.action = Marker.ADD
            edges.pose.orientation.w = 1.0
            edges.scale.x = 0.006
            edges.color.r = 0.95 if blocked else 0.35
            edges.color.g = 0.18 if blocked else 0.72
            edges.color.b = 0.18 if blocked else 0.95
            edges.color.a = 0.85
            edges.points.append(self._point(world_x, world_y, 0.025))
            edges.points.append(
                self._point(
                    world_x + left_bound_x * self.sensor_visual_range,
                    world_y + left_bound_y * self.sensor_visual_range,
                    0.025,
                )
            )
            edges.points.append(self._point(world_x, world_y, 0.025))
            edges.points.append(
                self._point(
                    world_x + right_bound_x * self.sensor_visual_range,
                    world_y + right_bound_y * self.sensor_visual_range,
                    0.025,
                )
            )
            markers.append(edges)

            threshold = Marker()
            threshold.header.frame_id = self.debug_frame_id
            threshold.header.stamp = now
            threshold.ns = 'arena_ir_threshold'
            threshold.id = base_id + 3
            threshold.type = Marker.LINE_STRIP
            threshold.action = Marker.ADD
            threshold.pose.orientation.w = 1.0
            threshold.scale.x = 0.005
            threshold.color.r = 0.95 if blocked else 0.30
            threshold.color.g = 0.26 if blocked else 0.78
            threshold.color.b = 0.18 if blocked else 0.95
            threshold.color.a = 0.8
            for arc_fraction in range(7):
                angle = -half_fov + (self.sensor_fov_rad * arc_fraction / 6.0)
                arc_x = dir_x * math.cos(angle) - dir_y * math.sin(angle)
                arc_y = dir_x * math.sin(angle) + dir_y * math.cos(angle)
                threshold.points.append(
                    self._point(
                        world_x + arc_x * self.sensor_visual_range,
                        world_y + arc_y * self.sensor_visual_range,
                        0.025,
                    )
                )
            markers.append(threshold)

            hit = Marker()
            hit.header.frame_id = self.debug_frame_id
            hit.header.stamp = now
            hit.ns = 'arena_ir_hits'
            hit.id = base_id + 4
            hit.type = Marker.SPHERE
            hit.action = Marker.ADD
            hit.pose.position.x = range_x
            hit.pose.position.y = range_y
            hit.pose.position.z = 0.02
            hit.pose.orientation.w = 1.0
            hit.scale.x = 0.04
            hit.scale.y = 0.04
            hit.scale.z = 0.04
            hit.color.r = 0.95
            hit.color.g = 0.25
            hit.color.b = 0.18
            hit_visible = blocked
            hit.color.a = 0.95 if hit_visible else 0.15
            markers.append(hit)

        return MarkerArray(markers=markers)

    def _publish_debug(self):
        if self.start_pose is None:
            return

        planned = Path()
        planned.header.stamp = self.get_clock().now().to_msg()
        planned.header.frame_id = self.debug_frame_id
        planned.poses = [self._make_pose(x_pos, y_pos, 0.0) for x_pos, y_pos in self._planned_points()]
        self.path_pub.publish(planned)

        trail = Path()
        trail.header.stamp = self.get_clock().now().to_msg()
        trail.header.frame_id = self.debug_frame_id
        trail.poses = self.robot_trail
        self.trail_pub.publish(trail)

        self.marker_pub.publish(self._build_markers())

        state_msg = String()
        state_msg.data = (
            f'state={self.state}, enabled={self.enabled}, trigger={self.last_trigger}, '
            f'progress={self.along_track_progress():.3f}/{self.goal_distance:.3f}, '
            f'line_error={self.cross_track_error():.3f}, target_offset={self.current_offset_target:.3f}, '
            f'front_clear_cross={self.front_clear_cross}, '
            f'front={self.sensor_active("front")}, fl={self.sensor_active("front_left")}, '
            f'fr={self.sensor_active("front_right")}, left={self.sensor_active("left")}, '
            f'right={self.sensor_active("right")}, ref_side={self.side_sensor_name(self.obstacle_reference_side)}, '
            f'ref_seen={self.side_seen_during_clear}, ref_clear_at={self.side_clear_progress}, cycles={self.shift_cycle_count}'
        )
        self.state_pub.publish(state_msg)

    def _control_loop(self):
        self.publish_lift(0.0)

        if self.start_pose is None:
            self.publish_twist(0.0, 0.0, 0.0)
            return

        if not self.enabled:
            self.state = 'WAIT_ENABLE'
            self.publish_twist(0.0, 0.0, 0.0)
            return

        if self.state == 'WAIT_ENABLE':
            self.state = 'MOVE_TO_GOAL'

        if self.state == 'MOVE_TO_GOAL':
            self.run_move_to_goal()
            return

        if self.state == 'BACKOFF':
            self.run_backoff()
            return

        if self.state == 'SHIFT_OUT':
            self.run_shift_out()
            return

        if self.state == 'ADVANCE_CLEAR':
            self.run_advance_clear()
            return

        if self.state == 'RETURN_TO_PATH':
            self.run_return_to_path()
            return

        self.publish_twist(0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCardinalBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
