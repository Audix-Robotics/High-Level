#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value, low, high):
    return max(low, min(high, value))


class CardinalMotionDebug(Node):
    def __init__(self):
        super().__init__('cardinal_motion_debug')

        self.declare_parameter('speed', 0.18)
        self.declare_parameter('start_delay_sec', 3.0)
        self.declare_parameter('goal_distance', 1.20)
        self.declare_parameter('goal_tolerance', 0.03)
        self.declare_parameter('line_kp', 1.2)
        self.declare_parameter('max_line_correction_speed', 0.10)
        self.declare_parameter('rotate_speed', 0.70)
        self.declare_parameter('rotate_angle_deg', 90.0)
        self.declare_parameter('lift_up_slider', 0.72)
        self.declare_parameter('lift_motion_sec', 2.5)
        self.declare_parameter('lift_pause_sec', 1.5)
        self.declare_parameter('final_stop_hold_sec', 2.0)
        self.declare_parameter('path_point_spacing', 0.01)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_offset_x', 0.0)
        self.declare_parameter('imu_offset_y', 0.0)
        self.declare_parameter('obstacle_warmup_sec', 3.0)
        self.declare_parameter('obstacle_trigger_distance', 0.22)
        self.declare_parameter('obstacle_side_clear_distance', 0.18)
        self.declare_parameter('obstacle_half_size', 0.05)
        self.declare_parameter('obstacle_safety_margin', 0.06)
        self.declare_parameter('obstacle_avoid_speed', 0.12)
        self.declare_parameter('obstacle_return_tolerance', 0.02)

        self.speed = float(self.get_parameter('speed').value)
        self.start_delay = float(self.get_parameter('start_delay_sec').value)
        self.goal_distance = float(self.get_parameter('goal_distance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.line_kp = float(self.get_parameter('line_kp').value)
        self.max_line_correction_speed = float(self.get_parameter('max_line_correction_speed').value)
        self.rotate_speed = float(self.get_parameter('rotate_speed').value)
        self.rotate_angle = math.radians(float(self.get_parameter('rotate_angle_deg').value))
        self.lift_up_slider = float(self.get_parameter('lift_up_slider').value)
        self.lift_motion_sec = float(self.get_parameter('lift_motion_sec').value)
        self.lift_pause_sec = float(self.get_parameter('lift_pause_sec').value)
        self.final_stop_hold_sec = float(self.get_parameter('final_stop_hold_sec').value)
        self.path_spacing = float(self.get_parameter('path_point_spacing').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.imu_offset_x = float(self.get_parameter('imu_offset_x').value)
        self.imu_offset_y = float(self.get_parameter('imu_offset_y').value)
        self.obstacle_warmup_sec = float(self.get_parameter('obstacle_warmup_sec').value)
        self.obstacle_trigger_distance = float(self.get_parameter('obstacle_trigger_distance').value)
        self.obstacle_side_clear_distance = float(self.get_parameter('obstacle_side_clear_distance').value)
        self.obstacle_half_size = float(self.get_parameter('obstacle_half_size').value)
        self.obstacle_safety_margin = float(self.get_parameter('obstacle_safety_margin').value)
        self.obstacle_avoid_speed = float(self.get_parameter('obstacle_avoid_speed').value)
        self.obstacle_return_tolerance = float(self.get_parameter('obstacle_return_tolerance').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(Float64, '/scissor_lift/slider', 10)
        self.plan_pub = self.create_publisher(Path, '/debug/planned_path', 10)
        self.trail_pub = self.create_publisher(Path, '/debug/robot_path', 10)
        self.state_pub = self.create_publisher(String, '/debug/state', 10)

        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

        self.ir = {
            'front': float('inf'),
            'front_left': float('inf'),
            'front_right': float('inf'),
            'left': float('inf'),
            'right': float('inf'),
            'back': float('inf'),
        }
        # SWAPPED mapping to account for URDF naming error (physical left/right swapped)
        ir_topics = {
            'front': '/ir_front/scan',
            'front_left': '/ir_front_right/scan',
            'front_right': '/ir_front_left/scan',
            'left': '/ir_right/scan',
            'right': '/ir_left/scan',
            'back': '/ir_back/scan',
        }
        for key, topic in ir_topics.items():
            self.create_subscription(
                LaserScan,
                topic,
                lambda msg, sensor_key=key: self.ir_cb(sensor_key, msg),
                10,
            )

        self.sensor_positions = {
            'back':        ( 0.16255, -0.00323),
            'right':       ( 0.00273,  0.15504),
            'front_right': (-0.17753,  0.12688),
            'front':       (-0.20195,  0.00482),
            'front_left':  (-0.18323, -0.11962),
            'left':        (-0.00537, -0.15346),
        }
        self.front_extent = abs(self.sensor_positions['front'][0] - self.imu_offset_x)
        self.back_extent = abs(self.sensor_positions['back'][0] - self.imu_offset_x)
        self.left_extent = abs(self.sensor_positions['left'][1] - self.imu_offset_y)
        self.right_extent = abs(self.sensor_positions['right'][1] - self.imu_offset_y)
        self.robot_length = self.front_extent + self.back_extent

        self.start_pose = None
        self.goal_pose = None
        self.current_base_pose = None
        self.current_imu_pose = None
        self.initial_yaw = None
        self.first_odom_time = None
        self.forward_unit = None
        self.left_unit = None
        self.max_heading_abs = 0.0
        self.current_lift_slider = 0.0
        self.current_angular_cmd = 0.0

        self.state = 'WAIT_START'
        self.state_started_at = None
        self.last_trail_xy = None
        self.robot_trail = []

        self.avoid_obstacle_side = None
        self.avoid_shift_direction = 0.0
        self.avoid_target_offset = 0.0
        self.avoid_start_progress = 0.0
        self.avoid_required_pass_distance = 0.0
        self.avoid_trigger_sensor = None

        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.2, self.publish_debug)

        self.get_logger().info(
            'Straight-line debug ready. Robot will drive to one endpoint, avoid a single obstacle '
            'with IR-aware clearance logic, then rotate in place and operate the lift at the goal.'
        )

    def ir_cb(self, key, msg):
        valid = [
            sample for sample in msg.ranges
            if not math.isnan(sample) and not math.isinf(sample) and msg.range_min <= sample < msg.range_max
        ]
        self.ir[key] = min(valid) if valid else float('inf')

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.current_base_pose = (x, y, yaw)
        self.current_imu_pose = self.imu_pose_from_base(x, y, yaw)

        if self.start_pose is None:
            self.start_pose = (self.current_imu_pose[0], self.current_imu_pose[1])
            self.initial_yaw = yaw
            self.first_odom_time = self.get_clock().now().nanoseconds / 1e9
            self.forward_unit = (-math.cos(yaw), -math.sin(yaw))
            self.left_unit = (math.sin(yaw), -math.cos(yaw))
            self.goal_pose = (
                self.start_pose[0] + self.goal_distance * self.forward_unit[0],
                self.start_pose[1] + self.goal_distance * self.forward_unit[1],
            )

        heading_abs = abs(self.normalize_angle(yaw - self.initial_yaw))
        if heading_abs > self.max_heading_abs:
            self.max_heading_abs = heading_abs

        if self.last_trail_xy is None:
            self.last_trail_xy = (self.current_imu_pose[0], self.current_imu_pose[1])
            self.robot_trail.append(self.make_pose(self.current_imu_pose[0], self.current_imu_pose[1], yaw))
            return

        dx = self.current_imu_pose[0] - self.last_trail_xy[0]
        dy = self.current_imu_pose[1] - self.last_trail_xy[1]
        if math.hypot(dx, dy) >= self.path_spacing:
            self.last_trail_xy = (self.current_imu_pose[0], self.current_imu_pose[1])
            self.robot_trail.append(self.make_pose(self.current_imu_pose[0], self.current_imu_pose[1], yaw))

    def imu_pose_from_base(self, x, y, yaw):
        imu_x = x + math.cos(yaw) * self.imu_offset_x - math.sin(yaw) * self.imu_offset_y
        imu_y = y + math.sin(yaw) * self.imu_offset_x + math.cos(yaw) * self.imu_offset_y
        return imu_x, imu_y, yaw

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def publish_twist(self, vx=0.0, vy=0.0, wz=0.0):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
        self.current_angular_cmd = wz
        self.cmd_pub.publish(cmd)

    def publish_imu_centered_rotation(self, wz):
        vx = wz * self.imu_offset_y
        vy = -wz * self.imu_offset_x
        self.publish_twist(vx, vy, wz)

    def publish_lift(self, slider):
        msg = Float64()
        msg.data = clamp(float(slider), 0.0, 1.0)
        self.current_lift_slider = msg.data
        self.lift_pub.publish(msg)

    def along_track_progress(self):
        dx = self.current_imu_pose[0] - self.start_pose[0]
        dy = self.current_imu_pose[1] - self.start_pose[1]
        return dx * self.forward_unit[0] + dy * self.forward_unit[1]

    def cross_track_error(self):
        dx = self.current_imu_pose[0] - self.start_pose[0]
        dy = self.current_imu_pose[1] - self.start_pose[1]
        return dx * self.left_unit[0] + dy * self.left_unit[1]

    def planned_points(self):
        if self.start_pose is None or self.goal_pose is None:
            return []

        x0, y0 = self.start_pose
        x1, y1 = self.goal_pose
        length = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(math.ceil(length / self.path_spacing)))
        points = []
        for index in range(steps + 1):
            ratio = index / steps
            points.append((x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio))
        return points

    def side_sensor_names(self, side):
        if side == 'left':
            return ['front_left', 'left']
        return ['front_right', 'right']

    def front_sensor_names(self):
        return ['front', 'front_left', 'front_right']

    def choose_trigger_sensor(self, now):
        if self.first_odom_time is None or now - self.first_odom_time < self.obstacle_warmup_sec:
            return None

        candidates = [
            ('front', self.ir['front']),
            ('front_left', self.ir['front_left']),
            ('front_right', self.ir['front_right']),
            ('left', self.ir['left']),
            ('right', self.ir['right']),
        ]
        hits = [item for item in candidates if item[1] < self.obstacle_trigger_distance]
        if not hits:
            return None
        hits.sort(key=lambda item: item[1])
        return hits[0][0]

    def start_avoidance(self, sensor_name):
        if sensor_name in ('front_left', 'left'):
            obstacle_side = 'left'
        elif sensor_name in ('front_right', 'right'):
            obstacle_side = 'right'
        else:
            left_min = min(self.ir['front_left'], self.ir['left'])
            right_min = min(self.ir['front_right'], self.ir['right'])
            obstacle_side = 'left' if right_min >= left_min else 'right'

        if obstacle_side == 'left':
            self.avoid_shift_direction = -1.0
            obstacle_side_extent = self.left_extent
        else:
            self.avoid_shift_direction = 1.0
            obstacle_side_extent = self.right_extent

        self.avoid_obstacle_side = obstacle_side
        self.avoid_trigger_sensor = sensor_name
        self.avoid_target_offset = self.avoid_shift_direction * (
            obstacle_side_extent + self.obstacle_half_size + self.obstacle_safety_margin
        )
        self.avoid_start_progress = self.along_track_progress()
        self.avoid_required_pass_distance = self.robot_length + (2.0 * self.obstacle_half_size) + self.obstacle_safety_margin
        self.state = 'AVOID_SHIFT'
        self.get_logger().warn(
            'Obstacle detected by %s. Obstacle assumed on %s, shifting %s until clear.' % (
                sensor_name,
                obstacle_side,
                'left' if self.avoid_shift_direction > 0.0 else 'right',
            )
        )

    def obstacle_side_clear(self):
        return all(self.ir[name] > self.obstacle_side_clear_distance for name in self.side_sensor_names(self.avoid_obstacle_side))

    def front_clear(self):
        return all(self.ir[name] > self.obstacle_trigger_distance for name in self.front_sensor_names())

    def all_ir_clear(self):
        threshold = min(self.obstacle_side_clear_distance, self.obstacle_trigger_distance)
        return all(value > threshold for value in self.ir.values())

    def run_nominal_drive(self, now):
        remaining = self.goal_distance - self.along_track_progress()
        if remaining <= self.goal_tolerance:
            self.state = 'ROTATE_CENTER'
            self.state_started_at = now
            self.publish_twist(0.0, 0.0, 0.0)
            self.get_logger().info('Goal reached. Starting in-place rotation at endpoint.')
            return

        trigger_sensor = self.choose_trigger_sensor(now)
        if trigger_sensor is not None:
            self.start_avoidance(trigger_sensor)
            return

        line_error = self.cross_track_error()
        vy = clamp(-self.line_kp * line_error, -self.max_line_correction_speed, self.max_line_correction_speed)
        self.publish_twist(-self.speed, vy, 0.0)

    def run_avoid_shift(self):
        line_error = self.cross_track_error()
        error = self.avoid_target_offset - line_error
        vy = clamp(error * 1.5, -self.obstacle_avoid_speed, self.obstacle_avoid_speed)
        self.publish_twist(0.0, vy, 0.0)

        if abs(line_error - self.avoid_target_offset) <= self.obstacle_return_tolerance and self.obstacle_side_clear():
            self.state = 'AVOID_ADVANCE'
            self.get_logger().info('Lateral clearance reached. Advancing past obstacle.')

    def run_avoid_advance(self):
        progress = self.along_track_progress() - self.avoid_start_progress
        line_error = self.cross_track_error()
        offset_error = self.avoid_target_offset - line_error
        vy = clamp(offset_error * 1.2, -self.max_line_correction_speed, self.max_line_correction_speed)
        self.publish_twist(-self.obstacle_avoid_speed, vy, 0.0)

        if not self.obstacle_side_clear():
            self.avoid_target_offset += self.avoid_shift_direction * 0.03
            self.state = 'AVOID_SHIFT'
            self.get_logger().info('Obstacle still too close on obstacle side. Increasing lateral clearance.')
            return

        if progress >= self.avoid_required_pass_distance and self.obstacle_side_clear() and self.front_clear():
            self.state = 'AVOID_RETURN'
            self.get_logger().info(
                'Obstacle side %s is clear with robot-length margin. Returning to nominal path.' %
                self.avoid_obstacle_side
            )

    def run_avoid_return(self):
        line_error = self.cross_track_error()
        vy = clamp(-self.line_kp * line_error, -self.obstacle_avoid_speed, self.obstacle_avoid_speed)
        self.publish_twist(0.0, vy, 0.0)

        if not self.obstacle_side_clear() or not self.front_clear():
            self.state = 'AVOID_ADVANCE'
            self.get_logger().info('Return path not fully clear yet. Continuing past obstacle before rejoining.')
            return

        if abs(line_error) <= self.obstacle_return_tolerance and self.all_ir_clear():
            self.state = 'MOVE_TO_GOAL'
            self.avoid_obstacle_side = None
            self.avoid_shift_direction = 0.0
            self.avoid_target_offset = 0.0
            self.avoid_start_progress = 0.0
            self.avoid_required_pass_distance = 0.0
            self.avoid_trigger_sensor = None
            self.get_logger().info('Rejoined nominal path. Continuing to endpoint.')

    def publish_debug(self):
        if self.start_pose is None:
            return

        planned = Path()
        planned.header.stamp = self.get_clock().now().to_msg()
        planned.header.frame_id = 'odom'
        planned.poses = [self.make_pose(x, y, 0.0) for x, y in self.planned_points()]
        self.plan_pub.publish(planned)

        trail = Path()
        trail.header.stamp = self.get_clock().now().to_msg()
        trail.header.frame_id = 'odom'
        trail.poses = self.robot_trail
        self.trail_pub.publish(trail)

        msg = String()
        msg.data = (
            f'state={self.state}, trigger={self.avoid_trigger_sensor or "NONE"}, obstacle_side={self.avoid_obstacle_side or "NONE"}, '
            f'progress={self.along_track_progress():.3f}/{self.goal_distance:.3f}, line_error={self.cross_track_error():.3f}, '
            f'front={self.ir["front"]:.3f}, fl={self.ir["front_left"]:.3f}, fr={self.ir["front_right"]:.3f}, '
            f'left={self.ir["left"]:.3f}, right={self.ir["right"]:.3f}, lift={self.current_lift_slider:.2f}, '
            f'max_heading_abs_deg={math.degrees(self.max_heading_abs):.3f}, angular_cmd={self.current_angular_cmd:.3f}'
        )
        self.state_pub.publish(msg)

    def control_loop(self):
        if self.current_imu_pose is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if self.state == 'WAIT_START':
            self.publish_lift(0.0)
            self.publish_twist(0.0, 0.0, 0.0)
            if self.first_odom_time is not None and now - self.first_odom_time >= self.start_delay:
                self.state = 'MOVE_TO_GOAL'
                self.state_started_at = now
                self.get_logger().info('Starting straight-line obstacle test toward endpoint.')
            return

        if self.state == 'MOVE_TO_GOAL':
            self.publish_lift(0.0)
            self.run_nominal_drive(now)
            return

        if self.state == 'AVOID_SHIFT':
            self.publish_lift(0.0)
            self.run_avoid_shift()
            return

        if self.state == 'AVOID_ADVANCE':
            self.publish_lift(0.0)
            self.run_avoid_advance()
            return

        if self.state == 'AVOID_RETURN':
            self.publish_lift(0.0)
            self.run_avoid_return()
            return

        if self.state == 'ROTATE_CENTER':
            self.publish_lift(0.0)
            rotate_duration = abs(self.rotate_angle) / max(abs(self.rotate_speed), 1e-6)
            wz = math.copysign(abs(self.rotate_speed), self.rotate_angle)
            if now - self.state_started_at < rotate_duration:
                self.publish_imu_centered_rotation(wz)
                return
            self.publish_twist(0.0, 0.0, 0.0)
            self.state = 'LIFT_UP'
            self.state_started_at = now
            self.get_logger().info('Rotation complete. Raising scissor lift at endpoint.')
            return

        if self.state == 'LIFT_UP':
            self.publish_twist(0.0, 0.0, 0.0)
            self.publish_lift(self.lift_up_slider)
            if now - self.state_started_at >= self.lift_motion_sec:
                self.state = 'LIFT_PAUSE'
                self.state_started_at = now
                self.get_logger().info('Lift reached target height. Holding before lowering.')
            return

        if self.state == 'LIFT_PAUSE':
            self.publish_twist(0.0, 0.0, 0.0)
            self.publish_lift(self.lift_up_slider)
            if now - self.state_started_at >= self.lift_pause_sec:
                self.state = 'LIFT_DOWN'
                self.state_started_at = now
                self.get_logger().info('Lowering scissor lift.')
            return

        if self.state == 'LIFT_DOWN':
            self.publish_twist(0.0, 0.0, 0.0)
            self.publish_lift(0.0)
            if now - self.state_started_at >= self.lift_motion_sec:
                self.state = 'FINAL_STOP'
                self.state_started_at = now
                self.get_logger().info('Lift down. Entering final stop hold.')
            return

        if self.state == 'FINAL_STOP':
            self.publish_twist(0.0, 0.0, 0.0)
            self.publish_lift(0.0)
            if now - self.state_started_at >= self.final_stop_hold_sec:
                self.state = 'DONE'
                self.get_logger().info('Straight-line obstacle test complete.')
            return

        if self.state == 'DONE':
            self.publish_twist(0.0, 0.0, 0.0)
            self.publish_lift(0.0)


def main(args=None):
    rclpy.init(args=args)
    node = CardinalMotionDebug()
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