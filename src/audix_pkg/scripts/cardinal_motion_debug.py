#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class CardinalMotionDebug(Node):
    def __init__(self):
        super().__init__('cardinal_motion_debug')

        self.declare_parameter('speed', 0.2)
        self.declare_parameter('start_delay_sec', 3.0)
        self.declare_parameter('settle_sec', 0.5)
        self.declare_parameter('path_point_spacing', 0.01)
        self.declare_parameter('odom_topic', '/odom')

        self.speed = float(self.get_parameter('speed').value)
        self.start_delay = float(self.get_parameter('start_delay_sec').value)
        self.settle_sec = float(self.get_parameter('settle_sec').value)
        self.path_spacing = float(self.get_parameter('path_point_spacing').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.plan_pub = self.create_publisher(Path, '/debug/planned_path', 10)
        self.trail_pub = self.create_publisher(Path, '/debug/robot_path', 10)
        self.state_pub = self.create_publisher(String, '/debug/state', 10)

        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

        self.start_pose = None
        self.current_pose = None
        self.initial_yaw = None
        self.max_yaw_drift = 0.0
        self.last_trail_xy = None
        self.robot_trail = []

        self.started = False
        self.sequence_done = False
        self.step_index = 0
        self.step_started_at = None
        self.stop_started_at = None

        self.steps = [
            ('FORWARD', -self.speed, 0.0, 0.20),
            ('RIGHT', 0.0, -self.speed, 0.20),
            ('BACK', self.speed, 0.0, 0.10),
            ('LEFT', 0.0, self.speed, 0.30),
        ]

        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.2, self.publish_debug)

        self.get_logger().info(
            'Cardinal motion debug ready. Sequence: forward 20 cm, right 20 cm, '
            'back 10 cm, left 30 cm. angular.z is forced to 0.0 for the entire run.'
        )

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.current_pose = (x, y, yaw)

        if self.start_pose is None:
            self.start_pose = (x, y)
            self.initial_yaw = yaw

        yaw_drift = abs(normalize_angle(yaw - self.initial_yaw))
        if yaw_drift > self.max_yaw_drift:
            self.max_yaw_drift = yaw_drift

        if self.last_trail_xy is None:
            self.last_trail_xy = (x, y)
            self.robot_trail.append(self.make_pose(x, y, yaw))
            return

        dx = x - self.last_trail_xy[0]
        dy = y - self.last_trail_xy[1]
        if math.hypot(dx, dy) >= self.path_spacing:
            self.last_trail_xy = (x, y)
            self.robot_trail.append(self.make_pose(x, y, yaw))

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def publish_twist(self, vx=0.0, vy=0.0):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def planned_points(self):
        if self.start_pose is None:
            return []

        x0, y0 = self.start_pose
        pts = [(x0, y0)]
        x, y = x0, y0
        for _, vx, vy, distance in self.steps:
            if abs(vx) > 0.0:
                x += math.copysign(distance, vx)
            if abs(vy) > 0.0:
                y += math.copysign(distance, vy)
            pts.append((x, y))
        return pts

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
        if self.sequence_done:
            msg.data = (
                f'state=DONE, max_yaw_drift_deg={math.degrees(self.max_yaw_drift):.4f}, '
                'angular_cmd=0.0 always'
            )
        elif self.step_index < len(self.steps):
            label = self.steps[self.step_index][0]
            msg.data = (
                f'state={label}, step={self.step_index + 1}/{len(self.steps)}, '
                f'max_yaw_drift_deg={math.degrees(self.max_yaw_drift):.4f}, '
                'angular_cmd=0.0 always'
            )
        else:
            msg.data = 'state=WAITING'
        self.state_pub.publish(msg)

    def control_loop(self):
        if self.current_pose is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if not self.started:
            first_seen = self.robot_trail[0].header.stamp.sec + self.robot_trail[0].header.stamp.nanosec / 1e9
            if now - first_seen < self.start_delay:
                self.publish_twist(0.0, 0.0)
                return
            self.started = True
            self.step_started_at = now
            self.get_logger().info('Starting cardinal motion sequence.')

        if self.sequence_done:
            self.publish_twist(0.0, 0.0)
            return

        if self.step_index >= len(self.steps):
            self.sequence_done = True
            self.publish_twist(0.0, 0.0)
            self.get_logger().info(
                'Cardinal sequence complete. Max yaw drift = %.6f deg' %
                math.degrees(self.max_yaw_drift)
            )
            return

        label, vx, vy, distance = self.steps[self.step_index]
        duration = distance / self.speed

        if self.stop_started_at is not None:
            self.publish_twist(0.0, 0.0)
            if now - self.stop_started_at >= self.settle_sec:
                self.step_index += 1
                self.stop_started_at = None
                self.step_started_at = now
                if self.step_index < len(self.steps):
                    self.get_logger().info('Starting step %d: %s' % (self.step_index + 1, self.steps[self.step_index][0]))
            return

        if now - self.step_started_at < duration:
            self.publish_twist(vx, vy)
        else:
            self.publish_twist(0.0, 0.0)
            self.stop_started_at = now
            self.get_logger().info('Completed step %d: %s' % (self.step_index + 1, label))


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