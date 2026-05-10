#!/usr/bin/env python3

import math

import rclpy
import tf_transformations
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


yaw = 0.0
x = 0.0
y = 0.0
last_time = None

waypoints = [
    (0.0, 0.0, 0.0),
    (1.0, 0.0, 0.0),
    (1.0, 1.0, math.pi / 2),
    (0.0, 1.0, math.pi),
]


class WaypointsControl(Node):
    def __init__(self):
        super().__init__('waypoints_control')

        self.current_waypoint = 0
        self.state = 'rotate_to_target'
        self.linear_vel = 0.0

        self.cmd_pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_callback, 10)
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        self.timer = self.create_timer(0.02, self.control_step)

    def imu_callback(self, msg: Imu):
        global yaw
        q = msg.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        (_, _, yaw_value) = tf_transformations.euler_from_quaternion(quaternion)
        yaw = self.normalize_angle(yaw_value)

    def odom_callback(self, msg: Odometry):
        global x, y
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.linear_vel = msg.twist.twist.linear.x

    def clock_callback(self, _msg: Clock):
        pass

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_step(self):
        global last_time

        current_time = self.get_clock().now()
        if last_time is None:
            last_time = current_time
            return
        last_time = current_time

        target_x, target_y, target_yaw = waypoints[self.current_waypoint]
        dx = target_x - x
        dy = target_y - y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        angle_to_target = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_target - yaw)
        final_yaw_error = self.normalize_angle(target_yaw - yaw)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'RobotBody'

        if self.state == 'rotate_to_target':
            if abs(angle_error) > 0.005:
                cmd.twist.angular.z = -1.5 * angle_error
            else:
                self.state = 'move_forward'

        elif self.state == 'move_forward':
            if distance > 0.05:
                cmd.twist.linear.x = min(0.5 * distance, 0.3)
            else:
                self.state = 'rotate_to_final'

        elif self.state == 'rotate_to_final':
            if abs(final_yaw_error) > 0.005:
                cmd.twist.angular.z = -1.5 * final_yaw_error
            else:
                self.current_waypoint = (self.current_waypoint + 1) % len(waypoints)
                self.state = 'rotate_to_target'

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointsControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
