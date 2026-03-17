#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )

        self.publisher = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10,
        )

        self.safe_distance = 0.5
        self.get_logger().info('Obstacle Avoidance Node Started')

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0)

        if len(ranges) == 0:
            return

        front_ranges = np.concatenate((ranges[:30], ranges[-30:]))
        min_distance = np.min(front_ranges)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'RobotBody'

        if min_distance < self.safe_distance:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.5
            self.get_logger().info('Obstacle detected -> Turning')
        else:
            cmd.twist.linear.x = 0.2
            cmd.twist.angular.z = 0.0
            self.get_logger().info('Path clear -> Moving forward')

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
