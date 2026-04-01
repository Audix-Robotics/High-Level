#!/usr/bin/env python3

from copy import deepcopy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


class EkfInputAdapter(Node):
    def __init__(self):
        super().__init__('ekf_input_adapter')

        self.declare_parameter('raw_odom_topic', 'odom')
        self.declare_parameter('adapted_odom_topic', 'ekf/odom')
        self.declare_parameter('raw_imu_topic', 'imu')
        self.declare_parameter('adapted_imu_topic', 'ekf/imu')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('imu_frame', 'imu_link')

        self.raw_odom_topic = str(self.get_parameter('raw_odom_topic').value)
        self.adapted_odom_topic = str(self.get_parameter('adapted_odom_topic').value)
        self.raw_imu_topic = str(self.get_parameter('raw_imu_topic').value)
        self.adapted_imu_topic = str(self.get_parameter('adapted_imu_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.imu_frame = str(self.get_parameter('imu_frame').value)

        self.odom_pub = self.create_publisher(Odometry, self.adapted_odom_topic, 20)
        self.imu_pub = self.create_publisher(Imu, self.adapted_imu_topic, 50)

        self.create_subscription(Odometry, self.raw_odom_topic, self._odom_cb, 20)
        self.create_subscription(Imu, self.raw_imu_topic, self._imu_cb, 50)

        self.get_logger().info(
            'Adapting EKF inputs: '
            f'{self.raw_odom_topic} -> {self.adapted_odom_topic} '
            f'({self.odom_frame} -> {self.base_frame}), '
            f'{self.raw_imu_topic} -> {self.adapted_imu_topic} '
            f'({self.imu_frame})'
        )

    def _odom_cb(self, msg: Odometry) -> None:
        adapted = deepcopy(msg)
        adapted.header.frame_id = self.odom_frame
        adapted.child_frame_id = self.base_frame
        self.odom_pub.publish(adapted)

    def _imu_cb(self, msg: Imu) -> None:
        adapted = deepcopy(msg)
        adapted.header.frame_id = self.imu_frame
        self.imu_pub.publish(adapted)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkfInputAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()