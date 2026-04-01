#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('robot_body_frame_flip_180', False)

        self._odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self._odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self._flip_180 = bool(self.get_parameter('robot_body_frame_flip_180').value)
        self._latest_translation = None
        self._latest_rotation = None
        self._received_odom = False
        self._published_tf = False

        self.get_logger().info(
            f'Listening on {self._odom_topic} and publishing {self._odom_frame} -> {self._base_frame}'
        )

        self._broadcaster = TransformBroadcaster(self)
        self._subscription = self.create_subscription(
            Odometry,
            self._odom_topic,
            self._handle_odom,
            20,
        )
        self._timer = self.create_timer(1.0 / 30.0, self._publish_latest_transform)

    def _handle_odom(self, msg: Odometry) -> None:
        if not self._received_odom:
            self.get_logger().info('Received first odometry message for TF broadcast.')
            self._received_odom = True

        self._latest_translation = msg.pose.pose.position

        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        if self._flip_180:
            yaw = math.atan2(math.sin(yaw - math.pi), math.cos(yaw - math.pi))
        self._latest_rotation = (
            0.0,
            0.0,
            math.sin(yaw * 0.5),
            math.cos(yaw * 0.5),
        )

    def _publish_latest_transform(self) -> None:
        if self._latest_translation is None or self._latest_rotation is None:
            return

        if not self._published_tf:
            self.get_logger().info('Publishing odom TF transform.')
            self._published_tf = True

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame

        transform.transform.translation.x = self._latest_translation.x
        transform.transform.translation.y = self._latest_translation.y
        transform.transform.translation.z = self._latest_translation.z
        transform.transform.rotation.x = self._latest_rotation[0]
        transform.transform.rotation.y = self._latest_rotation[1]
        transform.transform.rotation.z = self._latest_rotation[2]
        transform.transform.rotation.w = self._latest_rotation[3]

        self._broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()