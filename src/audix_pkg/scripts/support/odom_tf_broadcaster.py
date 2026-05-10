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
        self.declare_parameter('yaw_offset_rad', 0.0)
        self.declare_parameter('flatten_to_yaw', False)

        self._odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self._odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self._yaw_offset_rad = self.get_parameter('yaw_offset_rad').get_parameter_value().double_value
        self._flatten_to_yaw = self.get_parameter('flatten_to_yaw').get_parameter_value().bool_value
        self._latest_translation = None
        self._latest_rotation = None
        self._received_odom = False
        self._published_tf = False

        self.get_logger().info(
            f'Listening on {self._odom_topic} and publishing {self._odom_frame} -> {self._base_frame} '
            f'with yaw_offset_rad={self._yaw_offset_rad}'
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
        norm = math.sqrt(
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        if norm > 0.0:
            self._latest_rotation = (
                orientation.x / norm,
                orientation.y / norm,
                orientation.z / norm,
                orientation.w / norm,
            )
        else:
            self._latest_rotation = (0.0, 0.0, 0.0, 1.0)

    def _apply_yaw_offset(self, rotation):
        if not self._flatten_to_yaw and abs(self._yaw_offset_rad) <= 1e-9:
            return rotation

        if self._flatten_to_yaw:
            x_val, y_val, z_val, w_val = rotation
            siny_cosp = 2.0 * (w_val * z_val + x_val * y_val)
            cosy_cosp = 1.0 - 2.0 * (y_val * y_val + z_val * z_val)
            yaw = math.atan2(siny_cosp, cosy_cosp) + self._yaw_offset_rad
            return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))

        half_yaw = 0.5 * self._yaw_offset_rad
        offset_z = math.sin(half_yaw)
        offset_w = math.cos(half_yaw)

        x_val, y_val, z_val, w_val = rotation
        return (
            x_val * offset_w + y_val * offset_z,
            -x_val * offset_z + y_val * offset_w,
            w_val * offset_z + z_val * offset_w,
            w_val * offset_w - z_val * offset_z,
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
        rotation = self._apply_yaw_offset(self._latest_rotation)
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]

        self._broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
