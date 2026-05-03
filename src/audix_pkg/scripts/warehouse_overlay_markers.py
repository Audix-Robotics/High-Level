#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


class WarehouseOverlayMarkers(Node):
    def __init__(self):
        super().__init__('warehouse_overlay_markers')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('publish_period', 1.0)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_period = self.get_parameter('publish_period').get_parameter_value().double_value

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, '/debug/warehouse_overlay', qos)
        self.timer = self.create_timer(max(publish_period, 0.1), self.publish_overlay)

    def _point(self, x_pos, y_pos, z_pos):
        point = Point()
        point.x = float(x_pos)
        point.y = float(y_pos)
        point.z = float(z_pos)
        return point

    def publish_overlay(self):
        timestamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        marker_id = 0
        grey = (0.55, 0.55, 0.55, 0.32)

        def add_box(name, x_pos, y_pos, z_pos, size_x, size_y, size_z, yaw=0.0):
            nonlocal marker_id

            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = timestamp
            marker.ns = 'warehouse_overlay_space'
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x_pos)
            marker.pose.position.y = float(y_pos)
            marker.pose.position.z = float(z_pos)
            marker.pose.orientation.z = math.sin(0.5 * yaw)
            marker.pose.orientation.w = math.cos(0.5 * yaw)
            marker.scale.x = float(size_x)
            marker.scale.y = float(size_y)
            marker.scale.z = float(size_z)
            marker.color.r = grey[0]
            marker.color.g = grey[1]
            marker.color.b = grey[2]
            marker.color.a = grey[3]
            markers.markers.append(marker)
            marker_id += 1

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = timestamp
            label.ns = 'warehouse_overlay_labels'
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(x_pos)
            label.pose.position.y = float(y_pos)
            label.pose.position.z = float(z_pos) + float(size_z) * 0.6
            label.pose.orientation.w = 1.0
            label.scale.z = 0.10
            label.color.r = 0.88
            label.color.g = 0.88
            label.color.b = 0.88
            label.color.a = 0.70
            label.text = name
            markers.markers.append(label)
            marker_id += 1

        for name, x_pos, y_pos in [
            ('Scan Rack C', 0.70, -2.40),
            ('Scan Rack B', -0.70, 0.0),
            ('Scan Rack A', 0.70, 2.40),
        ]:
            add_box(name, x_pos, y_pos, 0.16915, 0.82, 0.40, 0.3383, yaw=1.570796)

        self.publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseOverlayMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()