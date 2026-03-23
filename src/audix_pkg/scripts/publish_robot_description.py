#!/usr/bin/env python3
"""
Publish expanded xacro/URDF to /robot_description at 1 Hz until killed.
Usage: publish_robot_description.py <xacro_path> <robot_namespace>
"""
import os
import sys
import time
import subprocess

from rclpy.node import Node
import rclpy
from std_msgs.msg import String


def expand_xacro(xacro_path, robot_namespace):
    try:
        cmd = ['xacro', xacro_path, f'robot_namespace:={robot_namespace}']
        proc = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return proc.stdout
    except Exception as e:
        raise RuntimeError(f'xacro expansion failed: {e}')


class RobotDescriptionPublisher(Node):
    def __init__(self, urdf_xml: str):
        super().__init__('publish_robot_description')
        self.pub = self.create_publisher(String, '/robot_description', 1)
        self.urdf = urdf_xml
        self.timer = self.create_timer(1.0, self._tick)
        self.get_logger().info('Publishing /robot_description at 1Hz')

    def _tick(self):
        msg = String()
        msg.data = self.urdf
        self.pub.publish(msg)


def main(argv=None):
    argv = argv or sys.argv[1:]
    if len(argv) < 2:
        print('Usage: publish_robot_description.py <xacro_path> <robot_namespace>')
        return 2
    xacro_path = argv[0]
    robot_ns = argv[1]

    urdf = expand_xacro(xacro_path, robot_ns)

    rclpy.init()
    node = RobotDescriptionPublisher(urdf)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
