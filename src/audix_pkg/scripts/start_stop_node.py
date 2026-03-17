#!/usr/bin/env python3
"""
Start/Stop signal node.

Publishes Bool on /robot_enable.
Auto-starts after configurable delay, or can be triggered manually.
Publishing False at any time = immediate emergency stop.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class StartStopNode(Node):
    def __init__(self):
        super().__init__('start_stop_node')

        self.declare_parameter('auto_start', True)
        self.declare_parameter('start_delay', 6.0)

        self.auto_start = self.get_parameter('auto_start').value
        self.delay = self.get_parameter('start_delay').value

        self.pub = self.create_publisher(Bool, '/robot_enable', 10)
        self.started = False

        if self.auto_start:
            self.get_logger().info(
                f'Auto-start in {self.delay}s. '
                f'To stop: ros2 topic pub /robot_enable std_msgs/msg/Bool '
                f'"{{data: false}}" --once'
            )
            self.create_timer(self.delay, self._auto_start_cb)
        else:
            self.get_logger().info(
                'Manual mode. To start: ros2 topic pub /robot_enable '
                'std_msgs/msg/Bool "{data: true}" --once'
            )

        # Keep publishing current state at 2 Hz
        self.create_timer(0.5, self._heartbeat)

    def _auto_start_cb(self):
        if not self.started:
            self.started = True
            msg = Bool()
            msg.data = True
            self.pub.publish(msg)
            self.get_logger().info('START signal sent!')

    def _heartbeat(self):
        if self.started:
            msg = Bool()
            msg.data = True
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StartStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
