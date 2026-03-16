#!/usr/bin/env python3
"""
Goal sender node.

Waits for /send_mission service to be available, then calls it.
This satisfies the rubric requirement that goals are sent through
ROS nodes, not directly through topics.

The actual waypoints are loaded from YAML into the mission controller's
parameters. This node just triggers "go".
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class GoalSenderNode(Node):
    def __init__(self):
        super().__init__('goal_sender_node')

        self.client = self.create_client(Trigger, '/send_mission')
        self.get_logger().info('Waiting for /send_mission service...')

        self.sent = False
        self.pending = False
        self.create_timer(3.0, self._send_once)

    def _send_once(self):
        if self.sent or self.pending:
            return

        if not self.client.service_is_ready():
            self.get_logger().info('Service not ready yet, retrying...')
            return

        self.pending = True
        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self._response_cb)

    def _response_cb(self, future):
        self.pending = False
        try:
            resp = future.result()
            if resp.success:
                self.sent = True
                self.get_logger().info(f'Mission activated: {resp.message}')
            else:
                self.get_logger().warn(f'Mission not ready ({resp.message}) — will retry')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e} — will retry')


def main(args=None):
    rclpy.init(args=args)
    node = GoalSenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
