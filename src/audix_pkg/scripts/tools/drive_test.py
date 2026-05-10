#!/usr/bin/env python3
"""
Drive test: forward 20 cm, right 20 cm, back 10 cm, left 30 cm.

Robot body frame vs. world frame at spawn (yaw=0):
  "forward"  → body -X  → linear.x = -speed
  "backward" → body +X  → linear.x = +speed
  "right"    → body -Y  → linear.y = -speed
  "left"     → body +Y  → linear.y = +speed

Motion is driven by the Gazebo VelocityControl plugin (direct body velocity),
which makes all 4 directions work correctly in simulation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


SPEED = 0.2        # m/s for all moves
TOPIC = '/cmd_vel'


def make_cmd(vx=0.0, vy=0.0):
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = 0.0
    return msg


def run_move(node, pub, label, vx, vy, distance):
    duration = distance / SPEED
    end_time = time.time() + duration
    print(f"[drive_test] {label} ({distance*100:.0f} cm) ...")
    while time.time() < end_time:
        pub.publish(make_cmd(vx, vy))
        rclpy.spin_once(node, timeout_sec=0.05)
    # Stop
    for _ in range(10):
        pub.publish(make_cmd(0.0, 0.0))
        rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.5)


def main():
    rclpy.init()
    node = Node('drive_test')
    pub = node.create_publisher(Twist, TOPIC, 10)

    # Wait for subscriber to connect
    time.sleep(1.5)

    # ── Sequence ──────────────────────────────────────────────
    # Forward 20 cm  (body -X)
    run_move(node, pub, 'FORWARD', vx=-SPEED, vy=0.0, distance=0.20)

    # Right 20 cm  (body -Y)
    run_move(node, pub, 'RIGHT',   vx=0.0, vy=-SPEED, distance=0.20)

    # Back 10 cm  (body +X)
    run_move(node, pub, 'BACK',    vx=+SPEED, vy=0.0, distance=0.10)

    # Left 30 cm  (body +Y)
    run_move(node, pub, 'LEFT',    vx=0.0, vy=+SPEED, distance=0.30)
    # ─────────────────────────────────────────────────────────

    print('[drive_test] Sequence complete.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
