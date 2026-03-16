#!/usr/bin/env python3
"""
Mecanum drive kinematics node.

Subscribes to /cmd_vel (geometry_msgs/Twist) with vx, vy, wz.
Publishes individual wheel velocities to the JointGroupVelocityController.
Also computes forward-kinematics odometry and publishes /mecanum_odom.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
def _quat_from_euler(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return [sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy]


class MecanumKinematics(Node):
    def __init__(self):
        super().__init__('mecanum_kinematics')

        # Parameters (from mission_params.yaml via launch)
        self.declare_parameter('wheel_radius', 0.0485)
        self.declare_parameter('wheel_base_half', 0.09)
        self.declare_parameter('track_width_half', 0.1574)

        self.r = self.get_parameter('wheel_radius').value
        self.lx = self.get_parameter('wheel_base_half').value
        self.ly = self.get_parameter('track_width_half').value

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        # Publishers
        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/mecanum_velocity_controller/commands',
            10
        )
        self.odom_pub = self.create_publisher(Odometry, '/mecanum_odom', 10)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = None
        self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]  # fl, fr, bl, br

        # Joint name order must match controller config
        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'back_left_wheel_joint',
            'back_right_wheel_joint',
        ]

        self.get_logger().info(
            f'Mecanum kinematics ready: r={self.r}, lx={self.lx}, ly={self.ly}'
        )

    def cmd_vel_cb(self, msg: Twist):
        """Inverse kinematics: Twist -> 4 wheel angular velocities."""
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        k = self.lx + self.ly

        # Mecanum IK (wheel angular velocities in rad/s)
        w_fl = (1.0 / self.r) * (vx - vy - k * wz)
        w_fr = (1.0 / self.r) * (vx + vy + k * wz)
        w_bl = (1.0 / self.r) * (vx + vy - k * wz)
        w_br = (1.0 / self.r) * (vx - vy + k * wz)

        cmd = Float64MultiArray()
        cmd.data = [w_fl, w_fr, w_bl, w_br]
        self.wheel_pub.publish(cmd)

    def joint_state_cb(self, msg: JointState):
        """Forward kinematics from wheel velocities -> odometry."""
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 1.0:
            self.last_time = now
            return
        self.last_time = now

        # Extract wheel velocities
        vels = [0.0, 0.0, 0.0, 0.0]
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.velocity):
                    vels[i] = msg.velocity[idx]

        w_fl, w_fr, w_bl, w_br = vels

        # Mecanum FK (body velocities)
        vx = (self.r / 4.0) * (w_fl + w_fr + w_bl + w_br)
        vy = (self.r / 4.0) * (-w_fl + w_fr + w_bl - w_br)
        wz = (self.r / (4.0 * (self.lx + self.ly))) * (-w_fl + w_fr - w_bl + w_br)

        # Integrate in world frame
        self.yaw += wz * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'RobotBody'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = _quat_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # Covariance (diagonal)
        pc = [0.001, 0.001, 1e6, 1e6, 1e6, 0.01]
        for i in range(6):
            odom.pose.covariance[i * 7] = pc[i]
        tc = [0.001, 0.001, 1e6, 1e6, 1e6, 0.01]
        for i in range(6):
            odom.twist.covariance[i * 7] = tc[i]

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
