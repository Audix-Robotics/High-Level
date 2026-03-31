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
        self.declare_parameter('robot_body_frame_flip_180', True)
        self.declare_parameter('invert_linear_x', True)
        self.declare_parameter('publish_odom', False)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        self.r = self.get_parameter('wheel_radius').value
        self.lx = self.get_parameter('wheel_base_half').value
        self.ly = self.get_parameter('track_width_half').value
        self.flip_180 = self.get_parameter('robot_body_frame_flip_180').value
        self.invert_linear_x = bool(self.get_parameter('invert_linear_x').value)
        self.publish_odom = bool(self.get_parameter('publish_odom').value)
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.initial_yaw = float(self.get_parameter('initial_yaw').value)

        # PID parameters for per-wheel PI velocity controllers (exposed as ROS params)
        self.declare_parameter('kp_vel', 7.0)
        self.declare_parameter('ki_vel', 0.63392)
        self.declare_parameter('deadband', 0.1)
        self.declare_parameter('max_output', 50.0)
        self.declare_parameter('integral_clamp', 20.0)

        self.kp_vel = float(self.get_parameter('kp_vel').value)
        self.ki_vel = float(self.get_parameter('ki_vel').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.max_output = float(self.get_parameter('max_output').value)
        self.integral_clamp = float(self.get_parameter('integral_clamp').value)

        # PID state
        self._integral = {
            'front_left': 0.0,
            'front_right': 0.0,
            'back_left': 0.0,
            'back_right': 0.0,
        }
        self._actual_vel = {
            'front_left': 0.0,
            'front_right': 0.0,
            'back_left': 0.0,
            'back_right': 0.0,
        }
        self._last_pid_time = None
        self._have_joint_feedback = False

        # Subscribers
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        # Always subscribe to joint_states; joint_state_cb will update PID feedback
        # and also perform odometry when publish_odom is True.
        self.create_subscription(JointState, 'joint_states', self.joint_state_cb, 10)

        # Publishers
        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            'mecanum_velocity_controller/commands',
            10
        )
        self.odom_pub = self.create_publisher(Odometry, 'mecanum_odom', 10) if self.publish_odom else None

        # Odometry state
        self.x = self.initial_x
        self.y = self.initial_y
        self.yaw = self.initial_yaw
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
            (
                f'Mecanum kinematics ready: r={self.r}, lx={self.lx}, ly={self.ly}, '
                f'pose=({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f})'
            )
        )

    def cmd_vel_cb(self, msg: Twist):
        """Inverse kinematics: Twist -> 4 wheel angular velocities."""
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # Mission controller works in RobotBody frame. The physical wheel model is
        # aligned with base_link, which is rotated by pi from RobotBody.
        if self.flip_180:
            vx = -vx
            vy = -vy

        # Wheel joint positive velocity is opposite the robot's forward direction.
        if self.invert_linear_x:
            vx = -vx

        k = self.lx + self.ly

        # Mecanum IK for O-wheel roller orientation (wheel angular velocities in rad/s)
        w_fl = (1.0 / self.r) * (vx + vy - k * wz)
        w_fr = (1.0 / self.r) * (vx - vy + k * wz)
        w_bl = (1.0 / self.r) * (vx - vy - k * wz)
        w_br = (1.0 / self.r) * (vx + vy + k * wz)

        cmd = Float64MultiArray()
        cmd.data = [w_fl, w_fr, w_bl, w_br]
        self.wheel_pub.publish(cmd)

    def joint_state_cb(self, msg: JointState):
        """Forward kinematics from wheel velocities -> odometry.

        Also used to update PID feedback state every message.
        """
        # Always update PID feedback regardless of odom setting
        try:
            self._joint_state_cb(msg)
        except Exception:
            pass

        if not self.publish_odom or self.odom_pub is None:
            return

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

        # Mecanum FK (body velocities in base_link frame)
        vx = (self.r / 4.0) * (w_fl + w_fr + w_bl + w_br)
        vy = (self.r / 4.0) * (-w_fl + w_fr + w_bl - w_br)
        wz = (self.r / (4.0 * (self.lx + self.ly))) * (-w_fl + w_fr - w_bl + w_br)

        if self.invert_linear_x:
            vx = -vx

        # Convert odometry to RobotBody frame for EKF/mission consistency.
        if self.flip_180:
            vx = -vx
            vy = -vy

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

    def _joint_state_cb(self, msg: JointState):
        """Update actual wheel velocities used by the PID controllers."""
        key_map = {
            'front_left_wheel_joint':  'front_left',
            'front_right_wheel_joint': 'front_right',
            'back_left_wheel_joint':   'back_left',
            'back_right_wheel_joint':  'back_right',
        }
        for joint_name, key in key_map.items():
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                self._actual_vel[key] = msg.velocity[idx] if idx < len(msg.velocity) else 0.0

        self._have_joint_feedback = True


def main(args=None):
    rclpy.init(args=args)
    node = MecanumKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
