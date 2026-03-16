#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ScissorLiftMapper(Node):
    def __init__(self):
        super().__init__('scissor_lift_mapper')

        # Nominal target pose from your manual calibration:
        # bottom stud = 35 mm, level1/2 = 30 deg, upper levels = 60 deg.
        # This nominal pose is reached at slider = 0.75.
        self.declare_parameter('nominal_stroke_m', 0.035)
        self.declare_parameter('nominal_theta_level12_rad', math.radians(30.0))
        self.declare_parameter('nominal_theta_upper_rad', math.radians(60.0))
        self.declare_parameter('nominal_slider_value', 0.75)

        self.nominal_stroke = float(self.get_parameter('nominal_stroke_m').value)
        self.nominal_theta_level12 = float(self.get_parameter('nominal_theta_level12_rad').value)
        self.nominal_theta_upper = float(self.get_parameter('nominal_theta_upper_rad').value)
        self.nominal_slider = float(self.get_parameter('nominal_slider_value').value)

        if self.nominal_slider <= 0.0:
            self.nominal_slider = 0.75

        self._latest_slider = 0.0

        # Scale factors from slider units to joint targets.
        self.stroke_per_slider = self.nominal_stroke / self.nominal_slider
        self.theta12_per_slider = self.nominal_theta_level12 / self.nominal_slider
        self.theta3456_per_slider = self.nominal_theta_upper / self.nominal_slider

        self.slider_sub = self.create_subscription(
            Float64,
            '/scissor_lift/slider',
            self.slider_callback,
            10,
        )

        # Legacy input support (meters of bottom stud stroke).
        self.cmd_sub = self.create_subscription(
            Float64,
            '/camera_lift/command',
            self.legacy_stroke_callback,
            10,
        )
        self.cmd_sub_legacy = self.create_subscription(
            Float64,
            '/scissor_lift/command',
            self.legacy_stroke_callback,
            10,
        )

        self.joint_publishers = {
            'bottom_stud_joint': self.create_publisher(Float64, '/model/audix/joint/bottom_stud_joint/cmd_pos', 10),
            'left_joint1': self.create_publisher(Float64, '/model/audix/joint/left_joint1/cmd_pos', 10),
            'right_joint1': self.create_publisher(Float64, '/model/audix/joint/right_joint1/cmd_pos', 10),
            'left_joint2': self.create_publisher(Float64, '/model/audix/joint/left_joint2/cmd_pos', 10),
            'right_joint2': self.create_publisher(Float64, '/model/audix/joint/right_joint2/cmd_pos', 10),
            'left_joint3': self.create_publisher(Float64, '/model/audix/joint/left_joint3/cmd_pos', 10),
            'right_joint3': self.create_publisher(Float64, '/model/audix/joint/right_joint3/cmd_pos', 10),
            'left_joint4': self.create_publisher(Float64, '/model/audix/joint/left_joint4/cmd_pos', 10),
            'right_joint4': self.create_publisher(Float64, '/model/audix/joint/right_joint4/cmd_pos', 10),
            'left_joint5': self.create_publisher(Float64, '/model/audix/joint/left_joint5/cmd_pos', 10),
            'right_joint5': self.create_publisher(Float64, '/model/audix/joint/right_joint5/cmd_pos', 10),
            'left_joint6': self.create_publisher(Float64, '/model/audix/joint/left_joint6/cmd_pos', 10),
            'right_joint6': self.create_publisher(Float64, '/model/audix/joint/right_joint6/cmd_pos', 10),
            'camera_joint': self.create_publisher(Float64, '/model/audix/joint/camera_joint/cmd_pos', 10),
        }

        # Republish at a fixed rate so Gazebo controller always has a fresh command.
        self.timer = self.create_timer(0.05, self.publish_mapped_command)

        self.get_logger().info(
            'Scissor mapper ready. Send Float64 slider to /scissor_lift/slider in [-1, 1]. '
            f'At slider={self.nominal_slider:.2f}: stroke={self.nominal_stroke:.3f} m, '
            f'level12={math.degrees(self.nominal_theta_level12):.1f} deg, '
            f'upper={math.degrees(self.nominal_theta_upper):.1f} deg.'
        )

    def slider_callback(self, msg: Float64) -> None:
        self._latest_slider = max(-1.0, min(1.0, float(msg.data)))

    def legacy_stroke_callback(self, msg: Float64) -> None:
        # Convert legacy meter command to slider units.
        stroke = float(msg.data)
        if self.stroke_per_slider > 0.0:
            self._latest_slider = max(-1.0, min(1.0, stroke / self.stroke_per_slider))

    def publish_mapped_command(self) -> None:
        slider = self._latest_slider

        # Coupled mathematical mapping from one slider to all scissor joints.
        stroke = slider * self.stroke_per_slider
        theta12 = slider * self.theta12_per_slider
        theta3456 = slider * self.theta3456_per_slider

        targets = {
            'bottom_stud_joint': stroke,
            'left_joint1': theta12,
            'right_joint1': theta12,
            'left_joint2': theta12,
            'right_joint2': theta12,
            'left_joint3': -theta3456,
            'right_joint3': -theta3456,
            'left_joint4': theta3456,
            'right_joint4': theta3456,
            'left_joint5': -theta3456,
            'right_joint5': -theta3456,
            'left_joint6': -theta3456,
            'right_joint6': -theta3456,
            # Camera counter-rotates to stay level: compensates the net rotation
            # accumulated through the scissor chain (level-1 + upper levels).
            'camera_joint': -(theta12 + theta3456),
        }

        for joint_name, value in targets.items():
            msg = Float64()
            msg.data = float(value)
            self.joint_publishers[joint_name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScissorLiftMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
