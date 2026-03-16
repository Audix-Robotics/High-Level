#!/usr/bin/env python3

import math
import os
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


class ScissorLiftMapper(Node):
    def __init__(self):
        super().__init__('scissor_lift_mapper')

        # Slider input is expected in [0.0, 1.0] (GUI publishes percent/100).
        # Mapping is defined as DELTAS from the baseline pose captured at startup.
        # This ensures the motion is exactly the requested travel/angles regardless
        # of the current joint state when the node starts.
        deg40 = math.radians(40.0)
        deg80 = math.radians(80.0)

        # Order is aligned with controllers.yaml scissor_position_controller.joints.
        self._delta_at_full = {
            # Prismatic stroke: +40.14 mm
            'bottom_stud_joint': 0.04014,
            'top_stud_joint': 0.0,
            # Replace 30deg (0.522 rad) with 40deg
            'left_joint1': deg40,
            'right_joint1': deg40,
            'left_joint2': deg40,
            'right_joint2': deg40,
            # Replace 60deg (1.044 rad) with 80deg (preserve signs)
            'left_joint3': -deg80,
            'right_joint3': -deg80,
            'left_joint4': deg80,
            'right_joint4': deg80,
            'left_joint5': -deg80,
            'right_joint5': -deg80,
            'left_joint6': -deg80,
            'right_joint6': -deg80,
            'camera_joint': -deg40,
        }

        self._joint_order = list(self._delta_at_full.keys())
        self._latest_slider = 0.0

        self._baseline_joint_positions = None
        self._joint_limits = self._load_joint_limits()

        self.slider_sub = self.create_subscription(
            Float64,
            '/scissor_lift/slider',
            self.slider_callback,
            10,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
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

        # Publishes to the ros2_control forward_command_controller.
        # Joint order MUST match the scissor_position_controller joints list in controllers.yaml.
        self.cmd_publisher = self.create_publisher(
            Float64MultiArray,
            '/scissor_position_controller/commands',
            10,
        )

        # Republish at a fixed rate so Gazebo controller always has a fresh command.
        self.timer = self.create_timer(0.05, self.publish_mapped_command)

        self.get_logger().info(
            'Scissor mapper ready. Send Float64 slider to /scissor_lift/slider in [0, 1]. '
            'Mapping: baseline + slider * requested deltas (40.14mm stroke, 40deg/80deg joints).'
        )

    def slider_callback(self, msg: Float64) -> None:
        self._latest_slider = max(0.0, min(1.0, float(msg.data)))

    def joint_state_callback(self, msg: JointState) -> None:
        if self._baseline_joint_positions is not None:
            return

        name_to_pos = dict(zip(msg.name, msg.position))
        baseline = {}
        for joint_name in self._joint_order:
            if joint_name in name_to_pos:
                baseline[joint_name] = float(name_to_pos[joint_name])

        # Only lock baseline once we have at least the primary prismatic joint.
        if 'bottom_stud_joint' in baseline:
            self._baseline_joint_positions = baseline

    def legacy_stroke_callback(self, msg: Float64) -> None:
        # Back-compat: treat legacy command as "slider" in [0, 1].
        self._latest_slider = max(0.0, min(1.0, float(msg.data)))

    def publish_mapped_command(self) -> None:
        slider = self._latest_slider

        baseline = self._baseline_joint_positions or {}

        # Linear mapping:
        # - slider=0.0 -> baseline pose (current pose treated as "zero")
        # - slider=1.0 -> baseline + requested deltas
        commands = []
        for joint_name in self._joint_order:
            base = float(baseline.get(joint_name, 0.0))
            delta = float(self._delta_at_full[joint_name])
            target = base + delta * slider

            # Final clamp to limits.
            limit = self._joint_limits.get(joint_name)
            if limit is not None:
                lower, upper = limit
                target = min(upper, max(lower, target))

            commands.append(float(target))

        msg = Float64MultiArray()
        msg.data = commands
        self.cmd_publisher.publish(msg)

    def _load_joint_limits(self) -> dict:
        """Returns {joint_name: (lower, upper)} for controlled joints that define limits."""
        try:
            pkg_share = get_package_share_directory('audix')
            urdf_path = os.path.join(pkg_share, 'urdf', 'audix.urdf')
            tree = ET.parse(urdf_path)
            root = tree.getroot()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Failed to parse URDF for joint limits: {exc}')
            return {}

        limits = {}
        for joint in root.findall('joint'):
            name = joint.get('name')
            if name not in self._delta_at_full:
                continue

            limit = joint.find('limit')
            if limit is None:
                continue

            try:
                lower = float(limit.get('lower', 'nan'))
                upper = float(limit.get('upper', 'nan'))
            except ValueError:
                continue

            if math.isnan(lower) or math.isnan(upper):
                continue

            limits[name] = (lower, upper)

        return limits


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
