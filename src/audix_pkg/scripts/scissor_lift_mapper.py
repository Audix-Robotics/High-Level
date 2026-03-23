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
        # This model does not solve the scissor linkage passively, so the full
        # coordinated joint pose still has to be commanded explicitly.
        deg40 = math.radians(40.0)
        deg80 = math.radians(80.0)

        self._delta_at_full = {
            'bottom_stud_joint': 0.04014,
            'top_stud_joint': 0.0,
            'left_joint1': deg40,
            'right_joint1': deg40,
            'left_joint2': deg40,
            'right_joint2': deg40,
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
        self._last_published_slider = None
        self._command_dirty = True

        self._baseline_joint_positions = None
        self._joint_limits = self._load_joint_limits()
        self._baseline_ready_logged = False

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

        # Publish only when the desired command changes. Reasserting the exact same
        # target every 50 ms makes the lift hunt around the setpoint in simulation.
        self.timer = self.create_timer(0.05, self.publish_mapped_command)

        self.get_logger().info(
            'Scissor mapper ready. Send Float64 slider to /scissor_lift/slider in [0, 1]. '
            'Mapping: baseline + coordinated scissor deltas for the full lift stroke.'
        )

    def slider_callback(self, msg: Float64) -> None:
        slider = max(0.0, min(1.0, float(msg.data)))
        if abs(slider - self._latest_slider) > 1e-6:
            self._latest_slider = slider
            self._command_dirty = True

    def joint_state_callback(self, msg: JointState) -> None:
        if self._baseline_joint_positions is not None:
            return

        name_to_pos = dict(zip(msg.name, msg.position))
        baseline = {}
        for joint_name in self._joint_order:
            if joint_name in name_to_pos:
                baseline[joint_name] = float(name_to_pos[joint_name])

        # Lock baseline only after all controlled joints are available. Capturing a
        # partial baseline makes the lift jump and bounce around zero.
        if len(baseline) == len(self._joint_order):
            self._baseline_joint_positions = baseline
            self._command_dirty = True
            if not self._baseline_ready_logged:
                self.get_logger().info('Scissor mapper baseline captured from full joint state set.')
                self._baseline_ready_logged = True

    def legacy_stroke_callback(self, msg: Float64) -> None:
        # Back-compat: treat legacy command as "slider" in [0, 1].
        slider = max(0.0, min(1.0, float(msg.data)))
        if abs(slider - self._latest_slider) > 1e-6:
            self._latest_slider = slider
            self._command_dirty = True

    def publish_mapped_command(self) -> None:
        if self._baseline_joint_positions is None:
            return
        if not self._command_dirty:
            return

        slider = self._latest_slider

        baseline = self._baseline_joint_positions

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
        self._last_published_slider = slider
        self._command_dirty = False

    def _load_joint_limits(self) -> dict:
        """Returns {joint_name: (lower, upper)} for controlled joints that define limits."""
        try:
            pkg_share = get_package_share_directory('audix')
            pkg_urdf_dir = os.path.join(pkg_share, 'urdf')
            urdf_static = os.path.join(pkg_urdf_dir, 'audix.urdf')
            urdf_xacro = os.path.join(pkg_urdf_dir, 'audix.urdf.xacro')

            if os.path.exists(urdf_static):
                tree = ET.parse(urdf_static)
                root = tree.getroot()
            elif os.path.exists(urdf_xacro):
                # Expand the xacro at runtime and parse the resulting URDF XML.
                try:
                    import subprocess

                    proc = subprocess.run(['xacro', urdf_xacro], capture_output=True, text=True, check=True)
                    root = ET.fromstring(proc.stdout)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f'Failed to expand xacro for joint limits: {exc}')
                    return {}
            else:
                raise FileNotFoundError(f'No URDF or xacro found in {pkg_urdf_dir}')
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
