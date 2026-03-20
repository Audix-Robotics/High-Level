#!/usr/bin/env python3

import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import os

# Prevent this standalone avoidance node from running by default. The integrated
# `mission_controller.py` now performs vector-repulsion avoidance to guarantee a
# single writer to `/cmd_vel`. To enable the separate node for testing set
# `AUDIX_ALLOW_EXTERNAL_AVOIDANCE=1` in the environment before launching.
if os.environ.get('AUDIX_ALLOW_EXTERNAL_AVOIDANCE', '0') != '1':
    print('External obstacle_avoidance disabled by environment; exiting.')
    raise SystemExit(0)


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Map six IR topics to sensor names
        # SWAPPED mapping to account for URDF naming issue (physical left/right reversed)
        self.sensor_topics = {
            'front': '/ir_front/scan',
            'front_left': '/ir_front_right/scan',
            'front_right': '/ir_front_left/scan',
            'left': '/ir_right/scan',
            'right': '/ir_left/scan',
            'back': '/ir_back/scan',
        }

        # Debounce counters and blocked state (keep minimal debounce for noisy sensors)
        self.declare_parameter('debounce_count', 1)
        self.debounce_count = max(1, int(self.get_parameter('debounce_count').value))
        self._counters = {name: 0 for name in self.sensor_topics}
        self.ir_blocked = {name: False for name in self.sensor_topics}
        # Phantom-tail timestamps for force decay after sensor clears
        self.last_trigger_time = {name: 0.0 for name in self.sensor_topics}

        # Sensor physical bounds (used if LaserScan contains ranges)
        self.ir_min = 0.05
        self.ir_max = 0.15

        # Movement parameters (pull + push blending)
        self.declare_parameter('forward_speed', 0.20)    # base pull forward
        self.declare_parameter('vx_max_forward', 0.30)
        self.declare_parameter('vx_max_brake', 0.25)
        self.declare_parameter('vy_max', 0.35)

        # Evade magnitudes (fixed-magnitude binary pushes)
        self.declare_parameter('M_front', 0.30)
        self.declare_parameter('M_front_corner', 0.30)
        self.declare_parameter('M_side', 0.25)
        self.declare_parameter('M_back', 0.12)

        # Phantom tail decay: keep pushing for this many seconds after sensor clears
        self.declare_parameter('force_decay_sec', 1.0)

        # Angular flare gain (proportional to lateral push)
        self.declare_parameter('evade_angular_gain', 1.2)

        # Smoothing / ramping
        self.declare_parameter('ramp_step_linear', 0.05)
        self.declare_parameter('ramp_step_lateral', 0.05)
        self.declare_parameter('control_rate', 10.0)

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.vx_max_forward = float(self.get_parameter('vx_max_forward').value)
        self.vx_max_brake = float(self.get_parameter('vx_max_brake').value)
        self.vy_max = float(self.get_parameter('vy_max').value)

        self.M_front = float(self.get_parameter('M_front').value)
        self.M_front_corner = float(self.get_parameter('M_front_corner').value)
        self.M_side = float(self.get_parameter('M_side').value)
        self.M_back = float(self.get_parameter('M_back').value)

        self.force_decay_sec = float(self.get_parameter('force_decay_sec').value)
        self.evade_angular_gain = float(self.get_parameter('evade_angular_gain').value)

        self.ramp_step_linear = float(self.get_parameter('ramp_step_linear').value)
        self.ramp_step_lateral = float(self.get_parameter('ramp_step_lateral').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        # Publish suggestions to /avoid_cmd_vel (roamer will optionally accept)
        self.publisher = self.create_publisher(Twist, '/avoid_cmd_vel', 10)

        # Subscribe sensors
        for name, topic in self.sensor_topics.items():
            self.create_subscription(LaserScan, topic, lambda msg, n=name: self.scan_callback(n, msg), 10)

        # Optional odom subscription kept for future enhancement
        # from nav_msgs.msg import Odometry
        # self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)


        # Internal command smoothing state
        self.current_cmd = Twist()
        self.target_cmd = Twist()

        # Control loop
        self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('Obstacle Avoidance Node Started (smooth /avoid_cmd_vel)')

    def scan_callback(self, sensor_name: str, msg: LaserScan):
        ranges = np.array(msg.ranges)
        if ranges.size == 0:
            return
        finite_mask = np.isfinite(ranges)
        in_range_mask = (ranges >= self.ir_min) & (ranges <= self.ir_max)
        sample_blocked = bool(np.any(finite_mask & in_range_mask))

        # update debounce counter
        if sample_blocked:
            self._counters[sensor_name] = min(self.debounce_count, self._counters[sensor_name] + 1)
        else:
            self._counters[sensor_name] = max(0, self._counters[sensor_name] - 1)

        # effective blocked only when counter reaches threshold
        prev = self.ir_blocked[sensor_name]
        self.ir_blocked[sensor_name] = (self._counters[sensor_name] >= self.debounce_count)
        # update last trigger timestamp when blocked
        if self.ir_blocked[sensor_name]:
            self.last_trigger_time[sensor_name] = time.time()

    def _ramp_toward(self, current, target, step):
        if current < target:
            return min(current + step, target)
        if current > target:
            return max(current - step, target)
        return current

    def control_loop(self):
        # Vector Repulsion approach: compute per-sensor repulsion forces and blend
        tcmd = Twist()
        now = time.time()

        # sensor angles in robot frame (radians): front=0, front_left=+35deg, front_right=-35deg, left=+90deg, right=-90deg, back=180deg
        angles = {
            'front': 0.0,
            'front_left': np.deg2rad(35.0),
            'front_right': -np.deg2rad(35.0),
            'left': np.pi / 2.0,
            'right': -np.pi / 2.0,
            'back': np.pi,
        }

        # accumulate repulsion
        rx = 0.0
        ry = 0.0

        # helper to get min valid distance from last scan if available
        def _min_distance_for(sensor_name):
            # We don't store ranges per sensor long-term; rely on binary detection.
            # If more precise distances are required, extend scan_callback to save min distances.
            return None

        for name in self.sensor_topics:
            active = False
            # active if currently blocked or within decay window from last trigger
            if self.ir_blocked.get(name, False):
                active = True
            else:
                last = self.last_trigger_time.get(name, 0.0)
                if (now - last) <= self.force_decay_sec:
                    active = True

            if not active:
                continue

            # choose M_max for sensor
            if name == 'front':
                M = self.M_front
            elif name in ('front_left', 'front_right'):
                M = self.M_front_corner
            elif name in ('left', 'right'):
                M = self.M_side
            else:
                M = self.M_back

            theta = angles[name]
            # repulsion directed away from obstacle: negative of sensor direction
            rx += -M * np.cos(theta)
            ry += -M * np.sin(theta)

        # Pull (goal) vector
        gx = self.forward_speed
        gy = 0.0

        cmd_x = gx + rx
        cmd_y = gy + ry

        # clamp
        cmd_x = float(max(-self.vx_max_brake, min(self.vx_max_forward, cmd_x)))
        cmd_y = float(max(-self.vy_max, min(self.vy_max, cmd_y)))

        # angular flare proportional to lateral push
        cmd_w = float(self.evade_angular_gain * (-cmd_y))

        tcmd.linear.x = cmd_x
        tcmd.linear.y = cmd_y
        tcmd.angular.z = cmd_w

        # ramp toward target for smoothness
        cur_x = self.current_cmd.linear.x
        cur_y = self.current_cmd.linear.y
        cur_w = self.current_cmd.angular.z

        step_lin = self.ramp_step_linear
        step_lat = self.ramp_step_lateral
        step_ang = 0.08

        self.current_cmd.linear.x = self._ramp_toward(cur_x, tcmd.linear.x, step_lin)
        self.current_cmd.linear.y = self._ramp_toward(cur_y, tcmd.linear.y, step_lat)
        self.current_cmd.angular.z = self._ramp_toward(cur_w, tcmd.angular.z, step_ang)

        out = Twist()
        out.linear.x = float(self.current_cmd.linear.x)
        out.linear.y = float(self.current_cmd.linear.y)
        out.angular.z = float(self.current_cmd.angular.z)
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
