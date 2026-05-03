#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


STATE_TOPICS = {
    'front': '/ir/front/blocked',
    'front_left': '/ir/front_left/blocked',
    'front_right': '/ir/front_right/blocked',
    'left': '/ir/left/blocked',
    'right': '/ir/right/blocked',
    'back': '/ir/back/blocked',
}

SIM_SCAN_TOPICS = {
    'front': '/ir_front/scan',
    'front_left': '/ir_front_right/scan',
    'front_right': '/ir_front_left/scan',
    'left': '/ir_left/scan',
    'right': '/ir_right/scan',
    'back': '/ir_back/scan',
}

DIGITAL_TOPICS = {
    'front': '/ir_front_digital',
    'front_left': '/ir_front_right_digital',
    'front_right': '/ir_front_left_digital',
    'left': '/ir_left_digital',
    'right': '/ir_right_digital',
    'back': '/ir_back_digital',
}


class ArenaIrStateAdapter(Node):
    def __init__(self):
        super().__init__('arena_ir_state_adapter')

        self.declare_parameter('source_type', 'scan')
        self.declare_parameter('publish_period_sec', 0.1)
        self.declare_parameter('source_timeout_sec', 0.5)
        self.declare_parameter('ir_low_sample_count', 3)
        self.declare_parameter('ir_trip_distance_front', 0.08)
        self.declare_parameter('ir_trip_distance_front_left', 0.085)
        self.declare_parameter('ir_trip_distance_front_right', 0.11)
        self.declare_parameter('ir_trip_distance_left', 0.085)
        self.declare_parameter('ir_trip_distance_right', 0.095)
        self.declare_parameter('ir_trip_distance_back', 0.08)

        self.source_type = str(self.get_parameter('source_type').value).strip().lower()
        self.publish_period_sec = float(self.get_parameter('publish_period_sec').value)
        self.source_timeout_sec = float(self.get_parameter('source_timeout_sec').value)
        self.ir_low_sample_count = max(1, int(self.get_parameter('ir_low_sample_count').value))
        self.ir_trip_distance = {
            'front': float(self.get_parameter('ir_trip_distance_front').value),
            'front_left': float(self.get_parameter('ir_trip_distance_front_left').value),
            'front_right': float(self.get_parameter('ir_trip_distance_front_right').value),
            'left': float(self.get_parameter('ir_trip_distance_left').value),
            'right': float(self.get_parameter('ir_trip_distance_right').value),
            'back': float(self.get_parameter('ir_trip_distance_back').value),
        }

        self.blocked_state = {name: False for name in STATE_TOPICS}
        self.last_update_sec = {name: None for name in STATE_TOPICS}
        self.state_publishers = {
            name: self.create_publisher(Bool, topic, 10)
            for name, topic in STATE_TOPICS.items()
        }

        if self.source_type == 'scan':
            for name, topic in SIM_SCAN_TOPICS.items():
                self.create_subscription(
                    LaserScan,
                    topic,
                    lambda msg, sensor_name=name: self._scan_cb(sensor_name, msg),
                    10,
                )
        elif self.source_type == 'digital':
            for name, topic in DIGITAL_TOPICS.items():
                self.create_subscription(
                    Bool,
                    topic,
                    lambda msg, sensor_name=name: self._digital_cb(sensor_name, msg),
                    10,
                )
        else:
            raise ValueError(f'Unsupported source_type: {self.source_type}')

        self.create_timer(self.publish_period_sec, self._publish_all)
        self.get_logger().info(f'IR state adapter ready. source_type={self.source_type}')

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _representative_scan_range(self, valid_samples):
        if not valid_samples:
            return float('inf')
        samples = sorted(valid_samples)
        if len(samples) < self.ir_low_sample_count:
            return samples[0]
        lowest = samples[:self.ir_low_sample_count]
        return sum(lowest) / len(lowest)

    def _scan_cb(self, sensor_name, msg):
        rmin = msg.range_min if msg.range_min > 0.0 else 0.01
        rmax = msg.range_max if msg.range_max > 0.0 else float('inf')
        valid = [
            sample for sample in msg.ranges
            if not math.isnan(sample) and sample >= rmin and sample < rmax
        ]
        representative_range = self._representative_scan_range(valid)
        self.blocked_state[sensor_name] = (
            math.isfinite(representative_range)
            and representative_range <= self.ir_trip_distance[sensor_name]
        )
        self.last_update_sec[sensor_name] = self._now_sec()

    def _digital_cb(self, sensor_name, msg):
        self.blocked_state[sensor_name] = bool(msg.data)
        self.last_update_sec[sensor_name] = self._now_sec()

    def _effective_blocked_state(self, sensor_name):
        last_update = self.last_update_sec[sensor_name]
        if last_update is None:
            return False
        if self._now_sec() - last_update > self.source_timeout_sec:
            return False
        return self.blocked_state[sensor_name]

    def _publish_all(self):
        for sensor_name, publisher in self.state_publishers.items():
            msg = Bool()
            msg.data = self._effective_blocked_state(sensor_name)
            publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArenaIrStateAdapter()
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