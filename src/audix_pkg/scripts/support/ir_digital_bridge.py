#!/usr/bin/env python3
"""
IR Digital Bridge

Converts 6 digital HIGH/LOW signals from ESP32 (std_msgs/Bool via micro-ROS)
into sensor_msgs/LaserScan messages that arena_roamer.py consumes unchanged.

ESP32 publishes (one per sensor):
  /ir_front_digital
  /ir_front_left_digital
  /ir_front_right_digital
  /ir_left_digital
  /ir_right_digital
  /ir_back_digital

This node publishes:
  /ir_front/scan
  /ir_front_left/scan
  /ir_front_right/scan
  /ir_left/scan
  /ir_right/scan
  /ir_back/scan

Encoding:
    blocked (True)  -> ranges = [0.079] * 10  <- inside range_max, reads as obstacle
    clear  (False)  -> ranges = [0.30] * 10   <- above range_max=0.08, filtered as inf
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

# Must match URDF gpu_lidar definitions exactly
RANGE_MIN = 0.01
RANGE_MAX = 0.08
BLOCKED_RANGE = 0.079  # < range_max, passes _ir_cb filter, triggers detection
CLEAR_RANGE   = 0.30   # > range_max, filtered out by _ir_cb, reads as inf = clear
NUM_SAMPLES   = 10
ANGLE_MIN     = -0.30543
ANGLE_MAX     =  0.30543

# If ESP32 stops sending for this long, default to clear (safe, don't phantom-block)
TIMEOUT_SEC   = 0.5

SENSORS = {
    'front':       ('ir_front_link',       '/ir_front_digital',       '/ir_front/scan'),
    'front_left':  ('ir_front_left_link',  '/ir_front_left_digital',  '/ir_front_left/scan'),
    'front_right': ('ir_front_right_link', '/ir_front_right_digital', '/ir_front_right/scan'),
    'left':        ('ir_left_link',        '/ir_left_digital',        '/ir_left/scan'),
    'right':       ('ir_right_link',       '/ir_right_digital',       '/ir_right/scan'),
    'back':        ('ir_back_link',        '/ir_back_digital',        '/ir_back/scan'),
}


class IrDigitalBridge(Node):
    def __init__(self):
        super().__init__('ir_digital_bridge')

        self._blocked   = {k: False for k in SENSORS}
        self._last_time = {k: None  for k in SENSORS}

        self._pubs = {}
        for key, (frame_id, digital_topic, scan_topic) in SENSORS.items():
            self._pubs[key] = self.create_publisher(LaserScan, scan_topic, 10)
            self.create_subscription(
                Bool,
                digital_topic,
                lambda msg, k=key: self._cb(k, msg),
                10,
            )

        # Publish at 10 Hz — matches URDF sensor update_rate and keeps
        # arena_roamer's sensor_timeout (0.35s) from triggering
        self.create_timer(0.1, self._publish_all)

        self.get_logger().info('IR digital bridge ready.')

    def _cb(self, key: str, msg: Bool) -> None:
        self._blocked[key]   = bool(msg.data)
        self._last_time[key] = self.get_clock().now().nanoseconds / 1e9

    def _make_scan(self, key: str) -> LaserScan:
        now     = self.get_clock().now()
        frame_id = SENSORS[key][0]

        # Timeout check — stale ESP32 data defaults to clear
        t = self._last_time[key]
        if t is None or (now.nanoseconds / 1e9 - t) > TIMEOUT_SEC:
            blocked = False
        else:
            blocked = self._blocked[key]

        scan                 = LaserScan()
        scan.header.stamp    = now.to_msg()
        scan.header.frame_id = frame_id
        scan.angle_min       = ANGLE_MIN
        scan.angle_max       = ANGLE_MAX
        scan.angle_increment = (ANGLE_MAX - ANGLE_MIN) / (NUM_SAMPLES - 1)
        scan.time_increment  = 0.0
        scan.scan_time       = 0.1
        scan.range_min       = RANGE_MIN
        scan.range_max       = RANGE_MAX
        scan.ranges          = [BLOCKED_RANGE if blocked else CLEAR_RANGE] * NUM_SAMPLES

        return scan

    def _publish_all(self) -> None:
        for key, pub in self._pubs.items():
            pub.publish(self._make_scan(key))


def main(args=None):
    rclpy.init(args=args)
    node = IrDigitalBridge()
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
