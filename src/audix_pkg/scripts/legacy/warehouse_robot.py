#!/usr/bin/env python3
"""
warehouse_robot.py

Monolithic fusion launcher that runs both `ArenaRoamer` and
`MissionController` in the same process and arbitrates `/cmd_vel` so
the mission controller has priority when a mission is active.

This loader-based approach keeps the original classes untouched and
avoids duplicating large amounts of code.
"""
import importlib.machinery
import importlib.util
import os
import sys
from ament_index_python.packages import get_package_prefix

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String


def _load_module(module_name, path):
    loader = importlib.machinery.SourceFileLoader(module_name, path)
    spec = importlib.util.spec_from_loader(loader.name, loader)
    mod = importlib.util.module_from_spec(spec)
    loader.exec_module(mod)
    return mod


def main():
    rclpy.init()

    # Resolve installed package script locations (works both in-source and after colcon install)
    pkg_lib = os.path.join(get_package_prefix('audix'), 'lib', 'audix')
    arena_path = os.path.join(pkg_lib, 'arena_roamer.py')
    mission_path = os.path.join(pkg_lib, 'mission_controller.py')

    arena_mod = _load_module('arena_roamer_mod', arena_path)
    mission_mod = _load_module('mission_controller_mod', mission_path)

    ArenaRoamer = getattr(arena_mod, 'ArenaRoamer')
    MissionController = getattr(mission_mod, 'MissionController')

    # Create a small arbiter node which will host the shared publishers.
    arbiter = Node('warehouse_arbiter')
    cmd_pub = arbiter.create_publisher(Twist, '/cmd_vel', 10)
    planned_path_pub = arbiter.create_publisher(Path, '/debug/planned_path', 10)
    robot_trail_pub = arbiter.create_publisher(Path, '/debug/robot_path', 10)
    marker_pub = arbiter.create_publisher(MarkerArray, '/debug/targets', 10)
    state_pub = arbiter.create_publisher(String, '/debug/state', 10)

    # Instantiate both nodes (they create their own timers/subscriptions)
    roamer = ArenaRoamer()
    # Ensure RViz debug markers use the odom fixed frame in the warehouse environment
    # (ArenaRoamer defaults to 'arena10' which is not connected to 'odom' in this world).
    roamer.debug_frame_id = 'odom'
    mission = MissionController()

    # Attach arbiter publishers directly (attribute names are verified)
    # Roamer publishers (verified attribute names from arena_roamer.py)
    roamer.cmd_pub    = cmd_pub
    roamer.path_pub   = planned_path_pub
    roamer.trail_pub  = robot_trail_pub
    roamer.marker_pub = marker_pub
    roamer.state_pub  = state_pub

    # Mission publishers (verified attribute names from mission_controller.py)
    # Do NOT touch mission.lift_pub — it is internal to MissionController
    mission.cmd_pub          = cmd_pub
    mission.planned_path_pub = planned_path_pub
    mission.robot_trail_pub  = robot_trail_pub
    mission.marker_pub       = marker_pub
    mission.state_pub        = state_pub

    # Monkeypatch roamer._publish_cmd so it yields to mission when mission is active.
    if hasattr(roamer, '_publish_cmd') and hasattr(mission, 'mission_loaded'):
        roamer._publish_cmd_original = roamer._publish_cmd

        def _roamer_publish_cmd_guard(vx=0.0, vy=0.0, wz=0.0):
            # If mission is loaded, suppress roamer immediately so mission owns motion.
            if getattr(mission, 'mission_loaded', False):
                return
            return roamer._publish_cmd_original(vx, vy, wz)

        roamer._publish_cmd = _roamer_publish_cmd_guard

        # Guard the roamer's debug publisher so it doesn't overwrite
        # the mission controller's path/markers in RViz when mission is running.
        roamer._publish_debug_original = roamer._publish_debug

        def _roamer_debug_guard():
            # Suppress roamer debug publishing as soon as a mission is loaded.
            if getattr(mission, 'mission_loaded', False):
                return
            return roamer._publish_debug_original()

        roamer._publish_debug = _roamer_debug_guard

    # Mission uses its own _publish_cmd implementation; no passthrough proxy required.

    # Spin both nodes using a MultiThreadedExecutor so each node's timers/callbacks run concurrently.
    executor = MultiThreadedExecutor()
    executor.add_node(arbiter)
    executor.add_node(roamer)
    executor.add_node(mission)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(roamer)
        executor.remove_node(mission)
        executor.remove_node(arbiter)
        roamer.destroy_node()
        mission.destroy_node()
        arbiter.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
