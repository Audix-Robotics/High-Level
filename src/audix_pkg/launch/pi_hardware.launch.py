"""Main Raspberry Pi launch path for the real Audix hardware stack.

This launch keeps the high-level contract explicit:
- Pi runs the protected arena_roamer behavior
- Pi runs robot_localization EKF and publishes /odometry/filtered
- ESP32 stays low-level and communicates through micro-ROS
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('audix')
    mission_config = os.path.join(pkg_share, 'config', 'mission_params.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'hardware', 'ekf.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'full_mission.rviz')

    serial_device = LaunchConfiguration('serial_device')
    serial_baud = LaunchConfiguration('serial_baud')
    use_micro_ros_agent = LaunchConfiguration('use_micro_ros_agent')
    use_start_stop = LaunchConfiguration('use_start_stop')
    auto_start = LaunchConfiguration('auto_start')
    start_delay = LaunchConfiguration('start_delay')
    use_rviz = LaunchConfiguration('use_rviz')

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', serial_device, '-b', serial_baud],
        output='screen',
        condition=IfCondition(use_micro_ros_agent),
    )

    ir_bridge = Node(
        package='audix',
        executable='ir_digital_bridge.py',
        name='ir_digital_bridge',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': False}],
    )

    arena_roamer = Node(
        package='audix',
        executable='arena_roamer.py',
        name='arena_roamer',
        output='screen',
        parameters=[mission_config, {'use_sim_time': False}],
    )

    start_stop = Node(
        package='audix',
        executable='start_stop_node.py',
        name='start_stop_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'auto_start': auto_start,
                'start_delay': start_delay,
            }
        ],
        condition=IfCondition(use_start_stop),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        DeclareLaunchArgument('use_micro_ros_agent', default_value='true'),
        DeclareLaunchArgument('use_start_stop', default_value='true'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('start_delay', default_value='6.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        micro_ros_agent,
        ir_bridge,
        ekf,
        arena_roamer,
        start_stop,
        rviz,
    ])