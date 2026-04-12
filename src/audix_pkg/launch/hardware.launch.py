"""Legacy transitional hardware launch.

This file is kept for reference while the supported real-hardware entry point is
pi_hardware.launch.py. It reflects an older architecture that mixed Pi-side
high-level behavior with local kinematics/helper nodes.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share    = get_package_share_directory('audix')
    ekf_config   = os.path.join(pkg_share, 'config', 'ekf.yaml')
    mission_config = os.path.join(pkg_share, 'config', 'mission_params.yaml')
    experiment_config = os.path.join(pkg_share, 'config', 'full_mission_params.yaml')
    rviz_config  = os.path.join(pkg_share, 'rviz', 'full_mission.rviz')

    use_rviz       = LaunchConfiguration('use_rviz')
    use_spawn_panel = LaunchConfiguration('use_spawn_panel')

    # micro-ROS agent (bridges ESP32 to ROS2)
    microros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB0', '-b', '115200'],
        output='screen',
    )

    # IR digital -> LaserScan bridge
    ir_bridge = Node(
        package='audix',
        executable='ir_digital_bridge.py',
        name='ir_digital_bridge',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # Mecanum kinematics (reads /cmd_vel, publishes wheel commands)
    mecanum = Node(
        package='audix',
        executable='mecanum_kinematics.py',
        output='screen',
        parameters=[mission_config, {'use_sim_time': False, 'publish_odom': False}],
    )

    # EKF — fuses /odom from ESP32 + /imu from ESP32
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': False}],
    )

    # Scissor lift mapper
    scissor_mapper = Node(
        package='audix',
        executable='scissor_lift_mapper.py',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # Arena roamer (navigation brain — identical to sim)
    arena_roamer = Node(
        package='audix',
        executable='arena_roamer.py',
        name='arena_roamer',
        output='screen',
        parameters=[mission_config, {'use_sim_time': False}],
    )

    # Obstacle manager
    obstacle_manager = Node(
        package='audix',
        executable='arena_obstacle_manager.py',
        name='arena_obstacle_manager',
        output='screen',
        parameters=[experiment_config, {'use_sim_time': False}],
    )

    spawn_panel = Node(
        package='audix',
        executable='arena_spawn_panel.py',
        name='arena_spawn_panel',
        output='screen',
        parameters=[experiment_config],
        condition=IfCondition(use_spawn_panel),
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
        DeclareLaunchArgument('use_rviz',        default_value='true'),
        DeclareLaunchArgument('use_spawn_panel', default_value='true'),
        microros_agent,
        ir_bridge,
        mecanum,
        ekf,
        scissor_mapper,
        arena_roamer,
        obstacle_manager,
        TimerAction(period=2.0, actions=[spawn_panel]),
        rviz,
    ])
