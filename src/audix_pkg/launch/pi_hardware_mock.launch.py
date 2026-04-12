"""Mock Raspberry Pi launch path for validating arena_roamer without hardware.

This launch reuses the real Pi-side stack from pi_hardware.launch.py and swaps
only the low-level inputs with deterministic mock publishers.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('audix')
    use_rviz = LaunchConfiguration('use_rviz')
    ir_mode = LaunchConfiguration('ir_mode')
    ir_scenario = LaunchConfiguration('ir_scenario')
    scenario_loop = LaunchConfiguration('scenario_loop')
    scenario_start_delay = LaunchConfiguration('scenario_start_delay')
    blocked_sensors = LaunchConfiguration('blocked_sensors')
    linear_x = LaunchConfiguration('linear_x')
    linear_y = LaunchConfiguration('linear_y')
    angular_z = LaunchConfiguration('angular_z')

    pi_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'pi_hardware.launch.py')
        ]),
        launch_arguments={
            'use_micro_ros_agent': 'false',
            'use_start_stop': 'false',
            'use_rviz': use_rviz,
        }.items(),
    )

    mock_imu = Node(
        package='audix',
        executable='mock_imu_publisher.py',
        name='mock_imu_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    mock_odom = Node(
        package='audix',
        executable='mock_odom_publisher.py',
        name='mock_odom_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'linear_x': linear_x,
                'linear_y': linear_y,
                'angular_z': angular_z,
            }
        ],
    )

    mock_ir = Node(
        package='audix',
        executable='mock_ir_digital_publisher.py',
        name='mock_ir_digital_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'mode': ir_mode,
                'ir_scenario': ir_scenario,
                'scenario_loop': scenario_loop,
                'scenario_start_delay': scenario_start_delay,
                'blocked_sensors': blocked_sensors,
            }
        ],
    )

    mock_enable = Node(
        package='audix',
        executable='mock_robot_enable_publisher.py',
        name='mock_robot_enable_publisher',
        output='screen',
        parameters=[{'use_sim_time': False, 'enabled': True}],
    )

    mock_limit_switch = Node(
        package='audix',
        executable='mock_limit_switch_publisher.py',
        name='mock_limit_switch_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('publish_limit_switch')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('ir_mode', default_value='scripted'),
        DeclareLaunchArgument('ir_scenario', default_value='all_clear'),
        DeclareLaunchArgument('scenario_loop', default_value='false'),
        DeclareLaunchArgument('scenario_start_delay', default_value='1.0'),
        DeclareLaunchArgument('blocked_sensors', default_value=''),
        DeclareLaunchArgument('linear_x', default_value='0.0'),
        DeclareLaunchArgument('linear_y', default_value='0.0'),
        DeclareLaunchArgument('angular_z', default_value='0.0'),
        DeclareLaunchArgument('publish_limit_switch', default_value='false'),
        pi_hardware,
        mock_imu,
        mock_odom,
        mock_ir,
        mock_enable,
        mock_limit_switch,
    ])