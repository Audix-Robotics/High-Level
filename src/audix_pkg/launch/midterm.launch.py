"""
Audix midterm master launch file.

Launches:
  1. Gazebo Harmonic with warehouse world
  2. Robot state publisher
  3. Robot spawn
  4. gz_ros2_bridge (all topics including 6 IR sensors)
  5. Controller spawners (joint_state_broadcaster, mecanum_velocity, scissor_position)
  6. EKF (robot_localization)
  7. Mecanum kinematics node
  8. Mission controller
  9. Start/stop node
  10. Goal sender node
  11. Static/dynamic obstacle spawning from YAML
"""

import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('audix')
    pkg_parent = os.path.dirname(pkg_share)

    urdf_path = os.path.join(pkg_share, 'urdf', 'audix.urdf')
    world_path = os.path.join(pkg_share, 'world', 'debug_empty.sdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    mission_config = os.path.join(pkg_share, 'config', 'mission_params.yaml')

    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
    # Obstacles disabled in straight-line debug mode.
    robot_description = Command([
        'xacro ', urdf_path,
        ' controllers_yaml:=', controllers_yaml,
    ])

    # --- Environment ---
    gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{pkg_parent}:{pkg_share}'
    )
    ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f'{pkg_parent}:{pkg_share}'
    )

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items(),
    )

    # --- Robot State Publisher ---
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': True,
        }],
    )

    # --- Spawn robot (delayed: Gazebo needs ~5s to load the world) ---
    spawn = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-world', 'warehouse',
                '-topic', 'robot_description',
                '-name', 'audix',
                '-x', '0.5', '-y', '1.1', '-z', '0.06',
                '-R', '0.0', '-P', '0.0', '-Y', '3.14159',
            ],
        )],
    )

    # --- GZ Bridge ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # joint_states: published directly to ROS by joint_state_broadcaster
            # via gz_ros2_control — do NOT bridge it here (gz.msgs.Model is wrong type)
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # 6 IR sensors
            '/ir_front/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_front_left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_front_right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_back/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
    )

    # --- Controller spawners ---
    # Delayed: controller_manager (inside gz_ros2_control) needs Gazebo fully loaded
    # + robot spawned before it appears. 10s is conservative but reliable.
    jsb_spawner = TimerAction(
        period=10.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            parameters=[{'use_sim_time': True}],
        )],
    )

    mecanum_spawner = TimerAction(
        period=11.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['mecanum_velocity_controller'],
            parameters=[{'use_sim_time': True}],
        )],
    )

    scissor_spawner = TimerAction(
        period=11.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['scissor_position_controller'],
            parameters=[{'use_sim_time': True}],
        )],
    )

    # --- EKF ---
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
    )

    # --- Mecanum kinematics ---
    mecanum_kin = Node(
        package='audix',
        executable='mecanum_kinematics.py',
        name='mecanum_kinematics',
        output='screen',
        parameters=[mission_config, {'use_sim_time': True}],
    )

    # --- Mission controller ---
    mission = Node(
        package='audix',
        executable='mission_controller.py',
        name='mission_controller',
        output='screen',
        parameters=[mission_config, {'use_sim_time': True}],
    )

    # --- Start/stop ---
    start_stop = Node(
        package='audix',
        executable='start_stop_node.py',
        name='start_stop_node',
        output='screen',
        parameters=[mission_config, {'use_sim_time': True}],
    )

    # --- Goal sender (delayed to let everything initialize) ---
    goal_sender = TimerAction(
        period=4.0,
        actions=[Node(
            package='audix',
            executable='goal_sender_node.py',
            name='goal_sender_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )],
    )

    # --- RViz debug view (robot path, planned path, waypoints, IR scans) ---
    rviz = TimerAction(
        period=2.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_audix_debug',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
        )],
    )

    obstacle_nodes = []

    return LaunchDescription([
        gz_resource,
        ign_resource,
        gazebo,
        rsp,
        bridge,
        spawn,
        jsb_spawner,
        mecanum_spawner,
        scissor_spawner,
        ekf,
        mecanum_kin,
        mission,
        start_stop,
        goal_sender,
        rviz,
    ] + obstacle_nodes)
