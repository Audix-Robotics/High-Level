"""Audix midterm launch built on top of the working scissor Gazebo stack."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('audix')
    pkg_parent = os.path.dirname(pkg_share)

    world_path = os.path.join(pkg_share, 'world', 'debug_empty.sdf')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    mission_config = os.path.join(pkg_share, 'config', 'mission_params.yaml')

    use_rviz = LaunchConfiguration('use_rviz')
    use_slider_gui = LaunchConfiguration('use_slider_gui')
    auto_start = LaunchConfiguration('auto_start')
    auto_send_goal = LaunchConfiguration('auto_send_goal')
    run_navigation = LaunchConfiguration('run_navigation')
    run_cardinal_test = LaunchConfiguration('run_cardinal_test')

    # --- Environment ---
    gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{pkg_parent}:{pkg_share}'
    )
    ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f'{pkg_parent}:{pkg_share}'
    )

    # --- Base Gazebo + scissor stack ---
    base_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'scissor_gazebo.launch.py')
        ]),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_slider_gui': use_slider_gui,
            'world_file': world_path,
            'world_name': 'warehouse',
            'spawn_x': '0.0',
            'spawn_y': '0.0',
            'spawn_z': '0.06',
            'spawn_yaw': '0.0',
        }.items(),
    )

    # --- GZ Bridge ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/ir_front/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_front_left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_front_right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_back/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
    )

    # --- EKF ---
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        condition=IfCondition(run_navigation),
    )

    # --- Mission controller ---
    mission = Node(
        package='audix',
        executable='mission_controller.py',
        name='mission_controller',
        output='screen',
        parameters=[mission_config, {'use_sim_time': True}],
        condition=IfCondition(run_navigation),
    )

    # --- Start/stop ---
    start_stop = Node(
        package='audix',
        executable='start_stop_node.py',
        name='start_stop_node',
        output='screen',
        parameters=[
            mission_config,
            {
                'use_sim_time': True,
                'auto_start': ParameterValue(auto_start, value_type=bool),
            },
        ],
        condition=IfCondition(run_navigation),
    )

    cardinal_test = TimerAction(
        period=5.0,
        actions=[Node(
            package='audix',
            executable='cardinal_motion_debug.py',
            name='cardinal_motion_debug',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(run_cardinal_test),
        )],
    )

    # --- Goal sender (delayed to let everything initialize) ---
    goal_sender = TimerAction(
        period=8.0,
        actions=[Node(
            package='audix',
            executable='goal_sender_node.py',
            name='goal_sender_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(auto_send_goal),
        )],
    )

    obstacle_nodes = []

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('use_slider_gui', default_value='false', description='Launch scissor slider GUI'),
        DeclareLaunchArgument('auto_start', default_value='false', description='Publish /robot_enable automatically'),
        DeclareLaunchArgument('auto_send_goal', default_value='false', description='Call /send_mission automatically'),
        DeclareLaunchArgument('run_navigation', default_value='false', description='Launch mission controller stack'),
        DeclareLaunchArgument('run_cardinal_test', default_value='true', description='Run pure cardinal motion test'),
        gz_resource,
        ign_resource,
        base_sim,
        bridge,
        ekf,
        mission,
        start_stop,
        goal_sender,
        cardinal_test,
    ] + obstacle_nodes)
