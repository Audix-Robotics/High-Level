import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('audix')
    pkg_parent = os.path.dirname(pkg_share)

    models_path = os.path.join(pkg_share, 'models')
    world_path = os.path.join(pkg_share, 'world', 'warehouse.sdf')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    world_config = os.path.join(pkg_share, 'config', 'arena_world.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'full_mission.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_spawn_panel = LaunchConfiguration('use_spawn_panel')
    world_name = LaunchConfiguration('world_name')

    gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{models_path}:{pkg_parent}:{pkg_share}',
    )
    ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f'{models_path}:{pkg_parent}:{pkg_share}',
    )

    base_sim = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_share, 'launch', 'scissor_gazebo.launch.py')
                ]),
                launch_arguments={
                    'use_rviz': 'false',
                    'use_gazebo_gui': 'true',
                    'use_slider_gui': 'false',
                    'world_file': world_path,
                    'world_name': world_name,
                    'spawn_x': '0.0',
                    'spawn_y': '-3.9',
                    'spawn_z': '0.06',
                    'spawn_yaw': '-1.570796',
                }.items(),
            )
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/ir_front/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_front_left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_front_right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_back/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
    )

    obstacle_manager = Node(
        package='audix',
        executable='arena_obstacle_manager.py',
        name='arena_obstacle_manager',
        output='screen',
        parameters=[world_config, {'use_sim_time': True, 'world_name': world_name}],
    )

    spawn_panel = Node(
        package='audix',
        executable='arena_spawn_panel.py',
        name='arena_spawn_panel',
        output='screen',
        parameters=[world_config],
        condition=IfCondition(use_spawn_panel),
    )

    warehouse_overlay = Node(
        package='audix',
        executable='warehouse_overlay_markers.py',
        name='warehouse_overlay_markers',
        output='screen',
        parameters=[{'use_sim_time': True, 'frame_id': 'odom'}],
    )

    arena_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='arena_alias_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'arena', 'arena10'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    start_spawn_panel = TimerAction(period=2.0, actions=[spawn_panel])

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_spawn_panel', default_value='true'),
        DeclareLaunchArgument('world_name', default_value='warehouse'),
        gz_resource,
        ign_resource,
        base_sim,
        bridge,
        ekf,
        obstacle_manager,
        start_spawn_panel,
        warehouse_overlay,
        arena_alias_tf,
        rviz_node,
    ])