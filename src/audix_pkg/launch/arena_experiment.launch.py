import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('audix')
    pkg_parent = os.path.dirname(pkg_share)

    world_path = os.path.join(pkg_share, 'world', 'debug_empty.sdf')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    experiment_config = os.path.join(pkg_share, 'config', 'arena_experiment_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'arena_experiment.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_spawn_panel = LaunchConfiguration('use_spawn_panel')
    world_name = LaunchConfiguration('world_name')

    gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{pkg_parent}:{pkg_share}',
    )
    ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f'{pkg_parent}:{pkg_share}',
    )

    base_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'scissor_gazebo.launch.py')
        ]),
        launch_arguments={
            'use_rviz': 'false',
                'use_gazebo_gui': 'true',
            'use_slider_gui': 'false',
            'world_file': world_path,
            'world_name': world_name,
            'spawn_x': '-3.6',
            'spawn_y': '0.0',
            'spawn_z': '0.06',
            'spawn_yaw': '3.141592653589793',
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/ir_front/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_front_left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_front_right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ir_back/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
    )

    mecanum_kinematics = Node(
        package='audix',
        executable='mecanum_kinematics.py',
        name='mecanum_kinematics',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
    )

    roamer = Node(
        package='audix',
        executable='arena_roamer.py',
        name='arena_roamer',
        output='screen',
        parameters=[experiment_config, {'use_sim_time': True}],
    )

    obstacle_manager = Node(
        package='audix',
        executable='arena_obstacle_manager.py',
        name='arena_obstacle_manager',
        output='screen',
        parameters=[experiment_config, {'use_sim_time': True, 'world_name': world_name}],
    )

    spawn_panel = Node(
        package='audix',
        executable='arena_spawn_panel.py',
        name='arena_spawn_panel',
        output='screen',
        parameters=[experiment_config],
        condition=IfCondition(use_spawn_panel),
    )


    arena_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='arena_alias_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'arena', 'arena10'],
        parameters=[{'use_sim_time': True}],
    )

    # RViz start is handled externally by scripts/clean_launch_arena.sh; leave the
    # `use_rviz` argument declared so callers can still pass it, but do not
    # launch RViz from this launch file to avoid duplicate windows.

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2 with click-to-spawn tooling'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true', description='Launch Gazebo GUI client'),
        DeclareLaunchArgument('use_spawn_panel', default_value='true', description='Launch the obstacle spawn preset panel'),
        DeclareLaunchArgument('world_name', default_value='debug_empty', description='Gazebo world name for the arena sandbox'),
        gz_resource,
        ign_resource,
        base_sim,
        bridge,
        ekf,
        arena_alias_tf,
        roamer,
        obstacle_manager,
        TimerAction(period=2.0, actions=[spawn_panel]),
    ])