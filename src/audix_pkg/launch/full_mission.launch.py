import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('audix')
    pkg_parent = os.path.dirname(pkg_share)

    models_path = os.path.join(pkg_share, 'models')
    world_path = os.path.join(pkg_share, 'world', 'warehouse.sdf')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    experiment_config = os.path.join(pkg_share, 'config', 'full_mission_params.yaml')
    mission_config = os.path.join(pkg_share, 'config', 'mission_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'full_mission.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_slider_gui = LaunchConfiguration('use_slider_gui')
    use_spawn_panel = LaunchConfiguration('use_spawn_panel')
    world_name = LaunchConfiguration('world_name')
    auto_start = LaunchConfiguration('auto_start')
    robot_namespace = LaunchConfiguration('robot_namespace')

    # Environment so Gazebo can find meshes
    gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{models_path}:{pkg_parent}:{pkg_share}',
    )
    ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f'{models_path}:{pkg_parent}:{pkg_share}',
    )

    # Include base scissor gazebo stack
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
            'spawn_x': '0.0',
            'spawn_y': '-3.9',
            'spawn_z': '0.06',
            'spawn_yaw': '-1.570796',
            'robot_namespace': robot_namespace,
        }.items(),
    )

    # Bridges
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

    # EKF
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
    )

    # Arena nodes

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

    # Arena roamer node (use ArenaRoamer for warehouse navigation + lift)
    roamer = Node(
        package='audix',
        executable='arena_roamer.py',
        name='arena_roamer',
        namespace=robot_namespace,
        output='screen',
        parameters=[mission_config, {'use_sim_time': True}],
        remappings=[
            ('/odometry/filtered', 'odometry/filtered'),
            ('/cmd_vel', 'cmd_vel'),
            ('/scissor_lift/slider', 'scissor_lift/slider'),
        ],
    )

    # start_stop and goal_sender removed: ArenaRoamer does not use /robot_enable or /send_mission

    # TF alias
    arena_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='arena_alias_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'arena', 'arena10'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    # Start/Stop node to publish /robot_enable
    start_stop_node = Node(
        package='audix',
        executable='start_stop_node.py',
        name='start_stop_node',
        output='screen',
        parameters=[{'use_sim_time': True, 'auto_start': auto_start}],
    )

    # Delay RViz startup slightly so other systems are ready
    # Match the wrapper script: give simulator/bridges time to initialize
    start_rviz_after = TimerAction(period=8.0, actions=[rviz_node])

    # Start/Stop GUI (the graphical frontend). This mirrors
    # scripts/clean_launch_arena.sh which launches the GUI after a short delay.
    # Use the workspace source path so the GUI is available when running from
    # the repo root (as the wrapper script does).
    start_stop_gui_path = os.path.join(os.getcwd(), 'src', 'audix_pkg', 'scripts', 'start_stop_gui.py')
    start_stop_gui_proc = ExecuteProcess(
        cmd=['python3', start_stop_gui_path],
        output='screen',
    )
    start_gui_after = TimerAction(period=8.0, actions=[start_stop_gui_proc])

    start_spawn_panel = TimerAction(period=2.0, actions=[spawn_panel])

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true', description='Launch Gazebo GUI client'),
        DeclareLaunchArgument('use_slider_gui', default_value='false', description='Launch scissor slider GUI'),
        DeclareLaunchArgument('use_spawn_panel', default_value='true', description='Launch the obstacle spawn preset panel'),
        DeclareLaunchArgument('world_name', default_value='warehouse', description='Gazebo world name for the arena sandbox'),
        DeclareLaunchArgument('auto_start', default_value='true', description='Publish /robot_enable automatically'),
        DeclareLaunchArgument('robot_namespace', default_value='audix', description='Namespace for this robot instance'),
        gz_resource,
        ign_resource,
        base_sim,
        bridge,
        ekf,
        obstacle_manager,
        start_spawn_panel,
        roamer,
        arena_alias_tf,
        start_stop_node,
        start_rviz_after,
        start_gui_after,
    ])
