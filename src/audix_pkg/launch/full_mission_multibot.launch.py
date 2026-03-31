import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('audix')
    pkg_parent = os.path.dirname(pkg_share)
    world_path = os.path.join(pkg_share, 'world', 'warehouse.sdf')
    models_path = os.path.join(pkg_share, 'models')
    warehouse_lanes_config = os.path.join(pkg_share, 'config', 'warehouse_lanes.yaml')
    warehouse_missions_config = os.path.join(pkg_share, 'config', 'warehouse_missions.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'full_mission.rviz')

    num_robots = LaunchConfiguration('num_robots')
    use_fleet_manager = LaunchConfiguration('use_fleet_manager')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_obstacle_manager = LaunchConfiguration('use_obstacle_manager')
    world_name = LaunchConfiguration('world_name')

    gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{models_path}:{pkg_parent}:{pkg_share}',
    )
    ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f'{models_path}:{pkg_parent}:{pkg_share}',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r -s -v 2 ', world_path]}.items(),
    )

    gazebo_gui = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['gz', 'sim', '-g', '-v', '2'],
                output='screen',
                condition=IfCondition(use_gazebo_gui),
            )
        ],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    odom_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', world_name, 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    obstacle_manager = Node(
        package='audix',
        executable='arena_obstacle_manager.py',
        name='arena_obstacle_manager',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'full_mission_params.yaml'),
            {
                'use_sim_time': True,
                'world_name': world_name,
                'click_topic': '/arena_clicked_point',
            },
        ],
        condition=IfCondition(use_obstacle_manager),
    )

    fleet_manager = Node(
        package='audix',
        executable='warehouse_fleet_manager.py',
        name='warehouse_fleet_manager',
        output='screen',
        parameters=[
            {
                'warehouse_config_path': warehouse_lanes_config,
                'missions_config_path': warehouse_missions_config,
                'world_name': world_name,
                'auto_spawn_default_fleet': False,
                'auto_spawn_num_robots': num_robots,
                'use_sim_time': True,
            }
        ],
        condition=IfCondition(use_fleet_manager),
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

    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='0', description='Number of robots to auto-spawn in fleet'),
        DeclareLaunchArgument('use_fleet_manager', default_value='true', description='Launch the fleet manager node'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true', description='Launch Gazebo GUI client'),
        DeclareLaunchArgument('use_obstacle_manager', default_value='true', description='Launch obstacle manager for RViz click spawning'),
        DeclareLaunchArgument('world_name', default_value='warehouse', description='Gazebo world name'),
        gz_resource,
        ign_resource,
        gazebo,
        gazebo_gui,
        clock_bridge,
        odom_frame,
        obstacle_manager,
        fleet_manager,
        rviz_node,
    ])