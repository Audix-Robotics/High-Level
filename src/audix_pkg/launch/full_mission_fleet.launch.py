import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('audix')
    warehouse_lanes_config = os.path.join(pkg_share, 'config', 'warehouse_lanes.yaml')
    warehouse_missions_config = os.path.join(pkg_share, 'config', 'warehouse_missions.yaml')
    warehouse_robot_params = os.path.join(pkg_share, 'config', 'warehouse_robot_params.yaml')

    num_robots = LaunchConfiguration('num_robots')
    use_fleet_manager = LaunchConfiguration('use_fleet_manager')
    use_spawn_panel = LaunchConfiguration('use_spawn_panel')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_obstacle_manager = LaunchConfiguration('use_obstacle_manager')
    world_name = LaunchConfiguration('world_name')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'full_mission_multibot.launch.py')
        ]),
        launch_arguments={
            'num_robots': num_robots,
            'use_fleet_manager': use_fleet_manager,
            'use_rviz': use_rviz,
            'use_gazebo_gui': use_gazebo_gui,
            'use_obstacle_manager': use_obstacle_manager,
            'world_name': world_name,
        }.items(),
    )

    mission_manager = Node(
        package='audix',
        executable='warehouse_mission_manager.py',
        name='warehouse_mission_manager',
        output='screen',
        parameters=[
            {
                'missions_config_path': warehouse_missions_config,
                'warehouse_config_path': warehouse_lanes_config,
                'robot_params_path': warehouse_robot_params,
                'use_sim_time': True,
            }
        ],
    )

    spawn_panel = Node(
        package='audix',
        executable='arena_spawn_panel_multibot.py',
        name='warehouse_spawn_panel',
        output='screen',
        parameters=[
            {
                'warehouse_config_path': warehouse_lanes_config,
                'missions_config_path': warehouse_missions_config,
                'use_sim_time': True,
            }
        ],
        condition=IfCondition(use_spawn_panel),
    )

    return LaunchDescription([
        DeclareLaunchArgument('num_robots', default_value='0', description='Number of robots to auto-spawn in fleet'),
        DeclareLaunchArgument('use_fleet_manager', default_value='true', description='Launch the fleet manager node'),
        DeclareLaunchArgument('use_spawn_panel', default_value='true', description='Launch the fleet GUI'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true', description='Launch Gazebo GUI client'),
        DeclareLaunchArgument('use_obstacle_manager', default_value='true', description='Launch obstacle manager for RViz click spawning'),
        DeclareLaunchArgument('world_name', default_value='warehouse', description='Gazebo world name'),
        base_launch,
        mission_manager,
        spawn_panel,
    ])