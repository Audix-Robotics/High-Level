import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('audix')
    pkg_parent = os.path.dirname(pkg_share)

    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    mission_config = os.path.join(pkg_share, 'config', 'mission_params.yaml')
    roamer_mission_config = os.path.join(pkg_share, 'config', 'arena_roamer_mission.yaml')
    world_config = os.path.join(pkg_share, 'config', 'arena_world.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'full_mission.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_spawn_panel = LaunchConfiguration('use_spawn_panel')
    world_name = LaunchConfiguration('world_name')

    # Arena roamer node (use ArenaRoamer for warehouse navigation + lift)
    roamer = Node(
        package='audix',
        executable='arena_roamer.py',
        name='arena_roamer',
        output='screen',
        parameters=[mission_config, roamer_mission_config, {'use_sim_time': True}],
    )

    world_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'full_mission_world.launch.py')
        ]),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_spawn_panel': use_spawn_panel,
            'world_name': world_name,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true', description='Launch Gazebo GUI client'),
        DeclareLaunchArgument('use_slider_gui', default_value='false', description='Launch scissor slider GUI'),
        DeclareLaunchArgument('use_spawn_panel', default_value='true', description='Launch the obstacle spawn preset panel'),
        DeclareLaunchArgument('world_name', default_value='warehouse', description='Gazebo world name for the arena sandbox'),
        DeclareLaunchArgument('auto_start', default_value='true', description='Publish /robot_enable automatically'),
        world_stack,
        roamer,
    ])
