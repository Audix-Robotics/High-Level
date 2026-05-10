import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('audix')

    mission_config = os.path.join(pkg_share, 'config', 'common', 'mission_params.yaml')
    stress_roamer_config = os.path.join(pkg_share, 'config', 'scenarios', 'arena_stress_roamer.yaml')
    ir_adapter_config = os.path.join(pkg_share, 'config', 'common', 'arena_ir_state_adapter.yaml')

    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_slider_gui = LaunchConfiguration('use_slider_gui')
    world_name = LaunchConfiguration('world_name')

    roamer = Node(
        package='audix',
        executable='arena_roamer.py',
        name='arena_roamer',
        output='screen',
        parameters=[mission_config, stress_roamer_config, ir_adapter_config, {'use_sim_time': True}],
    )

    world_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'stacks', 'arena_stress_world.launch.py')
        ]),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_gazebo_gui': use_gazebo_gui,
            'use_slider_gui': use_slider_gui,
            'world_name': world_name,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true', description='Launch Gazebo GUI client'),
        DeclareLaunchArgument('use_slider_gui', default_value='false', description='Launch scissor slider GUI'),
        DeclareLaunchArgument('world_name', default_value='arena10', description='Gazebo world name for the obstacle stress course'),
        world_stack,
        roamer,
    ])