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

    scenario_config = os.path.join(pkg_share, 'config', 'scenarios', 'simple_cardinal.yaml')
    course_file = os.path.join(pkg_share, 'config', 'scenarios', 'simple_cardinal_course.json')
    rviz_config = os.path.join(pkg_share, 'rviz', 'full_mission.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_slider_gui = LaunchConfiguration('use_slider_gui')
    use_start_stop_gui = LaunchConfiguration('use_start_stop_gui')
    world_name = LaunchConfiguration('world_name')

    simple_brain = Node(
        package='audix',
        executable='simple_cardinal_brain.py',
        name='simple_cardinal_brain',
        output='screen',
        parameters=[scenario_config, {'use_sim_time': True}],
    )

    start_stop_gui = Node(
        package='audix',
        executable='start_stop_gui.py',
        name='start_stop_gui',
        output='screen',
        parameters=[{'use_sim_time': True, 'initial_state': False}],
        condition=IfCondition(use_start_stop_gui),
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

    world_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'stacks', 'arena_stress_world.launch.py')
        ]),
        launch_arguments={
            'use_rviz': 'false',
            'use_gazebo_gui': use_gazebo_gui,
            'use_slider_gui': use_slider_gui,
            'world_name': world_name,
            'course_file': course_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true', description='Launch Gazebo GUI client'),
        DeclareLaunchArgument('use_slider_gui', default_value='false', description='Launch scissor slider GUI'),
        DeclareLaunchArgument('use_start_stop_gui', default_value='true', description='Launch start/stop control window for /robot_enable'),
        DeclareLaunchArgument('world_name', default_value='arena10', description='Gazebo world name for the obstacle stress course'),
        world_stack,
        rviz_node,
        simple_brain,
        start_stop_gui,
    ])