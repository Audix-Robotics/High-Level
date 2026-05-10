import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('audix')
    pkg_parent = os.path.dirname(pkg_share)

    models_path = os.path.join(pkg_share, 'models')
    world_path = os.path.join(pkg_share, 'world', 'arena_stress_course.sdf')
    ekf_config = os.path.join(pkg_share, 'config', 'common', 'ekf.yaml')
    ir_adapter_config = os.path.join(pkg_share, 'config', 'common', 'arena_ir_state_adapter.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'full_mission.rviz')
    default_course_file = os.path.join(pkg_share, 'config', 'scenarios', 'arena_stress_course.json')

    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_slider_gui = LaunchConfiguration('use_slider_gui')
    world_name = LaunchConfiguration('world_name')
    course_file = LaunchConfiguration('course_file')

    gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{models_path}:{pkg_parent}:{pkg_share}',
    )
    ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f'{models_path}:{pkg_parent}:{pkg_share}',
    )

    base_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'stacks', 'scissor_gazebo.launch.py')
        ]),
        launch_arguments={
            'use_rviz': 'false',
            'use_gazebo_gui': use_gazebo_gui,
            'use_slider_gui': use_slider_gui,
            'publish_encoder_odom': 'true',
            'world_file': world_path,
            'world_name': world_name,
            'spawn_x': '0.0',
            'spawn_y': '-3.45',
            'spawn_z': '0.06',
            'spawn_yaw': '-1.570796',
        }.items(),
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

    ir_state_adapter = Node(
        package='audix',
        executable='arena_ir_state_adapter.py',
        name='arena_ir_state_adapter',
        output='screen',
        parameters=[ir_adapter_config, {'source_type': 'scan', 'use_sim_time': True}],
    )

    course_spawner = Node(
        package='audix',
        executable='arena_course_spawner.py',
        name='arena_course_spawner',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'world_name': world_name,
                'course_file': course_file,
                'dynamic_update_period': 0.12,
            }
        ],
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

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true'),
        DeclareLaunchArgument('use_slider_gui', default_value='false'),
        DeclareLaunchArgument('world_name', default_value='arena10'),
        DeclareLaunchArgument('course_file', default_value=default_course_file),
        gz_resource,
        ign_resource,
        base_sim,
        bridge,
        ir_state_adapter,
        ekf,
        course_spawner,
        arena_alias_tf,
        rviz_node,
    ])