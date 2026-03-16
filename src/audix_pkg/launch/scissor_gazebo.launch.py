import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('audix')
    pkg_share_parent = os.path.dirname(pkg_share)

    model_path = os.path.join(pkg_share, 'urdf', 'audix.urdf')
    world_file_path = os.path.join(pkg_share, 'world', 'empty_world.sdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')

    robot_description = Command(['xacro ', model_path])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(robot_description, value_type=str)},
            {'use_sim_time': True},
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r -v 4 {world_file_path}'}.items(),
    )

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{pkg_share_parent}:{pkg_share}',
    )

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f'{pkg_share_parent}:{pkg_share}',
    )

    bridge_gz = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/audix/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/audix/joint/bottom_stud_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/audix/joint/left_joint1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/audix/joint/right_joint1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/audix/joint/left_joint2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/audix/joint/right_joint2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/audix/joint/left_joint3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/audix/joint/right_joint3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/audix/joint/left_joint4/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/audix/joint/right_joint4/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/audix/joint/left_joint5/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/audix/joint/right_joint5/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/audix/joint/left_joint6/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/audix/joint/right_joint6/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/audix/joint/camera_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[('/world/empty/model/audix/joint_state', '/joint_states')],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'empty',
            '-string', robot_description,
            '-name', 'audix',
            '-x', '0.0', '-y', '0.0', '-z', '0.025',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
        ],
    )

    world_to_base_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
    )

    scissor_mapper_node = Node(
        package='audix',
        executable='scissor_lift_mapper.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    scissor_slider_node = Node(
        package='audix',
        executable='scissor_slider_gui.py',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_slider_gui')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('use_slider_gui', default_value='true', description='Launch single scissor slider GUI'),
        gz_resource_path,
        ign_resource_path,
        gazebo,
        bridge_gz,
        robot_state_publisher_node,
        spawn_entity,
        world_to_base_node,
        TimerAction(period=2.0, actions=[scissor_mapper_node]),
        TimerAction(period=3.0, actions=[scissor_slider_node]),
        rviz_node,
    ])
