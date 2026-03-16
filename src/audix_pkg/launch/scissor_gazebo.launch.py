import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    robot_description = Command(['xacro ', model_path])

    # ── Environment so Gazebo can find meshes ─────────────────────────
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{pkg_share_parent}:{pkg_share}',
    )
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f'{pkg_share_parent}:{pkg_share}',
    )

    # ── Robot State Publisher ─────────────────────────────────────────
    # Parses the URDF and publishes TF for fixed joints.
    # Moveable joint TF comes from /joint_states (joint_state_broadcaster).
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(robot_description, value_type=str)},
            {'use_sim_time': True},
        ],
    )

    # ── Static TF: world → odom ─────────────────────────────────────
    # diff_drive_controller publishes odom → base_link. Publishing world → base_link
    # would give base_link two parents and break TF. This keeps a valid chain:
    # world → odom → base_link → ...
    world_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    # ── Gazebo Sim ────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': f'-r -v 2 {world_file_path}'}.items(),
    )

    # ── Spawn robot into Gazebo ───────────────────────────────────────
    # Uses -string so the xacro-processed URDF (with ros2_control tags) is passed
    # directly; gz_ros2_control plugin reads robot_description from RSP.
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

    # ── Clock bridge (sim time only) ──────────────────────────────────
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # ── Controller spawners ───────────────────────────────────────────
    # gz_ros2_control plugin creates /controller_manager when the robot spawns.
    # Each spawner retries for up to 30 s waiting for controller_manager to appear.
    #
    # joint_state_broadcaster  → publishes /joint_states → RSP → TF  ✅
    # diff_drive_controller    → subscribes /cmd_vel, drives wheels
    # scissor_position_controller → subscribes /scissor_position_controller/commands
    #                               (Float64MultiArray), moves scissor+camera joints
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--param-file', controllers_yaml,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'diff_drive_controller',
            '--param-file', controllers_yaml,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
    )

    scissor_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'scissor_position_controller',
            '--param-file', controllers_yaml,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
    )

    # ── Scissor mapper (converts single slider → all joint commands) ──
    scissor_mapper_node = Node(
        package='audix',
        executable='scissor_lift_mapper.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── Optional slider GUI ───────────────────────────────────────────
    scissor_slider_node = Node(
        package='audix',
        executable='scissor_slider_gui.py',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_slider_gui')),
    )

    # ── Optional RViz ─────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    start_rviz_after_diff = RegisterEventHandler(
        OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[rviz_node],
        )
    )

    start_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    start_diff_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    start_scissor_after_diff = RegisterEventHandler(
        OnProcessExit(
            target_action=diff_drive_spawner,
            on_exit=[scissor_controller_spawner],
        )
    )

    start_mapper_after_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=scissor_controller_spawner,
            on_exit=[scissor_mapper_node, scissor_slider_node],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('use_slider_gui', default_value='true', description='Launch scissor slider GUI'),

        gz_resource_path,
        ign_resource_path,
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        world_to_odom_node,
        bridge_clock,
        start_jsb_after_spawn,
        start_diff_after_jsb,
        start_scissor_after_diff,
        start_mapper_after_controller,
        start_rviz_after_diff,
    ])
