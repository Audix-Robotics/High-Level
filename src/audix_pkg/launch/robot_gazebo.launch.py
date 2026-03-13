import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('audix')
    pkg_share_parent = os.path.dirname(pkg_share)
    model_path = os.path.join(pkg_share, 'urdf', 'audix.urdf')
    world_file_path = os.path.join(pkg_share, 'world', 'my_world.sdf')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

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
            '/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[('/world/empty/model/audix/joint_state', '/joint_states')],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'empty',
            '-topic', 'robot_description',
            '-name', 'audix',
            '-x', '0.0', '-y', '0.0', '-z', '0.05',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
        ],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
    )

    return LaunchDescription([
        gz_resource_path,
        ign_resource_path,
        gazebo,
        bridge_gz,
        robot_state_publisher_node,
        spawn_entity,
        ekf_node,
    ])
