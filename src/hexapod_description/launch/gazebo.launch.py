
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_hexapod_description = get_package_share_directory('hexapod_description')

    # Parent directory of 'hexapod_description' in install/share
    # Required for Gazebo Sim to resolve 'model://hexapod_description/meshes/...'
    install_share_dir = os.path.dirname(pkg_hexapod_description)

    gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_share_dir
    )

    ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=install_share_dir
    )

    controllers_arg = DeclareLaunchArgument(
        name='load_controllers',
        default_value='true',
        description='Whether to automatically spawn ros2_control controllers'
    )
    load_controllers = LaunchConfiguration('load_controllers')

    robot_description_file = os.path.join(pkg_hexapod_description, 'urdf', 'hexapod.xacro')
    ros_gz_bridge_config = os.path.join(pkg_hexapod_description, 'config', 'ros_gz_bridge_gazebo.yaml')

    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', '/robot_description',
                    '-name', 'hexapod',
                    '-allow_renaming', 'false',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.25',
                    '-Y', '0.0'
                ],
                output='screen'
            )
        ]
    )

    # Automatically load ros2_control controllers after the robot is spawned
    load_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                condition=IfCondition(load_controllers),
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            )
        ]
    )

    load_forward_position_controller = TimerAction(
        period=6.0,
        actions=[
            Node(
                condition=IfCondition(load_controllers),
                package='controller_manager',
                executable='spawner',
                arguments=['forward_position_controller'],
                output='screen',
            )
        ]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        ign_resource_path,
        controllers_arg,
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        robot_state_publisher,
        load_joint_state_broadcaster,
        load_forward_position_controller,
    ])
