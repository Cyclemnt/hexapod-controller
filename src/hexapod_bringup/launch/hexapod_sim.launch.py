"""Launch file for Gazebo Sim simulation of the hexapod robot."""

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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Generate the launch description for Gazebo Sim and ros2_control."""
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_hexapod_description = get_package_share_directory('hexapod_description')
    pkg_hexapod_controller = get_package_share_directory('hexapod_controller')

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

    # Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(pkg_hexapod_description, 'worlds', 'hexapod_terrain.sdf'),
        description='Path to the Gazebo SDF world file'
    )
    world = LaunchConfiguration('world')

    headless_arg = DeclareLaunchArgument(
        name='headless',
        default_value='false',
        description='Run Gazebo headless (no GUI) if true'
    )
    headless = LaunchConfiguration('headless')

    load_controllers_arg = DeclareLaunchArgument(
        name='load_controllers',
        default_value='true',
        description='Whether to automatically spawn ros2_control controllers'
    )
    load_controllers = LaunchConfiguration('load_controllers')

    locomotion_config_arg = DeclareLaunchArgument(
        name='locomotion_config',
        default_value=os.path.join(pkg_hexapod_controller, 'config', 'locomotion_controller.yaml'),
        description='Path to locomotion controller parameters YAML'
    )
    locomotion_config = LaunchConfiguration('locomotion_config')

    # Robot Description (URDF/Xacro)
    robot_description_file = os.path.join(pkg_hexapod_description, 'urdf', 'hexapod.xacro')
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # Gazebo Sim: supports GUI (default) or headless via -s flag
    gz_args = PythonExpression([
        "'-s -r -v 3 ' + '", world,
        "' if '", headless, "' == 'true' else '-r -v 3 ' + '", world, "'"
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # Parameter Bridge: clock + IMU
    bridge_config_file = os.path.join(
        pkg_hexapod_description, 'config', 'ros_gz_bridge_gazebo.yaml'
    )
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config_file}, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Controllers config file
    controllers_file = os.path.join(
        pkg_hexapod_description, 'config', 'hexapod_controllers.yaml'
    )

    # Spawn Robot into Gazebo
    # Spawn at z=0.18m (close to nominal stance height ~0.11m so it drops gently into stance)
    spawn_robot = TimerAction(
        period=2.0,
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
                    '-z', '0.18',
                    '-Y', '0.0'
                ],
                output='screen'
            )
        ]
    )

    # Spawn ros2_control controllers
    load_joint_state_broadcaster = TimerAction(
        period=4.0,
        actions=[
            Node(
                condition=IfCondition(load_controllers),
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--param-file', controllers_file],
                output='screen',
            )
        ]
    )

    load_forward_position_controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                condition=IfCondition(load_controllers),
                package='controller_manager',
                executable='spawner',
                arguments=['forward_position_controller', '--param-file', controllers_file],
                output='screen',
            )
        ]
    )

    # Locomotion Engine Node
    locomotion_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='hexapod_controller',
                executable='locomotion_engine_node',
                name='locomotion_engine_node',
                output='screen',
                parameters=[
                    locomotion_config,
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )

    return LaunchDescription([
        gz_resource_path,
        ign_resource_path,
        use_sim_time_arg,
        world_arg,
        headless_arg,
        load_controllers_arg,
        locomotion_config_arg,
        gazebo,
        ros_gz_bridge,
        robot_state_publisher,
        spawn_robot,
        load_joint_state_broadcaster,
        load_forward_position_controller,
        locomotion_node,
    ])
