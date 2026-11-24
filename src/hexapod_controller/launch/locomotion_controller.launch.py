import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_hexapod_controller = get_package_share_directory('hexapod_controller')
    config_file = os.path.join(pkg_hexapod_controller, 'config', 'locomotion_controller.yaml')

    locomotion_engine_node = Node(
        package='hexapod_controller',
        executable='locomotion_engine_node',
        name='locomotion_engine_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        locomotion_engine_node,
    ])
