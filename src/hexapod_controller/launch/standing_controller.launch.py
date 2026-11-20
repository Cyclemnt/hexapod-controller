import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_hexapod_controller = get_package_share_directory('hexapod_controller')
    config_file = os.path.join(pkg_hexapod_controller, 'config', 'standing_controller.yaml')

    standing_controller_node = Node(
        package='hexapod_controller',
        executable='standing_controller_node',
        name='standing_controller_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        standing_controller_node,
    ])
