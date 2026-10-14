
"""Launch the Benewake lidar driver with YAML parameters."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    """Create the launch description for the lidar driver."""
    package_dir = get_package_share_directory(
        'benewake_ros2_driver'
    )
    config_file = os.path.join(
        package_dir,
        'config',
        'lidar.yaml',
    )

    lidar_node = Node(
        package='benewake_ros2_driver',
        executable='tfmini_s_driver',
        name='benewake_lidar_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([lidar_node])
