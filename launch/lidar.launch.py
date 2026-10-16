
"""Launch the Benewake lidar driver and distance monitor."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Create the lidar driver and distance monitor nodes."""
    package_dir = get_package_share_directory(
        'benewake_ros2_driver'
    )
    config_file = os.path.join(
        package_dir, 'config', 'lidar.yaml'
    )

    lidar_node = Node(
        package='benewake_ros2_driver',
        executable='tfmini_s_driver',
        name='benewake_lidar_node',
        output='screen',
        parameters=[config_file],
    )

    distance_monitor_node = Node(
        package='benewake_ros2_driver',
        executable='distance_monitor',
        name='distance_monitor',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        lidar_node,
        distance_monitor_node,
    ])
