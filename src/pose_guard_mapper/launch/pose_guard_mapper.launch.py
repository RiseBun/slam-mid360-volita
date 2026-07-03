from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory("pose_guard_mapper")
    config_file = os.path.join(package_share, "config", "pose_guard_mapper.yaml")

    return LaunchDescription([
        Node(
            package="pose_guard_mapper",
            executable="pose_guard_mapper_node",
            name="pose_guard_mapper",
            output="screen",
            parameters=[config_file],
        )
    ])
