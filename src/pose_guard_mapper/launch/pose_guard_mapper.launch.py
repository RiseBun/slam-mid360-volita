from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory("pose_guard_mapper")
    config_file = os.path.join(package_share, "config", "pose_guard_mapper.yaml")

    save_map_path = LaunchConfiguration("save_map_path")
    require_header_time_near_now = LaunchConfiguration("require_header_time_near_now")

    return LaunchDescription([
        DeclareLaunchArgument(
            "save_map_path",
            default_value=os.path.join(os.path.expanduser("~"), "slam_maps", "guarded_map.pcd"),
            description="Output path for the guarded PCD map",
        ),
        DeclareLaunchArgument(
            "require_header_time_near_now",
            default_value="true",
            description="Reject live messages whose header stamp is far from the local clock",
        ),
        Node(
            package="pose_guard_mapper",
            executable="pose_guard_mapper_node",
            name="pose_guard_mapper",
            output="screen",
            parameters=[
                config_file,
                {
                    "save_map_path": save_map_path,
                    "require_header_time_near_now": ParameterValue(
                        require_header_time_near_now, value_type=bool
                    ),
                },
            ],
        )
    ])
