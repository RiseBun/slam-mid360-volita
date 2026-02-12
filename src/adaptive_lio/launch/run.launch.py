import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('adaptive_lio')
    rviz_config = os.path.join(pkg_dir, 'config', 'adaptive_lio.rviz')

    return LaunchDescription([
        Node(
            package='adaptive_lio',
            executable='adaptive_lio_node',
            name='adaptive_lio',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
