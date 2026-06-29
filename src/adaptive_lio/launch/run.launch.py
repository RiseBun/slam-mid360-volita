import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = Path(get_package_share_directory('adaptive_lio'))
    rviz_config = os.path.join(str(pkg_dir), 'config', 'adaptive_lio.rviz')
    config_mode = LaunchConfiguration('config_mode')

    indoor_config = str(pkg_dir / 'config' / 'mapping_m.yaml')
    outdoor_config = str(pkg_dir / 'config' / 'mapping_high_altitude.yaml')
    orin_config = str(pkg_dir / 'config' / 'mapping_orin_nx.yaml')

    def launch_setup(context, *args, **kwargs):
        mode = config_mode.perform(context).strip().lower()
        if mode == 'outdoor':
            config_file = outdoor_config
        elif mode == 'orin':
            config_file = orin_config
        else:
            config_file = indoor_config

        return [
            Node(
                package='adaptive_lio',
                executable='adaptive_lio_node',
                name='adaptive_lio',
                output='screen',
                additional_env={'ADAPTIVE_LIO_CONFIG': config_file},
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_mode',
            default_value='indoor',
            description='adaptive_lio config profile: indoor, outdoor, orin',
        ),
        OpaqueFunction(function=launch_setup),
    ])
