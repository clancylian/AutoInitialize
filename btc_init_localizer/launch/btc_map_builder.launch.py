import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('btc_init_localizer')
    config_file = os.path.join(pkg_dir, 'config', 'btc_builder.yaml')
    
    return LaunchDescription([
        Node(
            package='btc_init_localizer',
            executable='btc_map_builder_node',
            name='btc_map_builder',
            output='screen',
            parameters=[config_file]
        )
    ])
