import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('scancontext_init_localizer')
    config_file = os.path.join(pkg_dir, 'config', 'sc_builder.yaml')
    
    return LaunchDescription([
        Node(
            package='scancontext_init_localizer',
            executable='sc_map_builder_node',
            name='sc_map_builder',
            output='screen',
            parameters=[config_file]
        )
    ])
