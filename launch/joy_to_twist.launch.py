import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir,LaunchConfiguration

def generate_launch_description():
    joy_param_file = LaunchConfiguration(
        'joy_param_file',
        default=os.path.join(
            get_package_share_directory('joy_to_twist'),
            'config','joy.yaml'))
    description = LaunchDescription([
        Node(
            package='joy',
            node_executable='joy_node',
            node_name='joy_node',
            parameters=[joy_param_file],
            output='screen'),
        Node(
            package='joy_to_twist',
            node_executable='joy_to_twist_node',
            node_name='joy_to_twist_node',
            output='screen'),
    ])
    return description