from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lab6_pkg'),
        'config',
        'rrt_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='lab6_pkg',
            executable='rrt_node',
            name='rrt_node',
            output='screen',
            parameters=[config]
        )
    ])
