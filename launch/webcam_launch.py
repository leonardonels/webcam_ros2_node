from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('webcam_publisher'),
        'config',
        'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='webcam_publisher',
            executable='webcam_node',
            name='webcam_publisher_node',
            output='screen',
            parameters=[params_file]
        )
    ])
