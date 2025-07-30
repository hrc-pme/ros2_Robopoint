# image_pub_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_image_pub_pkg',
            executable='image_pub',
            name='image_pub_node',
            output='screen'
        )
    ])