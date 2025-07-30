#!/usr/bin/env python3
"""
Launch file for the ROS2 controller node
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    dispatch_method_arg = DeclareLaunchArgument(
        'dispatch_method',
        default_value='shortest_queue',
        choices=['lottery', 'shortest_queue'],
        description='Dispatch method for worker selection'
    )
    
    # Controller node
    controller_node = Node(
        package='robopoint_controller',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[{
            'dispatch_method': LaunchConfiguration('dispatch_method')
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        dispatch_method_arg,
        controller_node
    ])
