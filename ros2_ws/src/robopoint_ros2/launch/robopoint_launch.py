#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'controller_url',
            default_value='http://localhost:21001',
            description='URL of the model controller server'
        ),
        
        DeclareLaunchArgument(
            'moderate',
            default_value='false',
            description='Enable content moderation'
        ),
        
        DeclareLaunchArgument(
            'temperature',
            default_value='1.0',
            description='Model temperature parameter'
        ),
        
        DeclareLaunchArgument(
            'top_p',
            default_value='0.7',
            description='Model top_p parameter'
        ),
        
        DeclareLaunchArgument(
            'max_output_tokens',
            default_value='512',
            description='Maximum output tokens'
        ),
        
        DeclareLaunchArgument(
            'model_name',
            default_value='',
            description='Specific model name to use'
        ),
        
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera/image_raw',
            description='Camera image topic'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to launch RViz'
        ),

        
        # RoboPoint Node
        Node(
            package='robopoint_ros2',
            executable='robopoint_node',
            name='robopoint_node',
            output='screen',
            parameters=[{
                'controller_url': LaunchConfiguration('controller_url'),
                'moderate': LaunchConfiguration('moderate'),
                'temperature': LaunchConfiguration('temperature'),
                'top_p': LaunchConfiguration('top_p'),
                'max_output_tokens': LaunchConfiguration('max_output_tokens'),
                'model_name': LaunchConfiguration('model_name'),
            }],
            remappings=[
                ('camera/image_raw', LaunchConfiguration('camera_topic')),
            ]
        ),
        
        # Optional: RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '$(find robopoint_ros2)/rviz/robopoint_config.rviz'],
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        ),
    ])