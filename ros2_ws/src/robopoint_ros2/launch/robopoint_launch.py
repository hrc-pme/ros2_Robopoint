#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments (removed controller_url since we use ROS2 services now)
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
            'service_timeout',
            default_value='30.0',
            description='Timeout for ROS2 service calls in seconds'
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
        
        DeclareLaunchArgument(
            'launch_controller',
            default_value='true',
            description='Whether to launch the model controller node'
        ),
        
        DeclareLaunchArgument(
            'launch_workers',
            default_value='true', 
            description='Whether to launch worker nodes'
        ),
        
        DeclareLaunchArgument(
            'worker_models',
            default_value='vicuna-13b,llava-v1.5-13b',
            description='Comma-separated list of models for workers'
        ),

        # Model Controller Node (provides refresh_all_workers, list_models, get_worker_address services)
        # Node(
        #     package='robopoint_ros2',
        #     executable='model_controller_node',
        #     name='model_controller',
        #     output='screen',
        #     condition=IfCondition(LaunchConfiguration('launch_controller')),
        #     parameters=[{
        #         'worker_models': LaunchConfiguration('worker_models'),
        #     }]
        # ),
        
        # RoboPoint Node (main affordance processing node)
        Node(
            package='robopoint_ros2',
            executable='robopoint_node',
            name='robopoint_node',
            output='screen',
            parameters=[{
                'moderate': LaunchConfiguration('moderate'),
                'temperature': LaunchConfiguration('temperature'),
                'top_p': LaunchConfiguration('top_p'),
                'max_output_tokens': LaunchConfiguration('max_output_tokens'),
                'model_name': LaunchConfiguration('model_name'),
                'service_timeout': LaunchConfiguration('service_timeout'),
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
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            output='screen'
        ),
    ])