# launch/model_worker.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 聲明 launch 參數
        DeclareLaunchArgument(
            'model_path',
            default_value='facebook/opt-350m',
            description='Path to the model'
        ),
        DeclareLaunchArgument(
            'model_base',
            default_value='',
            description='Base model path'
        ),
        DeclareLaunchArgument(
            'model_name',
            default_value='',
            description='Model name'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cuda',
            description='Device to run the model on'
        ),
        DeclareLaunchArgument(
            'load_8bit',
            default_value='false',
            description='Load model in 8-bit precision'
        ),
        DeclareLaunchArgument(
            'load_4bit',
            default_value='false',
            description='Load model in 4-bit precision'
        ),
        DeclareLaunchArgument(
            'use_flash_attn',
            default_value='false',
            description='Use Flash Attention'
        ),
        DeclareLaunchArgument(
            'limit_model_concurrency',
            default_value='5',
            description='Maximum concurrent model requests'
        ),
        DeclareLaunchArgument(
            'heartbeat_interval',
            default_value='10.0',
            description='Heartbeat interval in seconds'
        ),
        DeclareLaunchArgument(
            'controller_topic',
            default_value='/model_controller',
            description='Controller topic prefix'
        ),
        
        # 模型工作器節點
        Node(
            package='robopoint_worker',
            executable='ros2_model_worker',
            name='model_worker',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'model_base': LaunchConfiguration('model_base'),
                'model_name': LaunchConfiguration('model_name'),
                'device': LaunchConfiguration('device'),
                'load_8bit': LaunchConfiguration('load_8bit'),
                'load_4bit': LaunchConfiguration('load_4bit'),
                'use_flash_attn': LaunchConfiguration('use_flash_attn'),
                'limit_model_concurrency': LaunchConfiguration('limit_model_concurrency'),
                'heartbeat_interval': LaunchConfiguration('heartbeat_interval'),
                'controller_topic': LaunchConfiguration('controller_topic'),
            }],
            # 設定大的 buffer size 用於處理大型回應
            remappings=[
                # 可以在這裡重新映射 topic 名稱
            ]
        ),
    ])


# 範例使用多個模型工作器的 launch 檔案
# def generate_launch_description_multi_worker():
#     """啟動多個模型工作器的範例"""
#     return LaunchDescription([
#         # Worker 1
#         Node(
#             package='robopoint_worker',
#             executable='ros2_model_worker',
#             name='model_worker_1',
#             namespace='worker1',
#             output='screen',
#             parameters=[{
#                 'model_path': 'path/to/model1',
#                 'device': 'cuda:0',
#                 'limit_model_concurrency': 3,
#             }]
#         ),
        
#         # Worker 2
#         # Node(
#         #     package='robopoint_worker',
#         #     executable='model_worker',
#         #     name='model_worker_2',
#         #     namespace='worker2',
#         #     output='screen',
#         #     parameters=[{
#         #         'model_path': 'path/to/model2',
#         #         'device': 'cuda:1',
#         #         'limit_model_concurrency': 3,
#         #     }]
#         # ),
#     ])
