from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform publisher for livox_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='livox_tf_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'livox_frame']
        ),

        # Livox data acquisition node
        Node(
            package='lidar_get_points',
            executable='livox_node',
            name='livox_node',
            output='screen',
            parameters=[{
                'frame_id': 'livox_frame',
                'publish_freq': 10.0,  # 10Hz output
                'buffer_frames': 10,   # 10フレーム分を保持
                'integration_time_ms': 100,  # 100ms単位でフレーム統合
                'flip_yz': True  # Y-Z反転 (Lidar上下逆向き時はtrueに設定)
            }]
        )
    ])
