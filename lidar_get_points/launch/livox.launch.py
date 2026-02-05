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
            # パラメータはYAMLファイルから読み込まれます
            # (lidar_get_points/src/livox_node.cpp の LoadConfigFromYAML()参照)
            # コマンドライン引数でオーバーライド可能:
            # ros2 launch lidar_get_points livox.launch.py integration_time_ms:=500
        )
    ])
