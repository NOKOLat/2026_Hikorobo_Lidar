from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch all LiDAR components in a single container."""
    
    container = ComposableNodeContainer(
        name='lidar_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Component 1: Data acquisition
            ComposableNode(
                package='lidar_get_cloud_points',
                plugin='lidar_get_cloud_points::LidarGetCloudPointsComponent',
                name='lidar_get_cloud_points',
                parameters=[{
                    'publish_rate': 10.0,          # 10fps配信（メモリ削減）
                    'buffer_time_seconds': 0.1,    # 100msバッファ（メモリ削減）
                    'max_buffer_size': 5000,       # 最大5千点（メモリ削減・さらに安全に）
                }],
                extra_arguments=[{'use_intra_process_comms': False}]
            ),
            # Component 2: Preprocessing
            ComposableNode(
                package='lidar_preprocess',
                plugin='lidar_preprocess::LidarPreprocessComponent',
                name='lidar_preprocess',
                parameters=[{
                    'sor_mean_k': 50,
                    'sor_std_dev_mul_thresh': 1.0,

                    # Voxel Grid Filterの設定[m]（最小の点サイズを指定）
                    'voxel_leaf_size': 0.01,

                    ## 範囲指定（座標: Lidar基準、範囲外の点は削除される）

                    # Z軸の範囲指定[m](Lidar高さ方向)
                    'passthrough_min': -0.5,
                    'passthrough_max': 1.5,

                    # X軸の範囲指定[m](Lidar前後方向)
                    'passthrough_x_min': 0.0,
                    'passthrough_x_max': 3.0,

                    # Y軸の範囲指定[m](Lidar左右方向)
                    'passthrough_y_min': -1.0,
                    'passthrough_y_max': 1.0,
                }],
                extra_arguments=[{'use_intra_process_comms': False}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
