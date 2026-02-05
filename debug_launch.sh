#!/bin/bash

# デバッグログ付きで起動するスクリプト

cd /home/nokolat-1/2026_hikohobo_mid70

# ROS環境のセットアップ
source install/setup.bash

echo "=========================================="
echo "デバッグモードで起動"
echo "=========================================="

# デバッグログレベルで起動
ros2 launch lidar_get_points livox.launch.py --ros-args --log-level debug
