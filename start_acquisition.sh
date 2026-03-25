#!/bin/bash

# データ取得 + 前処理ノード 起動スクリプト
# lidar_get_points + lidar_preprocess のみを起動します

# スクリプトのディレクトリを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Livox SDK のスタックサイズ問題対策
ulimit -s unlimited

# ROS環境のセットアップ
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "Error: install/setup.bash not found."
    echo "ビルドが必要です。以下を実行してください:"
    echo "  ./build.sh"
    exit 1
fi

echo "=========================================="
echo "データ取得 + 前処理ノード 起動"
echo "=========================================="
echo ""

# lidar_get_points をバックグラウンドで起動
echo "データ取得ノード起動 (lidar_get_points)..."
ros2 launch lidar_get_points livox.launch.py &
GET_POINTS_PID=$!

sleep 2

# lidar_preprocess をフォアグラウンドで起動
echo "前処理ノード起動 (lidar_preprocess)..."
ros2 run lidar_preprocess preprocess_node

# フォアグラウンドプロセスが終了したらバックグラウンドも終了
kill $GET_POINTS_PID 2>/dev/null
wait $GET_POINTS_PID 2>/dev/null
