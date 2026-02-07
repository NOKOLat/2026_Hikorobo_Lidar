#!/bin/bash

# Livox MID-70 データ取得・処理システム 起動スクリプト

# スクリプトのディレクトリを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

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
echo "Livox MID-70 データ取得・処理システム"
echo "=========================================="
echo ""
echo "起動オプション:"
echo "  1) データ取得のみ (lidar_get_points)"
echo "  2) 前処理ノード (lidar_preprocess)"
echo "  3) 背景差分ノード (lidar_background_diff)"
echo "  4) クラスタリングノード (lidar_clustering)"
echo "  5) 全て同時起動 (データ取得 → 前処理 → 背景差分 → クラスタリング)"
echo ""
read -p "選択してください (1-5): " choice

case $choice in
    1)
        echo "データ取得モード起動..."
        ros2 launch lidar_get_points livox.launch.py
        ;;
    2)
        echo "前処理ノード起動..."
        ros2 run lidar_preprocess preprocess_node
        ;;
    3)
        echo "背景差分ノード起動..."
        ros2 run lidar_background_diff background_diff_node
        ;;
    4)
        echo "クラスタリングノード起動..."
        ros2 run lidar_clustering clustering_node
        ;;
    5)
        echo "全て同時起動..."
        # lidar_get_pointsをバックグラウンドで起動
        ros2 launch lidar_get_points livox.launch.py &
        GET_POINTS_PID=$!
        sleep 2
        # 前処理ノードをバックグラウンドで起動
        ros2 run lidar_preprocess preprocess_node &
        PREPROCESS_PID=$!
        sleep 2
        # 背景差分ノードをバックグラウンドで起動
        ros2 run lidar_background_diff background_diff_node &
        BACKGROUND_DIFF_PID=$!
        sleep 2
        # クラスタリングノードを起動（フォアグラウンド）
        ros2 run lidar_clustering clustering_node
        ;;
    *)
        echo "無効な選択です"
        exit 1
        ;;
esac
