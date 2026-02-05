#!/bin/bash

# integration_time_ms のテストスクリプト
# 0.1s ~ 1.5s の範囲で設定を変更し、出力ポイント数を確認

set -e

CONFIG_FILE="/home/nokolat-1/2026_hikohobo_mid70/config/livox_node.yml"
LOG_DIR="/tmp/claude-1000/-home-nokolat-1-2026-hikohobo-mid70/c1347104-a167-4bb3-98bb-513545e12cdb/scratchpad/integration_test_logs"

# ログディレクトリを作成
mkdir -p "$LOG_DIR"

# テスト結果サマリーファイル
SUMMARY_FILE="$LOG_DIR/test_summary.txt"
echo "Integration Time Test Results" > "$SUMMARY_FILE"
echo "=============================" >> "$SUMMARY_FILE"
echo "" >> "$SUMMARY_FILE"

# オリジナルの設定をバックアップ
cp "$CONFIG_FILE" "${CONFIG_FILE}.backup"

# テストする integration_time_ms の値（ミリ秒）
test_values=(100 200 300 500 1000 1500)

echo "テスト開始: integration_time_ms を ${test_values[@]} ms でテスト"
echo ""

for time_ms in "${test_values[@]}"; do
    echo "========================================" | tee -a "$SUMMARY_FILE"
    echo "テスト: integration_time_ms = ${time_ms} ms" | tee -a "$SUMMARY_FILE"
    echo "========================================" | tee -a "$SUMMARY_FILE"

    # 設定ファイルを更新
    cat > "$CONFIG_FILE" << EOF
livox_node:
  frame_id: "livox_frame"
  publish_freq: 10.0          # Hz - 発行周波数（10Hz = 100ms周期）
  integration_time_ms: ${time_ms}    # ms - 統合時間（データ数 = 100k * (integration_time_ms / 1000) )
  flip_yz: true               # Y-Z反転オプション（LiDAR上下逆向き対応）
EOF

    # ワークスペースをビルド
    echo "ビルド中..." | tee -a "$SUMMARY_FILE"
    cd /home/nokolat-1/2026_hikohobo_mid70
    source /opt/ros/humble/setup.bash
    colcon build --packages-select lidar_get_points --cmake-args -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1

    # システムを起動（10秒間実行）
    echo "システム起動中 (10秒間実行)..." | tee -a "$SUMMARY_FILE"
    source install/setup.bash

    # ulimit設定を反映してノードを起動
    LOG_FILE="$LOG_DIR/test_${time_ms}ms.log"
    timeout 10s bash -c "ulimit -s unlimited && ros2 run lidar_get_points livox_node" > "$LOG_FILE" 2>&1 || true

    # ログから点群サイズを抽出
    echo "" | tee -a "$SUMMARY_FILE"
    echo "結果:" | tee -a "$SUMMARY_FILE"
    grep "Published point cloud" "$LOG_FILE" | tail -n 5 | tee -a "$SUMMARY_FILE"

    # 平均ポイント数を計算
    avg_points=$(grep "Published point cloud" "$LOG_FILE" | grep -oP 'with \K[0-9]+' | awk '{sum+=$1; count++} END {if(count>0) print sum/count; else print 0}')
    echo "" | tee -a "$SUMMARY_FILE"
    echo "平均ポイント数: ${avg_points}" | tee -a "$SUMMARY_FILE"
    echo "" | tee -a "$SUMMARY_FILE"

    sleep 2
done

# 設定を元に戻す
mv "${CONFIG_FILE}.backup" "$CONFIG_FILE"

echo "========================================" | tee -a "$SUMMARY_FILE"
echo "テスト完了" | tee -a "$SUMMARY_FILE"
echo "結果サマリー: $SUMMARY_FILE" | tee -a "$SUMMARY_FILE"
echo "詳細ログ: $LOG_DIR" | tee -a "$SUMMARY_FILE"
echo "========================================" | tee -a "$SUMMARY_FILE"

# サマリーを表示
cat "$SUMMARY_FILE"
