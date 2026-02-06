#!/bin/bash

# integration_time_ms の簡易テストスクリプト
# 100ms, 500ms, 1000ms の3つでテスト

set -e

CONFIG_FILE="/home/nokolat-1/2026_hikohobo_mid70/config/livox_node.yml"
LOG_DIR="/tmp/claude-1000/-home-nokolat-1-2026-hikohobo-mid70/c1347104-a167-4bb3-98bb-513545e12cdb/scratchpad/integration_test"

mkdir -p "$LOG_DIR"

# オリジナルの設定をバックアップ
cp "$CONFIG_FILE" "${CONFIG_FILE}.backup"

# テストする integration_time_ms の値（ミリ秒）
test_values=(100 500 1000)

echo "========================================"
echo "Integration Time Test - Quick Version"
echo "========================================"
echo ""

for time_ms in "${test_values[@]}"; do
    echo "----------------------------------------"
    echo "Testing: integration_time_ms = ${time_ms} ms"
    echo "----------------------------------------"

    # 設定ファイルを更新
    cat > "$CONFIG_FILE" << EOF
livox_node:
  frame_id: "livox_frame"
  publish_freq: 10.0          # Hz - 発行周波数（10Hz = 100ms周期）
  integration_time_ms: ${time_ms}    # ms - 統合時間
  flip_yz: true               # Y-Z反転オプション（LiDAR上下逆向き対応）
EOF

    # システムを起動（8秒間実行）
    echo "Starting system (running for 8 seconds)..."
    cd /home/nokolat-1/2026_hikohobo_mid70
    source /opt/ros/humble/setup.bash
    source install/setup.bash

    LOG_FILE="$LOG_DIR/test_${time_ms}ms.log"
    timeout 8s bash -c "ulimit -s unlimited && ros2 run lidar_get_points livox_node" > "$LOG_FILE" 2>&1 || true

    echo ""
    echo "Results:"
    grep "Published:" "$LOG_FILE" | tail -n 5 || echo "No published messages found"

    # 平均ポイント数を計算
    avg_points=$(grep "Published:" "$LOG_FILE" | grep -oP 'Published: \K[0-9]+' | awk '{sum+=$1; count++} END {if(count>0) printf "%.0f", sum/count; else print 0}')

    if [ -n "$avg_points" ] && [ "$avg_points" != "0" ]; then
        echo ""
        echo "Average points: ${avg_points}"

        # 理論値との比較（Livox MID-70 は約100k points/sec）
        expected=$((time_ms * 100))
        echo "Expected points (approx): ${expected}"

        # パーセンテージ計算
        percentage=$(awk "BEGIN {printf \"%.1f\", ($avg_points / $expected) * 100}")
        echo "Actual vs Expected: ${percentage}%"
    else
        echo "WARNING: No data collected!"
    fi

    echo ""
    sleep 1
done

# 設定を元に戻す
mv "${CONFIG_FILE}.backup" "$CONFIG_FILE"

echo "========================================"
echo "Test Complete"
echo "Log directory: $LOG_DIR"
echo "========================================"
