#!/bin/bash

# integration_time_ms のテストスクリプト（launchファイル使用版）
# 0.1s ~ 1.5s の範囲で設定を変更し、出力ポイント数を確認

set -e

CONFIG_FILE="/home/nokolat-1/2026_hikohobo_mid70/config/livox_node.yml"
LOG_DIR="/tmp/claude-1000/-home-nokolat-1-2026-hikohobo-mid70/c1347104-a167-4bb3-98bb-513545e12cdb/scratchpad/integration_test_launch"

mkdir -p "$LOG_DIR"

# オリジナルの設定をバックアップ
cp "$CONFIG_FILE" "${CONFIG_FILE}.backup"

# テストする integration_time_ms の値（ミリ秒）
# 0.1s ~ 1.5s の範囲
test_values=(100 200 300 500 800 1000 1500)

echo "=========================================="
echo "Integration Time Test (with Launch File)"
echo "=========================================="
echo ""
echo "Testing values: ${test_values[@]} ms"
echo ""

# 結果サマリーファイル
SUMMARY_FILE="$LOG_DIR/summary.txt"
echo "Integration Time Test Results" > "$SUMMARY_FILE"
echo "=============================" >> "$SUMMARY_FILE"
echo "" >> "$SUMMARY_FILE"
printf "%-10s | %-15s | %-15s | %-10s | %-10s\n" "Time(ms)" "Avg Points" "Expected" "Ratio" "Frames" >> "$SUMMARY_FILE"
printf "%-10s-+-%-15s-+-%-15s-+-%-10s-+-%-10s\n" "----------" "---------------" "---------------" "----------" "----------" >> "$SUMMARY_FILE"

for time_ms in "${test_values[@]}"; do
    echo "========================================" | tee -a "$LOG_DIR/console.log"
    echo "Testing: integration_time_ms = ${time_ms} ms" | tee -a "$LOG_DIR/console.log"
    echo "========================================" | tee -a "$LOG_DIR/console.log"

    # 設定ファイルを更新
    cat > "$CONFIG_FILE" << EOF
livox_node:
  frame_id: "livox_frame"
  publish_freq: 10.0          # Hz - 発行周波数（10Hz = 100ms周期）
  integration_time_ms: ${time_ms}    # ms - 統合時間
  flip_yz: true               # Y-Z反転オプション（LiDAR上下逆向き対応）
EOF

    # システムを起動（10秒間実行）
    echo "Starting system (10 seconds)..." | tee -a "$LOG_DIR/console.log"
    cd /home/nokolat-1/2026_hikohobo_mid70
    source /opt/ros/humble/setup.bash
    source install/setup.bash

    LOG_FILE="$LOG_DIR/test_${time_ms}ms.log"

    # ulimit設定を反映してlaunchファイルで起動
    timeout 10s bash -c "ulimit -s unlimited && ros2 launch lidar_get_points livox.launch.py" > "$LOG_FILE" 2>&1 || true

    echo "" | tee -a "$LOG_DIR/console.log"

    # ログから発行メッセージを抽出
    if grep -q "Published:" "$LOG_FILE"; then
        echo "Results (last 5 publishes):" | tee -a "$LOG_DIR/console.log"
        grep "Published:" "$LOG_FILE" | tail -n 5 | tee -a "$LOG_DIR/console.log"

        # 統計を計算
        avg_points=$(grep "Published:" "$LOG_FILE" | grep -oP 'Published: \K[0-9]+' | awk '{sum+=$1; count++} END {if(count>0) printf "%.0f", sum/count; else print 0}')
        avg_frames=$(grep "Published:" "$LOG_FILE" | grep -oP 'from \K[0-9]+' | awk '{sum+=$1; count++} END {if(count>0) printf "%.1f", sum/count; else print 0}')

        if [ -n "$avg_points" ] && [ "$avg_points" != "0" ]; then
            # 理論値との比較（Livox MID-70 は約100k points/sec = 100 points/ms）
            expected=$((time_ms * 100))
            ratio=$(awk "BEGIN {printf \"%.2f\", $avg_points / $expected}")

            echo "" | tee -a "$LOG_DIR/console.log"
            echo "Statistics:" | tee -a "$LOG_DIR/console.log"
            echo "  Average points: ${avg_points}" | tee -a "$LOG_DIR/console.log"
            echo "  Average frames: ${avg_frames}" | tee -a "$LOG_DIR/console.log"
            echo "  Expected points (approx): ${expected}" | tee -a "$LOG_DIR/console.log"
            echo "  Ratio (actual/expected): ${ratio}" | tee -a "$LOG_DIR/console.log"

            # サマリーに追加
            printf "%-10s | %-15s | %-15s | %-10s | %-10s\n" "$time_ms" "$avg_points" "$expected" "$ratio" "$avg_frames" >> "$SUMMARY_FILE"
        else
            echo "WARNING: No valid data!" | tee -a "$LOG_DIR/console.log"
            printf "%-10s | %-15s | %-15s | %-10s | %-10s\n" "$time_ms" "NO DATA" "-" "-" "-" >> "$SUMMARY_FILE"
        fi
    else
        echo "WARNING: No published messages found!" | tee -a "$LOG_DIR/console.log"

        # デバイス接続状態を確認
        if grep -q "Device broadcast" "$LOG_FILE"; then
            echo "  Device was detected but no data published" | tee -a "$LOG_DIR/console.log"
        elif grep -q "discovering devices" "$LOG_FILE"; then
            echo "  Searching for devices (device may not be connected)" | tee -a "$LOG_DIR/console.log"
        fi

        printf "%-10s | %-15s | %-15s | %-10s | %-10s\n" "$time_ms" "NO DATA" "-" "-" "-" >> "$SUMMARY_FILE"
    fi

    echo "" | tee -a "$LOG_DIR/console.log"
    sleep 1
done

# 設定を元に戻す
mv "${CONFIG_FILE}.backup" "$CONFIG_FILE"

echo "" >> "$SUMMARY_FILE"
echo "Test completed at: $(date)" >> "$SUMMARY_FILE"

echo "========================================" | tee -a "$LOG_DIR/console.log"
echo "Test Complete" | tee -a "$LOG_DIR/console.log"
echo "========================================" | tee -a "$LOG_DIR/console.log"
echo "" | tee -a "$LOG_DIR/console.log"

# サマリーを表示
cat "$SUMMARY_FILE"

echo ""
echo "Detailed logs: $LOG_DIR"
