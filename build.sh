#!/bin/bash

# Livox MID-70 ビルドスクリプト

# スクリプトのディレクトリを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "Livox MID-70 ビルド開始"
echo "=========================================="
echo ""

# ビルドを実行
colcon build --packages-select lidar_get_points lidar_preprocess

# ビルド結果をチェック
if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✓ ビルド成功"
    echo "=========================================="
    echo ""
    echo "次に実行してください:"
    echo "  ./start.sh"
else
    echo ""
    echo "=========================================="
    echo "✗ ビルド失敗"
    echo "=========================================="
    exit 1
fi
