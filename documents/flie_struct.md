# リポジトリ構成

このプロジェクトの構成です

上から順番にデータを流していき、最終的に動体の座標と速度のデータが得られます

## データフロー

```
Livox MID-70 LiDAR（ハードウェア）
         ↓
  [lidar_get_points]
         ↓
    トピック: /livox/pointcloud
         ↓
  [lidar_preprocess]
    (ROI → Voxel → SOR フィルタ)
         ↓
    トピック: /preprocess/pointcloud
         ↓
  [lidar_background_diff]
    (背景モデル構築 → 差分抽出)
         ↓
    トピック: /background_diff/pointcloud
```

## パッケージの役割

| パッケージ | 機能 | 入力 | 出力 |
|-----------|------|------|------|
| **lidar_get_points** | LiDARから生のポイントクラウドを取得 | ハードウェア | `/livox/pointcloud` |
| **lidar_preprocess** | ROI/ボクセル/SORフィルタで前処理 | `/livox/pointcloud` | `/preprocess/pointcloud` |
| **lidar_background_diff** | 背景差分で動体を抽出 | `/preprocess/pointcloud` | `/background_diff/pointcloud` |

## ディレクトリツリー

```
2026_Hikorobo_Lidar/
├── .github/                          # GitHub設定ファイル
│   └── copilot-instructions.md      # AIコーディングガイドライン
├── .dev/                             # 開発環境設定
├── documents/                        # ドキュメント
│   ├── flie_struct.md               # （このファイル）
│   └── setup.md                     # 環境構築手順
├── config/                           # ROS 2 ノード設定ファイル
│   ├── livox_node.yml               # Livox ノード設定
│   ├── preprocess_node.yml          # 前処理ノード設定
│   └── background_diff_node.yml     # 背景差分ノード設定
│
├── lidar_get_points/                # Livox LiDAR データ取得パッケージ
│   ├── CMakeLists.txt              # CMake設定（Livox SDK リンク）
│   ├── package.xml                 # ROS 2 パッケージ定義
│   ├── include/
│   │   └── lidar_get_points/
│   │       └── livox_node.hpp      # LivoxNode クラス宣言
│   ├── src/
│   │   ├── livox_node.cpp          # LivoxNode 実装（SDK初期化、ポイントクラウド取得）
│   │   └── main.cpp                # エントリーポイント
│   └── launch/
│       └── livox.launch.py         # ROS 2 ランチファイル
│
├── lidar_preprocess/                # ポイントクラウド前処理パッケージ
│   ├── CMakeLists.txt              # CMake設定
│   ├── package.xml                 # ROS 2 パッケージ定義
│   ├── include/
│   │   └── lidar_preprocess/
│   │       ├── preprocess_node.hpp # PreprocessNode クラス宣言
│   │       ├── roi_filter.hpp      # ROI フィルタ（関心領域を抽出）
│   │       ├── voxel_filter.hpp    # ボクセルフィルタ（ダウンサンプリング）
│   │       └── sor_filter.hpp      # SOR フィルタ（外れ値除去）
│   └── src/
│       ├── preprocess_node.cpp     # PreprocessNode 実装
│       ├── roi_filter.cpp          # ROI フィルタ実装
│       ├── voxel_filter.cpp        # ボクセルフィルタ実装
│       ├── sor_filter.cpp          # SOR フィルタ実装
│       └── main.cpp                # エントリーポイント
│
├── lidar_background_diff/           # 背景差分パッケージ
│   ├── CMakeLists.txt              # CMake設定
│   ├── package.xml                 # ROS 2 パッケージ定義
│   ├── include/
│   │   └── lidar_background_diff/
│   │       └── background_diff_node.hpp # BackgroundDiffNode クラス宣言
│   └── src/
│       ├── background_diff_node.cpp     # BackgroundDiffNode 実装
│       └── main.cpp                # エントリーポイント
│
├── build.sh                         # ビルドスクリプト（colcon build）
├── start.sh                         # システム起動スクリプト（インタラクティブメニュー）
├── debug_launch.sh                  # デバッグ用ランチスクリプト
├── test/                            # テストコード
├── readme.md                        # プロジェクト概要・クイックスタート
└── .gitignore                       # Git 除外ファイル定義
```
