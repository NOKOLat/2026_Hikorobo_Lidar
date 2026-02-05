# Livox MID-70 LiDAR データ取得・処理システム - AI コーディングガイドライン

## プロジェクトアーキテクチャ

このプロジェクトは **ROS 2 ポイントクラウドパイプライン** で、Livox MID-70 LiDARデータを処理します：

- **`lidar_get_points/`**: Livox SDKを介してハードウェアに接続し、生のポイントクラウドをROS トピック `livox/pointcloud` に~10Hzで発行するノード
- **`lidar_preprocess/`**: リアルタイムでポイントクラウドをフィルタリングし、処理済みデータを `preprocess/pointcloud` に発行するパイプライン
- **データフロー**: ハードウェア → `livox_node` → ROS トピック → `preprocess_node` → (ROI → ボクセル → SOR フィルタ) → 出力トピック

## ビルド & 実行

**ビルドシステム**: CMake + ament (ROS 2 ビルドフレームワーク)

```bash
./build.sh              # 両パッケージをビルド: colcon build --packages-select lidar_get_points lidar_preprocess
./start.sh              # インタラクティブランチャー（3つのオプション）
```

**重要**: 実行前に `install/setup.bash` をソースします。ビルド成果物は `build/` と `install/` ディレクトリに生成されます。ログは `log/latest_build/` にあります。

## コード構造

### ファイル編成
```
lidar_get_points/
├── include/lidar_get_points/
│   └── livox_node.hpp              # クラス宣言
├── src/
│   ├── livox_node.cpp              # メソッド実装（Livox SDK統合含む）
│   └── main.cpp                    # エントリーポイント
├── CMakeLists.txt
└── package.xml

lidar_preprocess/
├── include/lidar_preprocess/
│   ├── preprocess_node.hpp         # クラス宣言
│   ├── roi_filter.hpp
│   ├── voxel_filter.hpp
│   └── sor_filter.hpp
├── src/
│   ├── preprocess_node.cpp         # メソッド実装
│   ├── main.cpp                    # エントリーポイント
│   ├── roi_filter.cpp
│   ├── voxel_filter.cpp
│   └── sor_filter.cpp
├── CMakeLists.txt
└── package.xml

config/
├── livox_node.yml                  # livox_nodeの設定
└── preprocess_node.yml             # preprocess_nodeの設定
```

### 設定ファイル（YAML）
- **`config/livox_node.yml`**: frame_id, publish_freq, buffer_frames, integration_time_ms, flip_yz を設定
- **`config/preprocess_node.yml`**: ROI範囲、ボクセルリーフサイズ、SOR パラメータを設定
- 両ノードは起動時に複数のパス（`config/`, `../config/`, `../../config/`）から設定を自動検索

## ROS 2 ノードアーキテクチャ

### LivoxNode (`lidar_get_points`)
- **ヘッダー**: `include/lidar_get_points/livox_node.hpp`
- **実装**: `src/livox_node.cpp`
- Livox SDK初期化: `InitLivoxSdk()`
- デバイス検出コールバック: `OnDeviceBroadcast()`
- デバイス状態変更コールバック: `OnDeviceChange()`
- ポイントクラウドコールバック: `GetPointCloudCallback()`
- フレームバッファリング: `IntegrateCurrentFrame()`、`PublishPointCloud()`
- パラメータ: `declare_parameter()` → `LoadConfigFromYAML()` で YAML から読み込み

### PreprocessNode (`lidar_preprocess`)
- **ヘッダー**: `include/lidar_preprocess/preprocess_node.hpp`
- **実装**: `src/preprocess_node.cpp`
- YAML設定読み込み: `LoadConfigFromYAML()`
- ポイントクラウド処理: `CloudCallback()` で ROI → ボクセル → SOR フィルタを順次適用
- パラメータ: roi_min_x, roi_max_x, roi_min_y, roi_max_y, roi_min_z, roi_max_z, voxel_leaf_size, sor_mean_k, sor_std_dev_mul

### ポイントクラウドフィルタリングパイプライン
3つのフィルタが `CloudCallback()` で順次適用されます：

1. **ROI フィルタ** (`roi_filter.hpp/.cpp`): 関心領域へのクロップ（デフォルト: x: 0.5-20m, y: -5-5m, z: -2-5m）
2. **ボクセルフィルタ** (`voxel_filter.hpp/.cpp`): リーフサイズによるダウンサンプリング（デフォルト: 0.01m/1cm）
3. **SOR フィルタ** (`sor_filter.hpp/.cpp`): 統計的外れ値除去（デフォルト: mean_k=50, std_dev_mul=1.0）

## 開発規約

- **C++ 標準**: C++14+ 、`-Wall -Wextra -Wpedantic` で厳密にコンパイル
- **言語**: すべてのコメントは日本語
- **PCL 使用**: `pcl::PointCloud<pcl::PointXYZ>`、`pcl::PointCloud<pcl::PointXYZI>` の直接使用
- **メッセージヘッダ**: タイムスタンプとフレーム ID を保持してパイプラインを通す
- **ロギング**: `RCLCPP_INFO()`, `RCLCPP_DEBUG()`, `RCLCPP_ERROR()` マクロを使用（std::cout は不可）
- **スレッド安全性**: `std::mutex` で共有バッファ（livox_node の frame_buffer_ など）を保護

## 外部依存関係

- **Livox SDK**: 静的リンク（`livox_sdk_static`）、ヘッダは `/usr/local/include/`、ライブラリは `/usr/local/lib/`
- **PCL**: ポイントクラウドライブラリ全体（フィルタ含む）
- **yaml-cpp**: YAML 設定ファイル解析用
- **ROS 2**: rclcpp, sensor_msgs, std_msgs, pcl_conversions, pcl_ros

## 一般的なタスク

**新しいフィルタを追加する**:
1. `lidar_preprocess/include/lidar_preprocess/new_filter.hpp` にヘッダを作成
2. `lidar_preprocess/src/new_filter.cpp` に実装（`filter(input, output)` メソッド必須）
3. `preprocess_node.hpp` でメンバー変数を宣言
4. `preprocess_node.cpp` のコンストラクタで初期化
5. `CloudCallback()` で適用順序に追加
6. `CMakeLists.txt` に `new_filter.cpp` を追加

**ポイントクラウドをテストする**:
```bash
ros2 topic echo /livox/pointcloud | head    # 生データを確認
ros2 topic echo /preprocess/pointcloud | head  # 処理済みデータを確認
```
RViz で PointCloud2 ディスプレイを使用して可視化します

**ハードウェアをデバッグする**:
- `OnDeviceBroadcast()` コールバック内にデバッグログを追加してデバイス検出を確認
- `GetLivoxSdkVersion()` で SDK バージョンを確認

**パラメータをチューニングする**:
```bash
# YAML ファイルで デフォルト値を編集、またはコマンドラインでオーバーライド
ros2 run lidar_preprocess preprocess_node --ros-args -p roi_min_x:=0.0 -p voxel_leaf_size:=0.02
```
