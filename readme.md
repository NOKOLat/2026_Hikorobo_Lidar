# Livox MID-70 データ取得・処理システム

## クイックスタート

### 初回セットアップ（ビルド）

```bash
./build.sh
```

### システム起動

```bash
./start.sh
```

システムの起動オプションメニューが表示されます。

**オプション:**
1. データ取得のみ (lidar_get_points)
2. 前処理ノード (lidar_preprocess)
3. 背景差分ノード (lidar_background_diff)
4. クラスタリングノード (lidar_clustering)
5. 全て同時起動 (データ取得 → 前処理 → 背景差分 → クラスタリング)

## システム構成

### データフロー

```
Livox MID-70 LiDAR
    ↓
[lidar_get_points] → トピック: livox/pointcloud
    ↓
[lidar_preprocess] → トピック: preprocess/pointcloud
    ↓
[lidar_background_diff] → トピック: background_diff/pointcloud
    ↓
[lidar_clustering] → トピック: clustering/markers (MarkerArray)
```

### パッケージ説明

- **lidar_get_points**: Livox SDK を使用してハードウェアから生のポイントクラウドを取得
- **lidar_preprocess**: ROI フィルタ、ボクセルフィルタ、SOR フィルタによるポイントクラウドの前処理
- **lidar_background_diff**: 最初の50フレーム(約5秒)で背景モデルを構築し、背景差分により動体のみを抽出
- **lidar_clustering**: ユークリッドクラスタリングにより動体をグループ化し、バウンディングボックスをRViz2に表示

## 詳細

詳細な設定・パラメータについては setup.md を参照してください。
