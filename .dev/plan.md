# PCL Pipeline for Drone Detection (Intensity-based)

このドキュメントは、PCL (Point Cloud Library) を使用したドローン検知システムの処理パイプラインを記述します。
反射強度（Intensity）を持つ点群データ (⁨`pcl::PointXYZI`⁩) を使用し、背景除去・ノイズ除去・床面除去を行った後、**「高輝度な動体」**を特定するための構成です。

## 目的

40m先で離陸してlivoxmid70に向かってくる発泡スチロール製の飛行物に位置情報を提供し、5m地点に着陸させるためのシステムの構築

基本的に視界中央に飛行物がある前提で、端のほうの精度は要求しない

### エリア

- 縦方向
    - 0.5m 〜 45mを使用
    - 壁は60m程度先にある 

- 横方向
    - -5m 〜 +5mを使用
    - 壁は20m程度先にある

- 高さ方向
    - -1m 〜 +5mを使用
    - 天井は20m程度先にある

- その他障害物
    - 付近の地面を通る人やエリア中央に障害物がある


## 0. Common Definition (共通定義)

すべての処理ステージにおいて`Intensity`（反射強度）情報を保持するため、点群の型定義を統一します。

またデータの積分時間は0.1秒（10Hz）とし、ここはユーザーが設定できるパラメータにします


```cpp
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// 座標(XYZ) + 反射強度(Intensity) を持つ型を使用
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
```

## 1. Preprocess (前処理)

計算量削減と信号品質向上のためのフィルタリング処理です。

### 1-1. 距離フィルター (PassThrough)
**役割:** 処理範囲（ROI: Region of Interest）を限定し、計算コストを削減します。
```cpp
#include <pcl/filters/passthrough.h>

pcl::PassThrough<PointT> pass;

// --- 1. Z軸 (奥行き) ---
pass.setInputCloud(cloud);          // 元の点群を入力
pass.setFilterFieldName("z");
pass.setFilterLimits(0.5, 45.0);
pass.filter(*cloud_filtered);       // 結果を cloud_filtered へ

// --- 2. X軸 (左右) ---
pass.setInputCloud(cloud_filtered); // ★直前の結果を入力にする
pass.setFilterFieldName("x");
pass.setFilterLimits(-5.0, 5.0);    // 必要な幅に設定
pass.filter(*cloud_filtered);       // 上書き保存

// --- 3. Y軸 (上下) ---
pass.setInputCloud(cloud_filtered); // ★直前の結果を入力にする
pass.setFilterFieldName("y");
pass.setFilterLimits(-1.0, 5.0);    // 必要な高さに設定
pass.filter(*cloud_filtered);       // 上書き保存
```

### 1-2. ダウンサンプリング (VoxelGrid)
**役割:** 点群密度を均一化し、データ量を削減します。
**重要:** `PointXYZI` を使用することで、ボクセル内の `Intensity` 値も平均化され、反射ノイズのバラつきが抑制されます。
**推奨設定:** LeafSize: 0.1m (10cm) - 40m先では5cmの密度は得られない可能性があるため、少し粗くして、確実に点を拾うことを優先します。

```cpp
#include <pcl/filters/voxel_grid.h>

pcl::VoxelGrid<PointT> vg;
vg.setInputCloud(cloud_filtered);
vg.setLeafSize(0.1f, 0.1f, 0.1f); // 10cm角のボクセルで間引き
// vg.filter(*cloud_filtered);
```

### 1-3. 外れ値除去 (SOR)
**役割:** 空間に浮遊する微細なスパイクノイズ（単発的なゴミ）を統計的に除去します。
**推奨設定:** MeanK: 10, Thresh: 1.0 - 高速移動するドローンは点群が尾を引くため、SORを強くかけすぎるとドローン自体が消えます。弱めに設定してください。

```cpp
#include <pcl/filters/statistical_outlier_removal.h>

pcl::StatisticalOutlierRemoval<PointT> sor;
sor.setInputCloud(cloud_filtered);
sor.setMeanK(10);             // 近傍10点を見て判断（弱めに設定）
sor.setStddevMulThresh(1.0);  // 標準偏差の1.0倍以上離れていたらノイズ
// sor.filter(*cloud_filtered);
```

## 2. Processing (動体検知・特定)

時間的な差分情報と、反射強度を利用してターゲット（ドローン）を抽出します。


### 2-1. 床面除去 (RANSAC + ExtractIndices) 
**役割:** 幾何学的に「平面」とみなせる床を検出し、除去します。これにより動体検知の誤検知を大幅に減らします。

```cpp
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// 1. 平面検出
pcl::SACSegmentation<PointT> seg;
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

seg.setOptimizeCoefficients(true);
seg.setModelType(pcl::SACMODEL_PLANE); // 平面モデル
seg.setMethodType(pcl::SAC_RANSAC);
seg.setDistanceThreshold(0.02);        // 厚み許容値 (2cm)
seg.setInputCloud(cloud_filtered);
seg.segment(*inliers, *coefficients);

// 2. 平面除去 (ExtractIndices)
if (inliers->indices.size() > 0) {
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);         // true = インデックスの点（床）を「消す」
    extract.filter(*cloud_filtered);
}
```

### 2-2. 背景差分処理 (SegmentDifferences)
**役割:** 起動直後の背景（何もない状態）を蓄積してから、現在フレームとの差分を計算。Livoxのノイズや風による揺れを考慮した安定した背景除去を実現します。

**グローバル/クラスメンバ変数:**
```cpp
#include <pcl/segmentation/segment_differences.h>

// グローバルまたはクラスメンバとして保持
pcl::PointCloud<PointT>::Ptr background_map(new pcl::PointCloud<PointT>);
bool is_background_captured = false;
int calibration_frames = 0;
const int CALIBRATION_LIMIT = 50; // 10Hzなら5秒間蓄積

// トラッキング用: 前フレームで検知したドローン位置
Eigen::Vector3f last_drone_position = Eigen::Vector3f::Zero();
bool drone_detected = false;
const float TRACKING_RADIUS = 3.0f; // トラッキング半径: 3m
```

**ループ内処理:**
```cpp
// cloud_in が現在のフレーム、cloud_filtered は前処理（1-1～1-3）を通したもの

if (!is_background_captured) {
    // 【フェーズ1: 背景蓄積】
    // 起動直後の誰もいない状態を学習します
    *background_map += *cloud_filtered;
    calibration_frames++;

    if (calibration_frames >= CALIBRATION_LIMIT) {
        // 蓄積完了後、データ量を減らすためにダウンサンプリング
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(background_map);
        vg.setLeafSize(0.1f, 0.1f, 0.1f); // 10cm刻みの密度にする
        pcl::PointCloud<PointT>::Ptr temp_map(new pcl::PointCloud<PointT>);
        vg.filter(*temp_map);
        background_map = temp_map; // マップ更新

        is_background_captured = true;
        std::cout << "Background Captured! Points: " << background_map->size() << std::endl;
    }
    // このフェーズでは処理をスキップ
    return;
} 
else {
    // 【フェーズ2: 背景差分実行】
    
    pcl::SegmentDifferences<PointT> seg_diff;
    seg_diff.setInputCloud(cloud_filtered);       // 現在のフレーム（ターゲット）
    seg_diff.setTargetCloud(background_map);      // 蓄積した背景（比較元）
    
    // 判定閾値: 背景点からこの距離以内なら「背景」とみなす
    // Livoxのノイズや風による揺れを考慮して少し大きめに (0.1 = 10cm)
    seg_diff.setDistanceThreshold(0.1); 

    pcl::PointCloud<PointT>::Ptr cloud_drone(new pcl::PointCloud<PointT>);
    seg_diff.segment(*cloud_drone); // 結果が cloud_drone に入る

    // cloud_drone には「背景に存在しなかった点（ドローン）」のみが残る
    // この cloud_drone を 2-3 のクラスタリング処理へ渡す
}
```

**参考: Octree Change Detector (非推奨)**
```cpp
// 注意: 以下は参考のみ。Livoxでは使用すると誤検知が増加するため非推奨
#include <pcl/octree/octree_pointcloud_changedetector.h>

float resolution = 0.1f;  // VoxelGrid のLeafSizeと同じ
pcl::octree::OctreePointCloudChangeDetector<PointT> octree(resolution);
octree.switchBuffers();
octree.setInputCloud(cloud_filtered);
octree.addPointsFromInputCloud();

std::vector<int> newPointIdxVector;
octree.getPointIndicesFromNewVoxels(newPointIdxVector);
pcl::PointCloud<PointT>::Ptr cloud_diff(new pcl::PointCloud<PointT>);
pcl::copyPointCloud(*cloud_filtered, newPointIdxVector, *cloud_diff);
```




### 2-3. トラッキングROI (オプション)
**役割:** 一度ドローンを検知したら、その位置の周囲3m以内に限定してクラスタリングを実行。計算コストを削減し、追跡精度を向上させます。

```cpp
#include <pcl/filters/passthrough.h>

// 2-2 から得た cloud_drone を使用
if (cloud_drone->size() == 0) {
    // 背景差分で何も残らなかった場合
    drone_detected = false;
    return;
}

pcl::PointCloud<PointT>::Ptr cloud_tracking = cloud_drone;

if (drone_detected) {
    // 【トラッキングモード】前フレームのドローン位置周囲のみを処理
    
    // 動的ROI: 前フレーム位置の周囲3m以内
    float x_min = last_drone_position(0) - TRACKING_RADIUS;
    float x_max = last_drone_position(0) + TRACKING_RADIUS;
    float y_min = last_drone_position(1) - TRACKING_RADIUS;
    float y_max = last_drone_position(1) + TRACKING_RADIUS;
    float z_min = last_drone_position(2) - TRACKING_RADIUS;
    float z_max = last_drone_position(2) + TRACKING_RADIUS;
    
    // PassThrough で周囲3mの領域を抽出
    pcl::PassThrough<PointT> pass;
    pcl::PointCloud<PointT>::Ptr cloud_roi(new pcl::PointCloud<PointT>);
    
    // X軸
    pass.setInputCloud(cloud_drone);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min, x_max);
    pass.filter(*cloud_roi);
    
    // Y軸
    pass.setInputCloud(cloud_roi);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min, y_max);
    pass.filter(*cloud_roi);
    
    // Z軸
    pass.setInputCloud(cloud_roi);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);
    pass.filter(*cloud_roi);
    
    cloud_tracking = cloud_roi;
}
```

### 2-4. クラスタリング (Euclidean Clustering)

**役割:** 背景差分後の点群をクラスタリング（またはトラッキング領域内で）。反射強度を「閾値」ではなく「重み」として利用し、クラスタリング後に強度で判定します。
**重要:** 40m先では強度が落ちます。閾値で切ると遠くで見失うため、全点を使ってクラスタリングし、抽出されたクラスタの「最大強度」や「平均強度」を見てドローン判定するロジックが良いでしょう。

```cpp
#include <pcl/segmentation/euclidean_cluster_extraction.h>
#include <pcl/kdtree/kdtree.h>

// 2-3 から得た cloud_tracking を使用
if (cloud_tracking->size() == 0) {
    // クラスタリング対象の点がない場合
    return;
}

// KD-Tree を構築
pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
tree->setInputCloud(cloud_tracking);

// ユークリッドクラスタリング実行
std::vector<pcl::PointIndices> cluster_indices;
pcl::EuclideanClusterExtraction<PointT> ec;
ec.setClusterTolerance(0.3);           // 許容距離: 30cm
ec.setMinClusterSize(5);               // 最小点数（弱めに設定）
ec.setMaxClusterSize(3000);            // 最大点数
ec.setSearchMethod(tree);
ec.setInputCloud(cloud_tracking);
ec.extract(cluster_indices);

// クラスタごとに反射強度を集計
bool detected_this_frame = false;
Eigen::Vector3f current_drone_position = Eigen::Vector3f::Zero();

for (const auto& cluster : cluster_indices) {
    float max_intensity = 0.0f;
    float avg_intensity = 0.0f;
    float intensity_sum = 0.0f;
    
    for (int idx : cluster.indices) {
        float intensity = cloud_tracking->points[idx].intensity;
        max_intensity = std::max(max_intensity, intensity);
        intensity_sum += intensity;
    }
    
    avg_intensity = intensity_sum / cluster.indices.size();
    
    // ドローン判定: 最大強度 > 100 かつ平均強度 > 50 の場合
    // （閾値はセンサー仕様・環境条件に合わせて調整）
    if (max_intensity > 100.0f && avg_intensity > 50.0f) {
        // このクラスタはドローンと判定
        detected_this_frame = true;
        
        // クラスタの重心を計算
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (int idx : cluster.indices) {
            centroid(0) += cloud_tracking->points[idx].x;
            centroid(1) += cloud_tracking->points[idx].y;
            centroid(2) += cloud_tracking->points[idx].z;
        }
        centroid /= cluster.indices.size();
        current_drone_position = centroid;
        
        // 位置情報などを出力
        std::cout << "Drone Detected at: (" 
                  << centroid(0) << ", " 
                  << centroid(1) << ", " 
                  << centroid(2) << ")" << std::endl;
        std::cout << "Max Intensity: " << max_intensity 
                  << ", Avg Intensity: " << avg_intensity << std::endl;
    }
}

// トラッキング位置の更新
if (detected_this_frame) {
    drone_detected = true;
    last_drone_position = current_drone_position;
} else {
    drone_detected = false;  // 検知できなくなったらトラッキング解除
}

```
