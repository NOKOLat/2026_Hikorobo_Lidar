#ifndef BACKGROUND_DIFF_NODE_HPP_
#define BACKGROUND_DIFF_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <yaml-cpp/yaml.h>
#include <memory>
#include <filesystem>
#include <vector>

#include "lidar_background_diff/ror_filter.hpp"

/**
 * @brief 背景差分ノード
 *
 * 最初の指定フレーム数（デフォルト50フレーム）を使用して背景モデルを構築し、
 * その後、入力ポイントクラウドから背景を差し引いて動体のみを抽出する。
 */
class BackgroundDiffNode : public rclcpp::Node
{
public:
    BackgroundDiffNode();

private:
    // YAMLファイルから設定を読み込む
    void LoadConfigFromYAML();

    // ポイントクラウドコールバック
    void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // 背景を構築する
    void BuildBackgroundModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    // 背景差分を計算する
    pcl::PointCloud<pcl::PointXYZ>::Ptr ComputeBackgroundDiff(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    // ROS インターフェース
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr diff_pub_;

    // 背景モデル関連
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> background_frames_; // 背景フレームの蓄積
    pcl::PointCloud<pcl::PointXYZ>::Ptr background_model_;               // 平均化された背景モデル
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr background_kdtree_;             // 背景モデルのKD-Tree

    // 設定パラメータ
    int background_frame_count_; // 背景構築に使用するフレーム数（デフォルト: 50）
    double distance_threshold_;  // 背景との距離閾値（デフォルト: 0.05m）
    float ror_min_range_;        // RORフィルタの最小距離（デフォルト: 0.5m）
    float ror_max_range_;        // RORフィルタの最大距離（デフォルト: 10.0m）

    // 状態管理
    bool is_background_ready_; // 背景モデルが構築済みかどうか
    int received_frame_count_; // 受信したフレーム数

    // フィルタ
    std::unique_ptr<RORFilter> ror_filter_; // Range-based Outlier Removal フィルタ
};

#endif // BACKGROUND_DIFF_NODE_HPP_
