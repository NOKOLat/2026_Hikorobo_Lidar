#ifndef CLUSTERING_NODE_HPP_
#define CLUSTERING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <yaml-cpp/yaml.h>
#include <memory>
#include <filesystem>
#include <vector>

/**
 * @brief クラスタリングノード
 *
 * 背景差分ノードから出力されるポイントクラウドに対して
 * ユークリッドクラスタリングを行い、各クラスタのバウンディングボックスを
 * MarkerArrayとしてRViz2にパブリッシュする。
 */
class ClusteringNode : public rclcpp::Node
{
public:
    ClusteringNode();

private:
    // YAMLファイルから設定を読み込む
    void LoadConfigFromYAML();

    // ポイントクラウドコールバック
    void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // ROS インターフェース
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // クラスタリングパラメータ
    double cluster_tolerance_;  // クラスタリングの距離閾値（デフォルト: 0.1m）
    int min_cluster_size_;      // クラスタの最小ポイント数（デフォルト: 10）
    int max_cluster_size_;      // クラスタの最大ポイント数（デフォルト: 10000）
};

#endif // CLUSTERING_NODE_HPP_
