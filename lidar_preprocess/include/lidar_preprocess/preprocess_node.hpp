#ifndef PREPROCESS_NODE_HPP_
#define PREPROCESS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_preprocess/roi_filter.hpp"
#include "lidar_preprocess/voxel_filter.hpp"
#include "lidar_preprocess/sor_filter.hpp"

#include <yaml-cpp/yaml.h>
#include <memory>
#include <filesystem>
#include <iostream>

class PreprocessNode : public rclcpp::Node
{
public:
    PreprocessNode();

private:
    // YAMLファイルから設定を読み込む
    void LoadConfigFromYAML();
    void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // フィルタ
    std::unique_ptr<ROIFilter> roi_filter_;
    std::unique_ptr<VoxelFilter> voxel_filter_;
    std::unique_ptr<SORFilter> sor_filter_;

    // ROS インターフェース
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    // 設定パラメータ
    double roi_min_x_;
    double roi_max_x_;
    double roi_min_y_;
    double roi_max_y_;
    double roi_min_z_;
    double roi_max_z_;
    double voxel_leaf_size_;
    int sor_mean_k_;
    double sor_std_dev_mul_;
};

#endif // PREPROCESS_NODE_HPP_
