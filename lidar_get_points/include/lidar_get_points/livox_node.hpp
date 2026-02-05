#ifndef LIVOX_NODE_HPP_
#define LIVOX_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "livox_sdk.h"

#include <yaml-cpp/yaml.h>
#include <deque>
#include <vector>
#include <mutex>
#include <memory>
#include <chrono>
#include <filesystem>
#include <iostream>

class LivoxNode : public rclcpp::Node
{
public:
    LivoxNode();
    ~LivoxNode();

private:
    // YAMLファイルから設定を読み込む
    void LoadConfigFromYAML();
    bool InitLivoxSdk();

    static void OnDeviceBroadcast(const BroadcastDeviceInfo *info);
    static void OnDeviceChange(const DeviceInfo *info, DeviceEvent type);
    static void OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data);
    static void GetPointCloudCallback(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data);

    void IntegrateCurrentFrame();
    void PublishPointCloud();

    // メンバー変数
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string frame_id_;
    double publish_freq_;     // 発行周波数 (デフォルト10Hz)
    int buffer_frames_;       // 保持するフレーム数 (デフォルト10)
    int integration_time_ms_; // フレーム統合時間 (デフォルト100ms)
    bool flip_yz_;            // Y-Z反転オプション (Lidar上下逆向き対応)

    static std::vector<uint8_t> device_handles_;
    static std::mutex cloud_mutex_;
    static std::deque<std::pair<std::chrono::steady_clock::time_point, std::vector<pcl::PointXYZI>>> frame_buffer_; // フレームとタイムスタンプのバッファ
    static std::vector<pcl::PointXYZI> current_frame_buffer_;                                                       // 現在のフレームバッファ
};

#endif // LIVOX_NODE_HPP_
