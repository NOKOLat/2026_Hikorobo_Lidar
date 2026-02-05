#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "livox_sdk.h"

#include <deque>
#include <vector>
#include <mutex>
#include <memory>
#include <chrono>

class LivoxNode : public rclcpp::Node
{
public:
    LivoxNode() : Node("livox_node")
    {
        // Declare parameters
        this->declare_parameter("frame_id", "livox_frame");
        this->declare_parameter("publish_freq", 10.0);       // 10Hz output
        this->declare_parameter("buffer_frames", 10);        // 10フレーム分 (1秒積分)
        this->declare_parameter("integration_time_ms", 100); // 100ms単位でフレーム統合
        this->declare_parameter("flip_yz", false);           // Y-Z反転オプション

        frame_id_ = this->get_parameter("frame_id").as_string();
        double publish_freq = this->get_parameter("publish_freq").as_double();
        buffer_frames_ = this->get_parameter("buffer_frames").as_int();
        integration_time_ms_ = this->get_parameter("integration_time_ms").as_int();
        flip_yz_ = this->get_parameter("flip_yz").as_bool();

        // Create publisher
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "livox/pointcloud", 10);

        // Initialize Livox SDK
        if (!InitLivoxSdk())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Livox SDK");
            return;
        }

        // Create timer for publishing at 10Hz
        auto period = std::chrono::duration<double>(1.0 / publish_freq);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&LivoxNode::PublishPointCloud, this));

        RCLCPP_INFO(this->get_logger(),
                    "Livox node started: publish_freq=%.1fHz, buffer_frames=%d, integration_time=%dms, flip_yz=%s",
                    publish_freq, buffer_frames_, integration_time_ms_, flip_yz_ ? "true" : "false");
    }

    ~LivoxNode()
    {
        Uninit();
    }

private:
    bool InitLivoxSdk()
    {
        if (!Init())
        {
            return false;
        }

        LivoxSdkVersion version;
        GetLivoxSdkVersion(&version);
        RCLCPP_INFO(this->get_logger(),
                    "Livox SDK version: %d.%d.%d",
                    version.major, version.minor, version.patch);

        SetBroadcastCallback(OnDeviceBroadcast);
        SetDeviceStateUpdateCallback(OnDeviceChange);

        // Start device discovery
        if (!Start())
        {
            Uninit();
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Livox SDK initialized, discovering devices...");
        return true;
    }

    static void OnDeviceBroadcast(const BroadcastDeviceInfo *info)
    {
        if (info == nullptr)
        {
            return;
        }

        RCLCPP_INFO(rclcpp::get_logger("livox_node"),
                    "Device broadcast: SN=%s, Type=%d, IP=%s",
                    info->broadcast_code, info->dev_type, info->ip);

        bool result = false;
        uint8_t handle = 0;
        result = AddLidarToConnect(info->broadcast_code, &handle);

        if (result == kStatusSuccess && handle < kMaxLidarCount)
        {
            SetDataCallback(handle, GetPointCloudCallback, nullptr);
            device_handles_.push_back(handle);
            RCLCPP_INFO(rclcpp::get_logger("livox_node"),
                        "Added device handle: %d", handle);
        }
    }

    static void OnDeviceChange(const DeviceInfo *info, DeviceEvent type)
    {
        if (info == nullptr)
        {
            return;
        }

        RCLCPP_INFO(rclcpp::get_logger("livox_node"),
                    "Device state change: SN=%s, Event=%d, State=%d",
                    info->broadcast_code, type, info->state);

        // Start sampling when device is connected and in normal state
        if (type == kEventConnect ||
            (type == kEventStateChange && info->state == kLidarStateNormal))
        {

            uint8_t handle = info->handle;

            // Check if device is in normal state before starting sampling
            if (info->state == kLidarStateNormal)
            {
                LidarStartSampling(handle, OnSampleCallback, nullptr);
                RCLCPP_INFO(rclcpp::get_logger("livox_node"),
                            "Starting sampling for device handle: %d", handle);
            }
        }
    }

    static void OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data)
    {
        (void)response;
        (void)data;
        if (status == kStatusSuccess)
        {
            RCLCPP_INFO(rclcpp::get_logger("livox_node"),
                        "Device %d start sampling successfully", handle);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("livox_node"),
                         "Device %d start sampling failed: %d", handle, status);
        }
    }

    static void GetPointCloudCallback(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data)
    {
        (void)handle;
        (void)client_data;

        if (data == nullptr)
        {
            return;
        }

        static int callback_count = 0;
        if (callback_count % 100 == 0)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("livox_node"),
                         "Data callback: type=%d, points=%d, current_frame_size=%zu",
                         data->data_type, data_num, current_frame_buffer_.size());
        }
        callback_count++;

        std::lock_guard<std::mutex> lock(cloud_mutex_);

        // Process point cloud data based on data type
        if (data->data_type == kCartesian)
        {
            LivoxRawPoint *p_point_data = (LivoxRawPoint *)data->data;

            for (uint32_t i = 0; i < data_num; i++)
            {
                pcl::PointXYZI point;
                point.x = p_point_data[i].x / 1000.0f; // mm to m
                point.y = p_point_data[i].y / 1000.0f;
                point.z = p_point_data[i].z / 1000.0f;
                point.intensity = p_point_data[i].reflectivity;

                current_frame_buffer_.push_back(point);
            }
        }
        else if (data->data_type == kExtendCartesian)
        {
            LivoxExtendRawPoint *p_point_data = (LivoxExtendRawPoint *)data->data;

            for (uint32_t i = 0; i < data_num; i++)
            {
                pcl::PointXYZI point;
                point.x = p_point_data[i].x / 1000.0f; // mm to m
                point.y = p_point_data[i].y / 1000.0f;
                point.z = p_point_data[i].z / 1000.0f;
                point.intensity = p_point_data[i].reflectivity;

                current_frame_buffer_.push_back(point);
            }
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("livox_node"),
                        "Unsupported data type: %d", data->data_type);
        }
    }

    // 定期的に現在のフレームバッファを統合バッファに追加
    void IntegrateCurrentFrame()
    {
        // 現在のフレームがある場合は統合バッファに追加
        if (!current_frame_buffer_.empty())
        {
            frame_buffer_.push_back(current_frame_buffer_);

            // バッファフレーム数を超えたら古いフレームを削除
            if (frame_buffer_.size() > static_cast<size_t>(buffer_frames_))
            {
                size_t removed_points = frame_buffer_.front().size();
                frame_buffer_.pop_front();

                RCLCPP_DEBUG(this->get_logger(),
                             "Removed old frame with %zu points, buffer now has %zu frames",
                             removed_points, frame_buffer_.size());
            }

            RCLCPP_DEBUG(this->get_logger(),
                         "Integrated frame with %zu points, buffer now has %zu frames",
                         current_frame_buffer_.size(), frame_buffer_.size());

            // 次のフレームのために現在のバッファをクリア
            current_frame_buffer_.clear();
        }
    }

    void PublishPointCloud()
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);

        // 現在のフレームを統合バッファに追加
        IntegrateCurrentFrame();

        if (frame_buffer_.empty())
        {
            static int empty_count = 0;
            if (empty_count % 100 == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Frame buffer is empty");
            }
            empty_count++;
            return;
        }

        // 全フレームの点群を統合
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

        size_t total_points = 0;
        for (const auto &frame : frame_buffer_)
        {
            total_points += frame.size();
        }

        cloud->points.reserve(total_points);

        for (const auto &frame : frame_buffer_)
        {
            for (const auto &point : frame)
            {
                pcl::PointXYZI p = point;
                // Y-Z反転処理 (Lidar上下逆向き対応)
                // X軸周りで180度回転 (Y と Z の符号反転)
                if (flip_yz_)
                {
                    p.y = -p.y;
                    p.z = -p.z;
                }
                cloud->points.push_back(p);
            }
        }

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.stamp = this->now();
        output.header.frame_id = frame_id_;

        cloud_pub_->publish(output);

        RCLCPP_INFO(this->get_logger(),
                    "Published point cloud with %zu points from %zu frames",
                    cloud->points.size(), frame_buffer_.size());
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string frame_id_;
    int buffer_frames_;       // 保持するフレーム数 (デフォルト10)
    int integration_time_ms_; // フレーム統合時間 (デフォルト100ms)
    bool flip_yz_;            // Y-Z反転オプション (Lidar上下逆向き対応)

    static std::vector<uint8_t> device_handles_;
    static std::mutex cloud_mutex_;
    static std::deque<std::vector<pcl::PointXYZI>> frame_buffer_; // 複数フレームのバッファ
    static std::vector<pcl::PointXYZI> current_frame_buffer_;     // 現在のフレームバッファ
};

// Static member initialization
std::vector<uint8_t> LivoxNode::device_handles_;
std::mutex LivoxNode::cloud_mutex_;
std::deque<std::vector<pcl::PointXYZI>> LivoxNode::frame_buffer_;
std::vector<pcl::PointXYZI> LivoxNode::current_frame_buffer_;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LivoxNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
