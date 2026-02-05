#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>

#include "livox_def.h"
#include "livox_sdk.h"

namespace lidar_get_cloud_points
{

    // Point structure for PCL-compatible storage
    struct LivoxPoint
    {
        float x;
        float y;
        float z;
        uint8_t reflectivity;
        uint8_t tag;
        uint16_t line;
    } __attribute__((packed));

    // Timestamped point for buffer management (same as mid70_pcl_fliter_test)
    struct TimestampedPoint
    {
        LivoxPoint point;
        uint64_t timestamp_us; // microseconds since epoch
    };

    class LidarGetCloudPointsComponent : public rclcpp::Node
    {
    public:
        explicit LidarGetCloudPointsComponent(const rclcpp::NodeOptions &options)
            : Node("lidar_get_cloud_points", options),
              is_initialized_(false),
              lidar_count_(0)
        {
            // Publisher for point cloud data
            cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "livox/cloud_points", 10);

            // Parameters
            this->declare_parameter<std::vector<std::string>>("broadcast_codes", std::vector<std::string>());
            this->declare_parameter<double>("publish_rate", 10.0);
            this->declare_parameter<double>("buffer_time_seconds", 0.1); // 0.1 seconds buffer (reduced for memory safety)
            this->declare_parameter<int>("max_buffer_size", 5000);      // Maximum points (further reduced for memory safety)

            auto broadcast_codes = this->get_parameter("broadcast_codes").as_string_array();
            double publish_rate = this->get_parameter("publish_rate").as_double();
            buffer_time_seconds_ = this->get_parameter("buffer_time_seconds").as_double();
            max_buffer_size_ = this->get_parameter("max_buffer_size").as_int();

            RCLCPP_INFO(this->get_logger(), "LiDAR Get Cloud Points Component initialized");
            RCLCPP_INFO(this->get_logger(), "Buffer time: %.1f seconds, Max buffer size: %d points",
                        buffer_time_seconds_, max_buffer_size_);

            // Pre-allocate buffer memory to avoid frequent reallocations
            timestamped_buffer_.reserve(max_buffer_size_);
            RCLCPP_INFO(this->get_logger(), "Pre-allocated buffer for %d points", max_buffer_size_);

            // Delay Livox SDK initialization to avoid memory conflicts with other components
            // Wait 2 seconds for other components to fully initialize
            RCLCPP_INFO(this->get_logger(), "Delaying Livox SDK initialization for 2 seconds...");

            init_timer_ = this->create_wall_timer(
                std::chrono::seconds(2),
                [this, broadcast_codes, publish_rate]() {
                    // This timer will only run once
                    init_timer_->cancel();

                    RCLCPP_INFO(this->get_logger(), "Starting Livox SDK initialization...");

                    // Initialize Livox SDK
                    if (InitLivoxSDK(broadcast_codes))
                    {
                        RCLCPP_INFO(this->get_logger(), "Livox SDK initialized successfully");

                        // Create timer for publishing accumulated point cloud
                        publish_timer_ = this->create_wall_timer(
                            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
                            std::bind(&LidarGetCloudPointsComponent::PublishPointCloud, this));
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Livox SDK");
                    }
                });
        }

        ~LidarGetCloudPointsComponent()
        {
            if (is_initialized_)
            {
                Uninit();
                RCLCPP_INFO(this->get_logger(), "Livox SDK deinitialized");
            }
        }

        // Static callback for Livox SDK data
        static void GetLidarDataCallback(uint8_t handle, LivoxEthPacket *data,
                                         uint32_t data_num, void *client_data)
        {
            if (!data || !data_num || !client_data)
            {
                return;
            }

            auto *node = static_cast<LidarGetCloudPointsComponent *>(client_data);
            node->ProcessLidarData(handle, data, data_num);
        }

        // Static callback for device broadcast
        static void OnDeviceBroadcast(const BroadcastDeviceInfo *info)
        {
            if (info == nullptr || g_lidar_component == nullptr)
            {
                return;
            }

            g_lidar_component->HandleDeviceBroadcast(info);
        }

        // Static callback for device state change
        static void OnDeviceChange(const DeviceInfo *info, DeviceEvent type)
        {
            if (info == nullptr || g_lidar_component == nullptr)
            {
                return;
            }

            g_lidar_component->HandleDeviceChange(info, type);
        }

    private:
        bool InitLivoxSDK(const std::vector<std::string> &broadcast_codes)
        {
            if (is_initialized_)
            {
                RCLCPP_WARN(this->get_logger(), "Livox SDK already initialized");
                return false;
            }

            if (!Init())
            {
                Uninit();
                RCLCPP_ERROR(this->get_logger(), "Livox SDK init failed");
                return false;
            }

            LivoxSdkVersion sdk_version;
            GetLivoxSdkVersion(&sdk_version);
            RCLCPP_INFO(this->get_logger(), "Livox SDK version: %d.%d.%d",
                        sdk_version.major, sdk_version.minor, sdk_version.patch);

            SetBroadcastCallback(OnDeviceBroadcast);
            SetDeviceStateUpdateCallback(OnDeviceChange);

            // Add broadcast codes to whitelist
            if (!broadcast_codes.empty())
            {
                for (const auto &code : broadcast_codes)
                {
                    AddBroadcastCodeToWhitelist(code.c_str());
                }
                RCLCPP_INFO(this->get_logger(), "Added %zu broadcast codes to whitelist",
                            broadcast_codes.size());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "No broadcast codes specified, using auto-connect mode");
            }

            if (!Start())
            {
                Uninit();
                RCLCPP_ERROR(this->get_logger(), "Livox SDK start failed");
                return false;
            }

            g_lidar_component = this;
            is_initialized_ = true;

            return true;
        }

        void AddBroadcastCodeToWhitelist(const char *broadcast_code)
        {
            if (whitelist_count_ >= kMaxLidarCount)
            {
                RCLCPP_WARN(this->get_logger(), "Whitelist is full");
                return;
            }

            strncpy(broadcast_code_whitelist_[whitelist_count_], broadcast_code, kBroadcastCodeSize - 1);
            whitelist_count_++;
        }

        bool FindInWhitelist(const char *broadcast_code)
        {
            for (uint32_t i = 0; i < whitelist_count_; i++)
            {
                if (strncmp(broadcast_code, broadcast_code_whitelist_[i], kBroadcastCodeSize) == 0)
                {
                    return true;
                }
            }
            return false;
        }

        void HandleDeviceBroadcast(const BroadcastDeviceInfo *info)
        {
            RCLCPP_INFO(this->get_logger(), "Received broadcast from device: %s", info->broadcast_code);

            if (info->dev_type == kDeviceTypeHub)
            {
                RCLCPP_WARN(this->get_logger(), "Hub device detected, skipping: %s", info->broadcast_code);
                return;
            }

            // Check whitelist
            if (whitelist_count_ > 0 && !FindInWhitelist(info->broadcast_code))
            {
                RCLCPP_WARN(this->get_logger(), "Device not in whitelist: %s", info->broadcast_code);
                return;
            }

            uint8_t handle = 0;
            livox_status result = AddLidarToConnect(info->broadcast_code, &handle);
            if (result == kStatusSuccess && handle < kMaxLidarCount)
            {
                SetDataCallback(handle, GetLidarDataCallback, this);
                RCLCPP_INFO(this->get_logger(), "Added LiDAR to connection queue: handle %d", handle);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to add LiDAR: result=%d, handle=%d", result, handle);
            }
        }

        void HandleDeviceChange(const DeviceInfo *info, DeviceEvent type)
        {
            uint8_t handle = info->handle;
            if (handle >= kMaxLidarCount)
            {
                return;
            }

            if (type == kEventConnect)
            {
                RCLCPP_INFO(this->get_logger(), "LiDAR connected: %s (handle: %d)",
                            info->broadcast_code, handle);
                lidar_count_++;

                // Start sampling
                LidarStartSampling(handle, nullptr, nullptr);
            }
            else if (type == kEventDisconnect)
            {
                RCLCPP_WARN(this->get_logger(), "LiDAR disconnected: %s (handle: %d)",
                            info->broadcast_code, handle);
                if (lidar_count_ > 0)
                {
                    lidar_count_--;
                }
            }
        }

        void ProcessLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num)
        {
            (void)handle; // Unused parameter
            std::lock_guard<std::mutex> lock(cloud_mutex_);

            for (uint32_t i = 0; i < data_num; i++)
            {
                LivoxEthPacket *eth_packet = &data[i];

                // Get timestamp
                uint64_t timestamp = *((uint64_t *)(eth_packet->timestamp));

                // Process based on data type
                if (eth_packet->data_type == kCartesian)
                {
                    ProcessCartesianData(eth_packet, timestamp);
                }
                else if (eth_packet->data_type == kExtendCartesian)
                {
                    ProcessExtendCartesianData(eth_packet, timestamp);
                }
            }
        }

        void ProcessCartesianData(LivoxEthPacket *eth_packet, uint64_t timestamp)
        {
            (void)timestamp; // Unused - we use current system time instead
            const uint32_t kPointSize = sizeof(LivoxRawPoint);
            const uint32_t kPacketHeaderSize = 18;
            const uint32_t kMaxPacketSize = 1024;
            uint32_t point_num = (kMaxPacketSize - kPacketHeaderSize) / kPointSize;

            LivoxRawPoint *points = (LivoxRawPoint *)eth_packet->data;

            // Get current time in microseconds (same as mid70_pcl_fliter_test)
            uint64_t current_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                           std::chrono::high_resolution_clock::now().time_since_epoch())
                                           .count();

            for (uint32_t j = 0; j < point_num; j++)
            {
                // Skip invalid points
                if (points[j].x == 0 && points[j].y == 0 && points[j].z == 0)
                {
                    continue;
                }

                // Convert to meters and check range
                float x = points[j].x / 1000.0f;
                float y = points[j].y / 1000.0f;
                float z = points[j].z / 1000.0f;

                if (std::abs(x) > 200.0f || std::abs(y) > 200.0f || std::abs(z) > 200.0f)
                {
                    continue;
                }

                // Apply coordinate transformation (same as mid70_pcl_fliter_test)
                // Y and Z axes are flipped
                float transformed_x = x;
                float transformed_y = -y; // Y座標を反転（左右反転のため）
                float transformed_z = -z; // Z座標を反転（LiDARが上下逆向きのため）

                // Add point to buffer
                LivoxPoint point;
                point.x = transformed_x;
                point.y = transformed_y;
                point.z = transformed_z;
                point.reflectivity = points[j].reflectivity;
                point.tag = 0;
                point.line = 0;

                // Add timestamped point to buffer (same as mid70_pcl_fliter_test)
                TimestampedPoint tp;
                tp.point = point;
                tp.timestamp_us = current_time_us;

                // Check buffer size before adding
                if (timestamped_buffer_.size() < static_cast<size_t>(max_buffer_size_))
                {
                    timestamped_buffer_.push_back(tp);
                }
                else
                {
                    // Buffer full, skip this point (already logged in CleanOldPoints)
                    continue;
                }
            }
        }

        void ProcessExtendCartesianData(LivoxEthPacket *eth_packet, uint64_t timestamp)
        {
            (void)timestamp; // Unused - we use current system time instead
            const uint32_t kPointSize = sizeof(LivoxExtendRawPoint);
            const uint32_t kPacketHeaderSize = 18;
            const uint32_t kMaxPacketSize = 1024;
            uint32_t point_num = (kMaxPacketSize - kPacketHeaderSize) / kPointSize;

            LivoxExtendRawPoint *points = (LivoxExtendRawPoint *)eth_packet->data;

            // Get current time in microseconds (same as mid70_pcl_fliter_test)
            uint64_t current_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                           std::chrono::high_resolution_clock::now().time_since_epoch())
                                           .count();

            for (uint32_t j = 0; j < point_num; j++)
            {
                // Skip invalid points
                if (points[j].x == 0 && points[j].y == 0 && points[j].z == 0)
                {
                    continue;
                }

                // Convert to meters and check range
                float x = points[j].x / 1000.0f;
                float y = points[j].y / 1000.0f;
                float z = points[j].z / 1000.0f;

                if (std::abs(x) > 200.0f || std::abs(y) > 200.0f || std::abs(z) > 200.0f)
                {
                    continue;
                }

                // Apply coordinate transformation (same as mid70_pcl_fliter_test)
                // Y and Z axes are flipped
                float transformed_x = x;
                float transformed_y = -y; // Y座標を反転（左右反転のため）
                float transformed_z = -z; // Z座標を反転（LiDARが上下逆向きのため）

                // Add point to buffer
                LivoxPoint point;
                point.x = transformed_x;
                point.y = transformed_y;
                point.z = transformed_z;
                point.reflectivity = points[j].reflectivity;
                point.tag = points[j].tag;
                point.line = 0; // LivoxExtendRawPoint does not have line field

                // Add timestamped point to buffer (same as mid70_pcl_fliter_test)
                TimestampedPoint tp;
                tp.point = point;
                tp.timestamp_us = current_time_us;

                // Check buffer size before adding
                if (timestamped_buffer_.size() < static_cast<size_t>(max_buffer_size_))
                {
                    timestamped_buffer_.push_back(tp);
                }
                else
                {
                    // Buffer full, skip this point
                    continue;
                }
            }
        }

        void CleanOldPoints()
        {
            // Remove points older than buffer_time_seconds_ (same as mid70_pcl_fliter_test)
            uint64_t current_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                           std::chrono::high_resolution_clock::now().time_since_epoch())
                                           .count();
            uint64_t cutoff_time = current_time_us - static_cast<uint64_t>(buffer_time_seconds_ * 1e6);

            auto it = std::remove_if(timestamped_buffer_.begin(), timestamped_buffer_.end(),
                                     [cutoff_time](const TimestampedPoint &tp)
                                     {
                                         return tp.timestamp_us < cutoff_time;
                                     });

            size_t removed = std::distance(it, timestamped_buffer_.end());
            timestamped_buffer_.erase(it, timestamped_buffer_.end());

            // Also enforce max buffer size
            if (timestamped_buffer_.size() > static_cast<size_t>(max_buffer_size_))
            {
                size_t excess = timestamped_buffer_.size() - max_buffer_size_;
                timestamped_buffer_.erase(timestamped_buffer_.begin(),
                                          timestamped_buffer_.begin() + excess);
                removed += excess;
            }

            if (removed > 0)
            {
                RCLCPP_DEBUG(this->get_logger(), "Cleaned %zu old points from buffer", removed);
            }
        }

        void PublishPointCloud()
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);

            // Clean old points first (same as mid70_pcl_fliter_test)
            CleanOldPoints();

            if (timestamped_buffer_.empty())
            {
                return;
            }

            // Extract points from timestamped buffer with size check
            std::vector<LivoxPoint> point_cloud_buffer;
            point_cloud_buffer.reserve(std::min(timestamped_buffer_.size(), static_cast<size_t>(max_buffer_size_)));
            for (const auto &tp : timestamped_buffer_)
            {
                point_cloud_buffer.push_back(tp.point);
            }

            // Create PointCloud2 message
            auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            cloud_msg->header.stamp = this->now();
            cloud_msg->header.frame_id = "livox_frame";

            cloud_msg->height = 1;
            cloud_msg->width = point_cloud_buffer.size();
            cloud_msg->is_dense = false;
            cloud_msg->is_bigendian = false;

            // Define point fields
            sensor_msgs::msg::PointField field;
            int offset = 0;

            field.name = "x";
            field.offset = offset;
            field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field.count = 1;
            cloud_msg->fields.push_back(field);
            offset += 4;

            field.name = "y";
            field.offset = offset;
            field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field.count = 1;
            cloud_msg->fields.push_back(field);
            offset += 4;

            field.name = "z";
            field.offset = offset;
            field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field.count = 1;
            cloud_msg->fields.push_back(field);
            offset += 4;

            field.name = "reflectivity";
            field.offset = offset;
            field.datatype = sensor_msgs::msg::PointField::UINT8;
            field.count = 1;
            cloud_msg->fields.push_back(field);
            offset += 1;

            field.name = "tag";
            field.offset = offset;
            field.datatype = sensor_msgs::msg::PointField::UINT8;
            field.count = 1;
            cloud_msg->fields.push_back(field);
            offset += 1;

            field.name = "line";
            field.offset = offset;
            field.datatype = sensor_msgs::msg::PointField::UINT16;
            field.count = 1;
            cloud_msg->fields.push_back(field);
            offset += 2;

            cloud_msg->point_step = sizeof(LivoxPoint);
            cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;

            // Copy point data
            cloud_msg->data.resize(cloud_msg->row_step);
            memcpy(cloud_msg->data.data(), point_cloud_buffer.data(), cloud_msg->row_step);

            // Publish
            cloud_pub_->publish(*cloud_msg);

            // Log buffer stats
            static int publish_count = 0;
            publish_count++;
            if (publish_count % 10 == 0)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Published #%d: %zu points (%.1f sec buffer)",
                            publish_count, point_cloud_buffer.size(), buffer_time_seconds_);
            }
        }

        // Member variables
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::TimerBase::SharedPtr init_timer_;

        bool is_initialized_;
        uint32_t lidar_count_;
        uint32_t whitelist_count_ = 0;
        char broadcast_code_whitelist_[kMaxLidarCount][kBroadcastCodeSize];

        std::vector<TimestampedPoint> timestamped_buffer_; // Time-based buffer (same as mid70_pcl_fliter_test)
        std::mutex cloud_mutex_;

        double buffer_time_seconds_; // Buffer duration in seconds
        int max_buffer_size_;        // Maximum number of points in buffer

        static inline LidarGetCloudPointsComponent *g_lidar_component = nullptr;
    };

} // namespace lidar_get_cloud_points

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_get_cloud_points::LidarGetCloudPointsComponent)
