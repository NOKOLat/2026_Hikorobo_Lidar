#include "lidar_get_points/livox_node.hpp"

// Static member initialization
std::vector<uint8_t> LivoxNode::device_handles_;
std::mutex LivoxNode::cloud_mutex_;
std::deque<std::pair<std::chrono::steady_clock::time_point, std::vector<pcl::PointXYZI>>> LivoxNode::frame_buffer_;
std::vector<pcl::PointXYZI> LivoxNode::current_frame_buffer_;

LivoxNode::LivoxNode() : Node("livox_node")
{
    // YAMLファイルから設定を読み込む
    LoadConfigFromYAML();

    // YAMLのデフォルト値を使用してパラメータを宣言
    this->declare_parameter("frame_id", frame_id_);
    this->declare_parameter("publish_freq", publish_freq_);
    this->declare_parameter("integration_time_ms", integration_time_ms_);
    this->declare_parameter("flip_yz", flip_yz_);

    // パラメータを取得（コマンドライン引数でオーバーライド可能）
    frame_id_ = this->get_parameter("frame_id").as_string();
    publish_freq_ = this->get_parameter("publish_freq").as_double();
    integration_time_ms_ = this->get_parameter("integration_time_ms").as_int();
    flip_yz_ = this->get_parameter("flip_yz").as_bool();

    // パブリッシャーを作成
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "livox/pointcloud", 10);

    // Livox SDKを初期化
    if (!InitLivoxSdk())
    {
        RCLCPP_ERROR(this->get_logger(), "Livox SDKの初期化に失敗しました");
        return;
    }

    // 発行用タイマーを作成
    auto period = std::chrono::duration<double>(1.0 / publish_freq_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&LivoxNode::PublishPointCloud, this));

    RCLCPP_INFO(this->get_logger(),
                "Livoxノードを開始: frame_id=%s, publish_freq=%.1f Hz, integration_time=%d ms, flip_yz=%s",
                frame_id_.c_str(), publish_freq_, integration_time_ms_, flip_yz_ ? "true" : "false");
}

LivoxNode::~LivoxNode()
{
    Uninit();
}

void LivoxNode::LoadConfigFromYAML()
{
    // 複数の場所から設定ファイルを探す
    std::vector<std::string> config_paths = {
        "config/livox_node.yml",
        "../config/livox_node.yml",
        "../../config/livox_node.yml"};

    std::string config_file;
    for (const auto &path : config_paths)
    {
        if (std::filesystem::exists(path))
        {
            config_file = path;
            break;
        }
    }

    if (config_file.empty())
    {
        RCLCPP_WARN(this->get_logger(),
                    "YAML設定ファイルが見つかりません。ハードコードされたデフォルト値を使用します。");
        // デフォルト値を設定
        frame_id_ = "livox_frame";
        publish_freq_ = 10.0;
        integration_time_ms_ = 1000;
        flip_yz_ = false;
        return;
    }

    try
    {
        YAML::Node config = YAML::LoadFile(config_file);

        if (config["livox_node"])
        {
            YAML::Node node_config = config["livox_node"];

            frame_id_ = node_config["frame_id"].as<std::string>("livox_frame");
            publish_freq_ = node_config["publish_freq"].as<double>(10.0);
            integration_time_ms_ = node_config["integration_time_ms"].as<int>(1000);
            flip_yz_ = node_config["flip_yz"].as<bool>(false);

            RCLCPP_INFO(this->get_logger(),
                        "設定を読み込みました: %s (frame_id=%s, freq=%.1f Hz, integration_time=%d ms, flip_yz=%s)",
                        config_file.c_str(), frame_id_.c_str(), publish_freq_, integration_time_ms_, flip_yz_ ? "true" : "false");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "YAML設定の読み込みに失敗しました: %s。デフォルト値を使用します。", e.what());
        // エラー時はデフォルト値を設定
        frame_id_ = "livox_frame";
        publish_freq_ = 10.0;
        integration_time_ms_ = 1000;
        flip_yz_ = false;
    }
}

bool LivoxNode::InitLivoxSdk()
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

    // デバイス検出を開始
    if (!Start())
    {
        Uninit();
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Livox SDK initialized, discovering devices...");
    return true;
}

void LivoxNode::OnDeviceBroadcast(const BroadcastDeviceInfo *info)
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

void LivoxNode::OnDeviceChange(const DeviceInfo *info, DeviceEvent type)
{
    if (info == nullptr)
    {
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("livox_node"),
                "Device state change: SN=%s, Event=%d, State=%d",
                info->broadcast_code, type, info->state);

    // デバイスが接続されて通常状態になったときにサンプリングを開始
    if (type == kEventConnect ||
        (type == kEventStateChange && info->state == kLidarStateNormal))
    {

        uint8_t handle = info->handle;

        // サンプリング開始前に通常状態を確認
        if (info->state == kLidarStateNormal)
        {
            LidarStartSampling(handle, OnSampleCallback, nullptr);
            RCLCPP_INFO(rclcpp::get_logger("livox_node"),
                        "Starting sampling for device handle: %d", handle);
        }
    }
}

void LivoxNode::OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data)
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

void LivoxNode::GetPointCloudCallback(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data)
{
    (void)handle;
    (void)client_data;

    if (data == nullptr)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(cloud_mutex_);

    // データ型に基づいてポイントクラウドデータを処理
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

void LivoxNode::IntegrateCurrentFrame(const std::chrono::steady_clock::time_point &timestamp)
{
    // 現在のフレームがある場合は統合バッファに追加（タイムスタンプ付き）
    if (!current_frame_buffer_.empty())
    {
        frame_buffer_.push_back({timestamp, current_frame_buffer_});

        RCLCPP_DEBUG(this->get_logger(),
                     "Integrated frame with %zu points, buffer now has %zu frames",
                     current_frame_buffer_.size(), frame_buffer_.size());

        // 次のフレームのために現在のバッファをクリア
        current_frame_buffer_.clear();
    }
}

void LivoxNode::PublishPointCloud()
{
    std::lock_guard<std::mutex> lock(cloud_mutex_);

    // 現在時刻を取得（タイムスタンプの一貫性のため）
    auto now = std::chrono::steady_clock::now();

    // 現在のフレームを統合バッファに追加
    IntegrateCurrentFrame(now);

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

    // 保持すべきフレーム数を計算
    // integration_time_ms と publish_freq から決定
    // 例: integration_time_ms=400, publish_freq=10Hz(100ms) → 400/100 = 4フレーム
    double frame_period_ms = 1000.0 / publish_freq_;
    int max_frames = static_cast<int>(std::round(integration_time_ms_ / frame_period_ms));

    // 最低1フレームは保持
    if (max_frames < 1) {
        max_frames = 1;
    }

    size_t points_removed = 0;
    int frames_removed = 0;

    // max_frames を超えるフレームを削除（FIFO）
    while (frame_buffer_.size() > static_cast<size_t>(max_frames))
    {
        size_t removed_points = frame_buffer_.front().second.size();
        auto frame_age = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - frame_buffer_.front().first);
        points_removed += removed_points;
        frames_removed++;
        frame_buffer_.pop_front();
        RCLCPP_DEBUG(this->get_logger(),
                     "Removed frame: age=%ld ms, removed %zu points (keeping %d frames)",
                     frame_age.count(), removed_points, max_frames);
    }

    // 統合時間内のフレームの点群を統合
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    size_t total_points = 0;
    for (const auto &frame_pair : frame_buffer_)
    {
        total_points += frame_pair.second.size();
    }

    cloud->points.reserve(total_points);

    // フレームの詳細情報を収集（デバッグ用）
    std::vector<size_t> frame_sizes;
    for (const auto &frame_pair : frame_buffer_)
    {
        frame_sizes.push_back(frame_pair.second.size());

        for (const auto &point : frame_pair.second)
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

    // 詳細なログ出力
    std::stringstream frame_info;
    frame_info << "[";
    for (size_t i = 0; i < frame_sizes.size(); ++i)
    {
        if (i > 0)
            frame_info << ", ";
        frame_info << frame_sizes[i];
    }
    frame_info << "]";

    RCLCPP_INFO(this->get_logger(),
                "Published: %zu points from %zu frames (max=%d, removed %d frames/%zu points) | integration_time=%d ms | frame_sizes=%s",
                cloud->points.size(), frame_buffer_.size(), max_frames, frames_removed, points_removed,
                integration_time_ms_, frame_info.str().c_str());
}
