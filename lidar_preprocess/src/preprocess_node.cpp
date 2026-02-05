#include "lidar_preprocess/preprocess_node.hpp"

PreprocessNode::PreprocessNode() : Node("preprocess_node")
{
    // YAMLファイルから設定を読み込む
    LoadConfigFromYAML();

    // YAMLのデフォルト値を使用してパラメータを宣言
    this->declare_parameter("roi_min_x", roi_min_x_);
    this->declare_parameter("roi_max_x", roi_max_x_);
    this->declare_parameter("roi_min_y", roi_min_y_);
    this->declare_parameter("roi_max_y", roi_max_y_);
    this->declare_parameter("roi_min_z", roi_min_z_);
    this->declare_parameter("roi_max_z", roi_max_z_);
    this->declare_parameter("voxel_leaf_size", voxel_leaf_size_);
    this->declare_parameter("sor_mean_k", sor_mean_k_);
    this->declare_parameter("sor_std_dev_mul", sor_std_dev_mul_);

    // パラメータを取得（コマンドライン引数でオーバーライド可能）
    roi_min_x_ = this->get_parameter("roi_min_x").as_double();
    roi_max_x_ = this->get_parameter("roi_max_x").as_double();
    roi_min_y_ = this->get_parameter("roi_min_y").as_double();
    roi_max_y_ = this->get_parameter("roi_max_y").as_double();
    roi_min_z_ = this->get_parameter("roi_min_z").as_double();
    roi_max_z_ = this->get_parameter("roi_max_z").as_double();
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
    sor_mean_k_ = this->get_parameter("sor_mean_k").as_int();
    sor_std_dev_mul_ = this->get_parameter("sor_std_dev_mul").as_double();

    // フィルタを初期化
    roi_filter_ = std::make_unique<ROIFilter>(
        roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_, roi_min_z_, roi_max_z_);
    voxel_filter_ = std::make_unique<VoxelFilter>(voxel_leaf_size_);
    sor_filter_ = std::make_unique<SORFilter>(sor_mean_k_, sor_std_dev_mul_);

    // サブスクライバーを作成
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "livox/pointcloud", 10,
        std::bind(&PreprocessNode::CloudCallback, this, std::placeholders::_1));

    // パブリッシャーを作成
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "preprocess/pointcloud", 10);

    RCLCPP_INFO(this->get_logger(), "前処理ノードを開始しました");
    RCLCPP_INFO(this->get_logger(),
                "ROI: x[%.2f, %.2f], y[%.2f, %.2f], z[%.2f, %.2f]",
                roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_, roi_min_z_, roi_max_z_);
    RCLCPP_INFO(this->get_logger(), "ボクセルリーフサイズ: %.4f m", voxel_leaf_size_);
    RCLCPP_INFO(this->get_logger(), "SOR: mean_k=%d, std_dev_mul=%.2f",
                sor_mean_k_, sor_std_dev_mul_);
}

void PreprocessNode::LoadConfigFromYAML()
{
    // 複数の場所から設定ファイルを探す
    std::vector<std::string> config_paths = {
        "config/preprocess_node.yml",
        "../config/preprocess_node.yml",
        "../../config/preprocess_node.yml"};

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
        roi_min_x_ = 0.5;
        roi_max_x_ = 20.0;
        roi_min_y_ = -5.0;
        roi_max_y_ = 5.0;
        roi_min_z_ = -2.0;
        roi_max_z_ = 5.0;
        voxel_leaf_size_ = 0.01;
        sor_mean_k_ = 50;
        sor_std_dev_mul_ = 1.0;
        return;
    }

    try
    {
        YAML::Node config = YAML::LoadFile(config_file);

        if (config["preprocess_node"])
        {
            YAML::Node node_config = config["preprocess_node"];

            // ROIフィルタパラメータ
            if (node_config["roi"])
            {
                YAML::Node roi = node_config["roi"];
                roi_min_x_ = roi["min_x"].as<double>(0.5);
                roi_max_x_ = roi["max_x"].as<double>(20.0);
                roi_min_y_ = roi["min_y"].as<double>(-5.0);
                roi_max_y_ = roi["max_y"].as<double>(5.0);
                roi_min_z_ = roi["min_z"].as<double>(-2.0);
                roi_max_z_ = roi["max_z"].as<double>(5.0);
            }

            // ボクセルフィルタパラメータ
            if (node_config["voxel"])
            {
                YAML::Node voxel = node_config["voxel"];
                voxel_leaf_size_ = voxel["leaf_size"].as<double>(0.01);
            }

            // 統計的外れ値除去フィルタパラメータ
            if (node_config["sor"])
            {
                YAML::Node sor = node_config["sor"];
                sor_mean_k_ = sor["mean_k"].as<int>(50);
                sor_std_dev_mul_ = sor["std_dev_mul"].as<double>(1.0);
            }

            RCLCPP_INFO(this->get_logger(),
                        "設定を読み込みました: %s", config_file.c_str());
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "YAML設定の読み込みに失敗しました: %s。デフォルト値を使用します。", e.what());
        // エラー時はデフォルト値を設定
        roi_min_x_ = 0.5;
        roi_max_x_ = 20.0;
        roi_min_y_ = -5.0;
        roi_max_y_ = 5.0;
        roi_min_z_ = -2.0;
        roi_max_z_ = 5.0;
        voxel_leaf_size_ = 0.01;
        sor_mean_k_ = 50;
        sor_std_dev_mul_ = 1.0;
    }
}

void PreprocessNode::CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // ROS メッセージを PCL ポイントクラウドに変換
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *cloud_input);

    // ROIフィルタを適用
    roi_filter_->filter(cloud_input, cloud_roi);
    RCLCPP_DEBUG(this->get_logger(), "ROI: %lu -> %lu ポイント",
                 cloud_input->size(), cloud_roi->size());

    // ボクセルフィルタを適用
    voxel_filter_->filter(cloud_roi, cloud_voxel);
    RCLCPP_DEBUG(this->get_logger(), "ボクセル: %lu -> %lu ポイント",
                 cloud_roi->size(), cloud_voxel->size());

    // 統計的外れ値除去フィルタを適用
    sor_filter_->filter(cloud_voxel, cloud_sor);
    RCLCPP_DEBUG(this->get_logger(), "SOR: %lu -> %lu ポイント",
                 cloud_voxel->size(), cloud_sor->size());

    // ROS メッセージに変換して発行
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_sor, output_msg);
    output_msg.header = msg->header;
    cloud_pub_->publish(output_msg);
}