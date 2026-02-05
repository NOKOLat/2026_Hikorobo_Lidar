#include "lidar_background_diff/background_diff_node.hpp"
#include <pcl/common/centroid.h>

BackgroundDiffNode::BackgroundDiffNode() : Node("background_diff_node")
{
    // YAMLファイルから設定を読み込む
    LoadConfigFromYAML();

    // YAMLのデフォルト値を使用してパラメータを宣言
    this->declare_parameter("background_frame_count", background_frame_count_);
    this->declare_parameter("distance_threshold", distance_threshold_);

    // パラメータを取得（コマンドライン引数でオーバーライド可能）
    background_frame_count_ = this->get_parameter("background_frame_count").as_int();
    distance_threshold_ = this->get_parameter("distance_threshold").as_double();

    // 状態を初期化
    is_background_ready_ = false;
    received_frame_count_ = 0;
    background_model_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    background_kdtree_ = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>);

    // サブスクライバーを作成（前処理済みポイントクラウドを入力）
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "preprocess/pointcloud", 10,
        std::bind(&BackgroundDiffNode::CloudCallback, this, std::placeholders::_1));

    // パブリッシャーを作成（背景差分の結果を出力）
    diff_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "background_diff/pointcloud", 10);

    RCLCPP_INFO(this->get_logger(), "背景差分ノードを開始しました");
    RCLCPP_INFO(this->get_logger(), "背景フレーム数: %d", background_frame_count_);
    RCLCPP_INFO(this->get_logger(), "距離閾値: %.3f m", distance_threshold_);
    RCLCPP_INFO(this->get_logger(), "最初の %d フレームで背景モデルを構築します...",
                background_frame_count_);
}

void BackgroundDiffNode::LoadConfigFromYAML()
{
    // 複数の場所から設定ファイルを探す
    std::vector<std::string> config_paths = {
        "config/background_diff_node.yml",
        "../config/background_diff_node.yml",
        "../../config/background_diff_node.yml"};

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
                    "YAML設定ファイルが見つかりません。デフォルト値を使用します。");
        // デフォルト値を設定
        background_frame_count_ = 50;
        distance_threshold_ = 0.05; // 5cm
        return;
    }

    try
    {
        YAML::Node config = YAML::LoadFile(config_file);

        if (config["background_diff_node"])
        {
            YAML::Node node_config = config["background_diff_node"];

            background_frame_count_ = node_config["background_frame_count"].as<int>(50);
            distance_threshold_ = node_config["distance_threshold"].as<double>(0.05);

            RCLCPP_INFO(this->get_logger(), "YAML設定ファイルを読み込みました: %s",
                        config_file.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "YAML内に 'background_diff_node' セクションが見つかりません。デフォルト値を使用します。");
            background_frame_count_ = 50;
            distance_threshold_ = 0.05;
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "YAML読み込みエラー: %s", e.what());
        RCLCPP_WARN(this->get_logger(), "デフォルト値を使用します。");
        background_frame_count_ = 50;
        distance_threshold_ = 0.05;
    }
}

void BackgroundDiffNode::CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // ROS メッセージをPCL形式に変換
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->points.empty())
    {
        RCLCPP_WARN(this->get_logger(), "空のポイントクラウドを受信しました");
        return;
    }

    // 背景モデルが未構築の場合、構築処理を行う
    if (!is_background_ready_)
    {
        BuildBackgroundModel(cloud);
        return;
    }

    // 背景差分を計算
    pcl::PointCloud<pcl::PointXYZ>::Ptr diff_cloud = ComputeBackgroundDiff(cloud);

    // 結果をパブリッシュ
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*diff_cloud, output_msg);
    output_msg.header = msg->header; // タイムスタンプとフレームIDを保持
    diff_pub_->publish(output_msg);

    RCLCPP_DEBUG(this->get_logger(), "背景差分完了: 入力=%zu点, 出力=%zu点",
                 cloud->points.size(), diff_cloud->points.size());
}

void BackgroundDiffNode::BuildBackgroundModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // 背景フレームを蓄積
    background_frames_.push_back(cloud);
    received_frame_count_++;

    RCLCPP_INFO(this->get_logger(), "背景フレーム蓄積中: %d / %d",
                received_frame_count_, background_frame_count_);

    // 指定フレーム数に達したら背景モデルを構築
    if (received_frame_count_ >= background_frame_count_)
    {
        RCLCPP_INFO(this->get_logger(), "背景モデルを構築中...");

        // 全フレームのポイントを統合
        background_model_->clear();
        for (const auto &frame : background_frames_)
        {
            *background_model_ += *frame;
        }

        RCLCPP_INFO(this->get_logger(), "背景モデル構築完了: 総ポイント数 = %zu",
                    background_model_->points.size());

        // KD-Treeを構築（高速な最近傍探索のため）
        if (!background_model_->points.empty())
        {
            background_kdtree_->setInputCloud(background_model_);
            RCLCPP_INFO(this->get_logger(), "KD-Tree構築完了");
        }

        // 蓄積したフレームを解放（メモリ節約）
        background_frames_.clear();
        background_frames_.shrink_to_fit();

        // 背景モデル構築完了フラグを立てる
        is_background_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "背景モデルの準備が完了しました。背景差分処理を開始します。");
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr BackgroundDiffNode::ComputeBackgroundDiff(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr diff_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 背景モデルが空の場合、全ポイントを返す
    if (background_model_->points.empty())
    {
        RCLCPP_WARN(this->get_logger(), "背景モデルが空です");
        return cloud;
    }

    // 各ポイントについて、背景モデルとの最近傍距離を計算
    for (const auto &point : cloud->points)
    {
        // 最近傍探索
        std::vector<int> nearest_indices(1);
        std::vector<float> nearest_distances(1);

        if (background_kdtree_->nearestKSearch(point, 1, nearest_indices, nearest_distances) > 0)
        {
            // 距離が閾値以上の場合、動体と判定して出力に追加
            float distance = std::sqrt(nearest_distances[0]);
            if (distance >= distance_threshold_)
            {
                diff_cloud->points.push_back(point);
            }
        }
        else
        {
            // 最近傍が見つからない場合も動体として扱う
            diff_cloud->points.push_back(point);
        }
    }

    // ポイントクラウドのプロパティを設定
    diff_cloud->width = diff_cloud->points.size();
    diff_cloud->height = 1;
    diff_cloud->is_dense = true;

    return diff_cloud;
}
