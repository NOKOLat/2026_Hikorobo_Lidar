#include "lidar_clustering/clustering_node.hpp"

ClusteringNode::ClusteringNode() : Node("clustering_node")
{
    // YAMLファイルから設定を読み込む
    LoadConfigFromYAML();

    // YAMLのデフォルト値を使用してパラメータを宣言
    this->declare_parameter("cluster_tolerance", cluster_tolerance_);
    this->declare_parameter("min_cluster_size", min_cluster_size_);
    this->declare_parameter("max_cluster_size", max_cluster_size_);

    // パラメータを取得（コマンドライン引数でオーバーライド可能）
    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
    max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();

    // サブスクライバーを作成（背景差分済みポイントクラウドを入力）
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "background_diff/pointcloud", 10,
        std::bind(&ClusteringNode::CloudCallback, this, std::placeholders::_1));

    // マーカーパブリッシャーを作成（バウンディングボックスをRViz2に表示）
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "clustering/markers", 10);

    RCLCPP_INFO(this->get_logger(), "クラスタリングノードを開始しました");
    RCLCPP_INFO(this->get_logger(), "クラスタ距離閾値: %.3f m", cluster_tolerance_);
    RCLCPP_INFO(this->get_logger(), "最小クラスタサイズ: %d 点", min_cluster_size_);
    RCLCPP_INFO(this->get_logger(), "最大クラスタサイズ: %d 点", max_cluster_size_);
}

void ClusteringNode::LoadConfigFromYAML()
{
    // 複数の場所から設定ファイルを探す
    std::vector<std::string> config_paths = {
        "config/clustering_node.yml",
        "../config/clustering_node.yml",
        "../../config/clustering_node.yml"};

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
        cluster_tolerance_ = 0.1;   // 10cm
        min_cluster_size_ = 10;
        max_cluster_size_ = 10000;
        return;
    }

    try
    {
        YAML::Node config = YAML::LoadFile(config_file);

        if (config["clustering_node"])
        {
            YAML::Node node_config = config["clustering_node"];

            cluster_tolerance_ = node_config["cluster_tolerance"].as<double>(0.1);
            min_cluster_size_ = node_config["min_cluster_size"].as<int>(10);
            max_cluster_size_ = node_config["max_cluster_size"].as<int>(10000);

            RCLCPP_INFO(this->get_logger(), "YAML設定ファイルを読み込みました: %s",
                        config_file.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "YAML内に 'clustering_node' セクションが見つかりません。デフォルト値を使用します。");
            cluster_tolerance_ = 0.1;
            min_cluster_size_ = 10;
            max_cluster_size_ = 10000;
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "YAML読み込みエラー: %s", e.what());
        RCLCPP_WARN(this->get_logger(), "デフォルト値を使用します。");
        cluster_tolerance_ = 0.1;
        min_cluster_size_ = 10;
        max_cluster_size_ = 10000;
    }
}

void ClusteringNode::CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // ROS メッセージをPCL形式に変換
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->points.empty())
    {
        // 空の場合、マーカーをクリアするために空のMarkerArrayをパブリッシュ
        visualization_msgs::msg::MarkerArray clear_markers;
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        clear_markers.markers.push_back(delete_marker);
        marker_pub_->publish(clear_markers);
        return;
    }

    // KD-Treeを作成
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // ユークリッドクラスタリングを実行
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // MarkerArrayを作成
    visualization_msgs::msg::MarkerArray marker_array;

    // まず前回のマーカーを全削除
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    int marker_id = 0;
    for (const auto &indices : cluster_indices)
    {
        // クラスタのポイントを抽出
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : indices.indices)
        {
            cluster_cloud->points.push_back(cloud->points[idx]);
        }

        // バウンディングボックスの最小・最大点を計算
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);

        // 重心を計算
        float center_x = (min_pt.x + max_pt.x) / 2.0f;
        float center_y = (min_pt.y + max_pt.y) / 2.0f;
        float center_z = (min_pt.z + max_pt.z) / 2.0f;

        // サイズを計算
        float size_x = max_pt.x - min_pt.x;
        float size_y = max_pt.y - min_pt.y;
        float size_z = max_pt.z - min_pt.z;

        // バウンディングボックスマーカーを作成
        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = "clustering";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 位置（重心）
        marker.pose.position.x = center_x;
        marker.pose.position.y = center_y;
        marker.pose.position.z = center_z;
        marker.pose.orientation.w = 1.0;

        // サイズ
        marker.scale.x = size_x;
        marker.scale.y = size_y;
        marker.scale.z = size_z;

        // 色（緑色、半透明）
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.3f;

        // 表示時間（次のフレームまで保持）
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);

        marker_array.markers.push_back(marker);
    }

    // マーカーをパブリッシュ
    marker_pub_->publish(marker_array);

    RCLCPP_DEBUG(this->get_logger(), "クラスタリング完了: 入力=%zu点, クラスタ数=%zu",
                 cloud->points.size(), cluster_indices.size());
}
