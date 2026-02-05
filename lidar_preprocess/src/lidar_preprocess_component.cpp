#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include "lidar_preprocess/passthrough_filter.hpp"
#include "lidar_preprocess/voxel_grid_filter.hpp"
#include "lidar_preprocess/statistical_outlier_removal_filter.hpp"

namespace lidar_preprocess
{

    // ===== LidarPreprocessComponent =====
    // LiDAR前処理コンポーネント

    class LidarPreprocessComponent : public rclcpp::Node
    {
    public:
        explicit LidarPreprocessComponent(const rclcpp::NodeOptions &options)
            : Node("lidar_preprocess", options)
        {
            // 設定ファイルからパラメータを読み込む
            loadConfig();

            // Subscriber生成
            cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "livox/cloud_points", 10,
                std::bind(&LidarPreprocessComponent::cloudCallback, this, std::placeholders::_1));

            // Publisher生成
            processed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "livox/processed_cloud", 10);

            RCLCPP_INFO(this->get_logger(), "LiDAR Preprocess Component initialized");
            RCLCPP_INFO(this->get_logger(), "Filter params: SOR(k=%d, thresh=%.1f), Voxel(%.3fm)",
                        sor_mean_k_, sor_std_dev_mul_thresh_, voxel_leaf_size_);
            RCLCPP_INFO(this->get_logger(), "  ROI X: %.1f to %.1f m", roi_x_min_, roi_x_max_);
            RCLCPP_INFO(this->get_logger(), "  ROI Y: %.1f to %.1f m", roi_y_min_, roi_y_max_);
            RCLCPP_INFO(this->get_logger(), "  ROI Z: %.1f to %.1f m", roi_z_min_, roi_z_max_);
        }

    private:
        // ===== 設定ファイル読み込み関数 =====
        void loadConfig()
        {
            // パッケージのshareディレクトリを取得
            std::string package_share_directory =
                ament_index_cpp::get_package_share_directory("lidar_preprocess");
            std::string config_file_path = package_share_directory + "/lidar_preprocess_config.yaml";

            // 設定ファイルが存在しない場合は、ソースディレクトリも確認
            if (!std::filesystem::exists(config_file_path))
            {
                // ビルドしていない場合のために、ソースディレクトリも確認
                std::string source_config_path = std::string(__FILE__);
                source_config_path = source_config_path.substr(0, source_config_path.find_last_of("/"));
                source_config_path = source_config_path.substr(0, source_config_path.find_last_of("/"));
                source_config_path += "/lidar_preprocess_config.yaml";

                if (std::filesystem::exists(source_config_path))
                {
                    config_file_path = source_config_path;
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Config file not found. Using default parameters.");
                    return;
                }
            }

            RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file_path.c_str());

            // YAMLファイルを読み込む
            YAML::Node config = YAML::LoadFile(config_file_path);

            // パラメータを読み込む
            sor_mean_k_ = config["sor_mean_k"].as<int>();
            sor_std_dev_mul_thresh_ = config["sor_std_dev_mul_thresh"].as<double>();
            voxel_leaf_size_ = config["voxel_leaf_size"].as<double>();
            roi_x_min_ = config["roi_x_min"].as<double>();
            roi_x_max_ = config["roi_x_max"].as<double>();
            roi_y_min_ = config["roi_y_min"].as<double>();
            roi_y_max_ = config["roi_y_max"].as<double>();
            roi_z_min_ = config["roi_z_min"].as<double>();
            roi_z_max_ = config["roi_z_max"].as<double>();

            RCLCPP_INFO(this->get_logger(), "Configuration loaded successfully");
        }

        // ===== コールバック関数 =====
        void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            // 空チェック
            if (msg->width * msg->height == 0)
            {
                RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
                return;
            }

            // ROS2 PointCloud2をPCL PointCloudに変換
            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *input_cloud);

            if (input_cloud->empty())
            {
                RCLCPP_WARN(this->get_logger(), "Input cloud is empty after conversion");
                return;
            }

            // 点群サイズチェック（メモリ安全性）
            if (input_cloud->size() > 100000)
            {
                RCLCPP_WARN(this->get_logger(), "Input cloud too large (%zu points), skipping", input_cloud->size());
                return;
            }

            // 追加の安全性チェック：極端な値を持つ点をチェック
            bool has_invalid_points = false;
            for (const auto &pt : input_cloud->points)
            {
                if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z) ||
                    std::abs(pt.x) > 1000.0f || std::abs(pt.y) > 1000.0f || std::abs(pt.z) > 1000.0f)
                {
                    has_invalid_points = true;
                    break;
                }
            }

            if (has_invalid_points)
            {
                RCLCPP_WARN(this->get_logger(), "Input cloud contains invalid or extreme values, skipping");
                return;
            }

            // 一時的なポイントクラウドポインタ
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            // 1. PassThroughフィルタ - X軸（ROIフィルタ）
            if (!PassThroughFilter::apply(input_cloud, temp_cloud1, "x", roi_x_min_, roi_x_max_))
            {
                RCLCPP_WARN(this->get_logger(), "PassThrough filter (X) failed");
                return;
            }

            // 2. PassThroughフィルタ - Y軸（ROIフィルタ）
            if (!PassThroughFilter::apply(temp_cloud1, temp_cloud2, "y", roi_y_min_, roi_y_max_))
            {
                RCLCPP_WARN(this->get_logger(), "PassThrough filter (Y) failed");
                return;
            }

            // 3. PassThroughフィルタ - Z軸（ROIフィルタ）
            if (!PassThroughFilter::apply(temp_cloud2, temp_cloud1, "z", roi_z_min_, roi_z_max_))
            {
                RCLCPP_WARN(this->get_logger(), "PassThrough filter (Z) failed");
                return;
            }

            // 4. VoxelGridフィルタ（ダウンサンプリング）
            if (!VoxelGridFilter::apply(temp_cloud1, temp_cloud2, voxel_leaf_size_))
            {
                RCLCPP_ERROR(this->get_logger(), "VoxelGrid filter failed");
                return;
            }

            // 5. 統計的外れ値除去（外れ値除去）
            if (!StatisticalOutlierRemovalFilter::apply(temp_cloud2, output_cloud, sor_mean_k_, sor_std_dev_mul_thresh_))
            {
                RCLCPP_WARN(this->get_logger(), "Statistical Outlier Removal filter failed");
                return;
            }

            if (output_cloud->empty())
            {
                RCLCPP_WARN(this->get_logger(), "Output cloud is empty after filtering");
                return;
            }

            // PCL PointCloudをROS2 PointCloud2に変換
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*output_cloud, output_msg);
            output_msg.header = msg->header;

            // 前処理済みポイントクラウドをパブリッシュ
            processed_cloud_pub_->publish(output_msg);
        }

        // ===== メンバ変数 =====
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_pub_;

        // ===== フィルタパラメータ（設定ファイルから読み込み） =====
        // 統計的外れ値除去パラメータ
        int sor_mean_k_ = 50;                 // 近傍点数 [-]
        double sor_std_dev_mul_thresh_ = 1.0; // 標準偏差乗数閾値 [-]

        // VoxelGridフィルタパラメータ
        double voxel_leaf_size_ = 0.025; // ボクセルサイズ [m] (2.5 cm)

        // ROI（関心領域）パラメータ
        double roi_x_min_ = 0.0;  // X軸最小値 [m]
        double roi_x_max_ = 25.0; // X軸最大値 [m]
        double roi_y_min_ = -5.0; // Y軸最小値 [m]
        double roi_y_max_ = 5.0;  // Y軸最大値 [m]
        double roi_z_min_ = -2.0; // Z軸最小値 [m]
        double roi_z_max_ = 5.0;  // Z軸最大値 [m]
    };

} // namespace lidar_preprocess

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_preprocess::LidarPreprocessComponent)
