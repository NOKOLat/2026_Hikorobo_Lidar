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
    PreprocessNode() : Node("preprocess_node")
    {
        // Load configuration from YAML file
        LoadConfigFromYAML();

        // Declare parameters with defaults from YAML
        this->declare_parameter("roi_min_x", roi_min_x_);
        this->declare_parameter("roi_max_x", roi_max_x_);
        this->declare_parameter("roi_min_y", roi_min_y_);
        this->declare_parameter("roi_max_y", roi_max_y_);
        this->declare_parameter("roi_min_z", roi_min_z_);
        this->declare_parameter("roi_max_z", roi_max_z_);
        this->declare_parameter("voxel_leaf_size", voxel_leaf_size_);
        this->declare_parameter("sor_mean_k", sor_mean_k_);
        this->declare_parameter("sor_std_dev_mul", sor_std_dev_mul_);

        // Get parameters (can be overridden by command line args)
        roi_min_x_ = this->get_parameter("roi_min_x").as_double();
        roi_max_x_ = this->get_parameter("roi_max_x").as_double();
        roi_min_y_ = this->get_parameter("roi_min_y").as_double();
        roi_max_y_ = this->get_parameter("roi_max_y").as_double();
        roi_min_z_ = this->get_parameter("roi_min_z").as_double();
        roi_max_z_ = this->get_parameter("roi_max_z").as_double();
        voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
        sor_mean_k_ = this->get_parameter("sor_mean_k").as_int();
        sor_std_dev_mul_ = this->get_parameter("sor_std_dev_mul").as_double();

        // Initialize filters
        roi_filter_ = std::make_unique<ROIFilter>(
            roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_, roi_min_z_, roi_max_z_);
        voxel_filter_ = std::make_unique<VoxelFilter>(voxel_leaf_size_);
        sor_filter_ = std::make_unique<SORFilter>(sor_mean_k_, sor_std_dev_mul_);

        // Create subscriber
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "livox/pointcloud", 10,
            std::bind(&PreprocessNode::CloudCallback, this, std::placeholders::_1));

        // Create publisher
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "preprocess/pointcloud", 10);

        RCLCPP_INFO(this->get_logger(), "Preprocess node started");
        RCLCPP_INFO(this->get_logger(),
                    "ROI: x[%.2f, %.2f], y[%.2f, %.2f], z[%.2f, %.2f]",
                    roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_, roi_min_z_, roi_max_z_);
        RCLCPP_INFO(this->get_logger(), "Voxel leaf size: %.4f m", voxel_leaf_size_);
        RCLCPP_INFO(this->get_logger(), "SOR: mean_k=%d, std_dev_mul=%.2f",
                    sor_mean_k_, sor_std_dev_mul_);
    }

private:
    // Configuration loading from YAML
    void LoadConfigFromYAML()
    {
        // Try to find config file in multiple locations
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
                        "YAML config file not found. Using hardcoded defaults.");
            // Set defaults
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

                // ROI parameters
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

                // Voxel filter parameters
                if (node_config["voxel"])
                {
                    YAML::Node voxel = node_config["voxel"];
                    voxel_leaf_size_ = voxel["leaf_size"].as<double>(0.01);
                }

                // SOR filter parameters
                if (node_config["sor"])
                {
                    YAML::Node sor = node_config["sor"];
                    sor_mean_k_ = sor["mean_k"].as<int>(50);
                    sor_std_dev_mul_ = sor["std_dev_mul"].as<double>(1.0);
                }

                RCLCPP_INFO(this->get_logger(),
                            "Loaded configuration from: %s", config_file.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to load YAML config: %s. Using defaults.", e.what());
            // Set defaults on error
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
    void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg, *cloud_input);

        // Apply ROI filter
        roi_filter_->filter(cloud_input, cloud_roi);
        RCLCPP_DEBUG(this->get_logger(), "ROI: %lu -> %lu points",
                     cloud_input->size(), cloud_roi->size());

        // Apply Voxel filter
        voxel_filter_->filter(cloud_roi, cloud_voxel);
        RCLCPP_DEBUG(this->get_logger(), "Voxel: %lu -> %lu points",
                     cloud_roi->size(), cloud_voxel->size());

        // Apply SOR filter
        sor_filter_->filter(cloud_voxel, cloud_sor);
        RCLCPP_DEBUG(this->get_logger(), "SOR: %lu -> %lu points",
                     cloud_voxel->size(), cloud_sor->size());

        // Convert back to ROS message and publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_sor, output_msg);
        output_msg.header = msg->header;
        cloud_pub_->publish(output_msg);
    }

    std::unique_ptr<ROIFilter> roi_filter_;
    std::unique_ptr<VoxelFilter> voxel_filter_;
    std::unique_ptr<SORFilter> sor_filter_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    // Configuration parameters
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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PreprocessNode>());
    rclcpp::shutdown();
    return 0;
}
