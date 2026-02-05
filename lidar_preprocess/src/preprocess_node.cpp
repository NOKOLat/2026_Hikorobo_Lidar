#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_preprocess/roi_filter.hpp"
#include "lidar_preprocess/voxel_filter.hpp"
#include "lidar_preprocess/sor_filter.hpp"

#include <memory>

class PreprocessNode : public rclcpp::Node
{
public:
    PreprocessNode() : Node("preprocess_node")
    {
        // Declare parameters
        this->declare_parameter("roi_min_x", 0.5);
        this->declare_parameter("roi_max_x", 20.0);
        this->declare_parameter("roi_min_y", -5.0);
        this->declare_parameter("roi_max_y", 5.0);
        this->declare_parameter("roi_min_z", -2.0);
        this->declare_parameter("roi_max_z", 5.0);
        this->declare_parameter("voxel_leaf_size", 0.01);
        this->declare_parameter("sor_mean_k", 50);
        this->declare_parameter("sor_std_dev_mul", 1.0);

        // Get parameters
        float roi_min_x = this->get_parameter("roi_min_x").as_double();
        float roi_max_x = this->get_parameter("roi_max_x").as_double();
        float roi_min_y = this->get_parameter("roi_min_y").as_double();
        float roi_max_y = this->get_parameter("roi_max_y").as_double();
        float roi_min_z = this->get_parameter("roi_min_z").as_double();
        float roi_max_z = this->get_parameter("roi_max_z").as_double();
        float voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();
        int sor_mean_k = this->get_parameter("sor_mean_k").as_int();
        double sor_std_dev_mul = this->get_parameter("sor_std_dev_mul").as_double();

        // Initialize filters
        roi_filter_ = std::make_unique<ROIFilter>(
            roi_min_x, roi_max_x, roi_min_y, roi_max_y, roi_min_z, roi_max_z);
        voxel_filter_ = std::make_unique<VoxelFilter>(voxel_leaf_size);
        sor_filter_ = std::make_unique<SORFilter>(sor_mean_k, sor_std_dev_mul);

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
                    roi_min_x, roi_max_x, roi_min_y, roi_max_y, roi_min_z, roi_max_z);
        RCLCPP_INFO(this->get_logger(), "Voxel leaf size: %.4f m", voxel_leaf_size);
        RCLCPP_INFO(this->get_logger(), "SOR: mean_k=%d, std_dev_mul=%.2f",
                    sor_mean_k, sor_std_dev_mul);
    }

private:
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
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PreprocessNode>());
    rclcpp::shutdown();
    return 0;
}
