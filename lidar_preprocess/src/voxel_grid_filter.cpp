#include "lidar_preprocess/voxel_grid_filter.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <iostream>

namespace lidar_preprocess
{

// Voxel Grid filter application function
bool VoxelGridFilter::apply(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
    double leaf_size)
{
    // Input validation
    if (!input_cloud || input_cloud->empty())
    {
        return false;
    }

    try
    {
        // Check point cloud bounds (overflow protection)
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*input_cloud, min_pt, max_pt);

        float range_x = max_pt.x - min_pt.x;
        float range_y = max_pt.y - min_pt.y;
        float range_z = max_pt.z - min_pt.z;

        // If range is too large, overflow may occur
        const float MAX_SAFE_RANGE = 100.0f; // 100m is safe
        if (range_x > MAX_SAFE_RANGE || range_y > MAX_SAFE_RANGE || range_z > MAX_SAFE_RANGE)
        {
            std::cerr << "[VoxelGrid] Point cloud range too large for leaf_size " << leaf_size << "m" << std::endl;
            std::cerr << "  Range X: " << range_x << "m (" << min_pt.x << " to " << max_pt.x << ")" << std::endl;
            std::cerr << "  Range Y: " << range_y << "m (" << min_pt.y << " to " << max_pt.y << ")" << std::endl;
            std::cerr << "  Range Z: " << range_z << "m (" << min_pt.z << " to " << max_pt.z << ")" << std::endl;
            return false;
        }

        // Configure Voxel Grid filter
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(input_cloud);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);

        // Apply filter
        vg.filter(*output_cloud);

        return !output_cloud->empty();
    }
    catch (const std::exception& e)
    {
        std::cerr << "[VoxelGrid] Exception: " << e.what() << std::endl;
        return false;
    }
}

} // namespace lidar_preprocess
