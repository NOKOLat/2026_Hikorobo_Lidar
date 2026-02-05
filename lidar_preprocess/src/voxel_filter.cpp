#include "lidar_preprocess/voxel_filter.hpp"
#include <pcl/filters/voxel_grid.h>

VoxelFilter::VoxelFilter(float leaf_size)
    : leaf_size_(leaf_size)
{
}

void VoxelFilter::filter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(input);
    voxel_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    voxel_filter.filter(*output);
}

void VoxelFilter::setLeafSize(float leaf_size)
{
    leaf_size_ = leaf_size;
}

float VoxelFilter::getLeafSize() const
{
    return leaf_size_;
}
