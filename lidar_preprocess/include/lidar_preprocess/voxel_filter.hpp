#ifndef VOXEL_FILTER_HPP_
#define VOXEL_FILTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @class VoxelFilter
 * @brief Voxel grid downsampling filter
 *
 * This filter reduces the point cloud density by creating a 3D voxel grid
 * and keeping only one point per voxel (the centroid).
 */
class VoxelFilter
{
public:
    /**
     * @brief Constructor
     * @param leaf_size Size of each voxel (in meters)
     */
    explicit VoxelFilter(float leaf_size = 0.01f);

    /**
     * @brief Apply voxel grid filter to point cloud
     * @param input Input point cloud
     * @param output Filtered output point cloud
     */
    void filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &output);

    /**
     * @brief Set voxel leaf size
     * @param leaf_size Size of each voxel (in meters)
     */
    void setLeafSize(float leaf_size);

    /**
     * @brief Get current voxel leaf size
     * @return Leaf size in meters
     */
    float getLeafSize() const;

private:
    float leaf_size_;
};

#endif // VOXEL_FILTER_HPP_
