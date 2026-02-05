#ifndef LIDAR_PREPROCESS__VOXEL_GRID_FILTER_HPP_
#define LIDAR_PREPROCESS__VOXEL_GRID_FILTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_preprocess
{

    /**
     * @brief Voxel Grid Filter utility class
     *
     * This class provides static methods for applying VoxelGrid downsampling
     * to point clouds.
     */
    class VoxelGridFilter
    {
    public:
        /**
         * @brief Apply VoxelGrid filter to point cloud
         *
         * @param input_cloud Input point cloud
         * @param output_cloud Output downsampled point cloud
         * @param leaf_size Size of voxel leaf (in meters)
         * @return true if filtering succeeded, false otherwise
         */
        static bool apply(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
            pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
            double leaf_size);
    };

} // namespace lidar_preprocess

#endif // LIDAR_PREPROCESS__VOXEL_GRID_FILTER_HPP_
