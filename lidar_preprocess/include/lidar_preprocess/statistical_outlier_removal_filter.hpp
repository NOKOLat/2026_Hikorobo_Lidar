#ifndef LIDAR_PREPROCESS__STATISTICAL_OUTLIER_REMOVAL_FILTER_HPP_
#define LIDAR_PREPROCESS__STATISTICAL_OUTLIER_REMOVAL_FILTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_preprocess
{

/**
 * @brief Statistical Outlier Removal Filter utility class
 * 
 * This class provides static methods for removing outlier points
 * from point clouds using statistical analysis.
 */
class StatisticalOutlierRemovalFilter
{
public:
    /**
     * @brief Apply Statistical Outlier Removal filter to point cloud
     * 
     * @param input_cloud Input point cloud
     * @param output_cloud Output filtered point cloud
     * @param mean_k Number of nearest neighbors to analyze
     * @param std_dev_mul_thresh Standard deviation multiplier threshold
     * @return true if filtering succeeded, false otherwise
     */
    static bool apply(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
        int mean_k,
        double std_dev_mul_thresh);
};

} // namespace lidar_preprocess

#endif // LIDAR_PREPROCESS__STATISTICAL_OUTLIER_REMOVAL_FILTER_HPP_
