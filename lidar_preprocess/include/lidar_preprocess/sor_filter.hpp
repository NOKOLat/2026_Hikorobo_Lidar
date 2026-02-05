#ifndef SOR_FILTER_HPP_
#define SOR_FILTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @class SORFilter
 * @brief Statistical Outlier Removal (SOR) filter
 *
 * This filter removes outliers by analyzing point cloud statistics.
 * Points with unusual distances to their neighbors are considered outliers
 * and removed.
 */
class SORFilter
{
public:
    /**
     * @brief Constructor
     * @param mean_k Number of nearest neighbors to consider
     * @param std_dev_mul Standard deviation multiplier for threshold
     */
    SORFilter(int mean_k = 50, float std_dev_mul = 1.0f);

    /**
     * @brief Apply Statistical Outlier Removal filter
     * @param input Input point cloud
     * @param output Filtered output point cloud
     */
    void filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &output);

    /**
     * @brief Set filter parameters
     * @param mean_k Number of nearest neighbors to consider
     * @param std_dev_mul Standard deviation multiplier for threshold
     */
    void setParameters(int mean_k, float std_dev_mul);

    /**
     * @brief Get mean_k parameter
     * @return Number of nearest neighbors
     */
    int getMeanK() const;

    /**
     * @brief Get standard deviation multiplier
     * @return Standard deviation multiplier
     */
    float getStdDevMul() const;

private:
    int mean_k_;
    float std_dev_mul_;
};

#endif // SOR_FILTER_HPP_
