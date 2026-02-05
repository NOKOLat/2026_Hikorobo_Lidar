#ifndef ROI_FILTER_HPP_
#define ROI_FILTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @class ROIFilter
 * @brief Region of Interest (ROI) filter for point clouds
 *
 * This filter removes points outside a specified 3D region.
 * Only points within the min/max bounds of x, y, z are kept.
 */
class ROIFilter
{
public:
    /**
     * @brief Constructor
     * @param min_x Minimum x coordinate
     * @param max_x Maximum x coordinate
     * @param min_y Minimum y coordinate
     * @param max_y Maximum y coordinate
     * @param min_z Minimum z coordinate
     * @param max_z Maximum z coordinate
     */
    ROIFilter(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);

    /**
     * @brief Default constructor with default bounds
     */
    ROIFilter();

    /**
     * @brief Apply ROI filter to point cloud
     * @param input Input point cloud
     * @param output Filtered output point cloud
     */
    void filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &output);

    /**
     * @brief Set ROI bounds
     * @param min_x Minimum x coordinate
     * @param max_x Maximum x coordinate
     * @param min_y Minimum y coordinate
     * @param max_y Maximum y coordinate
     * @param min_z Minimum z coordinate
     * @param max_z Maximum z coordinate
     */
    void setBounds(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);

private:
    float min_x_, max_x_;
    float min_y_, max_y_;
    float min_z_, max_z_;
};

#endif // ROI_FILTER_HPP_
