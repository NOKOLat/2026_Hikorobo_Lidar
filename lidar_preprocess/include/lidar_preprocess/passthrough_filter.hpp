#ifndef LIDAR_PREPROCESS__PASSTHROUGH_FILTER_HPP_
#define LIDAR_PREPROCESS__PASSTHROUGH_FILTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace lidar_preprocess
{

    /**
     * @brief PassThrough Filter utility class
     *
     * This class provides static methods for applying PassThrough filtering
     * to point clouds based on field values.
     */
    class PassThroughFilter
    {
    public:
        /**
         * @brief Apply PassThrough filter to point cloud
         *
         * @param input_cloud Input point cloud
         * @param output_cloud Output filtered point cloud
         * @param field_name Field name to filter (e.g., "x", "y", "z")
         * @param min_limit Minimum value for the field
         * @param max_limit Maximum value for the field
         * @return true if filtering succeeded, false otherwise
         */
        static bool apply(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
            pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
            const std::string &field_name,
            double min_limit,
            double max_limit);
    };

} // namespace lidar_preprocess

#endif // LIDAR_PREPROCESS__PASSTHROUGH_FILTER_HPP_
