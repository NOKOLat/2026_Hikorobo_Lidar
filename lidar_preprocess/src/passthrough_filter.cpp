#include "lidar_preprocess/passthrough_filter.hpp"
#include <pcl/filters/passthrough.h>

namespace lidar_preprocess
{

// PassThrough filter application function
bool PassThroughFilter::apply(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
    const std::string& field_name,
    double min_limit,
    double max_limit)
{
    // Input validation
    if (!input_cloud || input_cloud->empty())
    {
        return false;
    }

    try
    {
        // Configure PassThrough filter
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(input_cloud);
        pass.setFilterFieldName(field_name);
        pass.setFilterLimits(min_limit, max_limit);

        // Apply filter
        pass.filter(*output_cloud);

        return !output_cloud->empty();
    }
    catch (const std::exception& e)
    {
        return false;
    }
}

} // namespace lidar_preprocess
