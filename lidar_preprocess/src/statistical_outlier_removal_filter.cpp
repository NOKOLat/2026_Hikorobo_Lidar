#include "lidar_preprocess/statistical_outlier_removal_filter.hpp"
#include <pcl/filters/statistical_outlier_removal.h>

namespace lidar_preprocess
{

// Statistical Outlier Removal filter application function
bool StatisticalOutlierRemovalFilter::apply(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
    int mean_k,
    double std_dev_mul_thresh)
{
    // Input validation
    if (!input_cloud || input_cloud->empty())
    {
        return false;
    }

    try
    {
        // Configure Statistical Outlier Removal filter
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(input_cloud);
        sor.setMeanK(mean_k);
        sor.setStddevMulThresh(std_dev_mul_thresh);

        // Apply filter
        sor.filter(*output_cloud);

        return !output_cloud->empty();
    }
    catch (const std::exception& e)
    {
        return false;
    }
}

} // namespace lidar_preprocess
