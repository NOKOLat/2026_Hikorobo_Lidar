#include "lidar_preprocess/sor_filter.hpp"
#include <pcl/filters/statistical_outlier_removal.h>

SORFilter::SORFilter(int mean_k, float std_dev_mul)
    : mean_k_(mean_k), std_dev_mul_(std_dev_mul)
{
}

void SORFilter::filter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter;
    sor_filter.setInputCloud(input);
    sor_filter.setMeanK(mean_k_);
    sor_filter.setStddevMulThresh(std_dev_mul_);
    sor_filter.filter(*output);
}

void SORFilter::setParameters(int mean_k, float std_dev_mul)
{
    mean_k_ = mean_k;
    std_dev_mul_ = std_dev_mul;
}

int SORFilter::getMeanK() const
{
    return mean_k_;
}

float SORFilter::getStdDevMul() const
{
    return std_dev_mul_;
}
