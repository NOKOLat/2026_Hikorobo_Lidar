#include "lidar_preprocess/roi_filter.hpp"

ROIFilter::ROIFilter(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z)
    : min_x_(min_x), max_x_(max_x),
      min_y_(min_y), max_y_(max_y),
      min_z_(min_z), max_z_(max_z)
{
}

ROIFilter::ROIFilter()
    : min_x_(-5.0f), max_x_(5.0f),
      min_y_(-5.0f), max_y_(5.0f),
      min_z_(0.0f), max_z_(2.0f)
{
}

void ROIFilter::filter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
{
    output->clear();

    for (const auto &point : input->points)
    {
        if (point.x >= min_x_ && point.x <= max_x_ &&
            point.y >= min_y_ && point.y <= max_y_ &&
            point.z >= min_z_ && point.z <= max_z_)
        {
            output->push_back(point);
        }
    }

    output->width = output->size();
    output->height = 1;
    output->is_dense = true;
    output->header = input->header;
}

void ROIFilter::setBounds(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z)
{
    min_x_ = min_x;
    max_x_ = max_x;
    min_y_ = min_y;
    max_y_ = max_y;
    min_z_ = min_z;
    max_z_ = max_z;
}
