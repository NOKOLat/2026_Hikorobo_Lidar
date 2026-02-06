#include "lidar_background_diff/ror_filter.hpp"
#include <cmath>

RORFilter::RORFilter(float min_range, float max_range)
    : min_range_(min_range), max_range_(max_range)
{
}

void RORFilter::filter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &output)
{
    // 出力点群をクリア
    output->clear();
    output->points.reserve(input->points.size());

    // 各点について原点からの距離を計算し、範囲内の点のみを保持
    for (const auto &point : input->points)
    {
        // 原点からの距離を計算
        float range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

        // 距離が範囲内の場合、出力に追加
        if (range >= min_range_ && range <= max_range_)
        {
            output->points.push_back(point);
        }
    }

    // 点群のプロパティを設定
    output->width = output->points.size();
    output->height = 1;
    output->is_dense = true;
}

void RORFilter::setParameters(float min_range, float max_range)
{
    min_range_ = min_range;
    max_range_ = max_range;
}

float RORFilter::getMinRange() const
{
    return min_range_;
}

float RORFilter::getMaxRange() const
{
    return max_range_;
}
