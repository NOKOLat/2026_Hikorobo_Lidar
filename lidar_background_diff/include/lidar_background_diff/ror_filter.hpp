#ifndef ROR_FILTER_HPP_
#define ROR_FILTER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @class RORFilter
 * @brief Range-based Outlier Removal (ROR) filter
 *
 * 原点からの距離（Range）に基づいて外れ値を除去するフィルタ。
 * 指定された距離範囲外の点を外れ値として除去する。
 */
class RORFilter
{
public:
    /**
     * @brief コンストラクタ
     * @param min_range 最小距離（メートル単位、デフォルト: 0.5m）
     * @param max_range 最大距離（メートル単位、デフォルト: 10.0m）
     */
    RORFilter(float min_range = 0.5f, float max_range = 10.0f);

    /**
     * @brief Range-based Outlier Removal フィルタを適用
     * @param input 入力点群
     * @param output フィルタ済み出力点群
     */
    void filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &output);

    /**
     * @brief フィルタパラメータを設定
     * @param min_range 最小距離（メートル単位）
     * @param max_range 最大距離（メートル単位）
     */
    void setParameters(float min_range, float max_range);

    /**
     * @brief 最小距離パラメータを取得
     * @return 最小距離
     */
    float getMinRange() const;

    /**
     * @brief 最大距離パラメータを取得
     * @return 最大距離
     */
    float getMaxRange() const;

private:
    float min_range_; // 最小距離（メートル単位）
    float max_range_; // 最大距離（メートル単位）
};

#endif // ROR_FILTER_HPP_
