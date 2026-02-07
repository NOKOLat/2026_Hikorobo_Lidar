#include "lidar_clustering/clustering_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    // ROS 2の初期化
    rclcpp::init(argc, argv);

    // クラスタリングノードを作成
    auto node = std::make_shared<ClusteringNode>();

    // ノードを実行
    rclcpp::spin(node);

    // シャットダウン
    rclcpp::shutdown();
    return 0;
}
