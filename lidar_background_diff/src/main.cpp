#include "lidar_background_diff/background_diff_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    // ROS 2の初期化
    rclcpp::init(argc, argv);

    // 背景差分ノードを作成
    auto node = std::make_shared<BackgroundDiffNode>();

    // ノードを実行
    rclcpp::spin(node);

    // シャットダウン
    rclcpp::shutdown();
    return 0;
}
