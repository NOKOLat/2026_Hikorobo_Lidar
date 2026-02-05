#include "lidar_get_points/livox_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LivoxNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
