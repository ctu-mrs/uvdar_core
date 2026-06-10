#include <rclcpp/rclcpp.hpp>

#include "uvdar_core/app/tracker_node.hpp"

/**
 * @brief ROS2 entry point for tracker node.
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<uvdar_core::app::TrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
