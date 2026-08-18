#include <rclcpp/rclcpp.hpp>

#include "uvdar_core/app/led_manager_node.hpp"

/**
 * @brief ROS2 entry point for the LED manager node.
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<uvdar_core::app::LedManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
