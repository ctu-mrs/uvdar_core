#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "uvdar_core/app/bearing_node.hpp"

/** @brief ROS 2 entry point for the calibrated bearing endpoint. */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<uvdar_core::app::BearingNode>());
    rclcpp::shutdown();
    return 0;
}
