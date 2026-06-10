#include <rclcpp/rclcpp.hpp>

#include "uvdar_core/app/detector_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<uvdar_core::app::DetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}