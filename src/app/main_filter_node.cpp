#include "uvdar_core/app/filter_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<uvdar_core::app::FilterNode>());
    rclcpp::shutdown();
    return 0;
}
