#include "uvdar_core/app/pose_estimator_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<uvdar_core::app::PoseEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
