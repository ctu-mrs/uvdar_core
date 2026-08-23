#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "uvdar_core/app/calibrator_node.hpp"

/** @brief ROS 2 entry point for the autonomous camera calibrator. */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<uvdar_core::app::CalibratorNode>();
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 3U);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
