#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "uvdar_core/msg/pose_with_covariance_array_stamped.hpp"
#include "uvdar_core/pose_estimation/kf_pose.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

namespace uvdar_core::app {

class FilterNode final : public rclcpp::Node {
public:
    explicit FilterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void loadConfiguration(const std::string& config_path);
    void onMeasurement(const uvdar_core::msg::PoseWithCovarianceArrayStamped::ConstSharedPtr& msg);
    void onTimer();
    void publishStates(
        const std::vector<uvdar_core::pose_estimation::KfPoseState>& states,
        const rclcpp::Publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>::SharedPtr& publisher);
    void publishVisualization(
        const std::vector<uvdar_core::pose_estimation::KfPoseState>& validated_states,
        const std::vector<uvdar_core::pose_estimation::KfPoseState>& tentative_states);

    std::vector<rclcpp::Subscription<uvdar_core::msg::PoseWithCovarianceArrayStamped>::SharedPtr> subscriptions_;
    rclcpp::Publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>::SharedPtr filtered_publisher_;
    rclcpp::Publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>::SharedPtr tentative_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<uvdar_core::pose_estimation::KfPose> filter_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string output_frame_ = "local_origin";
    bool publish_visualization_ = false;
    double velocity_arrow_scale_sec_ = 1.0;
};

} // namespace uvdar_core::app
