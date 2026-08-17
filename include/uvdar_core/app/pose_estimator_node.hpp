#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "uvdar_core/msg/pose_with_covariance_array_stamped.hpp"
#include "uvdar_core/msg/tracker_output.hpp"
#include "uvdar_core/pose_estimation/i_pose_estimator.hpp"
#include "uvdar_core/pose_estimation/types.hpp"

namespace uvdar_core::app {

class PoseEstimatorNode final : public rclcpp::Node {
public:
    explicit PoseEstimatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    struct InputConfig {
        std::string name;
        std::string input_topic;
        std::string camera_frame;
        std::string calib_file;
    };

    void loadConfiguration(const std::string& config_path);
    void onTrackerOutput(const uvdar_core::msg::TrackerOutput::ConstSharedPtr& msg, std::size_t camera_index);
    void onScatterTimer();
    void publishMeasurements(
        const uvdar_core::pose_estimation::TimedPoseMeasurements& measurements,
        const rclcpp::Publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>::SharedPtr& publisher);

    std::vector<InputConfig> inputs_;
    std::vector<rclcpp::Subscription<uvdar_core::msg::TrackerOutput>::SharedPtr> subscriptions_;
    rclcpp::Publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>::SharedPtr measured_publisher_;
    rclcpp::Publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>::SharedPtr hypotheses_publisher_;
    rclcpp::Publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>::SharedPtr tentative_hypotheses_publisher_;
    rclcpp::TimerBase::SharedPtr scatter_timer_;

    std::unique_ptr<uvdar_core::pose_estimation::IPoseEstimator> pose_estimator_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string output_frame_;
    double latest_primary_input_stamp_ = 0.0;
    bool publish_constituents_ = false;

    long int frame_count_ = 0;
    long int usable_point_count_ = 0;
     
    long int publish_count_ = 0;
    long int populated_publish_count_ = 0;
};

} // namespace uvdar_core::app
