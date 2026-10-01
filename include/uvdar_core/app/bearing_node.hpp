#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "uvdar_core/calibration/i_lens_model.hpp"
#include "uvdar_core/msg/bearing_observation_array_stamped.hpp"
#include "uvdar_core/msg/tracker_output.hpp"

namespace uvdar_core::app {

/**
 * @brief Convert image tracks into calibrated rays in one rigid-body frame.
 */
class BearingNode final : public rclcpp::Node {
public:
    explicit BearingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~BearingNode() override = default;

private:
    struct InputPipeline {
        std::string name;
        std::string input_topic;
        std::string camera_frame;
        calibration::LensModelPtr lens;
        rclcpp::Subscription<uvdar_core::msg::TrackerOutput>::SharedPtr subscription;
    };

    void loadConfiguration(const std::string& config_path);
    void createInterfaces();
    void onTrackerOutput(const uvdar_core::msg::TrackerOutput::ConstSharedPtr& msg, std::size_t input_index);

    std::vector<InputPipeline> inputs_;
    rclcpp::Publisher<uvdar_core::msg::BearingObservationArrayStamped>::SharedPtr publisher_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string output_frame_;
    std::string output_topic_;
    std::size_t queue_depth_ = 10U;
    bool publish_predictions_ = true;
    bool publish_unidentified_ = true;
    double covariance_floor_px2_ = 1.0e-6;
    double fallback_pixel_variance_px2_ = 1.0;
};

} // namespace uvdar_core::app
