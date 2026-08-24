#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "uvdar_core/calibration/i_lens_model.hpp"
#include "uvdar_core/msg/bearing_observation_array_stamped.hpp"
#include "uvdar_core/msg/tracker_output.hpp"

namespace uvdar_core::app {

/**
 * @brief Convert identified image tracks into calibrated camera-frame bearings.
 */
class BearingNode final : public rclcpp::Node {
public:
    explicit BearingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~BearingNode() override = default;

private:
    struct InputPipeline {
        std::string name;
        std::string input_topic;
        std::string output_topic;
        std::string camera_frame;
        calibration::LensModelPtr lens;
        rclcpp::Subscription<uvdar_core::msg::TrackerOutput>::SharedPtr subscription;
        rclcpp::Publisher<uvdar_core::msg::BearingObservationArrayStamped>::SharedPtr publisher;
    };

    void loadConfiguration(const std::string& config_path);
    void createInterfaces();
    void onTrackerOutput(const uvdar_core::msg::TrackerOutput::ConstSharedPtr& msg, std::size_t input_index);

    std::vector<InputPipeline> inputs_;
    std::size_t queue_depth_ = 10U;
    bool publish_predictions_ = true;
    bool publish_unidentified_ = true;
    double covariance_floor_px2_ = 1.0e-6;
    double fallback_pixel_variance_px2_ = 1.0;
};

} // namespace uvdar_core::app
