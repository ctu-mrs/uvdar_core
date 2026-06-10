#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "uvdar_core/app/package_config.hpp"
#include "uvdar_core/msg/image_points_with_float_stamped.hpp"
#include "uvdar_core/msg/tracker_output.hpp"
#include "uvdar_core/tracking/ami/blink_processor.h"
#include "uvdar_core/utils/thread_pool.hpp"

namespace uvdar_core::app {

/**
 * @brief ROS2 node that tracks blinking markers using AMI signatures.
 */
class TrackerNode : public rclcpp::Node {
public:
    /**
     * @brief ROS2 tracker node constructor.
     */
    explicit TrackerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    /**
     * @brief Internal tracker input pipeline description.
     */
    struct InputPipeline {
        TrackerInputConfig config;
        std::unique_ptr<uvdar_core::tracking::ami::BlinkProcessor> blink_processor;
        bool tracker_initialized = false;
        cv::Mat latest_image;
        bool image_received = false;
        uvdar_core::msg::TrackerOutput latest_output;
        rclcpp::Subscription<uvdar_core::msg::ImagePointsWithFloatStamped>::SharedPtr input_subscription;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription;
        rclcpp::Publisher<uvdar_core::msg::TrackerOutput>::SharedPtr output_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_publisher;
        mutable std::mutex mutex;
    };

    /**
     * @brief Load configuration from YAML using configured path.
     */
    void loadConfig();
    /**
     * @brief Create publishers/subscribers for all enabled tracking inputs.
     */
    void createInterfaces();
    /**
     * @brief Image-points callback from detector output.
     */
    void onImagePoints(const uvdar_core::msg::ImagePointsWithFloatStamped::ConstSharedPtr& msg, std::size_t image_index);
    /**
     * @brief Input image callback for optional visualization.
     */
    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, std::size_t image_index);
    /**
     * @brief Run tracker pipeline on one frame.
     */
    void processImagePoints(const uvdar_core::msg::ImagePointsWithFloatStamped::ConstSharedPtr& image_msg, std::size_t image_index);
    /**
     * @brief Publish outputs and invoke visualization if configured.
     */
    void publishOutput(InputPipeline& pipeline, const uvdar_core::msg::TrackerOutput& output);
    /**
     * @brief Build and publish tracker visualization frame.
     */
    void publishVisualization(InputPipeline& pipeline, const uvdar_core::msg::TrackerOutput& output);
    /**
     * @brief Deterministic color mapping for blinker IDs.
     */
    static cv::Scalar idColor(int id);
    /**
     * @brief Convert builtin ROS time to floating-point seconds.
     */
    static double toSeconds(const builtin_interfaces::msg::Time& stamp);

    PackageConfig config_;
    std::vector<std::unique_ptr<InputPipeline>> pipelines_;
    std::unique_ptr<uvdar_core::utils::ThreadPool> thread_pool_;
    rclcpp::Time startup_time_;
    std::string config_path_;
};

} // namespace uvdar_core::app
