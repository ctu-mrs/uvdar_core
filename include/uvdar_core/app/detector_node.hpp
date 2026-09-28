#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "uvdar_core/app/package_config.hpp"
#include "uvdar_core/helpers/thread_pool.hpp"
#include "uvdar_core/app/visualization.hpp"
#include "uvdar_core/detection/fimd/cpu_detector.hpp"
#include "uvdar_core/detection/fimd/gpu_detector.hpp"
#include "uvdar_core/detection/i_detector.hpp"
#include "uvdar_core/msg/image_points_with_covariances_stamped.hpp"

namespace uvdar_core::app {

/**
 * @brief ROS2 node that runs configured FIMD detector pipelines.
 */
class DetectorNode : public rclcpp::Node {
public:
    /**
     * @brief Construct detector node.
     * @param options Node options for ROS2 initialization.
     */
    explicit DetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    /**
     * @brief Runtime state for one detector input stream.
     */
    struct InputPipeline {
        DetectorInputConfig config;
        std::vector<cv::Mat> masks;
        std::unique_ptr<uvdar_core::detection::IDetector> detector;
        bool detector_initialized = false;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
        rclcpp::Publisher<uvdar_core::msg::ImagePointsWithCovariancesStamped>::SharedPtr candidate_publisher;
        rclcpp::Publisher<uvdar_core::msg::ImagePointsWithCovariancesStamped>::SharedPtr sun_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_publisher;
        std::unique_ptr<uvdar_core::app::visualization::VisualizationWorker> visualization_worker;
        std::mutex mutex;
        std::mutex scheduling_mutex;
        sensor_msgs::msg::Image::ConstSharedPtr pending_image;
        bool worker_active = false;
        std::uint64_t dropped_pending_frames = 0;
    };

    /**
     * @brief Load node parameters and package config.
     */
    void loadConfig();
    /**
     * @brief Create subscriptions and publishers for all enabled detector streams.
     */
    void createInterfaces();
    /**
     * @brief Load optional mask image for one input stream.
     */
    std::vector<cv::Mat> loadMasks(const DetectorInputConfig& input_config) const;
    /**
     * @brief ROS image callback.
     */
    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, std::size_t image_index);
    /**
     * @brief Process the newest pending image until the bounded slot is empty.
     */
    void processLatestImages(std::size_t image_index);
    /**
     * @brief Run detector on one image and publish all outputs.
     */
    void processImage(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, std::size_t image_index);
    /**
     * @brief Publish candidate and sun points with intensity metadata.
     */
    void publishPoints(
        InputPipeline& pipeline,
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const uvdar_core::detection::DetectorOutput& output,
        const cv::Mat& image);
    /**
     * @brief Publish marker visualization image when configured.
     */
    void publishVisualization(
        InputPipeline& pipeline,
        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
        const cv::Mat& image,
        const uvdar_core::detection::DetectorOutput& output);

    PackageConfig config_;
    std::vector<std::unique_ptr<InputPipeline>> pipelines_;
    std::unique_ptr<uvdar_core::helpers::ThreadPool> thread_pool_;
    rclcpp::Time startup_time_;
    std::string config_path_;
};

} // namespace uvdar_core::app
