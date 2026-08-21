#include "uvdar_core/app/detector_node.hpp"

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "uvdar_core/app/visualization.hpp"

namespace uvdar_core::app {

DetectorNode::DetectorNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("detector", options)
{
    loadConfig();
    startup_time_ = now();
    thread_pool_  = std::make_unique<uvdar_core::helpers::ThreadPool>(config_.detector.thread_pool_size);
    createInterfaces();

    RCLCPP_INFO(get_logger(), "UVDAR detector node initialized.");
}

/**
 * @brief Load YAML config and setup runtime path.
 */
void DetectorNode::loadConfig()
{
    config_path_ = declare_parameter<std::string>("config_path", std::string { });
    config_      = loadPackageConfig(config_path_);
}

/**
 * @brief Parse optional mask from filesystem.
 */
std::vector<cv::Mat> DetectorNode::loadMasks(const DetectorInputConfig& input_config) const
{
    std::vector<cv::Mat> masks;
    if (input_config.mask_file.empty()) {
        return masks;
    }

    if (!std::filesystem::exists(input_config.mask_file)) {
        throw std::runtime_error("Mask '" + input_config.mask_file + "' does not exist.");
    }

    cv::Mat mask = cv::imread(input_config.mask_file, cv::IMREAD_GRAYSCALE);
    if (mask.empty()) {
        throw std::runtime_error("Mask '" + input_config.mask_file + "' could not be loaded.");
    }

    masks.push_back(mask);
    return masks;
}

/**
 * @brief Build all ROS interfaces for configured detector inputs.
 */
void DetectorNode::createInterfaces()
{
    pipelines_.clear();

    for (const DetectorInputConfig& input_config : config_.detector.inputs) {
        auto pipeline    = std::make_unique<InputPipeline>();
        pipeline->config = input_config;
        pipeline->masks  = loadMasks(input_config);
        if (input_config.backend == DetectorBackend::Cpu) {
            pipeline->detector = std::make_unique<uvdar_core::detection::fimd::CpuDetector>(uvdar_core::detection::fimd::CpuDetectorConfig {
                config_.detector.debug,
                input_config.detect_sun_points,
                input_config.threshold,
                input_config.threshold_diff,
                input_config.threshold_sun,
                input_config.min_sun_marker_distance,
                input_config.max_markers_count,
                input_config.max_sun_points_count,
                input_config.radii,
                pipeline->masks,
            });
        } else {
            pipeline->detector = std::make_unique<uvdar_core::detection::fimd::GpuDetector>(uvdar_core::detection::fimd::GpuDetectorConfig {
                config_.detector.debug,
                input_config.detect_sun_points,
                input_config.threshold,
                input_config.threshold_diff,
                input_config.threshold_sun,
                input_config.min_sun_marker_distance,
                input_config.max_markers_count,
                input_config.max_sun_points_count,
                input_config.radii,
                pipeline->masks,
            });
        }
        pipeline->candidate_publisher = create_publisher<uvdar_core::msg::ImagePointsWithCovariancesStamped>(
            input_config.output_topic,
            config_.detector.queue_depth);

        if (input_config.publish_sun_points) {
            pipeline->sun_publisher = create_publisher<uvdar_core::msg::ImagePointsWithCovariancesStamped>(
                input_config.sun_output_topic,
                config_.detector.queue_depth);
        }

        if (input_config.publish_visualization) {
            pipeline->visualization_publisher = create_publisher<sensor_msgs::msg::Image>(
                input_config.visualization_topic,
                config_.detector.queue_depth);
        }
        if (input_config.publish_visualization || config_.detector.gui) {
            pipeline->visualization_worker = std::make_unique<uvdar_core::app::visualization::VisualizationWorker>();
        }

        pipelines_.push_back(std::move(pipeline));
        const std::size_t index         = pipelines_.size() - 1;
        auto qos                        = rclcpp::QoS(rclcpp::KeepLast(config_.detector.queue_depth)).best_effort();
        pipelines_[index]->subscription = create_subscription<sensor_msgs::msg::Image>(
            pipelines_[index]->config.input_topic,
            qos,
            [this, index](const sensor_msgs::msg::Image::ConstSharedPtr image_msg) {
                onImage(image_msg, index);
            });
    }

    if (pipelines_.empty()) {
        throw std::runtime_error("No detector input pipelines configured.");
    }
}

/**
 * @brief Image callback that enforces startup delay and schedules processing.
 */
void DetectorNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, std::size_t image_index)
{
    if ((now() - startup_time_).seconds() < config_.detector.initial_delay_sec) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Ignoring messages during initial detector delay.");
        return;
    }

    thread_pool_->enqueue([this, image_msg, image_index]() {
        processImage(image_msg, image_index);
    });
}

/**
 * @brief Run detector for one image and publish outputs/visualization.
 */
void DetectorNode::processImage(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, std::size_t image_index)
{
    if (image_index >= pipelines_.size()) {
        return;
    }

    auto& pipeline = pipelines_[image_index];
    auto cv_image  = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);

    uvdar_core::detection::DetectorOutput output;
    {
        std::scoped_lock lock(pipeline->mutex);
        if (!pipeline->detector_initialized) {
            pipeline->detector_initialized = pipeline->detector->initDelayed(cv_image->image);
            if (!pipeline->detector_initialized) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Detector initialization is still pending.");
                return;
            }
        }

        const int mask_id = pipeline->masks.empty() ? -1 : 0;
        if (!pipeline->detector->processImage(cv_image->image, output, mask_id)) {
            RCLCPP_WARN(get_logger(), "FIMD detection failed for input '%s'.", pipeline->config.name.c_str());
            return;
        }

        pipeline->latest_image  = cv_image->image.clone();
        pipeline->latest_output = output;
    }

    if (output.detected_points.size() > config_.detector.max_points_per_image) {
        RCLCPP_WARN(get_logger(), "Received %zu detected points, dropping a noisy image.", output.detected_points.size());
        return;
    }

    publishPoints(*pipeline, image_msg, output, cv_image->image);
    publishVisualization(*pipeline, image_msg, cv_image->image, output);
}

/**
 * @brief Publish detected markers and optional sun points as custom message.
 */
void DetectorNode::publishPoints(
    InputPipeline& pipeline,
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
    const uvdar_core::detection::DetectorOutput& output,
    const cv::Mat& image)
{
    auto fill_message = [&](const std::vector<uvdar_core::detection::DetectorPoint>& points) {
        uvdar_core::msg::ImagePointsWithCovariancesStamped msg;
        msg.stamp        = image_msg->header.stamp;
        msg.image_height = static_cast<std::uint32_t>(image.rows);
        msg.image_width  = static_cast<std::uint32_t>(image.cols);
        msg.points.reserve(points.size());
        for (const auto& point : points) {
            uvdar_core::msg::Point2DWithCovariance msg_point;
            msg_point.x             = static_cast<double>(point.point.x);
            msg_point.y             = static_cast<double>(point.point.y);
            msg_point.covariance_00 = static_cast<double>(point.covariance_00);
            msg_point.covariance_01 = static_cast<double>(point.covariance_01);
            msg_point.covariance_10 = static_cast<double>(point.covariance_10);
            msg_point.covariance_11 = static_cast<double>(point.covariance_11);
            msg.points.push_back(msg_point);
        }
        return msg;
    };

    pipeline.candidate_publisher->publish(fill_message(output.detected_points));
    if (pipeline.config.publish_sun_points && pipeline.sun_publisher) {
        pipeline.sun_publisher->publish(fill_message(output.sun_points));
    }
}

/**
 * @brief Render and publish visualization when enabled.
 */
void DetectorNode::publishVisualization(
    InputPipeline& pipeline,
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
    const cv::Mat& image,
    const uvdar_core::detection::DetectorOutput& output)
{
    if (!pipeline.config.publish_visualization && !config_.detector.gui) {
        return;
    }

    uvdar_core::app::visualization::DetectionOverlay overlay;
    overlay.detected_points.reserve(output.detected_points.size());
    for (const auto& point : output.detected_points) {
        overlay.detected_points.emplace_back(point.point.x, point.point.y);
    }
    overlay.sun_points.reserve(output.sun_points.size());
    for (const auto& point : output.sun_points) {
        overlay.sun_points.emplace_back(point.point.x, point.point.y);
    }
    if (!pipeline.visualization_worker) {
        return;
    }
    const auto publisher = pipeline.visualization_publisher;
    const auto header = image_msg->header;
    const cv::Mat source = image.clone();
    const bool show_gui = config_.detector.gui;
    const std::string window_name = "uvdar_detection_" + pipeline.config.name;
    const auto logger = get_logger();
    pipeline.visualization_worker->submit([publisher, header, source, overlay = std::move(overlay), show_gui, window_name, logger] {
        try {
            const cv::Mat visualization = uvdar_core::app::visualization::renderDetectionOverlay(source, overlay);
            if (publisher && !visualization.empty()) {
                publisher->publish(*cv_bridge::CvImage(header, "bgr8", visualization).toImageMsg());
            }
            if (show_gui) {
                uvdar_core::app::visualization::showFrame(window_name, visualization);
            }
        } catch (const std::exception& ex) {
            RCLCPP_WARN(logger, "Detector visualization failed: %s", ex.what());
        }
    });
}

} // namespace uvdar_core::app
