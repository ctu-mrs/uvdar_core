#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include "uvdar_core/calibration/calibration_visualization.hpp"
#include "uvdar_core/calibration/calibrator.hpp"
#include "uvdar_core/calibration/pattern_detector.hpp"

namespace uvdar_core::app {

/** @brief Autonomous image-to-YAML camera calibration workflow. */
class CalibratorNode : public rclcpp::Node {
public:
    explicit CalibratorNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~CalibratorNode() override;

private:
    enum class Stage {
        Collecting,
        Initializing,
        Optimizing,
        Refining,
        Saving,
        Complete,
        Failed,
    };

    void loadParameters();
    void createInterfaces();
    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& message);
    void startCalibration();
    void runCalibration(
        std::vector<uvdar_core::calibration::CalibrationObservation> observations,
        cv::Size image_size);
    void updateProgress(
        const uvdar_core::calibration::CalibrationProgress& progress);
    void publishVisualization();
    void finishIfReady();
    bool sufficientlyDiverse(const Eigen::Vector4d& descriptor) const;
    Eigen::Vector4d patternDescriptor(
        const std::vector<cv::Point2f>& points,
        const cv::Size& image_size) const;
    static std::string stageName(Stage stage);

    std::string image_topic_;
    std::string visualization_topic_;
    std::string status_topic_;
    std::string output_file_;
    std::string model_name_;
    std::string pattern_name_;
    int required_frames_ = 20;
    double minimum_frame_interval_sec_ = 0.35;
    double minimum_frame_diversity_ = 0.07;
    double visualization_fps_ = 5.0;
    double completion_display_sec_ = 2.0;
    bool terminate_on_failure_ = true;

    uvdar_core::calibration::PatternDetectorOptions detector_options_;
    uvdar_core::calibration::CalibratorOptions calibrator_options_;
    std::unique_ptr<uvdar_core::calibration::CalibrationPatternDetector>
        pattern_detector_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr visualization_timer_;
    rclcpp::TimerBase::SharedPtr finish_timer_;

    mutable std::mutex mutex_;
    Stage stage_ = Stage::Collecting;
    std::string detail_ = "Waiting for calibration pattern";
    cv::Mat latest_image_;
    std_msgs::msg::Header latest_header_;
    cv::Size image_size_;
    uvdar_core::calibration::PatternDetection latest_detection_;
    std::vector<uvdar_core::calibration::CalibrationObservation> observations_;
    std::vector<Eigen::Vector4d> descriptors_;
    std::vector<cv::Point2f> accepted_centers_normalized_;
    uvdar_core::calibration::CalibrationProgress progress_;
    std::vector<double> cost_history_;
    std::optional<uvdar_core::calibration::CalibrationResult> result_;
    std::chrono::steady_clock::time_point last_accepted_time_ {};
    std::optional<std::chrono::steady_clock::time_point> finished_time_;
    bool worker_started_ = false;
    std::thread worker_;
};

} // namespace uvdar_core::app
