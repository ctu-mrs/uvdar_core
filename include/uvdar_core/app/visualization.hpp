#pragma once

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace uvdar_core::app::visualization {

/**
 * @brief Coalescing background worker for non-critical visualization work.
 *
 * Producers may submit frames from real-time callbacks.  Only the latest
 * pending task is retained, so expensive drawing never builds an unbounded
 * backlog behind detection, tracking, or pose estimation.
 */
class VisualizationWorker {
public:
    using Task = std::function<void()>;

    explicit VisualizationWorker(std::chrono::milliseconds minimum_interval = std::chrono::milliseconds(33));
    ~VisualizationWorker();

    VisualizationWorker(const VisualizationWorker&) = delete;
    VisualizationWorker& operator=(const VisualizationWorker&) = delete;

    /** @brief Replace the pending task with the newest visualization request. */
    void submit(Task task);

    /** @brief Stop the worker and join its thread. Safe to call repeatedly. */
    void stop();

private:
    void run();

    std::chrono::milliseconds minimum_interval_;
    std::mutex mutex_;
    std::condition_variable cv_;
    Task pending_task_;
    bool running_ = true;
    std::thread thread_;
};

/** @brief Detector points rendered over an input image. */
struct DetectionOverlay {
    std::vector<cv::Point2d> detected_points;
    std::vector<cv::Point2d> sun_points;
};

/** @brief Render the common detector overlay without any ROS dependency. */
cv::Mat renderDetectionOverlay(const cv::Mat& image, const DetectionOverlay& overlay);

/** @brief One tracker output marker rendered over an input image. */
struct TrackingOverlayMarker {
    cv::Point2d position;
    int signal_id = -1;
    bool has_prediction = false;
    cv::Point2d predicted_position;
    cv::Point2d confidence_half_extent = cv::Point2d(-1.0, -1.0);
    bool virtual_point = false;
};

/** @brief Stable display color for a decoded tracking signal. */
cv::Scalar trackingSignalColor(int signal_id);

/** @brief Render the common tracker overlay without any ROS dependency. */
cv::Mat renderTrackingOverlay(
    const cv::Mat& image,
    const cv::Size& fallback_size,
    const std::vector<TrackingOverlayMarker>& markers);

/** @brief A world-frame target pose for the multi-view pose overview. */
struct PoseVisualizationPose {
    int id = -1;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d position_covariance = Eigen::Matrix3d::Identity();
    /** Small-angle covariance in radians squared, ordered around world X/Y/Z. */
    Eigen::Matrix3d orientation_covariance = Eigen::Matrix3d::Identity();
};

/**
 * @brief Stateful renderer for the pose-estimator overview.
 *
 * It keeps the global metres-per-pixel scale until all views have enough
 * unused room to shrink it.  This avoids jittering axes while retaining a
 * common physical scale for the XY, XZ, and YZ views.
 */
class PoseOverviewRenderer {
public:
    cv::Mat render(const std::vector<PoseVisualizationPose>& poses, cv::Size canvas_size = cv::Size(1600, 900));

private:
    double metres_per_pixel_ = 0.0;
};

/** @brief Display a visualization frame under a process-wide OpenCV GUI lock. */
void showFrame(const std::string& window_name, const cv::Mat& frame);

} // namespace uvdar_core::app::visualization
