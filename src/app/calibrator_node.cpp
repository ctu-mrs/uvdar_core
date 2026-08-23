#include "uvdar_core/app/calibrator_node.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace uvdar_core::app {

namespace calibration = uvdar_core::calibration;

CalibratorNode::CalibratorNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("calibrator", options)
{
    loadParameters();
    pattern_detector_ =
        std::make_unique<calibration::CalibrationPatternDetector>(
            detector_options_);
    createInterfaces();
    RCLCPP_INFO(
        get_logger(),
        "Calibrating '%s' from %s patterns on %s; output: %s",
        model_name_.c_str(),
        pattern_name_.c_str(),
        image_topic_.c_str(),
        output_file_.c_str());
}

CalibratorNode::~CalibratorNode()
{
    if (worker_.joinable()) {
        worker_.join();
    }
}

void CalibratorNode::loadParameters()
{
    image_topic_ = declare_parameter<std::string>(
        "image_topic", "/camera/image_raw");
    visualization_topic_ = declare_parameter<std::string>(
        "visualization_topic", "~/visualization");
    status_topic_ = declare_parameter<std::string>(
        "status_topic", "~/status");
    output_file_ = declare_parameter<std::string>(
        "output_calibration_file", "/tmp/uvdar_camera_calibration.yaml");
    model_name_ = declare_parameter<std::string>(
        "calibration_model", "ocamcalib");
    pattern_name_ = declare_parameter<std::string>(
        "pattern_type", "checkerboard");
    required_frames_ = declare_parameter<int>("required_pattern_frames", 20);
    minimum_frame_interval_sec_ = declare_parameter<double>(
        "minimum_frame_interval_sec", 0.35);
    minimum_frame_diversity_ = declare_parameter<double>(
        "minimum_frame_diversity", 0.07);
    visualization_fps_ = declare_parameter<double>("visualization_fps", 5.0);
    completion_display_sec_ = declare_parameter<double>(
        "completion_display_sec", 2.0);
    terminate_on_failure_ = declare_parameter<bool>(
        "terminate_on_failure", true);

    detector_options_.pattern =
        calibration::calibrationPatternFromString(pattern_name_);
    detector_options_.rows = declare_parameter<int>("pattern_rows", 6);
    detector_options_.columns = declare_parameter<int>("pattern_columns", 8);
    detector_options_.spacing = declare_parameter<double>(
        "pattern_spacing", 0.04);
    detector_options_.maximum_candidates = declare_parameter<int>(
        "maximum_detection_candidates", 200);
    detector_options_.fimd_threshold = declare_parameter<int>(
        "fimd_threshold", 120);
    detector_options_.fimd_threshold_diff = declare_parameter<int>(
        "fimd_threshold_diff", 60);
    detector_options_.fimd_max_markers = static_cast<unsigned>(std::max<int64_t>(
        1,
        declare_parameter<int64_t>("fimd_max_markers", 300)));
    const std::vector<int64_t> radii =
        declare_parameter<std::vector<int64_t>>(
            "fimd_radii", std::vector<int64_t> {3, 5});
    detector_options_.fimd_radii.clear();
    for (const int64_t radius : radii) {
        if (radius > 0) {
            detector_options_.fimd_radii.push_back(
                static_cast<unsigned>(radius));
        }
    }
    if (detector_options_.fimd_radii.empty()) {
        throw std::invalid_argument("fimd_radii must contain a positive radius.");
    }
    detector_options_.hull_maximum_concave_angle = declare_parameter<double>(
        "hull_maximum_concave_angle", M_PI / 4.0);
    detector_options_.hull_similar_angle = declare_parameter<double>(
        "hull_similar_angle", M_PI / 9.0);

    calibrator_options_.model =
        calibration::calibrationModelFromString(model_name_);
    calibrator_options_.minimum_views = declare_parameter<int>(
        "minimum_valid_views", std::min(10, required_frames_));
    calibrator_options_.max_iterations = declare_parameter<int>(
        "maximum_optimization_iterations", 100);
    calibrator_options_.outlier_refinement_iterations = declare_parameter<int>(
        "outlier_refinement_iterations", 50);
    calibrator_options_.initial_damping = declare_parameter<double>(
        "initial_lm_damping", 1.0e-4);
    calibrator_options_.huber_delta_px = declare_parameter<double>(
        "huber_delta_px", 3.0);
    calibrator_options_.view_outlier_factor = declare_parameter<double>(
        "view_outlier_factor", 2.5);
    calibrator_options_.maximum_rms_px = declare_parameter<double>(
        "maximum_final_rms_px", 3.0);
    calibrator_options_.ocam_inverse_polynomial_order = declare_parameter<int>(
        "ocam_inverse_polynomial_order", 9);
    calibrator_options_.ocam_direct_polynomial_order = declare_parameter<int>(
        "ocam_direct_polynomial_order", 4);
    calibrator_options_.step_tolerance = declare_parameter<double>(
        "optimization_step_tolerance", 1.0e-9);
    calibrator_options_.gradient_tolerance = declare_parameter<double>(
        "optimization_gradient_tolerance", 1.0e-8);
    calibrator_options_.relative_cost_tolerance = declare_parameter<double>(
        "optimization_relative_cost_tolerance", 1.0e-10);

    if (image_topic_.empty() || output_file_.empty()) {
        throw std::invalid_argument(
            "image_topic and output_calibration_file must not be empty.");
    }
    if (required_frames_ < 3
        || calibrator_options_.minimum_views < 3
        || calibrator_options_.minimum_views > required_frames_) {
        throw std::invalid_argument(
            "Require 3 <= minimum_valid_views <= required_pattern_frames.");
    }
    if (visualization_fps_ <= 0.0 || minimum_frame_interval_sec_ < 0.0
        || minimum_frame_diversity_ < 0.0) {
        throw std::invalid_argument(
            "Visualization FPS must be positive and collection limits non-negative.");
    }
}

void CalibratorNode::createInterfaces()
{
    visualization_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        visualization_topic_, rclcpp::QoS(1).reliable());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
        status_topic_, rclcpp::QoS(1).reliable().transient_local());
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&CalibratorNode::onImage, this, std::placeholders::_1));

    visualization_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / visualization_fps_),
        std::bind(&CalibratorNode::publishVisualization, this));
    finish_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&CalibratorNode::finishIfReady, this));
}

Eigen::Vector4d CalibratorNode::patternDescriptor(
    const std::vector<cv::Point2f>& points,
    const cv::Size& image_size) const
{
    cv::Point2f centroid(0.0F, 0.0F);
    float minimum_x = std::numeric_limits<float>::infinity();
    float minimum_y = std::numeric_limits<float>::infinity();
    float maximum_x = -std::numeric_limits<float>::infinity();
    float maximum_y = -std::numeric_limits<float>::infinity();
    for (const cv::Point2f& point : points) {
        centroid += point;
        minimum_x = std::min(minimum_x, point.x);
        minimum_y = std::min(minimum_y, point.y);
        maximum_x = std::max(maximum_x, point.x);
        maximum_y = std::max(maximum_y, point.y);
    }
    centroid *= 1.0F / static_cast<float>(std::max<std::size_t>(1U, points.size()));
    const double area_fraction = std::max(0.0,
        static_cast<double>((maximum_x - minimum_x) * (maximum_y - minimum_y))
            / static_cast<double>(image_size.area()));
    const cv::Point2f direction = points.size() >= 2U
        ? points.back() - points.front() : cv::Point2f(1.0F, 0.0F);
    return {
        centroid.x / image_size.width,
        centroid.y / image_size.height,
        std::sqrt(area_fraction),
        std::atan2(direction.y, direction.x) / M_PI,
    };
}

bool CalibratorNode::sufficientlyDiverse(
    const Eigen::Vector4d& descriptor) const
{
    if (descriptors_.empty()) {
        return true;
    }
    for (const Eigen::Vector4d& existing : descriptors_) {
        Eigen::Vector4d delta = descriptor - existing;
        delta(3) = std::remainder(delta(3), 2.0);
        delta(2) *= 0.7;
        delta(3) *= 0.25;
        if (delta.norm() < minimum_frame_diversity_) {
            return false;
        }
    }
    return true;
}

void CalibratorNode::onImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& message)
{
    cv_bridge::CvImagePtr converted;
    try {
        converted = cv_bridge::toCvCopy(
            message, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception& exception) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Could not convert calibration image: %s", exception.what());
        return;
    }

    {
        std::scoped_lock lock(mutex_);
        latest_image_ = converted->image.clone();
        latest_header_ = message->header;
        image_size_ = converted->image.size();
        if (stage_ != Stage::Collecting) {
            return;
        }
    }

    calibration::PatternDetection detection;
    try {
        detection = pattern_detector_->detect(converted->image);
    } catch (const cv::Exception& exception) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Pattern detector failed: %s", exception.what());
        return;
    }

    bool should_start = false;
    {
        std::scoped_lock lock(mutex_);
        if (stage_ != Stage::Collecting) {
            return;
        }
        latest_detection_ = detection;
        detail_ = detection.detail;
        if (!detection.success) {
            return;
        }

        const auto current_time = std::chrono::steady_clock::now();
        const bool interval_elapsed = last_accepted_time_.time_since_epoch().count() == 0
            || std::chrono::duration<double>(
                   current_time - last_accepted_time_).count()
                >= minimum_frame_interval_sec_;
        const Eigen::Vector4d descriptor = patternDescriptor(
            detection.image_points, converted->image.size());
        if (!interval_elapsed) {
            detail_ = "Pattern locked; hold for next sample";
            return;
        }
        if (!sufficientlyDiverse(descriptor)) {
            detail_ = "Pattern locked; move or tilt the target";
            return;
        }

        calibration::CalibrationObservation observation;
        observation.image_points = detection.image_points;
        observation.object_points = detection.object_points;
        observations_.push_back(std::move(observation));
        descriptors_.push_back(descriptor);
        accepted_centers_normalized_.emplace_back(
            static_cast<float>(descriptor.x()),
            static_cast<float>(descriptor.y()));
        last_accepted_time_ = current_time;
        detail_ = "Accepted diverse pattern view";
        RCLCPP_INFO(
            get_logger(), "Accepted calibration view %zu/%d",
            observations_.size(), required_frames_);
        should_start = observations_.size()
            >= static_cast<std::size_t>(required_frames_);
    }
    if (should_start) {
        startCalibration();
    }
}

void CalibratorNode::startCalibration()
{
    std::vector<calibration::CalibrationObservation> observations;
    cv::Size image_size;
    {
        std::scoped_lock lock(mutex_);
        if (worker_started_ || stage_ != Stage::Collecting) {
            return;
        }
        worker_started_ = true;
        stage_ = Stage::Initializing;
        detail_ = "Starting calibration worker";
        observations = observations_;
        image_size = image_size_;
    }
    worker_ = std::thread(
        &CalibratorNode::runCalibration,
        this,
        std::move(observations),
        image_size);
}

void CalibratorNode::runCalibration(
    std::vector<calibration::CalibrationObservation> observations,
    const cv::Size image_size)
{
    try {
        const calibration::CameraCalibrator calibrator(calibrator_options_);
        calibration::CalibrationResult result = calibrator.calibrate(
            observations,
            image_size,
            [this](const calibration::CalibrationProgress& progress) {
                updateProgress(progress);
            });
        if (!result.valid) {
            std::scoped_lock lock(mutex_);
            result_ = std::move(result);
            stage_ = Stage::Failed;
            detail_ = result_->message;
            finished_time_ = std::chrono::steady_clock::now();
            RCLCPP_ERROR(
                get_logger(), "Calibration failed validation: %s",
                detail_.c_str());
            return;
        }

        {
            std::scoped_lock lock(mutex_);
            stage_ = Stage::Saving;
            detail_ = "Writing calibration YAML atomically";
        }
        calibration::writeCalibrationYaml(result, output_file_);
        const double final_rms = result.rms_px;
        const double calibration_seconds = result.total_seconds;
        {
            std::scoped_lock lock(mutex_);
            result_ = std::move(result);
            stage_ = Stage::Complete;
            detail_ = "Calibration saved successfully";
            finished_time_ = std::chrono::steady_clock::now();
        }
        RCLCPP_INFO(
            get_logger(),
            "Calibration complete in %.3f s: RMS %.4f px, saved to %s",
            calibration_seconds,
            final_rms,
            output_file_.c_str());
    } catch (const std::exception& exception) {
        {
            std::scoped_lock lock(mutex_);
            stage_ = Stage::Failed;
            detail_ = exception.what();
            finished_time_ = std::chrono::steady_clock::now();
        }
        RCLCPP_ERROR(get_logger(), "Calibration failed: %s", exception.what());
    }
}

void CalibratorNode::updateProgress(
    const calibration::CalibrationProgress& progress)
{
    std::scoped_lock lock(mutex_);
    progress_ = progress;
    detail_ = progress.detail;
    switch (progress.stage) {
        case calibration::CalibrationEngineStage::Initializing:
            stage_ = Stage::Initializing;
            break;
        case calibration::CalibrationEngineStage::Optimizing:
            stage_ = Stage::Optimizing;
            break;
        case calibration::CalibrationEngineStage::RejectingOutliers:
        case calibration::CalibrationEngineStage::Refining:
            stage_ = Stage::Refining;
            break;
        case calibration::CalibrationEngineStage::Validating:
        case calibration::CalibrationEngineStage::Finished:
            stage_ = Stage::Saving;
            break;
    }
    if (std::isfinite(progress.cost) && progress.cost > 0.0) {
        cost_history_.push_back(progress.cost);
        if (cost_history_.size() > 300U) {
            cost_history_.erase(cost_history_.begin());
        }
    }
}

std::string CalibratorNode::stageName(const Stage stage)
{
    switch (stage) {
        case Stage::Collecting:
            return "COLLECTING";
        case Stage::Initializing:
            return "INITIALIZING";
        case Stage::Optimizing:
            return "OPTIMIZING";
        case Stage::Refining:
            return "REFINING";
        case Stage::Saving:
            return "VALIDATING / SAVING";
        case Stage::Complete:
            return "COMPLETE";
        case Stage::Failed:
            return "FAILED";
    }
    return "UNKNOWN";
}

void CalibratorNode::publishVisualization()
{
    calibration::CalibrationVisualizationState snapshot;
    cv::Mat image;
    std_msgs::msg::Header header;
    {
        std::scoped_lock lock(mutex_);
        image = latest_image_.clone();
        header = latest_header_;
        snapshot.stage = stageName(stage_);
        snapshot.detail = detail_;
        snapshot.model = model_name_;
        snapshot.pattern = pattern_name_;
        snapshot.output_path = output_file_;
        snapshot.accepted_frames = static_cast<int>(observations_.size());
        snapshot.required_frames = required_frames_;
        snapshot.detected_candidates = static_cast<int>(
            latest_detection_.candidates.size());
        snapshot.pattern_rows = detector_options_.rows;
        snapshot.pattern_columns = detector_options_.columns;
        snapshot.iteration = progress_.iteration;
        snapshot.maximum_iterations = progress_.maximum_iterations;
        snapshot.rms_px = progress_.rms_px;
        snapshot.damping = progress_.damping;
        snapshot.elapsed_seconds = progress_.elapsed_seconds;
        snapshot.pattern_found = latest_detection_.success;
        snapshot.pattern_column_major = detector_options_.pattern
            == calibration::CalibrationPatternType::LedGrid;
        snapshot.successful = stage_ == Stage::Complete;
        snapshot.failed = stage_ == Stage::Failed;
        snapshot.candidates = latest_detection_.candidates;
        snapshot.hull_points = latest_detection_.hull_points;
        snapshot.detected_points = latest_detection_.image_points;
        snapshot.accepted_centers_normalized = accepted_centers_normalized_;
        snapshot.cost_history = cost_history_;
        snapshot.stage_index = stage_ == Stage::Collecting ? 0
            : stage_ == Stage::Initializing ? 1
            : stage_ == Stage::Optimizing ? 2
            : stage_ == Stage::Refining ? 3
            : stage_ == Stage::Saving || stage_ == Stage::Failed ? 4
            : 5;
        if (result_) {
            snapshot.rms_px = result_->rms_px;
            snapshot.elapsed_seconds = result_->total_seconds;
            for (std::size_t view = result_->projected_points.size();
                 view > 0U; --view) {
                const std::size_t index = view - 1U;
                if (!result_->projected_points[index].empty()
                    && index < observations_.size()) {
                    snapshot.measured_points = observations_[index].image_points;
                    snapshot.projected_points = result_->projected_points[index];
                    break;
                }
            }
        }
    }

    try {
        const cv::Mat visualization =
            calibration::renderCalibrationVisualization(image, snapshot);
        visualization_publisher_->publish(
            *cv_bridge::CvImage(header, "bgr8", visualization).toImageMsg());
        std_msgs::msg::String status;
        std::ostringstream stream;
        stream << snapshot.stage << ": " << snapshot.detail
               << " | frames=" << snapshot.accepted_frames << '/'
               << snapshot.required_frames;
        if (snapshot.rms_px > 0.0) {
            stream << " | rms_px=" << snapshot.rms_px;
        }
        if (snapshot.elapsed_seconds > 0.0) {
            stream << " | elapsed_sec=" << snapshot.elapsed_seconds;
        }
        status.data = stream.str();
        status_publisher_->publish(status);
    } catch (const std::exception& exception) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Calibration visualization failed: %s", exception.what());
    }
}

void CalibratorNode::finishIfReady()
{
    bool should_shutdown = false;
    Stage stage = Stage::Collecting;
    {
        std::scoped_lock lock(mutex_);
        if (!finished_time_) {
            return;
        }
        stage = stage_;
        const bool may_terminate = stage_ == Stage::Complete
            || (stage_ == Stage::Failed && terminate_on_failure_);
        should_shutdown = may_terminate
            && std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - *finished_time_).count()
                >= completion_display_sec_;
    }
    if (should_shutdown) {
        RCLCPP_INFO(
            get_logger(), "Calibrator terminating after %s.",
            stageName(stage).c_str());
        rclcpp::shutdown();
    }
}

} // namespace uvdar_core::app
