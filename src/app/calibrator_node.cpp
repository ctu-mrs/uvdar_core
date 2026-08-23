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

#include "uvdar_core/calibration/lens_model_loader.hpp"
#include "uvdar_core/helpers/yaml.hpp"

namespace uvdar_core::app {

namespace calibration = uvdar_core::calibration;

namespace {

std::vector<cv::Point2f> sampleModelProjection(
    const std::filesystem::path& calibration_file,
    const calibration::CalibrationResult& result)
{
    YAML::Node input;
    input["calib_file"] = calibration_file.string();
    const calibration::LensModelPtr model = calibration::loadLensModel(
        input, calibration_file);
    const Eigen::Vector2d optical_center = model->project(
        Eigen::Vector3d(0.0, 0.0, 1.0));
    if (!optical_center.allFinite()) {
        return {};
    }

    const double maximum_useful_radius = 1.25 * std::hypot(
        static_cast<double>(result.image_width),
        static_cast<double>(result.image_height));
    constexpr int sample_count = 181;
    constexpr double maximum_angle = 0.5 * M_PI - 1.0e-3;
    std::vector<cv::Point2f> curve;
    curve.reserve(sample_count);
    for (int sample = 0; sample < sample_count; ++sample) {
        const double angle = maximum_angle * sample / (sample_count - 1);
        const Eigen::Vector2d pixel = model->project(Eigen::Vector3d(
            std::sin(angle), 0.0, std::cos(angle)));
        if (!pixel.allFinite()) {
            continue;
        }
        const double radius = (pixel - optical_center).norm();
        if (!std::isfinite(radius) || radius > maximum_useful_radius) {
            continue;
        }
        curve.emplace_back(
            static_cast<float>(angle * 180.0 / M_PI),
            static_cast<float>(radius));
    }
    return curve;
}

} // namespace

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
    RCLCPP_INFO(
        get_logger(),
        "Processing calibration images at %.2f Hz; visualization at %.2f Hz",
        image_processing_fps_,
        visualization_fps_);
}

CalibratorNode::~CalibratorNode()
{
    if (worker_.joinable()) {
        worker_.join();
    }
}

void CalibratorNode::loadParameters()
{
    using uvdar_core::helpers::yaml::requireNode;
    using uvdar_core::helpers::yaml::requireScalar;
    using uvdar_core::helpers::yaml::optionalScalar;
    using uvdar_core::helpers::yaml::resolvePath;

    const std::string config_path_string =
        declare_parameter<std::string>("config_path", "");
    if (config_path_string.empty()) {
        throw std::runtime_error(
            "calibrator_node requires parameter 'config_path'.");
    }
    const std::filesystem::path config_path(config_path_string);
    const YAML::Node root =
        uvdar_core::helpers::yaml::loadFile(config_path_string);
    const YAML::Node config = requireNode(root, "calibrator");

    image_topic_ = requireScalar<std::string>(
        config, "image_topic", "calibrator");
    visualization_topic_ = requireScalar<std::string>(
        config, "visualization_topic", "calibrator");
    status_topic_ = requireScalar<std::string>(
        config, "status_topic", "calibrator");
    output_file_ = resolvePath(config_path, requireScalar<std::string>(
        config, "output_calibration_file", "calibrator"));
    model_name_ = requireScalar<std::string>(
        config, "calibration_model", "calibrator");
    pattern_name_ = requireScalar<std::string>(
        config, "pattern_type", "calibrator");
    required_frames_ = requireScalar<int>(
        config, "required_pattern_frames", "calibrator");
    minimum_frame_interval_sec_ = requireScalar<double>(
        config, "minimum_frame_interval_sec", "calibrator");
    minimum_frame_diversity_ = requireScalar<double>(
        config, "minimum_frame_diversity", "calibrator");
    image_processing_fps_ = optionalScalar<double>(
        config, "image_processing_fps", 2.0);
    visualization_fps_ = requireScalar<double>(
        config, "visualization_fps", "calibrator");
    completion_display_sec_ = requireScalar<double>(
        config, "completion_display_sec", "calibrator");
    terminate_on_failure_ = requireScalar<bool>(
        config, "terminate_on_failure", "calibrator");

    detector_options_.pattern =
        calibration::calibrationPatternFromString(pattern_name_);
    detector_options_.rows = requireScalar<int>(
        config, "pattern_rows", "calibrator");
    detector_options_.columns = requireScalar<int>(
        config, "pattern_columns", "calibrator");
    detector_options_.spacing = requireScalar<double>(
        config, "pattern_spacing", "calibrator");
    detector_options_.checkerboard_max_detection_height = optionalScalar<int>(
        config, "checkerboard_max_detection_height", 520);
    detector_options_.maximum_candidates = requireScalar<int>(
        config, "maximum_detection_candidates", "calibrator");
    detector_options_.fimd_threshold = requireScalar<int>(
        config, "fimd_threshold", "calibrator");
    detector_options_.fimd_threshold_diff = requireScalar<int>(
        config, "fimd_threshold_diff", "calibrator");
    const int fimd_max_markers = requireScalar<int>(
        config, "fimd_max_markers", "calibrator");
    if (fimd_max_markers <= 0) {
        throw std::invalid_argument("fimd_max_markers must be positive.");
    }
    detector_options_.fimd_max_markers =
        static_cast<unsigned>(fimd_max_markers);
    const YAML::Node radii = requireNode(config, "fimd_radii", "calibrator");
    if (!radii.IsSequence()) {
        throw std::invalid_argument("fimd_radii must be a sequence.");
    }
    detector_options_.fimd_radii.clear();
    for (const YAML::Node& radius_node : radii) {
        const int radius = radius_node.as<int>();
        if (radius > 0) {
            detector_options_.fimd_radii.push_back(
                static_cast<unsigned>(radius));
        } else {
            throw std::invalid_argument(
                "fimd_radii must contain only positive radii.");
        }
    }
    if (detector_options_.fimd_radii.empty()) {
        throw std::invalid_argument("fimd_radii must contain a positive radius.");
    }
    detector_options_.hull_maximum_concave_angle = requireScalar<double>(
        config, "hull_maximum_concave_angle", "calibrator");
    detector_options_.hull_similar_angle = requireScalar<double>(
        config, "hull_similar_angle", "calibrator");

    calibrator_options_.model =
        calibration::calibrationModelFromString(model_name_);
    calibrator_options_.minimum_views = requireScalar<int>(
        config, "minimum_valid_views", "calibrator");
    calibrator_options_.max_iterations = requireScalar<int>(
        config, "maximum_optimization_iterations", "calibrator");
    calibrator_options_.outlier_refinement_iterations = requireScalar<int>(
        config, "outlier_refinement_iterations", "calibrator");
    calibrator_options_.initial_damping = requireScalar<double>(
        config, "initial_lm_damping", "calibrator");
    calibrator_options_.huber_delta_px = requireScalar<double>(
        config, "huber_delta_px", "calibrator");
    calibrator_options_.view_outlier_factor = requireScalar<double>(
        config, "view_outlier_factor", "calibrator");
    calibrator_options_.maximum_rms_px = requireScalar<double>(
        config, "maximum_final_rms_px", "calibrator");
    calibrator_options_.ocam_inverse_polynomial_order = requireScalar<int>(
        config, "ocam_inverse_polynomial_order", "calibrator");
    calibrator_options_.ocam_direct_polynomial_order = requireScalar<int>(
        config, "ocam_direct_polynomial_order", "calibrator");
    calibrator_options_.step_tolerance = requireScalar<double>(
        config, "optimization_step_tolerance", "calibrator");
    calibrator_options_.gradient_tolerance = requireScalar<double>(
        config, "optimization_gradient_tolerance", "calibrator");
    calibrator_options_.relative_cost_tolerance = requireScalar<double>(
        config, "optimization_relative_cost_tolerance", "calibrator");

    if (image_topic_.empty() || visualization_topic_.empty()
        || status_topic_.empty() || output_file_.empty()) {
        throw std::invalid_argument(
            "Calibrator topics and output_calibration_file must not be empty.");
    }
    if (required_frames_ < 3
        || calibrator_options_.minimum_views < 3
        || calibrator_options_.minimum_views > required_frames_) {
        throw std::invalid_argument(
            "Require 3 <= minimum_valid_views <= required_pattern_frames.");
    }
    if (detector_options_.rows < 2 || detector_options_.columns < 2
        || detector_options_.spacing <= 0.0
        || detector_options_.checkerboard_max_detection_height < 64
        || detector_options_.maximum_candidates
            < detector_options_.rows * detector_options_.columns) {
        throw std::invalid_argument(
            "Pattern dimensions and spacing must be positive, checkerboard "
            "detection height must be at least 64 pixels, and "
            "maximum_detection_candidates must fit the complete pattern.");
    }
    if (detector_options_.fimd_threshold < 0
        || detector_options_.fimd_threshold > 255
        || detector_options_.fimd_threshold_diff < 0
        || detector_options_.fimd_threshold_diff > 255) {
        throw std::invalid_argument(
            "FIMD thresholds must be in the inclusive range [0, 255].");
    }
    if (detector_options_.hull_maximum_concave_angle <= 0.0
        || detector_options_.hull_maximum_concave_angle >= M_PI
        || detector_options_.hull_similar_angle <= 0.0
        || detector_options_.hull_similar_angle >= M_PI) {
        throw std::invalid_argument(
            "Hull angles must be strictly between zero and pi radians.");
    }
    if (calibrator_options_.max_iterations <= 0
        || calibrator_options_.outlier_refinement_iterations < 0
        || calibrator_options_.initial_damping <= 0.0
        || calibrator_options_.huber_delta_px <= 0.0
        || calibrator_options_.view_outlier_factor <= 0.0
        || calibrator_options_.maximum_rms_px <= 0.0
        || calibrator_options_.ocam_inverse_polynomial_order < 1
        || calibrator_options_.ocam_direct_polynomial_order < 2
        || calibrator_options_.step_tolerance < 0.0
        || calibrator_options_.gradient_tolerance < 0.0
        || calibrator_options_.relative_cost_tolerance < 0.0) {
        throw std::invalid_argument(
            "Calibration iteration counts, model orders, damping, robust "
            "limits, and convergence tolerances are outside valid ranges.");
    }
    if (image_processing_fps_ <= 0.0 || visualization_fps_ <= 0.0
        || minimum_frame_interval_sec_ < 0.0
        || minimum_frame_diversity_ < 0.0 || completion_display_sec_ < 0.0) {
        throw std::invalid_argument(
            "Image-processing and visualization FPS must be positive, and "
            "collection/display limits must be non-negative.");
    }
}

void CalibratorNode::createInterfaces()
{
    image_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    processing_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    visualization_callback_group_ = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    visualization_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        visualization_topic_, rclcpp::QoS(1).reliable());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
        status_topic_, rclcpp::QoS(1).reliable().transient_local());
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = image_callback_group_;
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&CalibratorNode::onImage, this, std::placeholders::_1),
        subscription_options);

    processing_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / image_processing_fps_),
        std::bind(&CalibratorNode::processLatestImage, this),
        processing_callback_group_);
    visualization_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / visualization_fps_),
        std::bind(&CalibratorNode::publishVisualization, this),
        visualization_callback_group_);
    finish_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&CalibratorNode::finishIfReady, this),
        visualization_callback_group_);
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
    std::scoped_lock lock(mutex_);
    if (stage_ != Stage::Collecting) {
        return;
    }
    latest_image_message_ = message;
    ++latest_image_sequence_;
}

void CalibratorNode::processLatestImage()
{
    sensor_msgs::msg::Image::ConstSharedPtr message;
    {
        std::scoped_lock lock(mutex_);
        if (stage_ != Stage::Collecting || !latest_image_message_
            || latest_image_sequence_ == processed_image_sequence_) {
            return;
        }
        message = latest_image_message_;
        processed_image_sequence_ = latest_image_sequence_;
    }

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
    const cv::Mat image = converted->image;
    const std_msgs::msg::Header header = message->header;

    calibration::PatternDetection detection;
    try {
        detection = pattern_detector_->detect(image);
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
        latest_processed_image_ = image;
        latest_processed_header_ = header;
        image_size_ = image.size();
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
            detection.image_points, image.size());
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
        observation_images_.push_back(image);
        observation_headers_.push_back(header);
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
            terminal_visualizations_published_ = 0;
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
        std::vector<cv::Point2f> projection_curve;
        try {
            projection_curve = sampleModelProjection(output_file_, result);
        } catch (const std::exception& exception) {
            RCLCPP_WARN(
                get_logger(),
                "Could not prepare final model projection plot: %s",
                exception.what());
        }
        const double final_rms = result.rms_px;
        const double calibration_seconds = result.total_seconds;
        {
            std::scoped_lock lock(mutex_);
            result_ = std::move(result);
            model_projection_curve_ = std::move(projection_curve);
            stage_ = Stage::Complete;
            detail_ = "Calibration saved successfully";
            finished_time_ = std::chrono::steady_clock::now();
            terminal_visualizations_published_ = 0;
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
            terminal_visualizations_published_ = 0;
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
        image = latest_processed_image_.clone();
        header = latest_processed_header_;
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
        snapshot.image_width = image_size_.width;
        snapshot.image_height = image_size_.height;
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
            snapshot.result_iterations = result_->iterations;
            snapshot.total_views = static_cast<int>(observations_.size());
            snapshot.retained_views = static_cast<int>(std::count(
                result_->retained_views.begin(),
                result_->retained_views.end(),
                true));
            snapshot.initialization_seconds = result_->initialization_seconds;
            snapshot.optimization_seconds = result_->optimization_seconds;
            snapshot.refinement_seconds = result_->outlier_refinement_seconds;
            snapshot.validation_seconds = result_->validation_seconds;
            snapshot.per_view_rms_px = result_->per_view_rms_px;
            snapshot.retained_view_mask = result_->retained_views;
            snapshot.model_projection_curve = model_projection_curve_;
            snapshot.intrinsics = result_->intrinsics;
            snapshot.distortion = result_->distortion;
            snapshot.direct_polynomial = result_->direct_polynomial;
            snapshot.inverse_polynomial = result_->inverse_polynomial;
            snapshot.center = cv::Point2d(
                result_->center.x(), result_->center.y());
            snapshot.affine = cv::Vec3d(
                result_->affine.x(),
                result_->affine.y(),
                result_->affine.z());
            snapshot.result_message = result_->message;

            std::size_t displayed_view = result_->projected_points.size();
            double largest_rms = -1.0;
            for (std::size_t index = 0U;
                 index < result_->projected_points.size(); ++index) {
                if (result_->projected_points[index].empty()
                    || index >= observations_.size()
                    || index >= observation_images_.size()) {
                    continue;
                }
                const double view_rms = index < result_->per_view_rms_px.size()
                    ? result_->per_view_rms_px[index] : 0.0;
                if (view_rms > largest_rms) {
                    largest_rms = view_rms;
                    displayed_view = index;
                }
            }
            if (displayed_view < result_->projected_points.size()) {
                image = observation_images_[displayed_view].clone();
                if (displayed_view < observation_headers_.size()) {
                    header = observation_headers_[displayed_view];
                }
                snapshot.displayed_view = static_cast<int>(displayed_view);
                snapshot.displayed_view_rms_px = largest_rms;
                snapshot.measured_points =
                    observations_[displayed_view].image_points;
                snapshot.projected_points =
                    result_->projected_points[displayed_view];
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
        if (snapshot.successful || snapshot.failed) {
            std::scoped_lock lock(mutex_);
            ++terminal_visualizations_published_;
        }
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
        const int required_terminal_frames = std::max(
            1,
            static_cast<int>(std::ceil(
                completion_display_sec_ * visualization_fps_)));
        should_shutdown = may_terminate
            && std::chrono::duration<double>(
                   std::chrono::steady_clock::now() - *finished_time_).count()
                >= completion_display_sec_
            && terminal_visualizations_published_ >= required_terminal_frames;
    }
    if (should_shutdown) {
        RCLCPP_INFO(
            get_logger(), "Calibrator terminating after %s.",
            stageName(stage).c_str());
        rclcpp::shutdown();
    }
}

} // namespace uvdar_core::app
