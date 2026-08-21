#include "uvdar_core/app/pose_estimator_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include <CvPlot/cvplot.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <yaml-cpp/yaml.h>

#include "uvdar_core/helpers/ros_conversions.hpp"
#include "uvdar_core/helpers/yaml.hpp"
#include "uvdar_core/calibration/lens_model_loader.hpp"
#include "uvdar_core/pose_estimation/body_model.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/geometric_solver.hpp"
#include "uvdar_core/pose_estimation/particle_filter/particle_filter.hpp"
#include "uvdar_core/pose_estimation/particle_filter/reprojection_model.hpp"

namespace uvdar_core::app {

namespace pe = uvdar_core::pose_estimation;
namespace gs = uvdar_core::pose_estimation::geometric_solver;
namespace pf = uvdar_core::pose_estimation::particle_filter;

namespace {
    using uvdar_core::helpers::yaml::optionalScalar;
    using uvdar_core::helpers::yaml::optionalScalarAny;
    using uvdar_core::helpers::yaml::optionalSequence;
    using uvdar_core::helpers::yaml::requireScalar;
    using uvdar_core::helpers::yaml::resolvePath;

    constexpr int kVisualizationWidth = 1600;
    constexpr int kVisualizationHeight = 900;
    
    // Strict layout bounds to force mathematically perfect square grids
    constexpr double kMarginL = 65.0;
    constexpr double kMarginR = 20.0;
    constexpr double kMarginT = 25.0;
    constexpr double kMarginB = 35.0;
    
    // Left plot is 925x900 -> Inner area: 840x840
    constexpr int kLeftPlotWidth = 925;
    constexpr int kLeftPlotHeight = 900;
    
    // Right plots are 475x450 -> Inner area: 390x390
    constexpr int kRightPlotWidth = 475;
    constexpr int kRightTopPlotHeight = 450;
    constexpr int kRightBottomPlotHeight = 450;
    
    // Shorter axes length factor (was 0.25, now 0.12)
    constexpr double kAxisLengthFactor = 0.12;
    constexpr double kRangePadding = 1.2;
    constexpr double kCovarianceEllipseScale = 2.0;
    const double kCovariancePi = std::acos(-1.0);

    enum class PosePlotPlane {
        XY = 0,
        XZ = 1,
        YZ = 2,
    };

    struct PoseForPlot {
        int id = -1;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        Eigen::Matrix3d position_covariance = Eigen::Matrix3d::Identity();
        Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX();
        Eigen::Vector3d y_axis = Eigen::Vector3d::UnitY();
        Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();
    };

    std::pair<double, double> extractPlaneCoordinate(const PoseForPlot& pose, PosePlotPlane plane)
    {
        switch (plane) {
            case PosePlotPlane::XY:
                // Plot XY with X pointing up and Y pointing right.
                return {pose.y, pose.x};
            case PosePlotPlane::XZ:
                return {pose.x, pose.z};
            case PosePlotPlane::YZ:
                return {pose.y, pose.z};
        }
        return {pose.x, pose.y};
    }

    std::pair<double, double> extractPlaneAxis(const PoseForPlot& pose, PosePlotPlane plane, int axis_index)
    {
        if (axis_index == 0) {
            switch (plane) {
                case PosePlotPlane::XY:
                    return {pose.x_axis.y(), pose.x_axis.x()};
                case PosePlotPlane::XZ:
                    return {pose.x_axis.x(), pose.x_axis.z()};
                case PosePlotPlane::YZ:
                    return {pose.y_axis.y(), pose.y_axis.z()};
            }
        }

        switch (plane) {
            case PosePlotPlane::XY:
                return {pose.y_axis.y(), pose.y_axis.x()};
            case PosePlotPlane::XZ:
                return {pose.z_axis.x(), pose.z_axis.z()};
            case PosePlotPlane::YZ:
                return {pose.z_axis.y(), pose.z_axis.z()};
        }
        return {pose.x_axis.x(), pose.x_axis.y()};
    }

    Eigen::Matrix2d extractPlaneCovariance(const PoseForPlot& pose, PosePlotPlane plane)
    {
        switch (plane) {
            case PosePlotPlane::XY:
                return (Eigen::Matrix2d() << pose.position_covariance(1, 1), pose.position_covariance(1, 0),
                    pose.position_covariance(0, 1), pose.position_covariance(0, 0))
                    .finished();
            case PosePlotPlane::XZ:
                return (Eigen::Matrix2d() << pose.position_covariance(0, 0), pose.position_covariance(0, 2),
                    pose.position_covariance(2, 0), pose.position_covariance(2, 2))
                    .finished();
            case PosePlotPlane::YZ:
                return (Eigen::Matrix2d() << pose.position_covariance(1, 1), pose.position_covariance(1, 2),
                    pose.position_covariance(2, 1), pose.position_covariance(2, 2))
                    .finished();
        }
        return (Eigen::Matrix2d() << pose.position_covariance(0, 0), pose.position_covariance(0, 1),
            pose.position_covariance(1, 0), pose.position_covariance(1, 1))
            .finished();
    }

    cv::Point2i toPixel(const double x, const double y, const double range_x, const double range_y, const int width, const int height)
    {
        const double plot_w = width - kMarginL - kMarginR;
        const double plot_h = height - kMarginT - kMarginB;
        
        const double cx = kMarginL + plot_w * 0.5;
        const double cy = kMarginT + plot_h * 0.5;
        
        // CvPlot pads extents by ~5% internally when auto-scaling limits
        const double scale_x = (plot_w / 2.0) / (range_x * 1.05);
        const double scale_y = (plot_h / 2.0) / (range_y * 1.05);
        
        return {
            static_cast<int>(std::lround(cx + x * scale_x)),
            static_cast<int>(std::lround(cy - y * scale_y)),
        };
    }

    std::array<cv::Scalar, 2> axisColorsForPlane(PosePlotPlane plane)
    {
        switch (plane) {
            case PosePlotPlane::XY:
                return {cv::Scalar(0, 0, 220), cv::Scalar(0, 200, 0)}; // R, G
            case PosePlotPlane::XZ:
                return {cv::Scalar(0, 0, 220), cv::Scalar(220, 0, 0)}; // R, B
            case PosePlotPlane::YZ:
                return {cv::Scalar(0, 200, 0), cv::Scalar(220, 0, 0)}; // G, B
        }
        return {cv::Scalar(0, 200, 0), cv::Scalar(220, 0, 0)};
    }

    cv::Mat ensurePlotImageSize(cv::Mat plot, const int width, const int height)
    {
        if (plot.empty()) {
            return plot;
        }
        if (plot.type() != CV_8UC3) {
            if (plot.channels() == 1) {
                cv::cvtColor(plot, plot, cv::COLOR_GRAY2BGR);
            } else if (plot.channels() == 4) {
                cv::cvtColor(plot, plot, cv::COLOR_BGRA2BGR);
            } else {
                return {};
            }
        }
        if (plot.cols == width && plot.rows == height) {
            return plot;
        }
        cv::Mat resized;
        cv::resize(plot, resized, cv::Size(width, height), 0.0, 0.0, cv::INTER_NEAREST);
        return resized;
    }

    std::vector<cv::Point2d> orientationAxisVector(const PoseForPlot& pose, PosePlotPlane plane)
    {
        if (plane == PosePlotPlane::XY) {
            return {cv::Point2d(pose.x_axis.y(), pose.x_axis.x()), cv::Point2d(pose.y_axis.y(), pose.y_axis.x())};
        }
        if (plane == PosePlotPlane::XZ) {
            return {cv::Point2d(pose.x_axis.x(), pose.x_axis.z()), cv::Point2d(pose.z_axis.x(), pose.z_axis.z())};
        }
        return {cv::Point2d(pose.y_axis.y(), pose.y_axis.z()), cv::Point2d(pose.z_axis.y(), pose.z_axis.z())};
    }

    void drawProjectedCovarianceEllipse(
        cv::Mat& plot,
        const PoseForPlot& pose,
        const PosePlotPlane plane,
        const double range_x,
        const double range_y,
        const cv::Scalar& color)
    {
        const auto [x, y] = extractPlaneCoordinate(pose, plane);
        const cv::Point2i center = toPixel(x, y, range_x, range_y, plot.cols, plot.rows);
        const Eigen::Matrix2d covariance = extractPlaneCovariance(pose, plane);
        const Eigen::Matrix2d symmetric_covariance = 0.5 * (covariance + covariance.transpose());
        if (!symmetric_covariance.allFinite()) {
            return;
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(symmetric_covariance);
        if (solver.info() != Eigen::Success) {
            return;
        }
        const Eigen::Vector2d eigenvalues = solver.eigenvalues().cwiseMax(0.0).cwiseSqrt();
        const Eigen::Matrix2d eigenvectors = solver.eigenvectors();
        if (!eigenvalues.allFinite() || !eigenvectors.allFinite()) {
            return;
        }
        if (eigenvalues[1] <= 0.0 || eigenvalues[0] <= 0.0) {
            return;
        }

        const Eigen::Vector2d major_axis = eigenvectors.col(1);
        const Eigen::Vector2d minor_axis = eigenvectors.col(0);

        const cv::Point2i major_end = toPixel(
            x + major_axis(0) * eigenvalues[1] * kCovarianceEllipseScale,
            y + major_axis(1) * eigenvalues[1] * kCovarianceEllipseScale,
            range_x,
            range_y,
            plot.cols,
            plot.rows);
        const cv::Point2i minor_end = toPixel(
            x + minor_axis(0) * eigenvalues[0] * kCovarianceEllipseScale,
            y + minor_axis(1) * eigenvalues[0] * kCovarianceEllipseScale,
            range_x,
            range_y,
            plot.cols,
            plot.rows);

        const cv::Point2d major_vector = major_end - center;
        const cv::Point2d minor_vector = minor_end - center;
        const int major_radius = static_cast<int>(std::lround(cv::norm(major_vector)));
        const int minor_radius = static_cast<int>(std::lround(cv::norm(minor_vector)));
        if (major_radius <= 0 || minor_radius <= 0) {
            return;
        }

        const double angle = std::atan2(major_vector.y, major_vector.x) * 180.0 / kCovariancePi;
        cv::ellipse(plot, center, cv::Size(major_radius, minor_radius), angle, 0.0, 360.0, color, 1, cv::LINE_AA);
    }

    cv::Mat renderPosePlot(
        const std::vector<PoseForPlot>& poses,
        const PosePlotPlane plane,
        const std::string& title,
        const std::string& x_label,
        const std::string& y_label,
        const double range_x,
        const double range_y,
        const int width,
        const int height)
    {
        if (width <= 0 || height <= 0) {
            return cv::Mat();
        }

        auto axes = CvPlot::makePlotAxes();

        // Feed limits silently to CvPlot for 1:1 bound generation (creates thin black axes)
        axes.create<CvPlot::Series>(std::vector<double>{-range_x, range_x}, std::vector<double>{0.0, 0.0}, "-k");
        axes.create<CvPlot::Series>(std::vector<double>{0.0, 0.0}, std::vector<double>{-range_y, range_y}, "-k");

        cv::Mat plot = axes.render(std::max(1, width), std::max(1, height));
        const int plot_cols = plot.cols;
        const int plot_rows = plot.rows;

        // Position nudged Labels (shifted slightly lower, higher, and left per instructions)
        cv::putText(plot, title, cv::Point(kMarginL + 25, kMarginT + 8), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        cv::putText(plot, x_label, cv::Point(plot_cols - kMarginR - 45, plot_rows - kMarginB / 2 + 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        cv::putText(plot, y_label, cv::Point(20, kMarginT - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        
        const auto axis_colors = axisColorsForPlane(plane);

        // Target IDs and Thicker RGB axes (center circles removed)
        for (const PoseForPlot& pose : poses) {
            const auto [x, y] = extractPlaneCoordinate(pose, plane);
            const auto pixel = toPixel(x, y, range_x, range_y, plot.cols, plot.rows);
            if (pixel.x < 0 || pixel.y < 0 || pixel.x >= plot.cols || pixel.y >= plot.rows) {
                continue;
            }

            drawProjectedCovarianceEllipse(plot, pose, plane, range_x, range_y, cv::Scalar(180, 180, 0));
            
            // Draw Shorter & Thicker (2px) RGB Axes natively in OpenCV
            const auto axis_vectors = orientationAxisVector(pose, plane);
            const auto& a0 = axis_vectors[0];
            const auto& a1 = axis_vectors[1];
            
            const double axis_length = std::max(0.2, kAxisLengthFactor * std::min(range_x, range_y));
            
            cv::Point2i p_axis0 = toPixel(x + a0.x * axis_length, y + a0.y * axis_length, range_x, range_y, plot.cols, plot.rows);
            cv::Point2i p_axis1 = toPixel(x + a1.x * axis_length, y + a1.y * axis_length, range_x, range_y, plot.cols, plot.rows);
            
            cv::line(plot, pixel, p_axis0, axis_colors[0], 2, cv::LINE_AA);
            cv::line(plot, pixel, p_axis1, axis_colors[1], 2, cv::LINE_AA);
            
            // Draw Target ID
            cv::putText(
                plot,
                "ID:" + std::to_string(pose.id),
                pixel + cv::Point(6, -6),
                cv::FONT_HERSHEY_SIMPLEX,
                0.55,
                cv::Scalar(0, 0, 0),
                1,
                cv::LINE_AA);
        }

        return plot;
    }

    std::vector<PoseForPlot> collectPoses(const pe::TimedPoseMeasurements& measurements)
    {
        std::vector<PoseForPlot> poses;
        poses.reserve(measurements.poses.size());
        for (const auto& measurement : measurements.poses) {
            const auto& orientation = measurement.pose.orientation;
            if (!std::isfinite(measurement.pose.position.x()) || !std::isfinite(measurement.pose.position.y())
                || !std::isfinite(measurement.pose.position.z()) || !std::isfinite(orientation.x())
                || !std::isfinite(orientation.y()) || !std::isfinite(orientation.z()) || !std::isfinite(orientation.w())) {
                continue;
            }
            PoseForPlot pose;
            pose.id = measurement.id;
            pose.x = measurement.pose.position.x();
            pose.y = measurement.pose.position.y();
            pose.z = measurement.pose.position.z();
            const double norm = orientation.norm();
            if (norm <= std::numeric_limits<double>::epsilon()) {
                continue;
            }
            const auto rotation = measurement.pose.orientation.normalized().toRotationMatrix();
            if (!rotation.allFinite()) {
                continue;
            }
            pose.x_axis = rotation.col(0);
            pose.y_axis = rotation.col(1);
            pose.z_axis = rotation.col(2);
            pose.position_covariance = measurement.covariance.topLeftCorner<3, 3>();
            if (!pose.position_covariance.allFinite()) {
                pose.position_covariance = Eigen::Matrix3d::Identity();
            }
            poses.push_back(std::move(pose));
        }
        return poses;
    }

} // namespace

PoseEstimatorNode::PoseEstimatorNode(const rclcpp::NodeOptions& options)
    : Node("pose_estimator", options)
    , tf_buffer_(get_clock())
    , tf_listener_(tf_buffer_)
{
    const std::string config_path = declare_parameter<std::string>("config_path", "");
    loadConfiguration(config_path);
}

PoseEstimatorNode::~PoseEstimatorNode()
{
    stopVisualizationThread();
}

void PoseEstimatorNode::loadConfiguration(const std::string& config_path_string)
{
    if (config_path_string.empty()) {
        throw std::runtime_error("pose_estimator requires parameter 'config_path'.");
    }

    const std::filesystem::path config_path(config_path_string);
    const YAML::Node root = uvdar_core::helpers::yaml::loadFile(config_path_string);
    const YAML::Node pose_node = root["pose_estimation"];
    if (!pose_node) {
        throw std::runtime_error("Missing pose_estimation config section.");
    }
    const std::string implementation = optionalScalar<std::string>(pose_node, "implementation", "particle_filter");

    output_frame_ = optionalScalar<std::string>(pose_node, "output_frame", "local_origin");
    const std::string output_topic = optionalScalar<std::string>(pose_node, "output_topic", "measuredPoses");
    publish_constituents_ = optionalScalar<bool>(pose_node, "publish_constituents", false);
    publish_visualization_ = optionalScalar<bool>(pose_node, "publish_visualization", false);
    visualization_topic_ = optionalScalar<std::string>(pose_node, "visualization_topic", "/pose_estimator/visualization");
    const double visualization_fps = optionalScalar<double>(pose_node, "visualization_fps", 5.0);
    visualization_period_sec_ = (visualization_fps > 0.0) ? (1.0 / visualization_fps) : 0.2;

    const std::string model_file = resolvePath(
        config_path,
        requireScalar<std::string>(pose_node, "model_file", "pose_estimation"));
    uvdar_core::pose_estimation::BodyModel body(model_file);
    const int signals_per_target = std::max(1, body.maxSignalId() + 1);

    std::vector<int> signal_ids = optionalSequence<int>(pose_node, "signal_ids");
    if (signal_ids.empty()) {
        signal_ids = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    }

    const YAML::Node inputs_node = pose_node["inputs"];
    if (!inputs_node || !inputs_node.IsSequence()) {
        throw std::runtime_error("pose_estimation.inputs must list tracker topics, camera frames, and calibration files.");
    }

    std::vector<pe::CameraModel> cameras;
    for (const YAML::Node& input_node : inputs_node) {
        InputConfig input;
        input.name = optionalScalar<std::string>(input_node, "name", "camera_" + std::to_string(inputs_.size()));
        input.input_topic = requireScalar<std::string>(input_node, "input_topic", "pose_estimation.inputs");
        input.camera_frame = requireScalar<std::string>(input_node, "camera_frame", "pose_estimation.inputs");
        input.calib_file = resolvePath(config_path, optionalScalar<std::string>(input_node, "calib_file", std::string {}));
        inputs_.push_back(input);

        pe::CameraModel camera;
        camera.lens = uvdar_core::calibration::loadLensModel(input_node, config_path);
        camera.image_width = camera.lens->imageWidth();
        camera.image_height = camera.lens->imageHeight();
        cameras.push_back(camera);
    }
    tf_logged_once_.resize(inputs_.size(), false);

    const YAML::Node particle_node = pose_node["particle_filter"];
    const YAML::Node geometric_node = pose_node["geometric_solver"];
    double publish_period_sec = optionalScalar<double>(pose_node, "publish_period_sec", 0.1);
    particle_filter_implementation_ = (implementation == "particle_filter");

    if (implementation == "particle_filter") {
        const auto [max_diameter, min_diameter] = body.maxMinVisibleDiameter();
        (void)min_diameter;

        pf::ParticleFilterConfig filter_config;
        filter_config.debug = optionalScalar<bool>(pose_node, "debug", false);
        filter_config.separate_by_distance = optionalScalarAny<bool>(particle_node, pose_node, "separate_by_distance", true);
        filter_config.max_cluster_distance = optionalScalarAny<double>(particle_node, pose_node, "max_cluster_distance", 100.0);
        filter_config.scatter_time_step = optionalScalarAny<double>(particle_node, pose_node, "scatter_time_step", 0.1);
        filter_config.mutation_position_max_step = optionalScalarAny<double>(
            particle_node,
            pose_node,
            "mutation_position_max_step",
            filter_config.mutation_position_max_step);
        filter_config.mutation_orientation_max_step = optionalScalarAny<double>(
            particle_node,
            pose_node,
            "mutation_orientation_max_step",
            filter_config.mutation_orientation_max_step);
        filter_config.mutation_velocity_max_step = optionalScalarAny<double>(
            particle_node,
            pose_node,
            "mutation_velocity_max_step",
            filter_config.mutation_velocity_max_step);
        filter_config.max_hypothesis_count = optionalScalarAny<int>(particle_node, pose_node, "max_hypothesis_count", 1000);
        filter_config.max_hypothesis_age = optionalScalarAny<double>(particle_node, pose_node, "max_hypothesis_age", 1.5);
        filter_config.output_frame = output_frame_;
        publish_period_sec = filter_config.scatter_time_step;

        pf::ReprojectionModel::Options solver_options;
        solver_options.debug = filter_config.debug;
        solver_options.signal_ids = signal_ids;
        solver_options.signals_per_target = signals_per_target;
        solver_options.max_diameter = max_diameter;
        solver_options.edge_detection_margin = optionalScalarAny<int>(particle_node, pose_node, "edge_detection_margin", 10);
        solver_options.initial_rough_hypothesis_count = optionalScalarAny<int>(particle_node, pose_node, "initial_rough_hypothesis_count", 200);
        solver_options.initial_hypothesis_count = optionalScalarAny<int>(particle_node, pose_node, "initial_hypothesis_count", 10);
        solver_options.max_init_iterations = optionalScalarAny<int>(particle_node, pose_node, "max_init_iterations", 10000);
        solver_options.max_mutation_refine_iterations = optionalScalarAny<int>(particle_node, pose_node, "max_mutation_refine_iterations", 1000);

        auto reprojection_model = std::make_shared<pf::ReprojectionModel>(cameras, body, solver_options);
        pose_estimator_ = std::make_unique<pf::ParticleFilter>(filter_config, body, signal_ids, reprojection_model);
    } else if (implementation == "geometric_solver") {
        gs::GeometricSolverConfig geometric_config;
        const std::string uncertainty_solver = optionalScalarAny<std::string>(geometric_node, pose_node, "uncertainty_solver", "jacobian_propagation");
        if (uncertainty_solver == "jacobian_propagation") {
            geometric_config.uncertainty_solver = gs::UncertaintySolver::JacobianPropagation;
        } else if (uncertainty_solver == "monte_carlo") {
            geometric_config.uncertainty_solver = gs::UncertaintySolver::MonteCarlo;
        } else if (uncertainty_solver == "ellipse_transform") {
            geometric_config.uncertainty_solver = gs::UncertaintySolver::EllipseTransform;
        } else {
            throw std::runtime_error("Unsupported geometric_solver.uncertainty_solver '" + uncertainty_solver + "'.");
        }
        geometric_config.debug = optionalScalar<bool>(pose_node, "debug", false);
        geometric_config.output_frame = output_frame_;
        geometric_config.signal_ids = signal_ids;
        geometric_config.signals_per_target = signals_per_target;
        geometric_config.enable_p2p = optionalScalarAny<bool>(geometric_node, pose_node, "enable_p2p", true);
        geometric_config.uncertainty_samples = optionalScalarAny<int>(geometric_node, pose_node, "uncertainty_samples", 5000);
        geometric_config.p4p_reprojection_threshold_rad = optionalScalarAny<double>(geometric_node, pose_node, "p4p_reprojection_threshold_rad", 0.01);
        geometric_config.covariance_regularization_px = optionalScalarAny<double>(geometric_node, pose_node, "covariance_regularization_px", 1.0e-6);
        geometric_config.refinement_iterations = optionalScalarAny<int>(geometric_node, pose_node, "refinement_iterations", 8);
        geometric_config.pnp_max_iterations = optionalScalarAny<int>(geometric_node, pose_node, "pnp_max_iterations", 40);
        geometric_config.pnp_damping = optionalScalarAny<double>(geometric_node, pose_node, "pnp_damping", 1.0e-8);
        geometric_config.pnp_finite_difference_eps = optionalScalarAny<double>(geometric_node, pose_node, "pnp_finite_difference_eps", 1.0e-6);
        geometric_config.pnp_step_tolerance = optionalScalarAny<double>(geometric_node, pose_node, "pnp_step_tolerance", 1.0e-10);
        geometric_config.pnp_residual_tolerance = optionalScalarAny<double>(geometric_node, pose_node, "pnp_residual_tolerance", 1.0e-10);
        publish_period_sec = optionalScalarAny<double>(geometric_node, pose_node, "publish_period_sec", publish_period_sec);
        pose_estimator_ = std::make_unique<gs::GeometricSolver>(geometric_config, body, cameras);
    } else {
        throw std::runtime_error("Unsupported pose_estimation.implementation '" + implementation + "'.");
    }

    measured_publisher_ = create_publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>(output_topic, 10);
    if (publish_constituents_) {
        hypotheses_publisher_ = create_publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>("constituentHypotheses", 10);
        tentative_hypotheses_publisher_ = create_publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>("constituentHypothesesTentative", 10);
    }
    if (publish_visualization_ && !visualization_topic_.empty()) {
        visualization_publisher_ = create_publisher<sensor_msgs::msg::Image>(visualization_topic_, 10);
    }

    for (std::size_t i = 0; i < inputs_.size(); ++i) {
        subscriptions_.push_back(create_subscription<uvdar_core::msg::TrackerOutput>(
            inputs_[i].input_topic,
            rclcpp::SensorDataQoS(),
            [this, i](const uvdar_core::msg::TrackerOutput::ConstSharedPtr msg) { onTrackerOutput(msg, i); }));
    }

    scatter_timer_ = create_wall_timer(
        std::chrono::duration<double>(std::max(0.001, publish_period_sec)),
        [this]() { onScatterTimer(); });

    startVisualizationThread();
}

void PoseEstimatorNode::onTrackerOutput(const uvdar_core::msg::TrackerOutput::ConstSharedPtr& msg, std::size_t camera_index)
{
    if (!pose_estimator_ || camera_index >= inputs_.size()) {
        return;
    }

    geometry_msgs::msg::TransformStamped camera_to_output_msg;
    geometry_msgs::msg::TransformStamped output_to_camera_msg;
    try {
        const rclcpp::Time stamp(msg->stamp);
        camera_to_output_msg = tf_buffer_.lookupTransform(
            output_frame_,
            inputs_[camera_index].camera_frame,
            stamp,
            tf2::durationFromSec(0.005));
        output_to_camera_msg = tf_buffer_.lookupTransform(
            inputs_[camera_index].camera_frame,
            output_frame_,
            stamp,
            tf2::durationFromSec(0.005));
        if (!tf_logged_once_[camera_index]) {
          RCLCPP_INFO(get_logger(),
              "Successfully looked up transform between '%s' and '%s'",
              output_frame_.c_str(), inputs_[camera_index].camera_frame.c_str());
          tf_logged_once_[camera_index] = true;
        }

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Could not get pose-estimation transform: %s", ex.what());
        return;
    }

    std::vector<pe::TrackedPoint> points;
    points.reserve(msg->blinkers.size());
    for (const auto& blinker : msg->blinkers) {
        pe::TrackedPoint point;
        point.x = blinker.x;
        point.y = blinker.y;
        point.id = blinker.id;
        point.virtual_point = blinker.virtual_point;
        point.associated_with_detection = blinker.associated_with_detection;
        point.covariance << blinker.covariance_00, blinker.covariance_01,
            blinker.covariance_10, blinker.covariance_11;
        point.has_prediction = blinker.poly_reg_computed || blinker.extended_search || blinker.virtual_point;
        point.predicted_position = Eigen::Vector2d(blinker.predicted_x, blinker.predicted_y);
        point.prediction_covariance << blinker.prediction_covariance_00, blinker.prediction_covariance_01,
            blinker.prediction_covariance_10, blinker.prediction_covariance_11;
        points.push_back(point);
    }
    if (camera_index == 0U) {
        latest_primary_input_stamp_ = uvdar_core::helpers::toSeconds(msg->stamp);
    }

    pose_estimator_->processFrame(
        camera_index,
        points,
        static_cast<int>(msg->image_width),
        static_cast<int>(msg->image_height),
        uvdar_core::helpers::toSeconds(msg->stamp),
        uvdar_core::helpers::toEigen(camera_to_output_msg),
        uvdar_core::helpers::toEigen(output_to_camera_msg));

    if (publish_visualization_ && visualization_publisher_ && !particle_filter_implementation_ && camera_index == 0U) {
        auto measurements = pose_estimator_->scatterAndMeasure(get_clock()->now().seconds(), uvdar_core::helpers::toSeconds(msg->stamp));
        queueVisualization(std::move(measurements), msg->stamp);
    }
}

void PoseEstimatorNode::onScatterTimer()
{
    if (!pose_estimator_) {
        return;
    }

    const double now_sec = get_clock()->now().seconds();
    const double stamp_sec = latest_primary_input_stamp_ > 0.0 ? latest_primary_input_stamp_ : now_sec;
    auto measurements = pose_estimator_->scatterAndMeasure(now_sec, stamp_sec);
    publishMeasurements(measurements, measured_publisher_);
    if (publish_visualization_ && visualization_publisher_ && particle_filter_implementation_) {
        queueVisualization(measurements, uvdar_core::helpers::toRosTime(stamp_sec));
    }

    if (publish_constituents_) {
        pe::TimedPoseMeasurements verified;
        verified.frame_id = measurements.frame_id;
        verified.stamp = measurements.stamp;
        verified.poses = pose_estimator_->verifiedHypotheses();
        publishMeasurements(verified, hypotheses_publisher_);

        pe::TimedPoseMeasurements tentative;
        tentative.frame_id = measurements.frame_id;
        tentative.stamp = measurements.stamp;
        tentative.poses = pose_estimator_->tentativeHypotheses();
        publishMeasurements(tentative, tentative_hypotheses_publisher_);
    }
}

void PoseEstimatorNode::queueVisualization(
    const pe::TimedPoseMeasurements& measurements,
    const builtin_interfaces::msg::Time& stamp)
{
    if (!publish_visualization_ || !visualization_publisher_) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(visualization_mutex_);
        pending_visualization_measurements_ = measurements;
        pending_visualization_stamp_ = stamp;
        visualization_pending_ = true;
    }
    visualization_cv_.notify_one();
}

void PoseEstimatorNode::startVisualizationThread()
{
    if (!publish_visualization_ || !visualization_publisher_) {
        return;
    }
    if (visualization_thread_.joinable()) {
        return;
    }

    visualization_thread_running_ = true;
    visualization_thread_ = std::thread(&PoseEstimatorNode::visualizationWorker, this);
}

void PoseEstimatorNode::stopVisualizationThread()
{
    {
        std::lock_guard<std::mutex> lock(visualization_mutex_);
        visualization_thread_running_ = false;
    }
    visualization_cv_.notify_all();
    if (visualization_thread_.joinable()) {
        visualization_thread_.join();
    }
}

void PoseEstimatorNode::visualizationWorker()
{
    const auto visualization_interval = std::chrono::duration<double>(visualization_period_sec_);

    while (true) {
        pe::TimedPoseMeasurements measurements;
        builtin_interfaces::msg::Time stamp;
        bool has_request = false;

        {
            std::unique_lock<std::mutex> lock(visualization_mutex_);
            visualization_cv_.wait_for(lock, visualization_interval, [this] {
                return !visualization_thread_running_ || visualization_pending_;
            });
            if (!visualization_thread_running_) {
                break;
            }
            if (!visualization_pending_) {
                continue;
            }

            measurements = std::move(pending_visualization_measurements_);
            stamp = pending_visualization_stamp_;
            visualization_pending_ = false;
            has_request = true;
        }

        if (!has_request) {
            continue;
        }
        try {
            publishVisualization(measurements, stamp);
        } catch (const std::exception& ex) {
            RCLCPP_WARN(get_logger(), "Publish visualization failed: %s", ex.what());
        }
    }
}

void PoseEstimatorNode::publishMeasurements(
    const pe::TimedPoseMeasurements& measurements,
    const rclcpp::Publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>::SharedPtr& publisher)
{
    if (!publisher) {
        return;
    }

    uvdar_core::msg::PoseWithCovarianceArrayStamped msg;
    msg.header.frame_id = measurements.frame_id;
    const auto stamp_nanoseconds = static_cast<int64_t>(measurements.stamp * 1.0e9);
    msg.header.stamp.sec = static_cast<int32_t>(stamp_nanoseconds / 1000000000LL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(stamp_nanoseconds % 1000000000LL);
    msg.poses.reserve(measurements.poses.size());
    for (const auto& measurement : measurements.poses) {
        uvdar_core::msg::PoseWithCovarianceIdentified pose_msg;
        pose_msg.id = measurement.id;
        pose_msg.pose = uvdar_core::helpers::toMsg(measurement.pose);
        pose_msg.covariance = uvdar_core::helpers::covarianceToMsg(measurement.covariance);
        msg.poses.push_back(std::move(pose_msg));
    }
    publisher->publish(msg);
}

void PoseEstimatorNode::publishVisualization(
        const pe::TimedPoseMeasurements& measurements,
        const builtin_interfaces::msg::Time& stamp)
{
    if (!visualization_publisher_) {
        return;
    }

    cv::Mat left_xy, top_xz, bottom_yz;

    try {
        const auto plot_poses = collectPoses(measurements);
        
        // Exact inner-area sizes to determine bounding limits
        const double W_inner_L = kLeftPlotWidth - kMarginL - kMarginR;
        const double H_inner_L = kLeftPlotHeight - kMarginT - kMarginB;
        const double W_inner_R = kRightPlotWidth - kMarginL - kMarginR;
        const double H_inner_R = kRightTopPlotHeight - kMarginT - kMarginB;

        double max_horiz_L = 0.0, max_vert_L = 0.0;
        double max_horiz_RT = 0.0, max_vert_RT = 0.0;
        double max_horiz_RB = 0.0, max_vert_RB = 0.0;
        
        for (const auto& p : plot_poses) {
            max_horiz_L = std::max(max_horiz_L, std::abs(p.y));
            max_vert_L = std::max(max_vert_L, std::abs(p.x));
            max_horiz_RT = std::max(max_horiz_RT, std::abs(p.x));
            max_vert_RT = std::max(max_vert_RT, std::abs(p.z));
            max_horiz_RB = std::max(max_horiz_RB, std::abs(p.y));
            max_vert_RB = std::max(max_vert_RB, std::abs(p.z));
        }

        // Establish the necessary universal scale 
        double req_S = 0.001; 
        req_S = std::max(req_S, (max_horiz_L * 2.0) / W_inner_L);
        req_S = std::max(req_S, (max_vert_L * 2.0) / H_inner_L);
        req_S = std::max(req_S, (max_horiz_RT * 2.0) / W_inner_R);
        req_S = std::max(req_S, (max_vert_RT * 2.0) / H_inner_R);
        req_S = std::max(req_S, (max_horiz_RB * 2.0) / W_inner_R);
        req_S = std::max(req_S, (max_vert_RB * 2.0) / H_inner_R);

        req_S *= kRangePadding;

        // Force bound expansion into integer 1-meter steps
        double target_rx_L = std::ceil((W_inner_L * req_S) / 2.0);
        if (target_rx_L < 1.0) target_rx_L = 1.0;
        
        // Hysteresis logic implemented utilizing a static cache variable
        static double cached_rx_L = 1.0;
        if (target_rx_L > cached_rx_L) {
            cached_rx_L = target_rx_L;
        } else if (cached_rx_L - target_rx_L >= 2.0) {
            cached_rx_L = target_rx_L;
        }
        target_rx_L = cached_rx_L;
        
        // Finalize static, uniform scaling mathematically tied to the left-plot step bounding
        double final_S = (target_rx_L * 2.0) / W_inner_L;
        
        const double rx_L = target_rx_L;
        const double ry_L = target_rx_L;
        const double rx_R = (W_inner_R * final_S) / 2.0;
        const double ry_R = (H_inner_R * final_S) / 2.0;

        left_xy = renderPosePlot(
            plot_poses, PosePlotPlane::XY, "XY top-view", "y [m]", "x [m]", 
            rx_L, ry_L, kLeftPlotWidth, kLeftPlotHeight);
        top_xz = renderPosePlot(
            plot_poses, PosePlotPlane::XZ, "XZ side-view", "x [m]", "z [m]", 
            rx_R, ry_R, kRightPlotWidth, kRightTopPlotHeight);
        bottom_yz = renderPosePlot(
            plot_poses, PosePlotPlane::YZ, "YZ side-view", "y [m]", "z [m]", 
            rx_R, ry_R, kRightPlotWidth, kRightBottomPlotHeight);

        left_xy = ensurePlotImageSize(std::move(left_xy), kLeftPlotWidth, kLeftPlotHeight);
        top_xz = ensurePlotImageSize(std::move(top_xz), kRightPlotWidth, kRightTopPlotHeight);
        bottom_yz = ensurePlotImageSize(std::move(bottom_yz), kRightPlotWidth, kRightBottomPlotHeight);

        if (left_xy.empty() || top_xz.empty() || bottom_yz.empty()) {
            RCLCPP_WARN(get_logger(), "Pose visualization render returned empty or unexpected image.");
            return;
        }

        cv::Mat canvas(kVisualizationHeight, kVisualizationWidth, CV_8UC3, cv::Scalar(255, 255, 255));
        
        const int offset_x = (kVisualizationWidth - (kLeftPlotWidth + kRightPlotWidth)) / 2;
        
        left_xy.copyTo(canvas(cv::Rect(offset_x, 0, kLeftPlotWidth, kLeftPlotHeight)));
        top_xz.copyTo(canvas(cv::Rect(offset_x + kLeftPlotWidth, 0, kRightPlotWidth, kRightTopPlotHeight)));
        bottom_yz.copyTo(canvas(cv::Rect(offset_x + kLeftPlotWidth, kRightTopPlotHeight, kRightPlotWidth, kRightBottomPlotHeight)));

        std_msgs::msg::Header header;
        header.frame_id = measurements.frame_id;
        header.stamp = stamp;
        const auto msg_ptr = cv_bridge::CvImage(header, "bgr8", canvas).toImageMsg();
        visualization_publisher_->publish(*msg_ptr);
    } catch (const cv::Exception& ex) {
        RCLCPP_WARN(get_logger(), "Pose visualization failed: %s", ex.what());
    } catch (const std::exception& ex) {
        RCLCPP_WARN(get_logger(), "Pose visualization failed: %s", ex.what());
    }
}

} // namespace uvdar_core::app
