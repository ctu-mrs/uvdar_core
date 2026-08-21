#include "uvdar_core/app/pose_estimator_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <yaml-cpp/yaml.h>

#include "uvdar_core/helpers/ros_conversions.hpp"
#include "uvdar_core/app/visualization.hpp"
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
namespace vis = uvdar_core::app::visualization;

namespace {
    using uvdar_core::helpers::yaml::optionalScalar;
    using uvdar_core::helpers::yaml::optionalScalarAny;
    using uvdar_core::helpers::yaml::optionalSequence;
    using uvdar_core::helpers::yaml::optionalSequenceAny;
    using uvdar_core::helpers::yaml::requireScalar;
    using uvdar_core::helpers::yaml::resolvePath;


    std::vector<vis::PoseVisualizationPose> collectPoses(const pe::TimedPoseMeasurements& measurements)
    {
        std::vector<vis::PoseVisualizationPose> poses;
        poses.reserve(measurements.poses.size());
        for (const auto& measurement : measurements.poses) {
            const auto& source_pose = measurement.pose;
            if (!source_pose.position.allFinite() || !source_pose.orientation.coeffs().allFinite()
                || source_pose.orientation.norm() <= std::numeric_limits<double>::epsilon()) {
                continue;
            }
            vis::PoseVisualizationPose pose;
            pose.id = measurement.id;
            pose.position = source_pose.position;
            pose.rotation = source_pose.orientation.normalized().toRotationMatrix();
            pose.position_covariance = measurement.covariance.topLeftCorner<3, 3>();
            pose.orientation_covariance = measurement.covariance.bottomRightCorner<3, 3>();
            if (!pose.rotation.allFinite() || !pose.position_covariance.allFinite() || !pose.orientation_covariance.allFinite()) {
                continue;
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
        p2p_odometry_topic_ = optionalScalarAny<std::string>(
            geometric_node,
            pose_node,
            "p2p_odometry_topic",
            "");
        p2p_odometry_maximum_age_sec_ = optionalScalarAny<double>(
            geometric_node,
            pose_node,
            "p2p_odometry_maximum_age_sec",
            p2p_odometry_maximum_age_sec_);
        if (geometric_config.enable_p2p && p2p_odometry_topic_.empty()) {
            throw std::runtime_error(
                "geometric_solver.enable_p2p requires geometric_solver.p2p_odometry_topic "
                "(nav_msgs/msg/Odometry, normally /mavros/local_position/odom).");
        }
        if (p2p_odometry_maximum_age_sec_ < 0.0) {
            throw std::runtime_error("geometric_solver.p2p_odometry_maximum_age_sec must be non-negative.");
        }
        const std::vector<double> p2p_model_gravity_axis = optionalSequenceAny<double>(
            geometric_node,
            pose_node,
            "p2p_model_gravity_axis");
        if (!p2p_model_gravity_axis.empty()) {
            if (p2p_model_gravity_axis.size() != 3U) {
                throw std::runtime_error("geometric_solver.p2p_model_gravity_axis must contain exactly three values.");
            }
            geometric_config.p2p_model_gravity_axis = Eigen::Vector3d(
                p2p_model_gravity_axis[0],
                p2p_model_gravity_axis[1],
                p2p_model_gravity_axis[2]);
            if (!geometric_config.p2p_model_gravity_axis.allFinite()
                || geometric_config.p2p_model_gravity_axis.squaredNorm() <= std::numeric_limits<double>::epsilon()) {
                throw std::runtime_error("geometric_solver.p2p_model_gravity_axis must be a finite non-zero vector.");
            }
            geometric_config.p2p_model_gravity_axis.normalize();
        }
        if (!geometric_config.enable_p2p) {
            // Keep disabled configurations self-contained: no unused odometry
            // subscription and no warning spam from the P2P input path.
            p2p_odometry_topic_.clear();
        }
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
        pose_visualization_renderer_ = std::make_shared<vis::PoseOverviewRenderer>();
        const auto interval = std::chrono::milliseconds(
            std::max(1, static_cast<int>(std::lround(visualization_period_sec_ * 1000.0))));
        visualization_worker_ = std::make_unique<vis::VisualizationWorker>(interval);
    }

    for (std::size_t i = 0; i < inputs_.size(); ++i) {
        subscriptions_.push_back(create_subscription<uvdar_core::msg::TrackerOutput>(
            inputs_[i].input_topic,
            rclcpp::SensorDataQoS(),
            [this, i](const uvdar_core::msg::TrackerOutput::ConstSharedPtr msg) { onTrackerOutput(msg, i); }));
    }
    if (!p2p_odometry_topic_.empty()) {
        p2p_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            p2p_odometry_topic_,
            rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { onP2POdometry(msg); });
    }

    scatter_timer_ = create_wall_timer(
        std::chrono::duration<double>(std::max(0.001, publish_period_sec)),
        [this]() { onScatterTimer(); });

}

void PoseEstimatorNode::onP2POdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    if (!msg) {
        return;
    }
    const auto& orientation = msg->pose.pose.orientation;
    const Eigen::Quaterniond rotation(orientation.w, orientation.x, orientation.y, orientation.z);
    if (!rotation.coeffs().allFinite() || rotation.squaredNorm() <= std::numeric_limits<double>::epsilon()) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Ignoring invalid orientation from geometric_solver.p2p_odometry_topic.");
        return;
    }

    std::scoped_lock lock(p2p_odometry_mutex_);
    latest_p2p_odometry_ = msg;
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

    if (auto* geometric_solver = dynamic_cast<gs::GeometricSolver*>(pose_estimator_.get())) {
        nav_msgs::msg::Odometry::ConstSharedPtr odometry;
        {
            std::scoped_lock lock(p2p_odometry_mutex_);
            odometry = latest_p2p_odometry_;
        }

        std::optional<Eigen::Vector3d> camera_up_axis;
        if (odometry) {
            const rclcpp::Time tracker_stamp(msg->stamp);
            const rclcpp::Time odometry_stamp(odometry->header.stamp);
            const double age = std::abs((tracker_stamp - odometry_stamp).seconds());
            const auto& orientation = odometry->pose.pose.orientation;
            const Eigen::Quaterniond navigation_to_output(
                orientation.w,
                orientation.x,
                orientation.y,
                orientation.z);
            if (std::isfinite(age)
                && (p2p_odometry_maximum_age_sec_ == 0.0 || age <= p2p_odometry_maximum_age_sec_)
                && navigation_to_output.coeffs().allFinite()
                && navigation_to_output.squaredNorm() > std::numeric_limits<double>::epsilon()) {
                // output_to_camera is the already verified, timestamped TF
                // from pose_estimation.output_frame to this camera. The
                // odometry attitude is interpreted in that same output frame.
                camera_up_axis = uvdar_core::helpers::toEigen(output_to_camera_msg).rotation()
                    * navigation_to_output.normalized().toRotationMatrix().transpose()
                    * Eigen::Vector3d::UnitZ();
            }
        }
        if (!camera_up_axis && p2p_odometry_subscription_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "P2P needs recent odometry on '%s'; two-LED pose estimates are unavailable until it arrives.",
                p2p_odometry_topic_.c_str());
        }
        geometric_solver->setCameraUpAxis(camera_index, camera_up_axis);
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
    if (!publish_visualization_ || !visualization_publisher_ || !visualization_worker_ || !pose_visualization_renderer_) {
        return;
    }
    const auto poses = collectPoses(measurements);
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = measurements.frame_id;
    const auto publisher = visualization_publisher_;
    const auto renderer = pose_visualization_renderer_;
    const auto logger = get_logger();
    visualization_worker_->submit([publisher, renderer, header, poses, logger] {
        try {
            const cv::Mat frame = renderer->render(poses);
            if (!frame.empty()) {
                publisher->publish(*cv_bridge::CvImage(header, "bgr8", frame).toImageMsg());
            }
        } catch (const std::exception& ex) {
            RCLCPP_WARN(logger, "Pose visualization failed: %s", ex.what());
        }
    });
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


} // namespace uvdar_core::app
