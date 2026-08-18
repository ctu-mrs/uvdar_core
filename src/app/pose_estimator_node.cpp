#include "uvdar_core/app/pose_estimator_node.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <yaml-cpp/yaml.h>

#include "uvdar_core/app/ros_conversions.hpp"
#include "uvdar_core/calibration/fisheye/equidistant_model.hpp"
#include "uvdar_core/calibration/fisheye/ocam_model.hpp"
#include "uvdar_core/calibration/fisheye/radial_model.hpp"
#include "uvdar_core/calibration/pinhole/pinhole_model.hpp"
#include "uvdar_core/pose_estimation/body_model.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/geometric_solver.hpp"
#include "uvdar_core/pose_estimation/particle_filter/particle_filter.hpp"
#include "uvdar_core/pose_estimation/particle_filter/reprojection_model.hpp"

namespace uvdar_core::app {

namespace pe = uvdar_core::pose_estimation;
namespace gs = uvdar_core::pose_estimation::geometric_solver;
namespace pf = uvdar_core::pose_estimation::particle_filter;

namespace {

template <typename T>
T optionalScalar(const YAML::Node& node, const std::string& key, T fallback)
{
    const YAML::Node value = node[key];
    return value ? value.as<T>() : fallback;
}

std::string requireString(const YAML::Node& node, const std::string& key)
{
    const YAML::Node value = node[key];
    if (!value) {
        throw std::runtime_error("Missing required pose_estimation key '" + key + "'.");
    }
    return value.as<std::string>();
}

std::string resolvePath(const std::filesystem::path& config_path, const std::string& value)
{
    if (value.empty()) {
        return {};
    }
    const std::filesystem::path path(value);
    if (path.is_absolute()) {
        return path.string();
    }
    return (config_path.parent_path() / path).lexically_normal().string();
}

std::vector<double> readVector(const YAML::Node& node, const std::string& key)
{
    std::vector<double> values;
    const YAML::Node vector_node = node[key];
    if (!vector_node || !vector_node.IsSequence()) {
        return values;
    }
    for (const YAML::Node& value : vector_node) {
        values.push_back(value.as<double>());
    }
    return values;
}

std::vector<double> readVectorAny(const YAML::Node& primary, const YAML::Node& fallback, const std::string& key)
{
    std::vector<double> values = readVector(primary, key);
    if (!values.empty()) {
        return values;
    }
    return readVector(fallback, key);
}

template <typename T>
T optionalScalarAny(const YAML::Node& primary, const YAML::Node& fallback, const std::string& key, T default_value)
{
    if (primary && primary[key]) {
        return primary[key].as<T>();
    }
    return optionalScalar<T>(fallback, key, default_value);
}

YAML::Node loadCameraConfigFile(const YAML::Node& input_node, const std::filesystem::path& config_path)
{
    const std::string calib_file = optionalScalar<std::string>(input_node, "calib_file", std::string {});
    if (calib_file.empty()) {
        return {};
    }

    const std::string resolved = resolvePath(config_path, calib_file);
    const std::filesystem::path path(resolved);
    if (path.extension() != ".yaml" && path.extension() != ".yml") {
        return {};
    }
    return YAML::LoadFile(resolved);
}

uvdar_core::calibration::fisheye::OcamModel loadOcamYamlModel(const YAML::Node& camera_node)
{
    uvdar_core::calibration::fisheye::OcamModel model;
    const std::vector<double> direct = readVector(camera_node, "direct_polynomial");
    const std::vector<double> inverse = readVector(camera_node, "inverse_polynomial");
    const std::vector<double> center = readVector(camera_node, "center");
    const std::vector<double> affine = readVector(camera_node, "affine");
    const std::vector<double> image_size = readVector(camera_node, "image_size");

    if (direct.empty() || direct.size() > static_cast<std::size_t>(uvdar_core::calibration::fisheye::max_polynomial_length)) {
        throw std::runtime_error("OCam YAML requires direct_polynomial with 1..64 coefficients.");
    }
    if (inverse.empty() || inverse.size() > static_cast<std::size_t>(uvdar_core::calibration::fisheye::max_polynomial_length)) {
        throw std::runtime_error("OCam YAML requires inverse_polynomial with 1..64 coefficients.");
    }
    if (center.size() != 2U) {
        throw std::runtime_error("OCam YAML requires center: [row, column].");
    }
    if (affine.size() != 3U) {
        throw std::runtime_error("OCam YAML requires affine: [c, d, e].");
    }
    if (image_size.size() != 2U) {
        throw std::runtime_error("OCam YAML requires image_size: [height, width].");
    }

    model.length_pol = static_cast<int>(direct.size());
    std::copy(direct.begin(), direct.end(), model.pol.begin());
    model.length_invpol = static_cast<int>(inverse.size());
    std::copy(inverse.begin(), inverse.end(), model.invpol.begin());
    model.xc = center[0];
    model.yc = center[1];
    model.c = affine[0];
    model.d = affine[1];
    model.e = affine[2];
    model.height = static_cast<int>(std::llround(image_size[0]));
    model.width = static_cast<int>(std::llround(image_size[1]));
    return model;
}

uvdar_core::calibration::LensModelPtr loadLensModel(const YAML::Node& input_node, const std::filesystem::path& config_path)
{
    const YAML::Node camera_node = loadCameraConfigFile(input_node, config_path);
    const std::string model_type = optionalScalarAny<std::string>(camera_node, input_node, "calibration_model", "ocamcalib");
    if (model_type == "ocamcalib") {
        if (camera_node) {
            return std::make_shared<uvdar_core::calibration::fisheye::OcamModel>(loadOcamYamlModel(camera_node));
        }
        const std::string calib_file = resolvePath(config_path, requireString(input_node, "calib_file"));
        return std::make_shared<uvdar_core::calibration::fisheye::OcamModel>(
            uvdar_core::calibration::fisheye::loadModel(calib_file));
    }

    const auto intrinsics = readVectorAny(camera_node, input_node, "intrinsics");
    const auto distortion = readVectorAny(camera_node, input_node, "distortion");
    const int width = optionalScalarAny<int>(camera_node, input_node, "image_width", 0);
    const int height = optionalScalarAny<int>(camera_node, input_node, "image_height", 0);
    if (intrinsics.size() < 4U) {
        throw std::runtime_error("Calibration model '" + model_type + "' requires intrinsics: [fx, fy, cx, cy].");
    }

    if (model_type == "pinhole") {
        uvdar_core::calibration::pinhole::PinholeModel::Parameters parameters;
        parameters.fx = intrinsics[0];
        parameters.fy = intrinsics[1];
        parameters.cx = intrinsics[2];
        parameters.cy = intrinsics[3];
        parameters.width = width;
        parameters.height = height;
        if (distortion.size() > 0U) parameters.k1 = distortion[0];
        if (distortion.size() > 1U) parameters.k2 = distortion[1];
        if (distortion.size() > 2U) parameters.p1 = distortion[2];
        if (distortion.size() > 3U) parameters.p2 = distortion[3];
        if (distortion.size() > 4U) parameters.k3 = distortion[4];
        return std::make_shared<uvdar_core::calibration::pinhole::PinholeModel>(parameters);
    }

    if (model_type == "fisheye_equidistant" || model_type == "equidistant") {
        uvdar_core::calibration::fisheye::EquidistantModel::Parameters parameters;
        parameters.fx = intrinsics[0];
        parameters.fy = intrinsics[1];
        parameters.cx = intrinsics[2];
        parameters.cy = intrinsics[3];
        parameters.width = width;
        parameters.height = height;
        if (distortion.size() > 0U) parameters.k1 = distortion[0];
        if (distortion.size() > 1U) parameters.k2 = distortion[1];
        if (distortion.size() > 2U) parameters.k3 = distortion[2];
        if (distortion.size() > 3U) parameters.k4 = distortion[3];
        return std::make_shared<uvdar_core::calibration::fisheye::EquidistantModel>(parameters);
    }

    if (model_type == "fisheye_equisolid" || model_type == "equisolid" || model_type == "equisolid_angle"
        || model_type == "fisheye_stereographic" || model_type == "stereographic"
        || model_type == "fisheye_orthographic" || model_type == "orthographic") {
        uvdar_core::calibration::fisheye::RadialModel::Parameters parameters;
        if (model_type == "fisheye_stereographic" || model_type == "stereographic") {
            parameters.projection = uvdar_core::calibration::fisheye::RadialModel::Projection::Stereographic;
        } else if (model_type == "fisheye_orthographic" || model_type == "orthographic") {
            parameters.projection = uvdar_core::calibration::fisheye::RadialModel::Projection::Orthographic;
        } else {
            parameters.projection = uvdar_core::calibration::fisheye::RadialModel::Projection::EquisolidAngle;
        }
        parameters.fx = intrinsics[0];
        parameters.fy = intrinsics[1];
        parameters.cx = intrinsics[2];
        parameters.cy = intrinsics[3];
        parameters.width = width;
        parameters.height = height;
        if (distortion.size() > 0U) parameters.k1 = distortion[0];
        if (distortion.size() > 1U) parameters.k2 = distortion[1];
        if (distortion.size() > 2U) parameters.k3 = distortion[2];
        if (distortion.size() > 3U) parameters.k4 = distortion[3];
        return std::make_shared<uvdar_core::calibration::fisheye::RadialModel>(parameters);
    }

    throw std::runtime_error("Unsupported calibration_model '" + model_type + "'.");
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
    const YAML::Node root = YAML::LoadFile(config_path_string);
    const YAML::Node pose_node = root["pose_estimation"];
    if (!pose_node) {
        throw std::runtime_error("Missing pose_estimation config section.");
    }
    const std::string implementation = optionalScalar<std::string>(pose_node, "implementation", "particle_filter");

    output_frame_ = optionalScalar<std::string>(pose_node, "output_frame", "local_origin");
    const std::string output_topic = optionalScalar<std::string>(pose_node, "output_topic", "measuredPoses");
    publish_constituents_ = optionalScalar<bool>(pose_node, "publish_constituents", false);

    const std::string model_file = resolvePath(config_path, requireString(pose_node, "model_file"));
    uvdar_core::pose_estimation::BodyModel body(model_file);
    const int signals_per_target = std::max(1, body.maxSignalId() + 1);

    std::vector<int> signal_ids;
    if (const YAML::Node signal_node = pose_node["signal_ids"]; signal_node && signal_node.IsSequence()) {
        for (const YAML::Node& value : signal_node) {
            signal_ids.push_back(value.as<int>());
        }
    }
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
        input.input_topic = requireString(input_node, "input_topic");
        input.camera_frame = requireString(input_node, "camera_frame");
        input.calib_file = resolvePath(config_path, optionalScalar<std::string>(input_node, "calib_file", std::string {}));
        inputs_.push_back(input);

        pe::CameraModel camera;
        camera.lens = loadLensModel(input_node, config_path);
        camera.image_width = camera.lens->imageWidth();
        camera.image_height = camera.lens->imageHeight();
        cameras.push_back(camera);
    }

    const YAML::Node particle_node = pose_node["particle_filter"];
    const YAML::Node geometric_node = pose_node["geometric_solver"];
    double publish_period_sec = optionalScalar<double>(pose_node, "publish_period_sec", 0.1);

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

    for (std::size_t i = 0; i < inputs_.size(); ++i) {
        subscriptions_.push_back(create_subscription<uvdar_core::msg::TrackerOutput>(
            inputs_[i].input_topic,
            rclcpp::SensorDataQoS(),
            [this, i](const uvdar_core::msg::TrackerOutput::ConstSharedPtr msg) { onTrackerOutput(msg, i); }));
    }

    scatter_timer_ = create_wall_timer(
        std::chrono::duration<double>(std::max(0.001, publish_period_sec)),
        [this]() { onScatterTimer(); });
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
        latest_primary_input_stamp_ = toSeconds(msg->stamp);
    }

    pose_estimator_->processFrame(
        camera_index,
        points,
        static_cast<int>(msg->image_width),
        static_cast<int>(msg->image_height),
        toSeconds(msg->stamp),
        toEigen(camera_to_output_msg),
        toEigen(output_to_camera_msg));
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
        pose_msg.pose = toMsg(measurement.pose);
        pose_msg.covariance = covarianceToMsg(measurement.covariance);
        msg.poses.push_back(std::move(pose_msg));
    }
    publisher->publish(msg);
}

} // namespace uvdar_core::app
