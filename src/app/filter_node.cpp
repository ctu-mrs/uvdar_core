#include "uvdar_core/app/filter_node.hpp"

#include "uvdar_core/helpers/math.hpp"
#include "uvdar_core/helpers/yaml.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>

#include "uvdar_core/helpers/ros_conversions.hpp"

namespace uvdar_core::app {

namespace pe = uvdar_core::pose_estimation;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kUnobservableAngleVariance = 666.0 * 666.0;
constexpr double kTfLookupTimeoutSec = 0.05;

using uvdar_core::helpers::yaml::optionalScalar;
using uvdar_core::helpers::yaml::optionalSequence;

std::array<double, 36> stateCovarianceToMsg(const Eigen::MatrixXd& input, bool velocity_state)
{
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
    if (velocity_state) {
        covariance.topLeftCorner<3, 3>() = input.topLeftCorner<3, 3>();
        covariance.bottomRightCorner<3, 3>() = input.bottomRightCorner<3, 3>();
    } else {
        covariance = input.topLeftCorner<6, 6>();
    }
    return uvdar_core::helpers::covarianceToMsg(covariance);
}

} // namespace

FilterNode::FilterNode(const rclcpp::NodeOptions& options)
    : Node("filter", options)
    , tf_buffer_(get_clock())
    , tf_listener_(tf_buffer_)
{
    loadConfiguration(declare_parameter<std::string>("config_path", ""));
}

void FilterNode::loadConfiguration(const std::string& config_path)
{
    if (config_path.empty()) {
        throw std::runtime_error("filter_node requires parameter 'config_path'.");
    }
    const YAML::Node root = uvdar_core::helpers::yaml::loadFile(config_path);
    const YAML::Node node = root["filtering"];
    if (!node) {
        throw std::runtime_error("Missing filtering config section.");
    }

    pe::KfPoseConfig config;
    config.debug = optionalScalar<bool>(node, "debug", false);
    config.anonymous_measurements = optionalScalar<bool>(node, "anonymous_measurements", false);
    config.indoor = optionalScalar<bool>(node, "indoor", false);
    config.use_velocity = optionalScalar<bool>(node, "use_velocity", false);
    config.process_acceleration_std_horizontal = optionalScalar<double>(
        node, "process_acceleration_std_horizontal", config.process_acceleration_std_horizontal);
    config.process_acceleration_std_vertical = optionalScalar<double>(
        node, "process_acceleration_std_vertical", config.process_acceleration_std_vertical);
    config.process_angular_velocity_std = optionalScalar<double>(
        node, "process_angular_velocity_std", config.process_angular_velocity_std);
    config.initial_velocity_std = optionalScalar<double>(
        node, "initial_velocity_std", config.initial_velocity_std);
    if (config.process_acceleration_std_horizontal < 0.0
        || config.process_acceleration_std_vertical < 0.0
        || config.process_angular_velocity_std < 0.0
        || config.initial_velocity_std <= 0.0) {
        throw std::runtime_error("Filter process-noise standard deviations must be non-negative and initial_velocity_std must be positive.");
    }
    config.min_measurements_to_validation = optionalScalar<int>(node, "min_measurements_to_validation", 10);
    config.decay_age_normal = optionalScalar<double>(node, "decay_age_normal", 3.0);
    config.decay_age_unvalidated = optionalScalar<double>(node, "decay_age_unvalidated", 1.0);
    config.match_level_threshold_associate = optionalScalar<double>(node, "match_level_threshold_associate", 0.3);
    config.match_level_threshold_remove = optionalScalar<double>(node, "match_level_threshold_remove", 0.5);
    output_frame_ = optionalScalar<std::string>(node, "output_frame", std::string("local_origin"));
    config.output_frame = output_frame_;
    config.accepts_correction = [this](const Eigen::Vector3d& position, const std::string& camera_frame, double stamp) {
        if (camera_frame.empty()) {
            return true;
        }
        geometry_msgs::msg::TransformStamped transform_msg;
        try {
            rclcpp::Time time(static_cast<int64_t>(std::llround(stamp * 1.0e9)), RCL_ROS_TIME);
            transform_msg = tf_buffer_.lookupTransform(
                camera_frame, output_frame_, time, tf2::durationFromSec(kTfLookupTimeoutSec));
        } catch (const tf2::TransformException&) {
            return false;
        }
        const Eigen::Vector3d target_camera = uvdar_core::helpers::toEigen(transform_msg) * position;
        const double norm = target_camera.norm();
        if (norm <= 1.5) {
            return false;
        }
        return target_camera.normalized().dot(Eigen::Vector3d::UnitZ()) > -0.173648;
    };
    filter_ = std::make_unique<pe::KfPose>(config);

    std::vector<std::string> input_topics = optionalSequence<std::string>(node, "measured_poses_topics");
    if (input_topics.empty()) {
        input_topics.push_back("/pose_estimator/measured_poses");
    }

    for (const auto& topic : input_topics) {
        subscriptions_.push_back(create_subscription<uvdar_core::msg::PoseWithCovarianceArrayStamped>(
            topic,
            10,
            [this](const uvdar_core::msg::PoseWithCovarianceArrayStamped::ConstSharedPtr msg) { onMeasurement(msg); }));
    }

    filtered_publisher_ = create_publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>(
        optionalScalar<std::string>(node, "output_topic", "filtered_poses"), 10);
    tentative_publisher_ = create_publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>(
        optionalScalar<std::string>(node, "tentative_output_topic", "filtered_poses/tentative"), 10);

    publish_visualization_ = optionalScalar<bool>(node, "publish_visualization", false);
    const std::string visualization_topic = optionalScalar<std::string>(
        node, "visualization_topic", "/filter/visualization");
    velocity_arrow_scale_sec_ = optionalScalar<double>(node, "velocity_arrow_scale_sec", 1.0);
    if (velocity_arrow_scale_sec_ <= 0.0) {
        throw std::runtime_error("filtering.velocity_arrow_scale_sec must be positive.");
    }
    if (publish_visualization_ && !visualization_topic.empty()) {
        visualization_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(visualization_topic, 10);
    }

    const double output_framerate = optionalScalar<double>(node, "output_framerate", 20.0);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, output_framerate)), [this]() { onTimer(); });
}

void FilterNode::onMeasurement(const uvdar_core::msg::PoseWithCovarianceArrayStamped::ConstSharedPtr& msg)
{
    if (!filter_ || msg->poses.empty()) {
        return;
    }

    geometry_msgs::msg::TransformStamped transform_msg;
    try {
        transform_msg = tf_buffer_.lookupTransform(
            output_frame_, msg->header.frame_id, msg->header.stamp,
            tf2::durationFromSec(kTfLookupTimeoutSec));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Could not transform filter measurement: %s", ex.what());
        return;
    }

    const Eigen::Isometry3d transform = uvdar_core::helpers::toEigen(transform_msg);
    std::vector<pe::KfPoseMeasurement> measurements;
    measurements.reserve(msg->poses.size());
    for (const auto& pose : msg->poses) {
        const Eigen::Vector3d position = transform * Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        const Eigen::Quaterniond orientation = Eigen::Quaterniond(transform.rotation())
            * Eigen::Quaterniond(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
        const Eigen::Vector3d rpy = uvdar_core::helpers::quaternionToRpy(orientation.normalized());

        pe::KfPoseMeasurement measurement;
        measurement.id = pose.id;
        measurement.x = Eigen::VectorXd::Zero(6);
        measurement.x << position.x(), position.y(), position.z(), rpy.x(), rpy.y(), rpy.z();
        measurement.covariance = uvdar_core::helpers::rotatePoseCovariance(uvdar_core::helpers::covarianceFromMsg(pose.covariance), transform.rotation());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> orientation_covariance(measurement.covariance.bottomRightCorner<3, 3>());
        if (orientation_covariance.info() == Eigen::Success) {
            Eigen::Vector3d eigenvalues = orientation_covariance.eigenvalues();
            bool changed = false;
            for (int i = 0; i < eigenvalues.size(); ++i) {
                if (eigenvalues(i) >= kPi * kPi) {
                    eigenvalues(i) = kUnobservableAngleVariance;
                    changed = true;
                }
            }
            if (changed) {
                measurement.covariance.bottomRightCorner<3, 3>() =
                    orientation_covariance.eigenvectors() * eigenvalues.asDiagonal() * orientation_covariance.eigenvectors().transpose();
            }
        }
        measurement.stamp = uvdar_core::helpers::toSeconds(msg->header.stamp);
        measurement.receipt_stamp = get_clock()->now().seconds();
        measurement.camera_frame = msg->header.frame_id;
        if (!measurement.x.array().isNaN().any() && !measurement.covariance.array().isNaN().any()) {
            measurements.push_back(std::move(measurement));
        }
    }

    filter_->applyMeasurements(measurements);
}

void FilterNode::onTimer()
{
    if (!filter_) {
        return;
    }
    filter_->spin(get_clock()->now().seconds());
    const auto validated_states = filter_->validatedStates();
    const auto tentative_states = filter_->tentativeStates();
    publishStates(validated_states, filtered_publisher_);
    publishStates(tentative_states, tentative_publisher_);
    publishVisualization(validated_states, tentative_states);
}

void FilterNode::publishVisualization(
    const std::vector<pe::KfPoseState>& validated_states,
    const std::vector<pe::KfPoseState>& tentative_states)
{
    if (!publish_visualization_ || !visualization_publisher_) {
        return;
    }

    visualization_msgs::msg::MarkerArray output;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    output.markers.push_back(clear);

    const auto stamp = get_clock()->now();
    int marker_id = 0;
    const auto append_states = [this, &output, &marker_id, &stamp](
                                   const std::vector<pe::KfPoseState>& states,
                                   bool validated) {
        for (const auto& state : states) {
            const int angle_offset = state.x.size() == 9 ? 6 : 3;
            if (state.x.size() < angle_offset + 3
                || state.covariance.rows() < angle_offset + 3
                || state.covariance.cols() < angle_offset + 3
                || !state.x.allFinite()
                || !state.covariance.allFinite()) {
                continue;
            }

            const Eigen::Vector3d position = state.x.head<3>();
            const Eigen::Matrix3d rotation = uvdar_core::helpers::rpyToQuaternion(
                state.x.segment<3>(angle_offset)).toRotationMatrix();
            const std_msgs::msg::ColorRGBA state_color = validated
                ? std_msgs::msg::ColorRGBA().set__r(0.1F).set__g(0.9F).set__b(0.2F).set__a(0.9F)
                : std_msgs::msg::ColorRGBA().set__r(1.0F).set__g(0.75F).set__b(0.1F).set__a(0.65F);

            const auto make_point = [](const Eigen::Vector3d& value) {
                geometry_msgs::msg::Point point;
                point.x = value.x();
                point.y = value.y();
                point.z = value.z();
                return point;
            };
            const auto initialize_marker = [this, &stamp, &marker_id](
                                               visualization_msgs::msg::Marker& marker,
                                               const std::string& marker_namespace,
                                               int type) {
                marker.header.frame_id = output_frame_;
                marker.header.stamp = stamp;
                marker.ns = marker_namespace;
                marker.id = marker_id++;
                marker.type = type;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.orientation.w = 1.0;
            };

            visualization_msgs::msg::Marker body;
            initialize_marker(body, validated ? "validated_pose" : "tentative_pose", visualization_msgs::msg::Marker::CUBE);
            body.pose.position = make_point(position);
            const Eigen::Quaterniond body_orientation(rotation);
            body.pose.orientation.x = body_orientation.x();
            body.pose.orientation.y = body_orientation.y();
            body.pose.orientation.z = body_orientation.z();
            body.pose.orientation.w = body_orientation.w();
            body.scale.x = 0.35;
            body.scale.y = 0.22;
            body.scale.z = 0.12;
            body.color = state_color;
            output.markers.push_back(std::move(body));

            visualization_msgs::msg::Marker axes;
            initialize_marker(axes, "pose_axes", visualization_msgs::msg::Marker::LINE_LIST);
            axes.scale.x = 0.035;
            const std::array<std_msgs::msg::ColorRGBA, 3> axis_colors {{
                std_msgs::msg::ColorRGBA().set__r(1.0F).set__a(1.0F),
                std_msgs::msg::ColorRGBA().set__g(1.0F).set__a(1.0F),
                std_msgs::msg::ColorRGBA().set__b(1.0F).set__a(1.0F),
            }};
            for (int axis = 0; axis < 3; ++axis) {
                axes.points.push_back(make_point(position));
                axes.points.push_back(make_point(position + rotation.col(axis) * 0.55));
                axes.colors.push_back(axis_colors[axis]);
                axes.colors.push_back(axis_colors[axis]);
            }
            output.markers.push_back(std::move(axes));

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> covariance_solver(
                0.5 * (state.covariance.topLeftCorner<3, 3>()
                    + state.covariance.topLeftCorner<3, 3>().transpose()));
            if (covariance_solver.info() == Eigen::Success) {
                Eigen::Matrix3d covariance_rotation = covariance_solver.eigenvectors();
                if (covariance_rotation.determinant() < 0.0) {
                    covariance_rotation.col(2) *= -1.0;
                }
                visualization_msgs::msg::Marker uncertainty;
                initialize_marker(uncertainty, "position_uncertainty_2sigma", visualization_msgs::msg::Marker::SPHERE);
                uncertainty.pose.position = make_point(position);
                const Eigen::Quaterniond uncertainty_orientation(covariance_rotation);
                uncertainty.pose.orientation.x = uncertainty_orientation.x();
                uncertainty.pose.orientation.y = uncertainty_orientation.y();
                uncertainty.pose.orientation.z = uncertainty_orientation.z();
                uncertainty.pose.orientation.w = uncertainty_orientation.w();
                const Eigen::Vector3d diameters = 4.0
                    * covariance_solver.eigenvalues().cwiseMax(0.0).cwiseSqrt();
                uncertainty.scale.x = std::max(0.01, diameters.x());
                uncertainty.scale.y = std::max(0.01, diameters.y());
                uncertainty.scale.z = std::max(0.01, diameters.z());
                uncertainty.color = state_color;
                uncertainty.color.a = validated ? 0.16F : 0.08F;
                output.markers.push_back(std::move(uncertainty));
            }

            Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
            if (state.x.size() == 9) {
                velocity = state.x.segment<3>(3);
                visualization_msgs::msg::Marker velocity_arrow;
                initialize_marker(velocity_arrow, "linear_velocity", visualization_msgs::msg::Marker::ARROW);
                velocity_arrow.points.push_back(make_point(position));
                velocity_arrow.points.push_back(make_point(position + velocity * velocity_arrow_scale_sec_));
                velocity_arrow.scale.x = 0.07;
                velocity_arrow.scale.y = 0.14;
                velocity_arrow.scale.z = 0.20;
                velocity_arrow.color = std_msgs::msg::ColorRGBA()
                    .set__r(0.9F).set__g(0.15F).set__b(0.95F).set__a(1.0F);
                output.markers.push_back(std::move(velocity_arrow));
            }

            visualization_msgs::msg::Marker label;
            initialize_marker(label, "state_labels", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
            label.pose.position = make_point(position + Eigen::Vector3d(0.0, 0.0, 0.45));
            label.scale.z = 0.22;
            label.color.r = label.color.g = label.color.b = 1.0F;
            label.color.a = 1.0F;
            std::ostringstream text;
            text << (validated ? "KF " : "KF tentative ") << "ID " << state.id
                 << "  |v|=" << std::fixed << std::setprecision(2) << velocity.norm() << " m/s";
            label.text = text.str();
            output.markers.push_back(std::move(label));
        }
    };
    append_states(validated_states, true);
    append_states(tentative_states, false);
    visualization_publisher_->publish(output);
}

void FilterNode::publishStates(
    const std::vector<pe::KfPoseState>& states,
    const rclcpp::Publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>::SharedPtr& publisher)
{
    uvdar_core::msg::PoseWithCovarianceArrayStamped msg;
    msg.header.frame_id = output_frame_;
    msg.header.stamp = get_clock()->now();
    msg.poses.reserve(states.size());
    for (const auto& state : states) {
        const int angle_offset = state.x.size() == 9 ? 6 : 3;
        const Eigen::Quaterniond q = uvdar_core::helpers::rpyToQuaternion(state.x.segment<3>(angle_offset));
        uvdar_core::msg::PoseWithCovarianceIdentified pose;
        pose.id = state.id;
        pose.pose.position.x = state.x[0];
        pose.pose.position.y = state.x[1];
        pose.pose.position.z = state.x[2];
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.covariance = stateCovarianceToMsg(state.covariance, state.x.size() == 9);
        msg.poses.push_back(std::move(pose));
    }
    publisher->publish(msg);
}

} // namespace uvdar_core::app
