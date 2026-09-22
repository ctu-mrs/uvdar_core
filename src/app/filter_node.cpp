#include "uvdar_core/app/filter_node.hpp"

#include "uvdar_core/app/visualization.hpp"
#include "uvdar_core/helpers/math.hpp"
#include "uvdar_core/helpers/yaml.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <yaml-cpp/yaml.h>

#include "uvdar_core/helpers/ros_conversions.hpp"

namespace uvdar_core::app {

namespace pe = uvdar_core::pose_estimation;
namespace vis = uvdar_core::app::visualization;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kUnobservableAngleVariance = 666.0 * 666.0;

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
            transform_msg = tf_buffer_.lookupTransform(camera_frame, output_frame_, time, tf2::durationFromSec(0.005));
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
    const double visualization_fps = optionalScalar<double>(node, "visualization_fps", 5.0);
    if (publish_visualization_ && !visualization_topic.empty()) {
        visualization_publisher_ = create_publisher<sensor_msgs::msg::Image>(visualization_topic, 10);
        visualization_renderer_ = std::make_shared<vis::PoseOverviewRenderer>();
        const auto interval = std::chrono::milliseconds(
            std::max(1, static_cast<int>(std::lround(1000.0 / std::max(0.1, visualization_fps)))));
        visualization_worker_ = std::make_unique<vis::VisualizationWorker>(interval);
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
        transform_msg = tf_buffer_.lookupTransform(output_frame_, msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(0.005));
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
    queueVisualization(validated_states, tentative_states);
}

void FilterNode::queueVisualization(
    const std::vector<pe::KfPoseState>& validated_states,
    const std::vector<pe::KfPoseState>& tentative_states)
{
    if (!publish_visualization_ || !visualization_publisher_ || !visualization_renderer_ || !visualization_worker_) {
        return;
    }

    std::vector<vis::PoseVisualizationPose> poses;
    poses.reserve(validated_states.size() + tentative_states.size());
    const auto append_states = [&poses](const std::vector<pe::KfPoseState>& states, const std::string& method) {
        for (const auto& state : states) {
            const int angle_offset = state.x.size() == 9 ? 6 : 3;
            if (state.x.size() < angle_offset + 3
                || state.covariance.rows() < angle_offset + 3
                || state.covariance.cols() < angle_offset + 3
                || !state.x.allFinite()
                || !state.covariance.allFinite()) {
                continue;
            }
            vis::PoseVisualizationPose pose;
            pose.id = state.id;
            pose.method = method;
            pose.position = state.x.head<3>();
            pose.rotation = uvdar_core::helpers::rpyToQuaternion(
                state.x.segment<3>(angle_offset)).toRotationMatrix();
            pose.position_covariance = state.covariance.topLeftCorner<3, 3>();
            pose.orientation_covariance = state.covariance.block<3, 3>(angle_offset, angle_offset);
            poses.push_back(std::move(pose));
        }
    };
    append_states(validated_states, "KF");
    append_states(tentative_states, "KF tentative");

    std_msgs::msg::Header header;
    header.stamp = get_clock()->now();
    header.frame_id = output_frame_;
    const auto publisher = visualization_publisher_;
    const auto renderer = visualization_renderer_;
    const auto logger = get_logger();
    visualization_worker_->submit([publisher, renderer, header, poses = std::move(poses), logger] {
        try {
            const cv::Mat frame = renderer->render(poses);
            if (!frame.empty()) {
                publisher->publish(*cv_bridge::CvImage(header, "bgr8", frame).toImageMsg());
            }
        } catch (const std::exception& ex) {
            RCLCPP_WARN(logger, "Filter visualization failed: %s", ex.what());
        }
    });
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
