#include "uvdar_core/app/filter_node.hpp"

#include "uvdar_core/helpers/frame_namespace.hpp"
#include "uvdar_core/helpers/math.hpp"

#include <filesystem>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <yaml-cpp/yaml.h>

#include "uvdar_core/helpers/ros_conversions.hpp"

namespace uvdar_core::app {

namespace pe = uvdar_core::pose_estimation;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kUnobservableAngleVariance = 666.0 * 666.0;

template <typename T>
T optionalScalar(const YAML::Node& node, const std::string& key, T fallback)
{
    const YAML::Node value = node[key];
    return value ? value.as<T>() : fallback;
}

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
    const YAML::Node root = YAML::LoadFile(config_path);
    const YAML::Node node = root["filtering"];
    if (!node) {
        throw std::runtime_error("Missing filtering config section.");
    }

    pe::KfPoseConfig config;
    config.debug = optionalScalar<bool>(node, "debug", false);
    config.anonymous_measurements = optionalScalar<bool>(node, "anonymous_measurements", false);
    config.indoor = optionalScalar<bool>(node, "indoor", false);
    config.odometry_available = optionalScalar<bool>(node, "odometry_available", true);
    config.use_velocity = optionalScalar<bool>(node, "use_velocity", false);
    config.min_measurements_to_validation = optionalScalar<int>(node, "min_measurements_to_validation", 10);
    config.decay_age_normal = optionalScalar<double>(node, "decay_age_normal", 3.0);
    config.decay_age_unvalidated = optionalScalar<double>(node, "decay_age_unvalidated", 1.0);
    config.match_level_threshold_associate = optionalScalar<double>(node, "match_level_threshold_associate", 0.3);
    config.match_level_threshold_remove = optionalScalar<double>(node, "match_level_threshold_remove", 0.5);
    output_frame_ = uvdar_core::helpers::resolveFrameName(optionalScalar<std::string>(node, "output_frame", std::string("local_origin")));
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

    std::vector<std::string> input_topics;
    if (const YAML::Node topics = node["measured_poses_topics"]; topics && topics.IsSequence()) {
        for (const YAML::Node& topic : topics) {
            input_topics.push_back(topic.as<std::string>());
        }
    }
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
    publishStates(filter_->validatedStates(), filtered_publisher_);
    publishStates(filter_->tentativeStates(), tentative_publisher_);
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
