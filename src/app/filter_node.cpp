#include "uvdar_core/app/filter_node.hpp"

#include <filesystem>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <yaml-cpp/yaml.h>

namespace uvdar_core::app {

namespace pe = uvdar_core::pose_estimation;

namespace {

template <typename T>
T optionalScalar(const YAML::Node& node, const std::string& key, T fallback)
{
    const YAML::Node value = node[key];
    return value ? value.as<T>() : fallback;
}

double toSeconds(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

Eigen::Isometry3d toEigen(const geometry_msgs::msg::TransformStamped& transform)
{
    const auto& t = transform.transform.translation;
    const auto& q = transform.transform.rotation;
    Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
    output.translation() = Eigen::Vector3d(t.x, t.y, t.z);
    output.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    return output;
}

Eigen::Matrix<double, 6, 6> covarianceFromMsg(const std::array<double, 36>& input)
{
    Eigen::Matrix<double, 6, 6> output;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            output(j, i) = input[static_cast<std::size_t>(6 * j + i)];
        }
    }
    return output;
}

std::array<double, 36> covarianceToMsg(const Eigen::MatrixXd& input, bool velocity_state)
{
    std::array<double, 36> output {};
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
    if (velocity_state) {
        covariance.topLeftCorner<3, 3>() = input.topLeftCorner<3, 3>();
        covariance.bottomRightCorner<3, 3>() = input.bottomRightCorner<3, 3>();
    } else {
        covariance = input.topLeftCorner<6, 6>();
    }
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            output[static_cast<std::size_t>(6 * j + i)] = covariance(j, i);
        }
    }
    return output;
}

Eigen::Matrix<double, 6, 6> rotateCovariance(const Eigen::Matrix<double, 6, 6>& covariance, const Eigen::Matrix3d& rotation)
{
    Eigen::Matrix<double, 6, 6> output = covariance;
    output.topLeftCorner<3, 3>() = rotation * covariance.topLeftCorner<3, 3>() * rotation.transpose();
    output.bottomRightCorner<3, 3>() = rotation * covariance.bottomRightCorner<3, 3>() * rotation.transpose();
    output.topRightCorner<3, 3>() = rotation * covariance.topRightCorner<3, 3>() * rotation.transpose();
    output.bottomLeftCorner<3, 3>() = rotation * covariance.bottomLeftCorner<3, 3>() * rotation.transpose();
    return output;
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

    pe::DkfPoseConfig config;
    config.debug = optionalScalar<bool>(node, "debug", false);
    config.anonymous_measurements = optionalScalar<bool>(node, "anonymous_measurements", false);
    config.indoor = optionalScalar<bool>(node, "indoor", false);
    config.odometry_available = optionalScalar<bool>(node, "odometry_available", true);
    config.use_velocity = optionalScalar<bool>(node, "use_velocity", false);
    config.min_measurements_to_validation = optionalScalar<int>(node, "min_measurements_to_validation", 3);
    config.decay_age_normal = optionalScalar<double>(node, "decay_age_normal", 1.0);
    config.decay_age_unvalidated = optionalScalar<double>(node, "decay_age_unvalidated", 0.3);
    config.match_level_threshold_associate = optionalScalar<double>(node, "match_level_threshold_associate", 0.15);
    config.match_level_threshold_remove = optionalScalar<double>(node, "match_level_threshold_remove", 0.35);
    output_frame_ = optionalScalar<std::string>(node, "output_frame", std::string("local_origin"));
    config.output_frame = output_frame_;
    filter_ = std::make_unique<pe::DkfPose>(config);

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

    const Eigen::Isometry3d transform = toEigen(transform_msg);
    std::vector<pe::DkfPoseMeasurement> measurements;
    measurements.reserve(msg->poses.size());
    for (const auto& pose : msg->poses) {
        const Eigen::Vector3d position = transform * Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        const Eigen::Quaterniond orientation = Eigen::Quaterniond(transform.rotation())
            * Eigen::Quaterniond(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
        const Eigen::Vector3d rpy = pe::quaternionToRpy(orientation.normalized());

        pe::DkfPoseMeasurement measurement;
        measurement.id = pose.id;
        measurement.x = Eigen::VectorXd::Zero(6);
        measurement.x << position.x(), position.y(), position.z(), rpy.x(), rpy.y(), rpy.z();
        measurement.covariance = rotateCovariance(covarianceFromMsg(pose.covariance), transform.rotation());
        measurement.stamp = toSeconds(msg->header.stamp);
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
    const std::vector<pe::DkfPoseState>& states,
    const rclcpp::Publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>::SharedPtr& publisher)
{
    uvdar_core::msg::PoseWithCovarianceArrayStamped msg;
    msg.header.frame_id = output_frame_;
    msg.header.stamp = get_clock()->now();
    msg.poses.reserve(states.size());
    for (const auto& state : states) {
        const int angle_offset = state.x.size() == 9 ? 6 : 3;
        const Eigen::Quaterniond q = pe::rpyToQuaternion(state.x.segment<3>(angle_offset));
        uvdar_core::msg::PoseWithCovarianceIdentified pose;
        pose.id = state.id;
        pose.pose.position.x = state.x[0];
        pose.pose.position.y = state.x[1];
        pose.pose.position.z = state.x[2];
        pose.pose.orientation.w = q.w();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.covariance = covarianceToMsg(state.covariance, state.x.size() == 9);
        msg.poses.push_back(std::move(pose));
    }
    publisher->publish(msg);
}

} // namespace uvdar_core::app
