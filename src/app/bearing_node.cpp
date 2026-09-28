#include "uvdar_core/app/bearing_node.hpp"

#include <cmath>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include "uvdar_core/calibration/lens_model_loader.hpp"
#include "uvdar_core/helpers/ros_conversions.hpp"
#include "uvdar_core/helpers/yaml.hpp"
#include "uvdar_core/msg/bearing_observation.hpp"
#include "uvdar_core/msg/tracked_blinker.hpp"
#include "uvdar_core/pose_estimation/bearing.hpp"

namespace uvdar_core::app {

namespace {

using uvdar_core::helpers::yaml::optionalScalar;
using uvdar_core::helpers::yaml::requireScalar;

bool isInsideImage(const Eigen::Vector2d& pixel, const uvdar_core::msg::TrackerOutput& msg,
                   const calibration::ILensModel& lens)
{
    const int width = msg.image_width > 0U ? static_cast<int>(msg.image_width) : lens.imageWidth();
    const int height = msg.image_height > 0U ? static_cast<int>(msg.image_height) : lens.imageHeight();
    return pixel.allFinite() && (width <= 0 || (pixel.x() >= 0.0 && pixel.x() < width)) &&
           (height <= 0 || (pixel.y() >= 0.0 && pixel.y() < height));
}

Eigen::Matrix2d measuredCovariance(const uvdar_core::msg::TrackedBlinker& blinker)
{
    Eigen::Matrix2d covariance;
    covariance << blinker.covariance_00, blinker.covariance_01, blinker.covariance_10, blinker.covariance_11;
    return covariance;
}

Eigen::Matrix2d predictionCovariance(const uvdar_core::msg::TrackedBlinker& blinker)
{
    Eigen::Matrix2d covariance;
    covariance << blinker.prediction_covariance_00, blinker.prediction_covariance_01, blinker.prediction_covariance_10,
        blinker.prediction_covariance_11;
    return covariance;
}

} // namespace

BearingNode::BearingNode(const rclcpp::NodeOptions& options)
    : Node("bearing", options)
    , tf_buffer_(get_clock())
    , tf_listener_(tf_buffer_)
{
    const std::string config_path = declare_parameter<std::string>("config_path", "");
    loadConfiguration(config_path);
    createInterfaces();
    RCLCPP_INFO(get_logger(), "UVDAR bearing node initialized with %zu camera input(s).", inputs_.size());
}

void BearingNode::loadConfiguration(const std::string& config_path_string)
{
    if (config_path_string.empty()) {
        throw std::runtime_error("bearing_node requires parameter 'config_path'.");
    }

    const std::filesystem::path config_path(config_path_string);
    const YAML::Node root = uvdar_core::helpers::yaml::loadFile(config_path_string);
    const YAML::Node bearing_node = root["bearing"];
    if (!bearing_node || !bearing_node.IsMap()) {
        throw std::runtime_error("Missing bearing config section.");
    }

    queue_depth_ = optionalScalar<std::size_t>(bearing_node, "queue_depth", queue_depth_);
    publish_predictions_ = optionalScalar<bool>(bearing_node, "publish_predictions", publish_predictions_);
    publish_unidentified_ = optionalScalar<bool>(bearing_node, "publish_unidentified", publish_unidentified_);
    covariance_floor_px2_ = optionalScalar<double>(bearing_node, "covariance_floor_px2", covariance_floor_px2_);
    fallback_pixel_variance_px2_ =
        optionalScalar<double>(bearing_node, "fallback_pixel_variance_px2", fallback_pixel_variance_px2_);
    output_frame_ = requireScalar<std::string>(bearing_node, "output_frame", "bearing");
    output_topic_ = requireScalar<std::string>(bearing_node, "output_topic", "bearing");

    if (queue_depth_ == 0U) {
        throw std::runtime_error("bearing.queue_depth must be positive.");
    }
    if (!std::isfinite(covariance_floor_px2_) || covariance_floor_px2_ < 0.0) {
        throw std::runtime_error("bearing.covariance_floor_px2 must be finite and non-negative.");
    }
    if (!std::isfinite(fallback_pixel_variance_px2_) || fallback_pixel_variance_px2_ <= 0.0) {
        throw std::runtime_error("bearing.fallback_pixel_variance_px2 must be finite and positive.");
    }

    const YAML::Node inputs_node = bearing_node["inputs"];
    if (!inputs_node || !inputs_node.IsSequence() || inputs_node.size() == 0U) {
        throw std::runtime_error("bearing.inputs must contain at least one camera pipeline.");
    }

    inputs_.reserve(inputs_node.size());
    for (const YAML::Node& input_node : inputs_node) {
        InputPipeline input;
        input.name = optionalScalar<std::string>(input_node, "name", "camera_" + std::to_string(inputs_.size()));
        input.input_topic = requireScalar<std::string>(input_node, "input_topic", "bearing.inputs");
        input.camera_frame = requireScalar<std::string>(input_node, "camera_frame", "bearing.inputs");
        input.lens = calibration::loadLensModel(input_node, config_path);
        if (!input.lens) {
            throw std::runtime_error("Could not load camera model for bearing input '" + input.name + "'.");
        }
        inputs_.push_back(std::move(input));
    }
}

void BearingNode::createInterfaces()
{
    // Match the tracker's sensor-data output on input. Publish the endpoint
    // reliably so both reliable and best-effort consumers can subscribe.
    const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(queue_depth_)).best_effort();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(queue_depth_)).reliable();
    publisher_ = create_publisher<uvdar_core::msg::BearingObservationArrayStamped>(output_topic_, output_qos);
    for (std::size_t index = 0U; index < inputs_.size(); ++index) {
        InputPipeline& input = inputs_[index];
        input.subscription = create_subscription<uvdar_core::msg::TrackerOutput>(
            input.input_topic, input_qos,
            [this, index](const uvdar_core::msg::TrackerOutput::ConstSharedPtr& msg) { onTrackerOutput(msg, index); });
    }
}

void BearingNode::onTrackerOutput(const uvdar_core::msg::TrackerOutput::ConstSharedPtr& msg,
                                  const std::size_t input_index)
{
    if (!msg || input_index >= inputs_.size()) {
        return;
    }

    InputPipeline& input = inputs_[input_index];
    Eigen::Isometry3d camera_to_output = Eigen::Isometry3d::Identity();
    if (input.camera_frame != output_frame_) {
        try {
            const auto transform = tf_buffer_.lookupTransform(
                output_frame_, input.camera_frame, msg->stamp, tf2::durationFromSec(0.005));
            camera_to_output = uvdar_core::helpers::toEigen(transform);
        } catch (const tf2::TransformException& exception) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Could not transform bearing input '%s' from '%s' to '%s': %s",
                input.name.c_str(), input.camera_frame.c_str(), output_frame_.c_str(), exception.what());
            return;
        }
    }
    const Eigen::Matrix3d camera_rotation = camera_to_output.rotation();

    uvdar_core::msg::BearingObservationArrayStamped output;
    output.header.stamp = msg->stamp;
    output.header.frame_id = output_frame_;
    output.observations.reserve(msg->blinkers.size());

    for (const auto& blinker : msg->blinkers) {
        if (blinker.id < 0 && !publish_unidentified_) {
            continue;
        }

        // A current detector association always wins over tracker prediction,
        // even if an inconsistent producer also sets virtual_point.
        const bool predicted = !blinker.associated_with_detection;
        if (predicted && !publish_predictions_) {
            continue;
        }

        const Eigen::Vector2d pixel = predicted ? Eigen::Vector2d(blinker.predicted_x, blinker.predicted_y)
                                                : Eigen::Vector2d(blinker.x, blinker.y);
        const Eigen::Matrix2d pixel_covariance =
            predicted ? predictionCovariance(blinker) : measuredCovariance(blinker);
        if (!isInsideImage(pixel, *msg, *input.lens)) {
            continue;
        }

        std::optional<uvdar_core::pose_estimation::BearingMeasurement> measurement;
        try {
            measurement = uvdar_core::pose_estimation::bearingFromPixel(
                *input.lens, pixel, pixel_covariance, covariance_floor_px2_, fallback_pixel_variance_px2_);
        } catch (const std::exception& exception) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Bearing conversion failed for input '%s': %s",
                                 input.name.c_str(), exception.what());
            continue;
        }
        if (!measurement) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Ignoring invalid tracked point on bearing input '%s'.", input.name.c_str());
            continue;
        }

        uvdar_core::msg::BearingObservation observation;
        observation.id = blinker.id;
        observation.track_id = blinker.track_id;
        const Eigen::Vector3d bearing = camera_rotation * measurement->vector;
        const Eigen::Matrix3d covariance =
            camera_rotation * measurement->covariance * camera_rotation.transpose();
        observation.bearing.x = bearing.x();
        observation.bearing.y = bearing.y();
        observation.bearing.z = bearing.z();
        observation.origin.x = camera_to_output.translation().x();
        observation.origin.y = camera_to_output.translation().y();
        observation.origin.z = camera_to_output.translation().z();
        observation.predicted = predicted;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                observation.covariance[static_cast<std::size_t>(3 * row + column)] =
                    covariance(row, column);
            }
        }
        output.observations.push_back(std::move(observation));
    }

    // All cameras publish on one endpoint in one body frame. If two cameras
    // see the same decoded ID, consumers receive two independent rays with
    // their respective camera origins and may update the target twice.
    publisher_->publish(std::move(output));
}

} // namespace uvdar_core::app
