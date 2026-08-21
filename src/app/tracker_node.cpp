#include "uvdar_core/app/tracker_node.hpp"

#include <cmath>
#include <sensor_msgs/image_encodings.hpp>

#include "uvdar_core/helpers/ros_conversions.hpp"

namespace uvdar_core::app {

namespace {

int publishedSignalId(int id, std::size_t sequence_count)
{
    return (0 <= id && id <= static_cast<int>(sequence_count)) ? id : -2;
}

} // namespace

TrackerNode::TrackerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("tracker", options)
{
    loadConfig();
    startup_time_ = now();
    thread_pool_ = std::make_unique<uvdar_core::helpers::ThreadPool>(config_.tracking.thread_pool_size);
    createInterfaces();

    RCLCPP_INFO(get_logger(), "UVDAR tracker node initialized.");
}

/**
 * @brief Parse ROS parameter and load package YAML configuration.
 */
void TrackerNode::loadConfig()
{
    config_path_ = declare_parameter<std::string>("config_path", std::string { });
    config_ = loadPackageConfig(config_path_);
    if (!config_.tracking.module.enabled) {
        throw std::runtime_error("tracking module is disabled in config.");
    }
    if (config_.tracking.module.implementation == "ami") {
        tracker_implementation_ = TrackerImplementation::Ami;
    } else if (config_.tracking.module.implementation == "generalized") {
        tracker_implementation_ = TrackerImplementation::Generalized;
    } else {
        throw std::runtime_error("Supported tracker implementations are 'ami' and 'generalized'.");
    }
}

/**
 * @brief Create all subscriptions, publishers, and pipeline state objects.
 */
void TrackerNode::createInterfaces()
{
    if (config_.tracking.inputs.empty()) {
        throw std::runtime_error("No tracking input pipelines configured.");
    }

    pipelines_.clear();
    pipelines_.reserve(config_.tracking.inputs.size());

    for (const TrackerInputConfig& input_config : config_.tracking.inputs) {
        auto pipeline = std::make_unique<InputPipeline>();
        pipeline->config = input_config;

        if (tracker_implementation_ == TrackerImplementation::Ami) {
            uvdar_core::tracking::ami::ParamsAMI params
                = uvdar_core::tracking::ami::ParamsAMI::create(config_.tracking.sequence_file, config_.tracking.debug, false);
            params.allowed_BER_per_seq = config_.tracking.allowed_BER_per_seq;
            params.stored_seq_len_factor = config_.tracking.stored_seq_len_factor;
            params.poly_order = config_.tracking.poly_order;
            params.max_px_shift_x = static_cast<double>(config_.tracking.max_px_shift_x);
            params.max_px_shift_y = static_cast<double>(config_.tracking.max_px_shift_y);
            params.max_zeros_consecutive = config_.tracking.max_zeros_consecutive;
            params.max_buffer_length = config_.tracking.max_buffer_length;
            params.decay_factor = config_.tracking.decay_factor;
            params.conf_probab_percent = config_.tracking.conf_probab_percent;
            pipeline->ami_processor = std::make_unique<uvdar_core::tracking::ami::BlinkProcessor>(params, config_.tracking.sequences);
        } else {
            uvdar_core::tracking::generalized::ParamsGeneralized params = uvdar_core::tracking::generalized::ParamsGeneralized::create(config_.tracking.debug);
            params.allowed_BER_per_seq = config_.tracking.allowed_BER_per_seq;
            params.stored_seq_len_factor = config_.tracking.stored_seq_len_factor;
            params.model_order = config_.tracking.poly_order;
            params.max_px_shift_x = static_cast<double>(config_.tracking.max_px_shift_x);
            params.max_px_shift_y = static_cast<double>(config_.tracking.max_px_shift_y);
            params.max_zeros_consecutive = config_.tracking.max_zeros_consecutive;
            params.max_buffer_length = config_.tracking.max_buffer_length;
            params.decay_factor = config_.tracking.decay_factor;
            params.conf_probab_percent = config_.tracking.conf_probab_percent;
            params.association_gate_sigma = config_.tracking.association_gate_sigma;
            params.default_measurement_variance = config_.tracking.default_measurement_variance;
            params.process_noise_variance = config_.tracking.process_noise_variance;
            pipeline->generalized_processor = std::make_unique<uvdar_core::tracking::generalized::BlinkProcessor>(params, config_.tracking.sequences);
        }
        pipeline->tracker_initialized = true;

        pipeline->output_publisher = create_publisher<uvdar_core::msg::TrackerOutput>(
            input_config.output_topic,
            rclcpp::QoS(rclcpp::KeepLast(config_.tracking.queue_depth)).best_effort());

        if (input_config.publish_visualization && !input_config.visualization_topic.empty()) {
            pipeline->visualization_publisher = create_publisher<sensor_msgs::msg::Image>(
                input_config.visualization_topic,
                rclcpp::QoS(rclcpp::KeepLast(config_.tracking.queue_depth)).best_effort());
            pipeline->visualization_worker = std::make_unique<uvdar_core::app::visualization::VisualizationWorker>();
        }

        const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(config_.tracking.queue_depth)).best_effort();
        pipeline->input_subscription = create_subscription<uvdar_core::msg::ImagePointsWithCovariancesStamped>(
            input_config.input_topic,
            image_qos,
            [this, image_index = pipelines_.size()](const uvdar_core::msg::ImagePointsWithCovariancesStamped::ConstSharedPtr& msg) {
                onImagePoints(msg, image_index);
            });

        if (!input_config.input_image_topic.empty()) {
            pipeline->image_subscription = create_subscription<sensor_msgs::msg::Image>(
                input_config.input_image_topic,
                image_qos,
                [this, image_index = pipelines_.size()](const sensor_msgs::msg::Image::ConstSharedPtr& image_msg) {
                    onImage(image_msg, image_index);
                });
        }

        pipelines_.push_back(std::move(pipeline));
    }

    if (pipelines_.empty()) {
        throw std::runtime_error("No tracking pipelines configured.");
    }
}

/**
 * @brief Queue incoming image point cloud for asynchronous processing.
 */
void TrackerNode::onImagePoints(const uvdar_core::msg::ImagePointsWithCovariancesStamped::ConstSharedPtr& msg, std::size_t image_index)
{
    if ((now() - startup_time_).seconds() < config_.tracking.initial_delay_sec) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Tracker startup delay active.");
        return;
    }

    thread_pool_->enqueue([this, msg, image_index]() {
        processImagePoints(msg, image_index);
    });
}

/**
 * @brief Store latest raw image to render visualization on top.
 */
void TrackerNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, std::size_t image_index)
{
    if (image_index >= pipelines_.size()) {
        return;
    }

    cv_bridge::CvImagePtr cv_image;
    try {
        cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception& exc) {
        RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", exc.what());
        return;
    }

    auto& pipeline = pipelines_[image_index];
    {
        std::scoped_lock lock(pipeline->mutex);
        pipeline->latest_image = cv_image->image.clone();
        pipeline->image_received = true;
    }
}

/**
 * @brief Run selected tracker pipeline and publish shared output.
 */
void TrackerNode::processImagePoints(const uvdar_core::msg::ImagePointsWithCovariancesStamped::ConstSharedPtr& image_msg, std::size_t image_index)
{
    if (image_index >= pipelines_.size() || !image_msg) {
        return;
    }

    auto& pipeline = pipelines_[image_index];
    if (!pipeline->tracker_initialized) {
        return;
    }

    uvdar_core::msg::TrackerOutput output;
    output.stamp = image_msg->stamp;
    output.image_width = image_msg->image_width;
    output.image_height = image_msg->image_height;

    if (tracker_implementation_ == TrackerImplementation::Ami) {
        if (!pipeline->ami_processor) {
            return;
        }

        uvdar_core::tracking::ImagePointsWithCovariancesStamped input_points;
        input_points.stamp = uvdar_core::helpers::toSeconds(image_msg->stamp);
        input_points.img_width = static_cast<uint16_t>(image_msg->image_width);
        input_points.img_height = static_cast<uint16_t>(image_msg->image_height);
        input_points.points.reserve(image_msg->points.size());

        for (const auto& point : image_msg->points) {
            uvdar_core::tracking::ImagePoint tracker_point;
            tracker_point.x = point.x;
            tracker_point.y = point.y;
            tracker_point.covariance.c00 = point.covariance_00;
            tracker_point.covariance.c01 = point.covariance_01;
            tracker_point.covariance.c10 = point.covariance_10;
            tracker_point.covariance.c11 = point.covariance_11;
            input_points.points.push_back(tracker_point);
        }

        std::vector<uvdar_core::tracking::TrackResult> detected;
        {
            std::scoped_lock lock(pipeline->mutex);
            detected = pipeline->ami_processor->processFrame(std::make_shared<const uvdar_core::tracking::ami::ImagePointsWithCovariancesStamped>(input_points));
        }

        if (detected.size() > config_.tracking.max_points_per_image) {
            RCLCPP_WARN(get_logger(), "Tracker output has %zu points > max_points_per_image.", detected.size());
            return;
        }

        output.blinkers.reserve(detected.size());
        for (const auto& detected_track : detected) {
            uvdar_core::msg::TrackedBlinker tracker_point;
            const auto& point_state = detected_track.state;
            tracker_point.x = point_state.position.x();
            tracker_point.y = point_state.position.y();
            tracker_point.id = publishedSignalId(detected_track.id, config_.tracking.sequences.size());
            tracker_point.track_id = detected_track.track_id;
            tracker_point.stamp = image_msg->stamp;
            tracker_point.covariance_00 = point_state.covariance.c00;
            tracker_point.covariance_01 = point_state.covariance.c01;
            tracker_point.covariance_10 = point_state.covariance.c10;
            tracker_point.covariance_11 = point_state.covariance.c11;
            tracker_point.measurement_covariance_00 = point_state.measurement_covariance.c00;
            tracker_point.measurement_covariance_01 = point_state.measurement_covariance.c01;
            tracker_point.measurement_covariance_10 = point_state.measurement_covariance.c10;
            tracker_point.measurement_covariance_11 = point_state.measurement_covariance.c11;
            tracker_point.predicted_x = point_state.position.x();
            tracker_point.predicted_y = point_state.position.y();
            tracker_point.prediction_covariance_00 = point_state.prediction_covariance.c00;
            tracker_point.prediction_covariance_01 = point_state.prediction_covariance.c01;
            tracker_point.prediction_covariance_10 = point_state.prediction_covariance.c10;
            tracker_point.prediction_covariance_11 = point_state.prediction_covariance.c11;
            tracker_point.confidence_x = point_state.x_statistics.confidence_interval;
            tracker_point.confidence_y = point_state.y_statistics.confidence_interval;
            tracker_point.prediction_reference_time = image_msg->stamp;
            tracker_point.poly_reg_computed = false;
            tracker_point.extended_search = false;
            tracker_point.virtual_point = false;
            tracker_point.associated_with_detection = true;

            for (const double coeff : point_state.x_statistics.coeff) {
                tracker_point.x_coeff.push_back(coeff);
            }
            for (const double coeff : point_state.y_statistics.coeff) {
                tracker_point.y_coeff.push_back(coeff);
            }
            output.blinkers.push_back(std::move(tracker_point));
        }
    } else {
        if (!pipeline->generalized_processor) {
            return;
        }

        uvdar_core::tracking::generalized::ImagePointsWithCovariancesStamped input_points;
        input_points.stamp = uvdar_core::helpers::toSeconds(image_msg->stamp);
        input_points.img_width = static_cast<uint16_t>(image_msg->image_width);
        input_points.img_height = static_cast<uint16_t>(image_msg->image_height);
        input_points.points.reserve(image_msg->points.size());

        for (const auto& point : image_msg->points) {
            uvdar_core::tracking::generalized::ImagePoint tracker_point;
            tracker_point.x = point.x;
            tracker_point.y = point.y;
            tracker_point.covariance.c00 = point.covariance_00;
            tracker_point.covariance.c01 = point.covariance_01;
            tracker_point.covariance.c10 = point.covariance_10;
            tracker_point.covariance.c11 = point.covariance_11;
            input_points.points.push_back(tracker_point);
        }

        std::vector<uvdar_core::tracking::generalized::TrackResult> detected;
        {
            std::scoped_lock lock(pipeline->mutex);
            detected = pipeline->generalized_processor->processFrame(
                std::make_shared<const uvdar_core::tracking::generalized::ImagePointsWithCovariancesStamped>(input_points));
        }

        if (detected.size() > config_.tracking.max_points_per_image) {
            RCLCPP_WARN(get_logger(), "Tracker output has %zu points > max_points_per_image.", detected.size());
            return;
        }

        output.blinkers.reserve(detected.size());
        for (const auto& detected_track : detected) {
            uvdar_core::msg::TrackedBlinker tracker_point;
            const auto& point_state = detected_track.state;
            tracker_point.x = point_state.position.x();
            tracker_point.y = point_state.position.y();
            tracker_point.id = publishedSignalId(detected_track.id, config_.tracking.sequences.size());
            tracker_point.track_id = detected_track.track_id;
            tracker_point.stamp = image_msg->stamp;
            tracker_point.covariance_00 = point_state.covariance.c00;
            tracker_point.covariance_01 = point_state.covariance.c01;
            tracker_point.covariance_10 = point_state.covariance.c10;
            tracker_point.covariance_11 = point_state.covariance.c11;
            tracker_point.measurement_covariance_00 = point_state.measurement_covariance.c00;
            tracker_point.measurement_covariance_01 = point_state.measurement_covariance.c01;
            tracker_point.measurement_covariance_10 = point_state.measurement_covariance.c10;
            tracker_point.measurement_covariance_11 = point_state.measurement_covariance.c11;
            tracker_point.predicted_x = point_state.predicted_position.x();
            tracker_point.predicted_y = point_state.predicted_position.y();
            tracker_point.prediction_covariance_00 = point_state.prediction_covariance.c00;
            tracker_point.prediction_covariance_01 = point_state.prediction_covariance.c01;
            tracker_point.prediction_covariance_10 = point_state.prediction_covariance.c10;
            tracker_point.prediction_covariance_11 = point_state.prediction_covariance.c11;
            tracker_point.confidence_x = point_state.x_statistics.confidence_interval;
            tracker_point.confidence_y = point_state.y_statistics.confidence_interval;
            tracker_point.prediction_reference_time = uvdar_core::helpers::toRosTime(point_state.x_statistics.reference_time);
            tracker_point.poly_reg_computed = point_state.x_statistics.model_reg_computed || point_state.y_statistics.model_reg_computed;
            tracker_point.extended_search = point_state.x_statistics.extended_search || point_state.y_statistics.extended_search;
            tracker_point.virtual_point = point_state.virtual_point;
            tracker_point.associated_with_detection = point_state.associated_with_detection;

            for (const double coeff : point_state.x_statistics.coeff) {
                tracker_point.x_coeff.push_back(coeff);
            }
            for (const double coeff : point_state.y_statistics.coeff) {
                tracker_point.y_coeff.push_back(coeff);
            }
            output.blinkers.push_back(std::move(tracker_point));
        }
    }

    publishOutput(*pipeline, output);
}

/**
 * @brief Publish tracker output and optional visualization image.
 */
void TrackerNode::publishOutput(InputPipeline& pipeline, const uvdar_core::msg::TrackerOutput& output)
{
    {
        std::scoped_lock lock(pipeline.mutex);
        pipeline.latest_output = output;
    }
    pipeline.output_publisher->publish(output);

    if (pipeline.config.publish_visualization) {
        publishVisualization(pipeline, output);
    }
}

/**
 * @brief Snapshot tracker output and render it on the dedicated visualization worker.
 */
void TrackerNode::publishVisualization(InputPipeline& pipeline, const uvdar_core::msg::TrackerOutput& output)
{
    const int width = static_cast<int>(output.image_width);
    const int height = static_cast<int>(output.image_height);
    if (width <= 0 || height <= 0) {
        return;
    }

    cv::Mat frame;
    {
        std::scoped_lock lock(pipeline.mutex);
        if (pipeline.image_received && !pipeline.latest_image.empty()) {
            frame = pipeline.latest_image.clone();
        }
    }
    std::vector<uvdar_core::app::visualization::TrackingOverlayMarker> markers;
    markers.reserve(output.blinkers.size());
    for (const auto& point : output.blinkers) {
        markers.push_back({
            {point.x, point.y},
            point.id,
            point.poly_reg_computed,
            {point.predicted_x, point.predicted_y},
            {point.confidence_x, point.confidence_y},
            point.virtual_point,
        });
    }
    if (!pipeline.visualization_worker || !pipeline.visualization_publisher) {
        return;
    }
    std_msgs::msg::Header header;
    header.stamp = output.stamp;
    const auto publisher = pipeline.visualization_publisher;
    const auto logger = get_logger();
    pipeline.visualization_worker->submit([publisher, header, frame = std::move(frame), markers = std::move(markers), width, height, logger] {
        try {
            const cv::Mat visualization = uvdar_core::app::visualization::renderTrackingOverlay(
                frame,
                {width, height},
                markers);
            if (!visualization.empty()) {
                publisher->publish(*cv_bridge::CvImage(header, "bgr8", visualization).toImageMsg());
            }
        } catch (const std::exception& ex) {
            RCLCPP_WARN(logger, "Tracker visualization failed: %s", ex.what());
        }
    });
}

} // namespace uvdar_core::app
