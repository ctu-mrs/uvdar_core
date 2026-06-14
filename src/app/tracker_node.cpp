#include "uvdar_core/app/tracker_node.hpp"

#include <cmath>
#include <sensor_msgs/image_encodings.hpp>

namespace uvdar_core::app {

namespace {

constexpr int kMinImageRows = 100;
constexpr int kMinImageCols = 100;
constexpr int kLineThickness = 1;

} // namespace

TrackerNode::TrackerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("tracker", options)
{
    loadConfig();
    startup_time_ = now();
    thread_pool_ = std::make_unique<uvdar_core::utils::ThreadPool>(config_.tracking.thread_pool_size);
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
    if (config_.tracking.module.implementation != "ami") {
        throw std::runtime_error("Only ami tracker implementation is supported in this build.");
    }
}

/**
 * @brief Convert builtin time to seconds.
 */
double TrackerNode::toSeconds(const builtin_interfaces::msg::Time& stamp)
{
    return rclcpp::Time(stamp).seconds();
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

        uvdar_core::tracking::ami::ParamsAMI params
            = uvdar_core::tracking::ami::ParamsAMI::create(config_.tracking.sequence_file, config_.tracking.debug, false);
        params.allowed_BER_per_seq = config_.tracking.allowed_BER_per_seq;
        params.stored_seq_len_factor = config_.tracking.stored_seq_len_factor;
        params.poly_order = config_.tracking.poly_order;
        params.max_px_shift = uvdar_core::tracking::ami::Point2D(config_.tracking.max_px_shift_x, config_.tracking.max_px_shift_y);
        params.max_zeros_consecutive = config_.tracking.max_zeros_consecutive;
        params.max_buffer_length = config_.tracking.max_buffer_length;
        params.decay_factor = config_.tracking.decay_factor;
        params.conf_probab_percent = config_.tracking.conf_probab_percent;
        pipeline->blink_processor = std::make_unique<uvdar_core::tracking::ami::BlinkProcessor>(params, config_.tracking.sequences);
        pipeline->tracker_initialized = true;

        pipeline->output_publisher = create_publisher<uvdar_core::msg::TrackerOutput>(
            input_config.output_topic,
            rclcpp::QoS(rclcpp::KeepLast(config_.tracking.queue_depth)).best_effort());

        if (input_config.publish_visualization && !input_config.visualization_topic.empty()) {
            pipeline->visualization_publisher = create_publisher<sensor_msgs::msg::Image>(
                input_config.visualization_topic,
                rclcpp::QoS(rclcpp::KeepLast(config_.tracking.queue_depth)).best_effort());
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
 * @brief Run AMI tracker pipeline and publish output.
 */
void TrackerNode::processImagePoints(const uvdar_core::msg::ImagePointsWithCovariancesStamped::ConstSharedPtr& image_msg, std::size_t image_index)
{
    if (image_index >= pipelines_.size() || !image_msg) {
        return;
    }

    auto& pipeline = pipelines_[image_index];
    if (!pipeline->tracker_initialized || !pipeline->blink_processor) {
        return;
    }

    uvdar_core::tracking::ami::ImagePointsWithCovariancesStamped input_points;
    input_points.stamp = toSeconds(image_msg->stamp);
    input_points.img_width = static_cast<uint16_t>(image_msg->image_width);
    input_points.img_height = static_cast<uint16_t>(image_msg->image_height);
    input_points.points.reserve(image_msg->points.size());

    for (const auto& point : image_msg->points) {
        input_points.points.emplace_back(
            static_cast<int>(std::llround(point.x)),
            static_cast<int>(std::llround(point.y)));
    }

    std::vector<std::pair<uvdar_core::tracking::ami::PointState, int>> detected;
    {
        std::scoped_lock lock(pipeline->mutex);
        detected = pipeline->blink_processor->processFrame(std::make_shared<const uvdar_core::tracking::ami::ImagePointsWithCovariancesStamped>(input_points));
    }

    if (detected.size() > config_.tracking.max_points_per_image) {
        RCLCPP_WARN(get_logger(), "Tracker output has %zu points > max_points_per_image.", detected.size());
        return;
    }

    uvdar_core::msg::TrackerOutput output;
    output.stamp = image_msg->stamp;
    output.image_width = image_msg->image_width;
    output.image_height = image_msg->image_height;
    output.blinkers.reserve(detected.size());

    for (const auto& detected_point : detected) {
        uvdar_core::msg::TrackedBlinker tracker_point;
        const auto& point_state = detected_point.first;
        tracker_point.x = point_state.px_cord.x;
        tracker_point.y = point_state.px_cord.y;
        tracker_point.id = detected_point.second;
        tracker_point.stamp = image_msg->stamp;
        tracker_point.predicted_x = point_state.x_statistics.predicted_coordinate;
        tracker_point.predicted_y = point_state.y_statistics.predicted_coordinate;
        tracker_point.confidence_x = point_state.x_statistics.confidence_interval;
        tracker_point.confidence_y = point_state.y_statistics.confidence_interval;
        tracker_point.poly_reg_computed = point_state.x_statistics.poly_reg_computed || point_state.y_statistics.poly_reg_computed;
        tracker_point.extended_search = point_state.x_statistics.extended_search || point_state.y_statistics.extended_search;
        tracker_point.virtual_point = !point_state.led_state;

        for (const double coeff : point_state.x_statistics.coeff) {
            tracker_point.x_coeff.push_back(coeff);
        }
        for (const double coeff : point_state.y_statistics.coeff) {
            tracker_point.y_coeff.push_back(coeff);
        }
        output.blinkers.push_back(std::move(tracker_point));
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
 * @brief Convert output into BGR image overlay and publish it.
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
    if (frame.empty()) {
        frame = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
    }
    if (frame.rows < kMinImageRows) {
        cv::resize(frame, frame, cv::Size(std::max(frame.cols, kMinImageCols), std::max(frame.rows, kMinImageRows)));
    }

    if (frame.channels() == 1) {
        cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
    }

    for (const auto& point : output.blinkers) {
        const cv::Point2i center_int(std::lround(point.x), std::lround(point.y));
        const cv::Scalar color = idColor(point.id);

        cv::circle(frame, center_int, 4, color, 2);
        cv::putText(frame, std::to_string(point.id), center_int + cv::Point(-6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.4, color, kLineThickness, cv::LINE_AA);

        if (point.poly_reg_computed) {
            const cv::Point2i predicted(std::lround(point.predicted_x), std::lround(point.predicted_y));
            cv::circle(frame, predicted, 3, cv::Scalar(255, 255, 0), 2);

            if (point.confidence_x >= 0.0 && point.confidence_y >= 0.0) {
                const int half_width = static_cast<int>(std::ceil(point.confidence_x));
                const int half_height = static_cast<int>(std::ceil(point.confidence_y));
                cv::rectangle(
                    frame,
                    predicted - cv::Point(half_width, half_height),
                    predicted + cv::Point(half_width, half_height),
                    cv::Scalar(255, 255, 0),
                    kLineThickness);
            }
        }

        if (point.virtual_point) {
            cv::circle(frame, center_int, 2, cv::Scalar(80, 80, 80), 2);
        }
    }

    if (!pipeline.config.visualization_topic.empty() && pipeline.visualization_publisher) {
        std_msgs::msg::Header header;
        header.stamp = output.stamp;
        auto image_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        pipeline.visualization_publisher->publish(*image_msg);
    }
}

/**
 * @brief Deterministic HSV-like color map per blinker id.
 */
cv::Scalar TrackerNode::idColor(int id)
{
    if (id < 0) {
        return cv::Scalar(160, 160, 160);
    }
    const int hue = (id * 37 + 17) % 255;
    return cv::Scalar(30 + hue % 226, 40 + hue / 2 % 215, 220 - hue / 3);
}

} // namespace uvdar_core::app
