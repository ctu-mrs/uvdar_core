#pragma once

#include <condition_variable>
#include <builtin_interfaces/msg/time.hpp>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/image.hpp>

#include "uvdar_core/msg/pose_with_covariance_array_stamped.hpp"
#include "uvdar_core/msg/tracker_output.hpp"
#include "uvdar_core/pose_estimation/i_pose_estimator.hpp"
#include "uvdar_core/pose_estimation/types.hpp"

namespace uvdar_core::app {

class PoseEstimatorNode final : public rclcpp::Node {
public:
    explicit PoseEstimatorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~PoseEstimatorNode() override;

private:
    struct InputConfig {
        std::string name;
        std::string input_topic;
        std::string camera_frame;
        std::string calib_file;
    };

    void loadConfiguration(const std::string& config_path);
    void onTrackerOutput(const uvdar_core::msg::TrackerOutput::ConstSharedPtr& msg, std::size_t camera_index);
    void onScatterTimer();
    void publishMeasurements(
        const uvdar_core::pose_estimation::TimedPoseMeasurements& measurements,
        const rclcpp::Publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>::SharedPtr& publisher);
    void publishVisualization(
        const uvdar_core::pose_estimation::TimedPoseMeasurements& measurements,
        const builtin_interfaces::msg::Time& stamp);
    void queueVisualization(
        const uvdar_core::pose_estimation::TimedPoseMeasurements& measurements,
        const builtin_interfaces::msg::Time& stamp);
    void startVisualizationThread();
    void stopVisualizationThread();
    void visualizationWorker();

    std::vector<InputConfig> inputs_;
    std::vector<rclcpp::Subscription<uvdar_core::msg::TrackerOutput>::SharedPtr> subscriptions_;
    rclcpp::Publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>::SharedPtr measured_publisher_;
    rclcpp::Publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>::SharedPtr hypotheses_publisher_;
    rclcpp::Publisher<uvdar_core::msg::PoseWithCovarianceArrayStamped>::SharedPtr tentative_hypotheses_publisher_;
    rclcpp::TimerBase::SharedPtr scatter_timer_;

    std::unique_ptr<uvdar_core::pose_estimation::IPoseEstimator> pose_estimator_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string output_frame_;
    double latest_primary_input_stamp_ = 0.0;
    bool publish_constituents_ = false;
    bool publish_visualization_ = false;
    std::string visualization_topic_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_publisher_;
    bool particle_filter_implementation_ = false;
    double visualization_period_sec_ = 0.2;
    bool visualization_thread_running_ = false;
    bool visualization_pending_ = false;
    std::mutex visualization_mutex_;
    std::condition_variable visualization_cv_;
    std::thread visualization_thread_;
    uvdar_core::pose_estimation::TimedPoseMeasurements pending_visualization_measurements_;
    builtin_interfaces::msg::Time pending_visualization_stamp_;
};

} // namespace uvdar_core::app
