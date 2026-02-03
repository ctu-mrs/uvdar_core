#pragma once

#include <mutex>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>

#include <uvdar/uv_led_detector/uv_led_detector.h>
#include <uvdar_ros/utils/ros_logger.h>
#include <uvdar_ros_msgs/msg/image_points_with_float_stamped.hpp>

namespace uvdar {

using namespace std::literals::chrono_literals;
using image_point_publisher_t = mrs_lib::PublisherHandler<uvdar_ros_msgs::msg::ImagePointsWithFloatStamped>;

/* CameraContext //{ */
struct CameraContext {
  std::string camera_topic;
  std::string detected_points_topic;

  mrs_lib::SubscriberHandler<sensor_msgs::msg::Image> sub;
  image_point_publisher_t pub_detected_points;
  image_point_publisher_t pub_sun_points;
  mrs_lib::PublisherHandler<sensor_msgs::msg::Image> pub_debug_dp_image;
  mrs_lib::PublisherHandler<sensor_msgs::msg::Image> pub_debug_sp_image;

  rclcpp::TimerBase::SharedPtr timer;
  sensor_msgs::msg::Image::ConstSharedPtr last_msg;

  cv::Size image_size{0, 0};
  cv::Mat current_image;
  std::unique_ptr<UvLedDetector> uv_detector;
  std::vector<cv::Point> detected_points;
  std::vector<cv::Point> sun_points;

  std::mutex mtx;
};
//}

/* UvLedDetectorComponent //{ */
class UvLedDetectorComponent : public rclcpp::Node {
 public:
  UvLedDetectorComponent(rclcpp::NodeOptions options);

 private:
  void initialize_();
  void initDetector_();

  void initRosInterface_();
  void initRosProcessImgSubs_();
  void initRosPublishers_();

  void loadParams_();
  void loadRosParams_();
  void loadUvLedDetectParams_();

  [[nodiscard]] bool areAllCamerasDetected_();
  [[nodiscard]] bool isReadyToProcess_(const int image_index);
  [[nodiscard]] bool hasInitialDelayElapsed_();
  void initGpuProgram_(const int image_index);

  void checkCameraInputTopics_();
  void checkDetectedPointsTopics_();

  void processImage_(const int image_index);

  void publishDetectedPoints_(const cv_bridge::CvImage& image, CameraContext& camera);
  void publishDetectedPointsImage_(const cv_bridge::CvImage& image, CameraContext& camera);
  void publishSunPoints_(const cv_bridge::CvImage& image, CameraContext& camera);
  void publishSunPointsImage_(const cv_bridge::CvImage& image, CameraContext& camera);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_init_;
  rclcpp::CallbackGroup::SharedPtr image_callback_group_{nullptr};
  rclcpp::CallbackGroup::SharedPtr processing_callback_group_{nullptr};

  std::shared_ptr<RosLogger> logger_;
  std::shared_ptr<mrs_lib::ParamLoader> param_loader_;

  bool initialized_{false};
  std::string uav_name_;
  double initial_delay_;
  bool initial_delay_started_flag_{false};
  std::atomic_bool initial_delay_done_flag_{false};
  rclcpp::Time initial_delay_start_;
  std::mutex initial_delay_mtx_;

  bool publish_sun_points_{false};
  size_t camera_count_{0};
  std::vector<std::string> camera_topics_;
  std::vector<std::string> detected_points_topics_;

  UvLedDetectConfig detect_cfg_;
  std::deque<CameraContext> cameras_;
  bool all_cameras_detected_{false};
};
//}

} // namespace uvdar