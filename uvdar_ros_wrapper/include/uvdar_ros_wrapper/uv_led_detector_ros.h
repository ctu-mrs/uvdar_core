#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "image_transport/image_transport.hpp"

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>

#include <uvdar_core/uv_led_detector/uv_led_detector.h>
#include <uvdar_ros_wrapper/utils/ros_logger.h>

namespace uvdar {

using namespace std::literals::chrono_literals;
// using image_callback_t = std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr&)>;

struct CameraContext {
  std::string topic;
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Image> sub;
  rclcpp::TimerBase::SharedPtr timer;

  cv::Size image_size{0, 0};
  cv::Mat current_image;
  std::vector<cv::Point> detected_points;
  std::vector<cv::Point> sun_points;
};

class UvLedDetectorComponent : public rclcpp::Node {
 public:
  UvLedDetectorComponent(rclcpp::NodeOptions options);

 private:
  void initialize_();
  void initDetector_();
  void initRosInterface_();

  void loadParams_();
  void loadRosParams_();
  void loadUvLedDetectParams_();

  void callbackImage_(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, int image_index);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_init_;
  rclcpp::CallbackGroup::SharedPtr image_callback_group_{nullptr};

  std::shared_ptr<RosLogger> logger_;
  std::shared_ptr<mrs_lib::ParamLoader> param_loader_;

  bool initialized_;
  std::string uav_name_;
  double initial_delay_;
  bool publish_visualization_flag_;

  size_t camera_count_;
  std::vector<std::string> camera_topics_;

  UvLedDetectConfig detect_cfg_;
  std::unique_ptr<UvLedDetector> uv_detector_;
  std::vector<CameraContext> cameras_;
};

} // namespace uvdar