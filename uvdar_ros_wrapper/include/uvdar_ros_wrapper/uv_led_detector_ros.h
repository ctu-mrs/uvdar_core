#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/param_loader.h>

#include <uvdar_core/uv_led_detector/uv_led_detector.h>
#include <uvdar_ros_wrapper/utils/ros_logger.h>

namespace uvdar {

using namespace std::literals::chrono_literals;

class UvLedDetectorComponent : public rclcpp::Node {
 public:
  UvLedDetectorComponent(rclcpp::NodeOptions options);

 private:
  void initialize_();
  void loadParams_();
  void loadRosParams_();
  void loadUvLedDetectParams_();

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_init_;

  std::shared_ptr<RosLogger> logger_;
  std::shared_ptr<mrs_lib::ParamLoader> param_loader_;

  std::string uav_name_;
  double initial_delay_;
  bool publish_visualization_flag_;

  UvLedDetectConfig detect_cfg_;
  std::unique_ptr<UvLedDetector> uv_detector_;
};

} // namespace uvdar