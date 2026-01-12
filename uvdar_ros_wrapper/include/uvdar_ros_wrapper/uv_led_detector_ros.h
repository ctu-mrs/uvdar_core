#pragma once

#include <rclcpp/rclcpp.hpp>
#include <uvdar_core/uv_led_detector/uv_led_detector.h>
#include <uvdar_ros_wrapper/utils/ros_logger.h>

namespace uvdar {

using namespace std::literals::chrono_literals;

class UvLedDetectorComponent : public rclcpp::Node {
 public:
  UvLedDetectorComponent(rclcpp::NodeOptions options);

 private:
  void Initialize_();

 private:
  std::shared_ptr<RosLogger> logger_;

  rclcpp::TimerBase::SharedPtr timer_init_;

  std::unique_ptr<UvLedDetector> uv_detector_;
};

} // namespace uvdar