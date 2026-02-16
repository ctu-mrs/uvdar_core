#pragma once

#include <rclcpp/rclcpp.hpp>

#include <uvdar_ros/utils/ros_logger.h>
#include <uvdar/blink_processor/blink_processor.h>

#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>

namespace uvdar::blink_processor {

class BlinkProcessorComponent : public mrs_lib::Node {
 public:
  BlinkProcessorComponent(rclcpp::NodeOptions options);

 private:
 private:
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<RosLogger> logger_;
  std::shared_ptr<mrs_lib::ParamLoader> param_loader_;
};

} // namespace uvdar::blink_processor