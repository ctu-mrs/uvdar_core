#pragma once

#include <rclcpp/rclcpp.hpp>

#include <uvdar/blink_processor/ami_tracker.h>
#include <uvdar_ros/utils/ros_logger.h>

#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>

namespace uvdar::ami {

class AmiTrackerComponent : public mrs_lib::Node {
 public:
  AmiTrackerComponent(rclcpp::NodeOptions options);

 private:
 private:
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<RosLogger> logger_;
  std::shared_ptr<mrs_lib::ParamLoader> param_loader_;
};

} // namespace uvdar::ami