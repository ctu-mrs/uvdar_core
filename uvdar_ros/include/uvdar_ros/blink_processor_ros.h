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
  void loadParams_();
  void loadRosParams_();
  void loadBlinkProcessorParams_();

  void checkLoadedParams_();
  [[nodiscard]] bool checkRosTopics_() const;
  [[nodiscard]] bool checkPatternsFile_();
  [[nodiscard]] bool checkBlinkProcessorConfig_() const;

  [[nodiscard]] bool loadPatternsFile_();
  [[nodiscard]] bool initBlinkProcessor_();

 private:
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<RosLogger> logger_;

  std::shared_ptr<mrs_lib::ParamLoader> param_loader_;
  BlinkProcessorConfig _cfg_;
  std::string _patterns_file_path_;
  std::vector<std::string> _detected_raw_points_topics_;
  std::vector<std::string> _detected_markers_topics_;

  std::unique_ptr<BlinkProcessor> blink_processor_;
  std::vector<Sequence> _blinking_patterns_;
};

} // namespace uvdar::blink_processor