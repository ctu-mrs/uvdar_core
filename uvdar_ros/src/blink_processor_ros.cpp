#include <uvdar_ros/blink_processor_ros.h>

namespace uvdar::blink_processor {

/* BlinkProcessorComponent //{ */
BlinkProcessorComponent::BlinkProcessorComponent(rclcpp::NodeOptions options)
    : mrs_lib::Node("BlinkProcessor", options) {
  node_ = this_node_ptr();

  logger_ = std::make_shared<RosLogger>(node_->get_logger());

  loadParams_();
}
//}

/* loadParams_ //{ */
void BlinkProcessorComponent::loadParams_() {
  param_loader_ = std::make_shared<mrs_lib::ParamLoader>(node_, node_->get_name());

  std::vector<std::string> config_files;
  param_loader_->loadParam("config_files", config_files);
  for (auto config_file : config_files) {
    RCLCPP_INFO(node_->get_logger(), "Loading config file '%s'", config_file.c_str());
    param_loader_->addYamlFile(config_file);
  }

  loadRosParams_();
  loadBlinkProcessorParams_();

  if (!param_loader_->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Some compulsory parameters were not loaded successfully!");
    rclcpp::shutdown();
  }
}
//}

/* loadRosParams_ //{ */
void BlinkProcessorComponent::loadRosParams_() {
  param_loader_->loadParam("uv_led_detector/detected_points_topics", detected_raw_points_topics_);
  param_loader_->loadParam("blink_processor/detected_markers_topics", detected_markers_topics_);
}
//}

/* loadBlinkProcessorParams_ //{ */
void BlinkProcessorComponent::loadBlinkProcessorParams_() {
}
//}

} // namespace uvdar::blink_processor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uvdar::blink_processor::BlinkProcessorComponent)
