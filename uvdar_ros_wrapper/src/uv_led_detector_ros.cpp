#include <uvdar_ros_wrapper/uv_led_detector_ros.h>

namespace uvdar {

/* UvLedDetectorComponent constructor //{ */
UvLedDetectorComponent::UvLedDetectorComponent(rclcpp::NodeOptions options) : Node("PCLFiltration", options) {
  timer_init_ = this->create_wall_timer(std::chrono::duration<double>(0.1s),
                                        std::bind(&UvLedDetectorComponent::initialize_, this));
}
//}

/* initialize_ //{ */
void UvLedDetectorComponent::initialize_() {
  node_   = this->shared_from_this();
  logger_ = std::make_shared<RosLogger>(node_->get_logger());

  loadParams_();
  uv_detector_ = std::make_unique<UvLedDetector>(*logger_, detect_cfg_);

  timer_init_->cancel();
}
//}

/* loadParams_ //{ */
void UvLedDetectorComponent::loadParams_() {
  param_loader_ = std::make_shared<mrs_lib::ParamLoader>(node_, node_->get_name());

  std::vector<std::string> config_files;
  param_loader_->loadParam("config_files", config_files);

  for (auto config_file : config_files) {
    RCLCPP_INFO(node_->get_logger(), "loading config file '%s'", config_file.c_str());
    param_loader_->addYamlFile(config_file);
  }

  loadRosParams_();
  loadUvLedDetectParams_();

  if (!param_loader_->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[UVDARDetector]: Some compulsory parameters were not loaded successfully, ending the node!");
    rclcpp::shutdown();
  }
}
//}

/* loadRosParams_ //{ */
void UvLedDetectorComponent::loadRosParams_() {
  param_loader_->loadParam("uav_name", uav_name_, std::string("uav1"));
  param_loader_->loadParam("publish_visualization", publish_visualization_flag_, false);
  param_loader_->loadParam("initial_delay", initial_delay_, 5.0);
}
//}

/* loadUvLedDetectParams_ //{ */
void UvLedDetectorComponent::loadUvLedDetectParams_() {
  param_loader_->loadParam("uv_led_detector/use_gpu", detect_cfg_.gpu, false);
  param_loader_->loadParam("uv_led_detector/gui", detect_cfg_.gui, false);
  param_loader_->loadParam("uv_led_detector/threshold", detect_cfg_.threshold, 200);
  param_loader_->loadParam("uv_led_detector/threshold_diff", detect_cfg_.threshold_diff, 100);
  param_loader_->loadParam("uv_led_detector/threshold_sun", detect_cfg_.threshold_sun, 150);
  param_loader_->loadParam("uv_led_detector/threshold_sun_dist", detect_cfg_.threshold_sun_dist, 25);
  param_loader_->loadParam("uv_led_detector/threshold_sun_merge", detect_cfg_.threshold_sun_merge, 20);
}
//}

} // namespace uvdar

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uvdar::UvLedDetectorComponent)