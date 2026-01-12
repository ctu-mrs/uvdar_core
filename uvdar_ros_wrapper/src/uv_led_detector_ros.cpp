#include <uvdar_ros_wrapper/uv_led_detector_ros.h>

namespace uvdar {

UvLedDetectorComponent::UvLedDetectorComponent(rclcpp::NodeOptions options) : Node("PCLFiltration", options) {
  timer_init_ = this->create_wall_timer(std::chrono::duration<double>(0.1s),
                                        std::bind(&UvLedDetectorComponent::Initialize_, this));
}

void UvLedDetectorComponent::Initialize_() {
  logger_ = std::make_shared<RosLogger>(this->get_logger());

  UvLedDetectConfig cfg;
  uv_detector_ = std::make_unique<UvLedDetector>(*logger_, cfg);
}

} // namespace uvdar

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uvdar::UvLedDetectorComponent)