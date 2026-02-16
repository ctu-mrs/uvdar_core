#include <uvdar_ros/blink_processor_ros.h>

namespace uvdar::blink_processor {

/* BlinkProcessorComponent //{ */
BlinkProcessorComponent::BlinkProcessorComponent(rclcpp::NodeOptions options)
    : mrs_lib::Node("BlinkProcessor", options) {
  node_ = this_node_ptr();

  logger_ = std::make_shared<RosLogger>(node_->get_logger());
}
//}

} // namespace uvdar::blink_processor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uvdar::blink_processor::BlinkProcessorComponent)
