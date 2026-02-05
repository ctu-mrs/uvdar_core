#include <uvdar_ros/ami_tracker_ros.h>

namespace uvdar::ami {

/* AmiTrackerComponent //{ */
AmiTrackerComponent::AmiTrackerComponent(rclcpp::NodeOptions options) : mrs_lib::Node("AmiTracker", options) {
  node_ = this_node_ptr();

  logger_ = std::make_shared<RosLogger>(node_->get_logger());
}
//}

} // namespace uvdar::ami