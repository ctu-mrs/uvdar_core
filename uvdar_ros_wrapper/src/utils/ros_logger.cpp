#include <uvdar_ros_wrapper/utils/ros_logger.h>

/* RosLogger constructor //{ */
RosLogger::RosLogger(const rclcpp::Logger& logger) : m_logger(logger) {
}
//}

/* RosLogger::log //{ */
void RosLogger::log(const LogLevel level, const std::string& msg) {
  if (level == LogLevel::Info) {
    RCLCPP_INFO_STREAM(m_logger, msg);
  } else if (level == LogLevel::Debug) {
    RCLCPP_DEBUG_STREAM(m_logger, msg);
  } else if (level == LogLevel::Warn) {
    RCLCPP_WARN_STREAM(m_logger, msg);
  } else if (level == LogLevel::Error) {
    RCLCPP_ERROR_STREAM(m_logger, msg);
  } else {
    RCLCPP_ERROR_STREAM(m_logger, "Given LogLevel is not supported!");
  }
}
//}