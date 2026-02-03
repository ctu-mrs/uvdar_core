#pragma once

#include <rclcpp/rclcpp.hpp>

#include <uvdar/utils/i_logger.h>

class RosLogger : public ILogger {
 public:
  RosLogger(const rclcpp::Logger& logger);
  void log(const LogLevel level, const std::string& msg) override;

 private:
  rclcpp::Logger m_logger;
};