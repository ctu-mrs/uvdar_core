#pragma once

#include <string>

#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>

#include "uvdar_core/helpers/yaml.hpp"

namespace uvdar_core::app {

/**
 * @brief Log every parameter set in one section of the YAML configuration.
 *
 * Emits one "[config] dotted.key: value" line per parameter so a recorded
 * terminal log identifies the exact configuration of a run and stays easy to
 * grep. Values are printed after environment-variable expansion.
 */
inline void logConfigSection(
    const rclcpp::Logger& logger,
    const std::string& config_path,
    const YAML::Node& root,
    const std::string& section)
{
    RCLCPP_INFO(logger, "[config] file: %s", config_path.c_str());
    for (const std::string& line : uvdar_core::helpers::yaml::flatten(root[section], section)) {
        RCLCPP_INFO(logger, "[config] %s", line.c_str());
    }
}

/**
 * @brief Load @p config_path and log one of its sections.
 */
inline void logConfigSection(
    const rclcpp::Logger& logger,
    const std::string& config_path,
    const std::string& section)
{
    logConfigSection(logger, config_path, uvdar_core::helpers::yaml::loadFile(config_path), section);
}

} // namespace uvdar_core::app
