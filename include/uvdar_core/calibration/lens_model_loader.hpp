#pragma once

#include <filesystem>

#include <yaml-cpp/yaml.h>

#include "uvdar_core/calibration/i_lens_model.hpp"

namespace uvdar_core::calibration {

/**
 * @brief Construct a lens model from an input-camera YAML node.
 *
 * The camera can either reference a YAML calibration file or specify model
 * parameters inline. Relative calibration paths are resolved against the
 * configuration file containing @p input_node.
 */
LensModelPtr loadLensModel(const YAML::Node& input_node, const std::filesystem::path& config_path);

} // namespace uvdar_core::calibration
