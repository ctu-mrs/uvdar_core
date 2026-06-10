#include "uvdar_core/app/package_config.hpp"

#include <filesystem>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace uvdar_core::app {

namespace {

    template <typename T>
    T requireScalar(const YAML::Node& node, const std::string& key)
    {
        const YAML::Node value = node[key];
        if (!value) {
            throw std::runtime_error("Missing required config key '" + key + "'.");
        }
        return value.as<T>();
    }

    YAML::Node requireNode(const YAML::Node& node, const std::string& key)
    {
        const YAML::Node value = node[key];
        if (!value) {
            throw std::runtime_error("Missing required config section '" + key + "'.");
        }
        return value;
    }

    std::string resolvePath(const std::filesystem::path& config_path, const std::string& value)
    {
        if (value.empty()) {
            return value;
        }

        const std::filesystem::path path_value(value);
        if (path_value.is_absolute()) {
            return path_value.string();
        }
        return (config_path.parent_path() / path_value).lexically_normal().string();
    }

    DetectorBackend parseBackend(const std::string& value)
    {
        if (value == "cpu") {
            return DetectorBackend::Cpu;
        }
        if (value == "gpu") {
            return DetectorBackend::Gpu;
        }
        throw std::runtime_error("Unsupported detector backend '" + value + "'.");
    }

    ModuleConfig parseModuleConfig(const YAML::Node& node)
    {
        return ModuleConfig {
            requireScalar<bool>(node, "enabled"),
            requireScalar<std::string>(node, "implementation"),
        };
    }

    std::vector<unsigned> parseRadii(const YAML::Node& node)
    {
        std::vector<unsigned> radii;
        for (const YAML::Node& radius_node : node) {
            const int radius = radius_node.as<int>();
            if (radius <= 0) {
                throw std::runtime_error("Configured detector radii must be positive.");
            }
            radii.push_back(static_cast<unsigned>(radius));
        }
        return radii;
    }

} // namespace

PackageConfig loadPackageConfig(const std::string& config_path_string)
{
    const std::filesystem::path config_path(config_path_string);
    if (config_path_string.empty()) {
        throw std::runtime_error("The 'config_path' parameter must point to a package default.yaml file.");
    }
    if (!std::filesystem::exists(config_path)) {
        throw std::runtime_error("The config file '" + config_path_string + "' does not exist.");
    }

    const YAML::Node root          = YAML::LoadFile(config_path_string);
    const YAML::Node detector_node = requireNode(root, "detector");
    const YAML::Node inputs_node   = requireNode(detector_node, "inputs");

    PackageConfig config;
    config.source_path                   = config_path_string;
    config.detector.debug                = requireScalar<bool>(detector_node, "debug");
    config.detector.gui                  = requireScalar<bool>(detector_node, "gui");
    config.detector.initial_delay_sec    = requireScalar<double>(detector_node, "initial_delay_sec");
    config.detector.queue_depth          = requireScalar<std::size_t>(detector_node, "queue_depth");
    config.detector.thread_pool_size     = requireScalar<std::size_t>(detector_node, "thread_pool_size");
    config.detector.max_points_per_image = requireScalar<std::size_t>(detector_node, "max_points_per_image");

    for (const YAML::Node& input_node : inputs_node) {
        config.detector.inputs.push_back(DetectorInputConfig {
            requireScalar<std::string>(input_node, "name"),
            requireScalar<bool>(input_node, "enabled"),
            requireScalar<std::string>(input_node, "input_topic"),
            requireScalar<std::string>(input_node, "output_topic"),
            requireScalar<bool>(input_node, "detect_sun_points"),
            requireScalar<bool>(input_node, "publish_sun_points"),
            requireScalar<std::string>(input_node, "sun_output_topic"),
            requireScalar<bool>(input_node, "publish_visualization"),
            requireScalar<std::string>(input_node, "visualization_topic"),
            resolvePath(config_path, requireScalar<std::string>(input_node, "mask_file")),
            parseBackend(requireScalar<std::string>(input_node, "backend")),
            requireScalar<int>(input_node, "threshold"),
            requireScalar<int>(input_node, "threshold_diff"),
            requireScalar<int>(input_node, "threshold_sun"),
            requireScalar<unsigned>(input_node, "min_sun_marker_distance"),
            requireScalar<unsigned>(input_node, "max_markers_count"),
            requireScalar<unsigned>(input_node, "max_sun_points_count"),
            parseRadii(requireNode(input_node, "radii")),
        });
    }

    if (config.detector.inputs.empty()) {
        throw std::runtime_error("The detector.inputs list must not be empty.");
    }

    config.tracking        = parseModuleConfig(requireNode(root, "tracking"));
    config.pose_estimation = parseModuleConfig(requireNode(root, "pose_estimation"));
    config.filtering       = parseModuleConfig(requireNode(root, "filtering"));
    config.calibration     = parseModuleConfig(requireNode(root, "calibration"));
    config.simulation      = parseModuleConfig(requireNode(root, "simulation"));

    return config;
}

} // namespace uvdar_core::app