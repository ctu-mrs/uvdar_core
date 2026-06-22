#include "uvdar_core/app/package_config.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <sstream>
#include <string>

#include <yaml-cpp/yaml.h>

namespace uvdar_core::app {

namespace {

/**
 * @brief Read optional scalar from YAML; return a fallback value if missing.
 */
template <typename T>
T optionalScalar(const YAML::Node& node, const std::string& key, T default_value)
{
    const YAML::Node value = node[key];
    if (!value) {
        return default_value;
    }
    return value.as<T>();
}

/**
 * @brief Read required scalar from YAML; throw when missing.
 */
template <typename T>
T requireScalar(const YAML::Node& node, const std::string& key)
{
    const YAML::Node value = node[key];
    if (!value) {
        throw std::runtime_error("Missing required config key '" + key + "'.");
    }
    return value.as<T>();
}

/**
 * @brief Read required subsection from YAML; throw when missing.
 */
YAML::Node requireNode(const YAML::Node& node, const std::string& key)
{
    const YAML::Node value = node[key];
    if (!value) {
        throw std::runtime_error("Missing required config section '" + key + "'.");
    }
    return value;
}

/**
 * @brief Resolve sequence/config paths relative to the current config file.
 */
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

/**
 * @brief Parse detector backend string from YAML.
 */
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

/**
 * @brief Parse module configuration with backward-compatible support for optional
 *        legacy key 'enabled'. New configurations only require 'implementation'.
 */
ModuleConfig parseModuleConfig(const YAML::Node& node)
{
    return ModuleConfig {
        optionalScalar<bool>(node, "enabled", true),
        requireScalar<std::string>(node, "implementation"),
    };
}

/**
 * @brief Trim whitespace from both ends of a string.
 */
std::string trim(const std::string& value)
{
    const auto start = value.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        return {};
    }
    const auto end = value.find_last_not_of(" \t\r\n") + 1;
    return value.substr(start, end - start);
}

/**
 * @brief Convert supported sequence syntaxes into boolean vectors.
 */
std::vector<bool> parseSequenceString(const std::string& definition, bool manchester_code)
{
    const std::string trimmed_definition = trim(definition);
    if (trimmed_definition.empty()) {
        return {};
    }

    std::vector<bool> sequence;

    if (trimmed_definition.find(':') != std::string::npos) {
        const auto split = trimmed_definition.find(':');
        const std::string len_s = trim(trimmed_definition.substr(0, split));
        const std::string hex_s = trim(trimmed_definition.substr(split + 1));
        if (len_s.empty()) {
            return {};
        }

        const unsigned len = static_cast<unsigned>(std::stoul(len_s));
        std::string hex = hex_s;
        if (hex.size() >= 2 && hex[0] == '0' && (hex[1] == 'x' || hex[1] == 'X')) {
            hex = hex.substr(2);
        }

        const unsigned long long value = hex.empty() ? 0ULL : std::stoull(hex, nullptr, 16);
        sequence.reserve(len);
        for (unsigned i = 0; i < len; ++i) {
            sequence.push_back((value >> (len - 1 - i)) & 1ULL);
        }
    } else if (trimmed_definition.find(',') != std::string::npos) {
        std::stringstream ss(trimmed_definition);
        std::string token;
        while (std::getline(ss, token, ',')) {
            token = trim(token);
            if (token.empty()) {
                continue;
            }
            if (token != "0" && token != "1") {
                return {};
            }
            sequence.push_back(token == "1");
        }
    } else {
        for (const char symbol : trimmed_definition) {
            if (symbol == '0' || symbol == '1') {
                sequence.push_back(symbol == '1');
            } else if (!std::isspace(static_cast<unsigned char>(symbol))) {
                return {};
            }
        }
    }

    if (!manchester_code) {
        return sequence;
    }

    std::vector<bool> encoded;
    encoded.reserve(sequence.size() * 2);
    for (const bool bit : sequence) {
        if (bit) {
            encoded.push_back(false);
            encoded.push_back(true);
        } else {
            encoded.push_back(true);
            encoded.push_back(false);
        }
    }
    return encoded;
}

/**
 * @brief Parse sequences passed directly in YAML config.
 */
std::vector<std::vector<bool>> parseSequencesNode(const YAML::Node& node, bool manchester_code)
{
    std::vector<std::vector<bool>> sequences;
    if (!node) {
        return sequences;
    }

    if (node.IsScalar()) {
        const auto parsed = parseSequenceString(node.as<std::string>(), manchester_code);
        if (!parsed.empty()) {
            sequences.push_back(parsed);
        }
        return sequences;
    }

    if (!node.IsSequence()) {
        return sequences;
    }

    for (const YAML::Node& sequence_node : node) {
        if (!sequence_node) {
            continue;
        }

        if (sequence_node.IsSequence()) {
            std::vector<bool> sequence;
            for (const YAML::Node& bit_node : sequence_node) {
                const auto bit = bit_node.as<int>();
                sequence.push_back(bit != 0);
            }
            if (!sequence.empty()) {
                sequences.push_back(std::move(sequence));
            }
            continue;
        }

        const auto parsed = parseSequenceString(sequence_node.as<std::string>(), manchester_code);
        if (!parsed.empty()) {
            sequences.push_back(parsed);
        }
    }
    return sequences;
}

/**
 * @brief Parse a text file containing one sequence per line.
 */
std::vector<std::vector<bool>> parseSequenceFile(const std::filesystem::path& config_path, const std::string& filename, bool manchester_code)
{
    const std::filesystem::path path = resolvePath(config_path, filename);
    if (path.empty() || !std::filesystem::exists(path)) {
        return {};
    }

    std::ifstream file(path);
    if (!file.is_open()) {
        return {};
    }

    std::vector<std::vector<bool>> sequences;
    std::string line;
    while (std::getline(file, line)) {
        const std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }
        const auto parsed = parseSequenceString(trimmed, manchester_code);
        if (!parsed.empty()) {
            sequences.push_back(parsed);
        }
    }
    return sequences;
}

/**
 * @brief Ensure tracking templates match assumptions used by AMI matching.
 */
void validateTrackingSequences(const std::vector<std::vector<bool>>& sequences)
{
    if (sequences.empty()) {
        throw std::runtime_error("At least one tracking blinking sequence is required.");
    }

    const std::size_t sequence_size = sequences.front().size();
    if (sequence_size == 0U) {
        throw std::runtime_error("Tracking blinking sequence 0 is empty.");
    }

    for (std::size_t index = 0; index < sequences.size(); ++index) {
        if (sequences[index].empty()) {
            throw std::runtime_error("Tracking blinking sequence " + std::to_string(index) + " is empty.");
        }
        if (sequences[index].size() != sequence_size) {
            throw std::runtime_error("All tracking blinking sequences must have equal length.");
        }
    }
}

/**
 * @brief Parse and validate configured marker radii.
 */
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

/**
 * @brief Load and validate full uvdar-core YAML configuration.
 */
PackageConfig loadPackageConfig(const std::string& config_path_string)
{
    const std::filesystem::path config_path(config_path_string);
    if (config_path_string.empty()) {
        throw std::runtime_error("The 'config_path' parameter must point to a package default.yaml file.");
    }
    if (!std::filesystem::exists(config_path)) {
        throw std::runtime_error("The config file '" + config_path_string + "' does not exist.");
    }

    const YAML::Node root = YAML::LoadFile(config_path_string);
    const YAML::Node detector_node = requireNode(root, "detector");
    const YAML::Node inputs_node   = requireNode(detector_node, "inputs");
    const YAML::Node tracking_node = requireNode(root, "tracking");

    PackageConfig config;
    config.source_path = config_path_string;

    // detector module
    config.detector.debug = requireScalar<bool>(detector_node, "debug");
    config.detector.gui = requireScalar<bool>(detector_node, "gui");
    config.detector.initial_delay_sec = requireScalar<double>(detector_node, "initial_delay_sec");
    config.detector.queue_depth = requireScalar<std::size_t>(detector_node, "queue_depth");
    config.detector.thread_pool_size = requireScalar<std::size_t>(detector_node, "thread_pool_size");
    config.detector.max_points_per_image = requireScalar<std::size_t>(detector_node, "max_points_per_image");

    for (const YAML::Node& input_node : inputs_node) {
        config.detector.inputs.push_back(DetectorInputConfig {
            requireScalar<std::string>(input_node, "name"),
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

    // tracking module
    config.tracking.module = parseModuleConfig(tracking_node);
    config.tracking.debug = optionalScalar<bool>(tracking_node, "debug", false);
    config.tracking.gui = optionalScalar<bool>(tracking_node, "gui", false);
    config.tracking.initial_delay_sec = optionalScalar<double>(tracking_node, "initial_delay_sec", 0.0);
    config.tracking.queue_depth = optionalScalar<std::size_t>(tracking_node, "queue_depth", 10);
    config.tracking.thread_pool_size = optionalScalar<std::size_t>(tracking_node, "thread_pool_size", 2);
    config.tracking.max_points_per_image = optionalScalar<std::size_t>(tracking_node, "max_points_per_image", 100);
    const bool legacy_ami = config.tracking.module.implementation == "ami";
    config.tracking.max_px_shift_x = optionalScalar<int>(tracking_node, "max_px_shift_x", legacy_ami ? 2 : 5);
    config.tracking.max_px_shift_y = optionalScalar<int>(tracking_node, "max_px_shift_y", legacy_ami ? 2 : 5);
    config.tracking.max_zeros_consecutive = optionalScalar<int>(tracking_node, "max_zeros_consecutive", legacy_ami ? 10 : 3);
    config.tracking.stored_seq_len_factor = optionalScalar<int>(tracking_node, "stored_seq_len_factor", legacy_ami ? 20 : 3);
    config.tracking.max_buffer_length = optionalScalar<int>(tracking_node, "max_buffer_length", legacy_ami ? 1000 : 2000);
    config.tracking.poly_order = optionalScalar<int>(tracking_node, "poly_order", legacy_ami ? 4 : 3);
    config.tracking.decay_factor = optionalScalar<double>(tracking_node, "decay_factor", legacy_ami ? 0.1 : 0.01);
    config.tracking.conf_probab_percent = optionalScalar<double>(
        tracking_node,
        "conf_probab_percent",
        optionalScalar<double>(tracking_node, "confidence_probability", legacy_ami ? 75.0 : 95.0));
    config.tracking.association_gate_sigma = optionalScalar<double>(tracking_node, "association_gate_sigma", 3.0);
    config.tracking.default_measurement_variance = optionalScalar<double>(tracking_node, "default_measurement_variance", 1.0);
    config.tracking.process_noise_variance = optionalScalar<double>(tracking_node, "process_noise_variance", 1.0);
    config.tracking.allowed_BER_per_seq = optionalScalar<int>(tracking_node, "allowed_BER_per_seq", legacy_ami ? 0 : 1);
    config.tracking.manchester_code = optionalScalar<bool>(tracking_node, "manchester_code", false);
    config.tracking.sequence_file = optionalScalar<std::string>(tracking_node, "sequence_file", std::string {});

    if (const YAML::Node custom_sequences = tracking_node["sequences"]; custom_sequences) {
        config.tracking.sequences = parseSequencesNode(custom_sequences, config.tracking.manchester_code);
    }
    if (!config.tracking.sequence_file.empty()) {
        const auto file_sequences = parseSequenceFile(config_path, config.tracking.sequence_file, config.tracking.manchester_code);
        if (file_sequences.empty()) {
            throw std::runtime_error("tracking.sequence_file is set but no valid blinking sequences were loaded.");
        }
        config.tracking.sequences = file_sequences;
    }
    if (config.tracking.sequences.empty()) {
        config.tracking.sequences = std::vector<std::vector<bool>> { { true, false, true, true, false, false } };
    }
    validateTrackingSequences(config.tracking.sequences);

    const YAML::Node tracking_inputs_node = tracking_node["inputs"];
    if (tracking_inputs_node && !tracking_inputs_node.IsNull()) {
        for (const YAML::Node& input_node : tracking_inputs_node) {
            TrackerInputConfig input;
            input.name = requireScalar<std::string>(input_node, "name");
            input.input_topic = requireScalar<std::string>(input_node, "input_topic");
            input.input_image_topic = optionalScalar<std::string>(input_node, "input_image_topic", std::string {});
            input.output_topic = requireScalar<std::string>(input_node, "output_topic");
            input.publish_visualization = optionalScalar<bool>(input_node, "publish_visualization", false);
            input.visualization_topic = optionalScalar<std::string>(input_node, "visualization_topic", std::string {});
            config.tracking.inputs.push_back(std::move(input));
        }
    }

    config.pose_estimation = parseModuleConfig(requireNode(root, "pose_estimation"));
    config.filtering = parseModuleConfig(requireNode(root, "filtering"));
    config.calibration = parseModuleConfig(requireNode(root, "calibration"));
    config.simulation = parseModuleConfig(requireNode(root, "simulation"));

    return config;
}

} // namespace uvdar_core::app
