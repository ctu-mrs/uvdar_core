#pragma once

#include <string>
#include <vector>

namespace uvdar_core::app {

/**
 * @brief Backend selection for detector pipelines.
 */
enum class DetectorBackend {
    Cpu,
    Gpu,
};

/**
 * @brief Configuration for one detector input stream.
 */
struct DetectorInputConfig {
    std::string name;
    bool enabled;
    std::string input_topic;
    std::string output_topic;
    bool detect_sun_points;
    bool publish_sun_points;
    std::string sun_output_topic;
    bool publish_visualization;
    std::string visualization_topic;
    std::string mask_file;
    DetectorBackend backend;
    int threshold;
    int threshold_diff;
    int threshold_sun;
    unsigned min_sun_marker_distance;
    unsigned max_markers_count;
    unsigned max_sun_points_count;
    std::vector<unsigned> radii;
};

/**
 * @brief Configuration for detector module execution.
 */
struct DetectorConfig {
    bool debug;
    bool gui;
    double initial_delay_sec;
    std::size_t queue_depth;
    std::size_t thread_pool_size;
    std::size_t max_points_per_image;
    std::vector<DetectorInputConfig> inputs;
};

/**
 * @brief Module state loaded from YAML (used by all modules).
 */
struct ModuleConfig {
    bool enabled;
    std::string implementation;
};

/**
 * @brief Configuration for one tracking input stream.
 */
struct TrackerInputConfig {
    std::string name;
    bool enabled;
    std::string input_topic;
    std::string input_image_topic;
    std::string output_topic;
    bool publish_visualization;
    std::string visualization_topic;
};

/**
 * @brief Configuration for tracking module execution.
 */
struct TrackerConfig {
    ModuleConfig module;
    bool debug;
    bool gui;
    double initial_delay_sec;
    std::size_t queue_depth;
    std::size_t thread_pool_size;
    std::size_t max_points_per_image;
    int max_px_shift_x;
    int max_px_shift_y;
    int max_zeros_consecutive;
    int stored_seq_len_factor;
    int max_buffer_length;
    int poly_order;
    double decay_factor;
    double conf_probab_percent;
    int allowed_BER_per_seq;
    bool manchester_code;
    std::string sequence_file;
    std::vector<std::vector<bool>> sequences;
    std::vector<TrackerInputConfig> inputs;
};

/**
 * @brief Full package configuration loaded from YAML.
 */
struct PackageConfig {
    std::string source_path;
    DetectorConfig detector;
    TrackerConfig tracking;
    ModuleConfig pose_estimation;
    ModuleConfig filtering;
    ModuleConfig calibration;
    ModuleConfig simulation;
};

/**
 * @brief Load complete package configuration from a YAML file.
 * @param config_path Path to the YAML configuration file.
 * @return Parsed configuration.
 */
PackageConfig loadPackageConfig(const std::string& config_path);

} // namespace uvdar_core::app
