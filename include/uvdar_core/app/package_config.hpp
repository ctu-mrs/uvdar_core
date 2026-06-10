#pragma once

#include <string>
#include <vector>

namespace uvdar_core::app {

enum class DetectorBackend {
    Cpu,
    Gpu,
};

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

struct DetectorConfig {
    bool debug;
    bool gui;
    double initial_delay_sec;
    std::size_t queue_depth;
    std::size_t thread_pool_size;
    std::size_t max_points_per_image;
    std::vector<DetectorInputConfig> inputs;
};

struct ModuleConfig {
    bool enabled;
    std::string implementation;
};

struct PackageConfig {
    std::string source_path;
    DetectorConfig detector;
    ModuleConfig tracking;
    ModuleConfig pose_estimation;
    ModuleConfig filtering;
    ModuleConfig calibration;
    ModuleConfig simulation;
};

PackageConfig loadPackageConfig(const std::string& config_path);

} // namespace uvdar_core::app