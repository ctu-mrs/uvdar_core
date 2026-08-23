#include "uvdar_core/calibration/intermediate_dataset.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>

#include "uvdar_core/calibration/calibration_visualization.hpp"

namespace uvdar_core::calibration {

namespace {

constexpr int dataset_schema_version = 1;

std::string framePrefix(const std::size_t frame_index)
{
    std::ostringstream stream;
    stream << "frame_" << std::setfill('0') << std::setw(5) << frame_index;
    return stream.str();
}

YAML::Node imagePointsNode(const std::vector<cv::Point2f>& points)
{
    YAML::Node output(YAML::NodeType::Sequence);
    for (const cv::Point2f& point : points) {
        output.push_back(std::vector<double> {point.x, point.y});
    }
    return output;
}

YAML::Node objectPointsNode(const std::vector<cv::Point3f>& points)
{
    YAML::Node output(YAML::NodeType::Sequence);
    for (const cv::Point3f& point : points) {
        output.push_back(std::vector<double> {point.x, point.y, point.z});
    }
    return output;
}

std::vector<cv::Point2f> readImagePoints(
    const YAML::Node& parent, const std::string& key)
{
    const YAML::Node sequence = parent[key];
    if (!sequence || !sequence.IsSequence()) {
        throw std::runtime_error("Missing point sequence '" + key + "'.");
    }
    std::vector<cv::Point2f> output;
    output.reserve(sequence.size());
    for (const YAML::Node& point : sequence) {
        if (!point.IsSequence() || point.size() != 2U) {
            throw std::runtime_error("Every '" + key + "' entry must contain x and y.");
        }
        const float x = point[0].as<float>();
        const float y = point[1].as<float>();
        if (!std::isfinite(x) || !std::isfinite(y)) {
            throw std::runtime_error("Every '" + key + "' entry must be finite.");
        }
        output.emplace_back(x, y);
    }
    return output;
}

std::vector<cv::Point3f> readObjectPoints(
    const YAML::Node& parent, const std::string& key)
{
    const YAML::Node sequence = parent[key];
    if (!sequence || !sequence.IsSequence()) {
        throw std::runtime_error("Missing point sequence '" + key + "'.");
    }
    std::vector<cv::Point3f> output;
    output.reserve(sequence.size());
    for (const YAML::Node& point : sequence) {
        if (!point.IsSequence() || point.size() != 3U) {
            throw std::runtime_error(
                "Every '" + key + "' entry must contain x, y, and z.");
        }
        const float x = point[0].as<float>();
        const float y = point[1].as<float>();
        const float z = point[2].as<float>();
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            throw std::runtime_error("Every '" + key + "' entry must be finite.");
        }
        output.emplace_back(x, y, z);
    }
    return output;
}

void requireUnusedTargets(const std::vector<std::filesystem::path>& paths)
{
    for (const std::filesystem::path& path : paths) {
        if (std::filesystem::exists(path)) {
            throw std::runtime_error(
                "Intermediate result already exists: '" + path.string()
                + "'. Select an empty directory for a new acquisition.");
        }
    }
}

void writeImage(const std::filesystem::path& path, const cv::Mat& image)
{
    if (!cv::imwrite(path.string(), image)) {
        throw std::runtime_error("Could not write image '" + path.string() + "'.");
    }
}

void writeYaml(const std::filesystem::path& path, const YAML::Node& root)
{
    YAML::Emitter emitter;
    emitter.SetDoublePrecision(16);
    emitter << root;
    if (!emitter.good()) {
        throw std::runtime_error("Could not serialize '" + path.string() + "'.");
    }
    std::ofstream stream(path, std::ios::trunc);
    if (!stream.is_open()) {
        throw std::runtime_error("Could not open '" + path.string() + "'.");
    }
    stream << emitter.c_str() << '\n';
    if (!stream.good()) {
        throw std::runtime_error("Could not finish writing '" + path.string() + "'.");
    }
}

void publishTemporaryFile(
    const std::filesystem::path& temporary,
    const std::filesystem::path& destination)
{
    std::error_code error;
    std::filesystem::rename(temporary, destination, error);
    if (error) {
        throw std::runtime_error(
            "Could not publish intermediate result '" + destination.string()
            + "': " + error.message());
    }
}

bool isFrameMetadata(const std::filesystem::path& path)
{
    const std::string name = path.filename().string();
    constexpr const char* prefix = "frame_";
    constexpr const char* suffix = "_detections.yaml";
    return name.rfind(prefix, 0U) == 0U
        && name.size() > std::char_traits<char>::length(prefix)
            + std::char_traits<char>::length(suffix)
        && name.compare(
            name.size() - std::char_traits<char>::length(suffix),
            std::char_traits<char>::length(suffix), suffix) == 0;
}

} // namespace

void storeIntermediateCalibrationFrame(
    const std::filesystem::path& directory,
    const std::size_t frame_index,
    const cv::Mat& original_image,
    const PatternDetection& detection,
    const PatternDetectorOptions& detector_options)
{
    if (directory.empty() || original_image.empty() || !detection.success
        || detection.image_points.size() != detection.object_points.size()
        || detection.image_points.size()
            != static_cast<std::size_t>(
                detector_options.rows * detector_options.columns)) {
        throw std::invalid_argument(
            "An intermediate frame requires a directory, image, and complete detection.");
    }
    std::filesystem::create_directories(directory);
    const std::string prefix = framePrefix(frame_index);
    const std::filesystem::path original = directory / (prefix + "_original.png");
    const std::filesystem::path overlay = directory / (prefix + "_overlay.png");
    const std::filesystem::path metadata = directory / (prefix + "_detections.yaml");
    requireUnusedTargets({original, overlay, metadata});

    CalibrationVisualizationState overlay_state;
    overlay_state.pattern_rows = detector_options.rows;
    overlay_state.pattern_columns = detector_options.columns;
    overlay_state.pattern_column_major =
        detector_options.pattern == CalibrationPatternType::LedGrid;
    overlay_state.candidates = detection.candidates;
    overlay_state.hull_points = detection.hull_points;
    overlay_state.detected_points = detection.image_points;
    const cv::Mat annotated = renderCalibrationDetectionOverlay(
        original_image, overlay_state);

    YAML::Node root;
    root["schema_version"] = dataset_schema_version;
    root["frame_index"] = static_cast<unsigned long long>(frame_index);
    root["pattern_type"] = toString(detector_options.pattern);
    root["pattern_rows"] = detector_options.rows;
    root["pattern_columns"] = detector_options.columns;
    root["pattern_spacing"] = detector_options.spacing;
    root["image_width"] = original_image.cols;
    root["image_height"] = original_image.rows;
    root["original_image"] = original.filename().string();
    root["overlay_image"] = overlay.filename().string();
    root["image_points"] = imagePointsNode(detection.image_points);
    root["object_points"] = objectPointsNode(detection.object_points);
    root["candidates"] = imagePointsNode(detection.candidates);
    root["hull_points"] = imagePointsNode(detection.hull_points);

    const std::filesystem::path original_temporary =
        directory / (prefix + "_original.tmp.png");
    const std::filesystem::path overlay_temporary =
        directory / (prefix + "_overlay.tmp.png");
    const std::filesystem::path metadata_temporary =
        directory / (prefix + "_detections.yaml.tmp");
    requireUnusedTargets(
        {original_temporary, overlay_temporary, metadata_temporary});
    writeImage(original_temporary, original_image);
    writeImage(overlay_temporary, annotated);
    writeYaml(metadata_temporary, root);
    publishTemporaryFile(original_temporary, original);
    publishTemporaryFile(overlay_temporary, overlay);
    publishTemporaryFile(metadata_temporary, metadata);
}

IntermediateCalibrationDataset loadIntermediateCalibrationDataset(
    const std::filesystem::path& directory,
    const PatternDetectorOptions& expected_detector_options)
{
    if (!std::filesystem::is_directory(directory)) {
        throw std::runtime_error(
            "Intermediate result directory does not exist: '"
            + directory.string() + "'.");
    }
    std::vector<std::filesystem::path> metadata_files;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file() && isFrameMetadata(entry.path())) {
            metadata_files.push_back(entry.path());
        }
    }
    std::sort(metadata_files.begin(), metadata_files.end());
    if (metadata_files.empty()) {
        throw std::runtime_error(
            "No frame detection YAML files found in '" + directory.string() + "'.");
    }

    IntermediateCalibrationDataset dataset;
    dataset.observations.reserve(metadata_files.size());
    dataset.images.reserve(metadata_files.size());
    dataset.detections.reserve(metadata_files.size());
    for (const std::filesystem::path& metadata_path : metadata_files) {
        const YAML::Node root = YAML::LoadFile(metadata_path.string());
        if (!root["schema_version"]
            || root["schema_version"].as<int>() != dataset_schema_version) {
            throw std::runtime_error(
                "Unsupported schema in '" + metadata_path.string() + "'.");
        }
        if (root["pattern_type"].as<std::string>()
                != toString(expected_detector_options.pattern)
            || root["pattern_rows"].as<int>() != expected_detector_options.rows
            || root["pattern_columns"].as<int>()
                != expected_detector_options.columns
            || std::abs(root["pattern_spacing"].as<double>()
                    - expected_detector_options.spacing) > 1.0e-12) {
            throw std::runtime_error(
                "Pattern configuration does not match '" + metadata_path.string() + "'.");
        }

        const std::filesystem::path original_path = metadata_path.parent_path()
            / root["original_image"].as<std::string>();
        cv::Mat image = cv::imread(original_path.string(), cv::IMREAD_UNCHANGED);
        if (image.empty()) {
            throw std::runtime_error(
                "Could not read source image '" + original_path.string() + "'.");
        }
        const cv::Size declared_size(
            root["image_width"].as<int>(), root["image_height"].as<int>());
        if (image.size() != declared_size
            || (!dataset.images.empty() && image.size() != dataset.image_size)) {
            throw std::runtime_error(
                "Inconsistent image dimensions in '" + metadata_path.string() + "'.");
        }

        PatternDetection detection;
        detection.success = true;
        detection.detail = "Loaded saved pattern detection";
        detection.image_points = readImagePoints(root, "image_points");
        detection.object_points = readObjectPoints(root, "object_points");
        detection.candidates = readImagePoints(root, "candidates");
        detection.hull_points = readImagePoints(root, "hull_points");
        if (detection.image_points.size() < 4U
            || detection.image_points.size() != detection.object_points.size()
            || detection.image_points.size()
                != static_cast<std::size_t>(
                    expected_detector_options.rows
                    * expected_detector_options.columns)) {
            throw std::runtime_error(
                "Incomplete correspondences in '" + metadata_path.string() + "'.");
        }
        if (dataset.images.empty()) {
            dataset.image_size = image.size();
        }
        CalibrationObservation observation;
        observation.image_points = detection.image_points;
        observation.object_points = detection.object_points;
        dataset.observations.push_back(std::move(observation));
        dataset.images.push_back(std::move(image));
        dataset.detections.push_back(std::move(detection));
    }
    return dataset;
}

} // namespace uvdar_core::calibration
