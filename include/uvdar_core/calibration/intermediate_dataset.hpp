#pragma once

#include <cstddef>
#include <filesystem>
#include <vector>

#include <opencv2/core.hpp>

#include "uvdar_core/calibration/calibrator.hpp"
#include "uvdar_core/calibration/pattern_detector.hpp"

namespace uvdar_core::calibration {

/** @brief Validated observations and source images loaded from one dataset directory. */
struct IntermediateCalibrationDataset {
    cv::Size image_size;
    std::vector<CalibrationObservation> observations;
    std::vector<cv::Mat> images;
    std::vector<PatternDetection> detections;
};

/**
 * @brief Store one accepted view as a lossless source image, overlay, and YAML.
 *
 * Existing files are never overwritten, which prevents views from separate
 * acquisition sessions from being mixed silently.
 */
void storeIntermediateCalibrationFrame(
    const std::filesystem::path& directory,
    std::size_t frame_index,
    const cv::Mat& original_image,
    const PatternDetection& detection,
    const PatternDetectorOptions& detector_options);

/** @brief Load and validate every stored frame without running a detector. */
IntermediateCalibrationDataset loadIntermediateCalibrationDataset(
    const std::filesystem::path& directory,
    const PatternDetectorOptions& expected_detector_options);

} // namespace uvdar_core::calibration
