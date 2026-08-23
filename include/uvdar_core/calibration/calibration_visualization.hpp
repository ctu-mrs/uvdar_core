#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace uvdar_core::calibration {

/** @brief Immutable data needed to render one calibrator status frame. */
struct CalibrationVisualizationState {
    std::string stage;
    std::string detail;
    std::string model;
    std::string pattern;
    std::string output_path;
    int stage_index = 0;
    int accepted_frames = 0;
    int required_frames = 0;
    int detected_candidates = 0;
    int iteration = 0;
    int maximum_iterations = 0;
    int pattern_rows = 0;
    int pattern_columns = 0;
    double rms_px = 0.0;
    double damping = 0.0;
    double elapsed_seconds = 0.0;
    bool pattern_found = false;
    bool pattern_column_major = false;
    bool successful = false;
    bool failed = false;
    std::vector<cv::Point2f> candidates;
    std::vector<cv::Point2f> hull_points;
    std::vector<cv::Point2f> detected_points;
    std::vector<cv::Point2f> measured_points;
    std::vector<cv::Point2f> projected_points;
    std::vector<cv::Point2f> accepted_centers_normalized;
    std::vector<double> cost_history;
};

/** @brief Render the image overlay, coverage, pipeline, and optimizer plot. */
cv::Mat renderCalibrationVisualization(
    const cv::Mat& image,
    const CalibrationVisualizationState& state);

} // namespace uvdar_core::calibration
