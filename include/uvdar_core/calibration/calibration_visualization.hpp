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
    int image_width = 0;
    int image_height = 0;
    int retained_views = 0;
    int total_views = 0;
    int result_iterations = 0;
    int displayed_view = -1;
    double rms_px = 0.0;
    double displayed_view_rms_px = 0.0;
    double damping = 0.0;
    double elapsed_seconds = 0.0;
    double initialization_seconds = 0.0;
    double optimization_seconds = 0.0;
    double refinement_seconds = 0.0;
    double validation_seconds = 0.0;
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
    std::vector<double> per_view_rms_px;
    std::vector<bool> retained_view_mask;
    std::vector<cv::Point2f> model_projection_curve;
    std::vector<double> intrinsics;
    std::vector<double> distortion;
    std::vector<double> direct_polynomial;
    std::vector<double> inverse_polynomial;
    cv::Point2d center;
    cv::Vec3d affine {1.0, 0.0, 0.0};
    std::string result_message;
};

/** @brief Render the image overlay, coverage, pipeline, and optimizer plot. */
cv::Mat renderCalibrationVisualization(
    const cv::Mat& image,
    const CalibrationVisualizationState& state);

} // namespace uvdar_core::calibration
