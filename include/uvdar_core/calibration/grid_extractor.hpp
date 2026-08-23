#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace uvdar_core::calibration {

/** @brief Geometric thresholds used while tracing the candidate-point hull. */
struct GridExtractorOptions {
    double maximum_concave_angle = 0.7853981633974483; // pi/4
    double similar_angle = 0.3490658503988659; // pi/9
};

/** @brief Ordered result of the bright-point hull/grid extraction. */
struct GridExtractionResult {
    bool success = false;
    bool x_axis_first = true;
    // Points are column-major: all rows in column zero precede column one.
    std::vector<cv::Point2f> image_points;
    std::vector<cv::Point2f> hull_points;
    std::string detail;
};

/**
 * @brief Recover an ordered rectangular grid from unordered FIMD centroids.
 *
 * Duplicate integer-pixel points are discarded before a concave hull is
 * traced. The sharpest hull corner seeds a projective cell graph whose
 * column-major traversal produces the image-to-pattern correspondences.
 */
GridExtractionResult extractGridFromFimdPoints(
    const std::vector<cv::Point2f>& points,
    int columns,
    int rows,
    const GridExtractorOptions& options = {});

} // namespace uvdar_core::calibration
