#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace uvdar_core::calibration {

enum class CalibrationPatternType {
    Checkerboard,
    LedGrid,
};

CalibrationPatternType calibrationPatternFromString(const std::string& name);
std::string toString(CalibrationPatternType pattern);

/** @brief Pattern geometry plus checkerboard/FIMD detection controls. */
struct PatternDetectorOptions {
    CalibrationPatternType pattern = CalibrationPatternType::Checkerboard;
    int rows = 6;
    int columns = 8;
    double spacing = 0.04;
    int checkerboard_max_detection_height = 520;
    int maximum_candidates = 200;
    int fimd_threshold = 120;
    int fimd_threshold_diff = 60;
    unsigned fimd_max_markers = 300;
    std::vector<unsigned> fimd_radii {3, 5};
    double hull_maximum_concave_angle = 0.7853981633974483;
    double hull_similar_angle = 0.3490658503988659;
};

/** @brief Ordered image/object correspondences and raw candidates for display. */
struct PatternDetection {
    bool success = false;
    std::vector<cv::Point2f> candidates;
    std::vector<cv::Point2f> hull_points;
    std::vector<cv::Point2f> image_points;
    std::vector<cv::Point3f> object_points;
    std::string detail;
};

/**
 * @brief Detect and order either checkerboard corners or a bright LED grid.
 *
 * LED candidate extraction delegates to the package's CPU FIMD implementation;
 * only generic grid ordering is performed here.
 */
class CalibrationPatternDetector {
public:
    explicit CalibrationPatternDetector(PatternDetectorOptions options);
    ~CalibrationPatternDetector();

    CalibrationPatternDetector(const CalibrationPatternDetector&) = delete;
    CalibrationPatternDetector& operator=(const CalibrationPatternDetector&) = delete;
    CalibrationPatternDetector(CalibrationPatternDetector&&) noexcept;
    CalibrationPatternDetector& operator=(CalibrationPatternDetector&&) noexcept;

    PatternDetection detect(const cv::Mat& grayscale_image);
    const PatternDetectorOptions& options() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace uvdar_core::calibration
