#pragma once

#include <cstddef>
#include <opencv2/core.hpp>

#include <vector>

namespace uvdar_core::detection {

/**
 * @brief Point extracted from a detector with optional covariance estimate.
 */
struct DetectorPoint {
    cv::Point2f point = { 0.0F, 0.0F };
    float covariance_00 = 0.0F;
    float covariance_01 = 0.0F;
    float covariance_10 = 0.0F;
    float covariance_11 = 0.0F;
    std::size_t sample_count = 1;
};

struct DetectorOutput {
    std::vector<DetectorPoint> detected_points;
    std::vector<DetectorPoint> sun_points;
};

class IDetector {
public:
    virtual ~IDetector() = default;

    /**
     * @brief Initialize detector state using first received frame.
     * @param image First image used for setup.
     * @return True when detector is ready.
     */
    virtual bool initDelayed(const cv::Mat& image)                                            = 0;
    /**
     * @brief Run one-frame detection.
     * @param image Input image.
     * @param output Result holder (detected and sun points).
     * @param mask_id Optional mask index, -1 disables masking.
     * @return True on successful processing.
     */
    virtual bool processImage(const cv::Mat& image, DetectorOutput& output, int mask_id = -1) = 0;
};

} // namespace uvdar_core::detection
