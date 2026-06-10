#pragma once

#include <opencv2/core.hpp>

#include <vector>

namespace uvdar_core::detection {

struct DetectorOutput {
    std::vector<cv::Point2i> detected_points;
    std::vector<cv::Point2i> sun_points;
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
