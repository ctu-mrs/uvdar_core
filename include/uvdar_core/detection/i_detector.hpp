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

    virtual bool initDelayed(const cv::Mat& image)                                            = 0;
    virtual bool processImage(const cv::Mat& image, DetectorOutput& output, int mask_id = -1) = 0;
};

} // namespace uvdar_core::detection