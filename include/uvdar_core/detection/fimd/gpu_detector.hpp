#pragma once

#include <memory>
#include <vector>

#include <opencv2/core.hpp>

#include "uvdar_core/detection/i_detector.hpp"

namespace uvdar_core::detection::fimd {

struct GpuDetectorConfig {
    bool debug                       = false;
    bool detect_sun_points           = true;
    int threshold                    = 120;
    int threshold_diff               = 60;
    int threshold_sun                = 240;
    unsigned min_sun_marker_distance = 20;
    unsigned max_markers_count       = 300;
    unsigned max_sun_points_count    = 10000;
    std::vector<unsigned> radii      = { 3, 5 };
    std::vector<cv::Mat> masks;
};

class GpuDetector final : public uvdar_core::detection::IDetector {
public:
    explicit GpuDetector(GpuDetectorConfig config = { });
    ~GpuDetector() override;

    bool initDelayed(const cv::Mat& image) override;
    bool processImage(const cv::Mat& image, DetectorOutput& output, int mask_id = -1) override;

    bool get_debug() const;
    void set_debug(bool debug);
    bool get_detect_sun_points() const;
    void set_detect_sun_points(bool detect_sun_points);
    int get_threshold() const;
    void set_threshold(int threshold);
    int get_threshold_diff() const;
    void set_threshold_diff(int threshold_diff);
    int get_threshold_sun() const;
    void set_threshold_sun(int threshold_sun);
    unsigned get_max_markers_count() const;
    void set_max_markers_count(unsigned max_markers_count);
    unsigned get_max_sun_points_count() const;
    void set_max_sun_points_count(unsigned max_sun_points_count);
    const std::vector<unsigned>& get_radii() const;
    void set_radii(std::vector<unsigned> radii);
    const std::vector<cv::Mat>& get_masks() const;
    void set_masks(std::vector<cv::Mat> masks);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace uvdar_core::detection::fimd