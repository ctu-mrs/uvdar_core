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
    /**
     * @brief Construct GPU detector.
     */
    explicit GpuDetector(GpuDetectorConfig config = { });
    /**
     * @brief Release GPU resources.
     */
    ~GpuDetector() override;

    /**
     * @brief Initialize detector when first frame arrives.
     */
    bool initDelayed(const cv::Mat& image) override;
    /**
     * @brief Process one frame on the GPU and decode outputs.
     */
    bool processImage(const cv::Mat& image, DetectorOutput& output, int mask_id = -1) override;

    /**
     * @brief Enable/disable verbose debug behavior.
     */
    bool get_debug() const;
    /**
     * @brief Toggle debug mode.
     */
    void set_debug(bool debug);
    /**
     * @brief Whether sun point detection is enabled.
     */
    bool get_detect_sun_points() const;
    /**
     * @brief Enable/disable sun point detection.
     */
    void set_detect_sun_points(bool detect_sun_points);
    /**
     * @brief Get current marker threshold.
     */
    int get_threshold() const;
    /**
     * @brief Set marker threshold.
     */
    void set_threshold(int threshold);
    /**
     * @brief Get center/boundary delta threshold.
     */
    int get_threshold_diff() const;
    /**
     * @brief Set center/boundary delta threshold.
     */
    void set_threshold_diff(int threshold_diff);
    /**
     * @brief Get sun-only intensity threshold.
     */
    int get_threshold_sun() const;
    /**
     * @brief Set sun-only intensity threshold.
     */
    void set_threshold_sun(int threshold_sun);
    /**
     * @brief Maximum number of detected markers.
     */
    unsigned get_max_markers_count() const;
    /**
     * @brief Set marker capacity and force reinitialization.
     */
    void set_max_markers_count(unsigned max_markers_count);
    /**
     * @brief Maximum number of detected sun points.
     */
    unsigned get_max_sun_points_count() const;
    /**
     * @brief Set sun-point capacity and force reinitialization.
     */
    void set_max_sun_points_count(unsigned max_sun_points_count);
    /**
     * @brief Read configured radii list.
     */
    const std::vector<unsigned>& get_radii() const;
    /**
     * @brief Set radii list and force reinitialization.
     */
    void set_radii(std::vector<unsigned> radii);
    /**
     * @brief Read loaded masks.
     */
    const std::vector<cv::Mat>& get_masks() const;
    /**
     * @brief Replace mask list.
     */
    void set_masks(std::vector<cv::Mat> masks);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace uvdar_core::detection::fimd
