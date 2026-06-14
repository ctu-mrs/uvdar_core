#pragma once

#include <memory>
#include <vector>

#include "uvdar_core/tracking/generalized/generalized_tracker.hpp"

namespace uvdar_core::tracking::generalized {

/**
 * @brief Wrapper around GeneralizedTracker with lifecycle helpers matching AMI BlinkProcessor.
 */
class BlinkProcessor {
public:
    /**
     * @brief Construct processor and initialize the underlying generalized tracker.
     * @param params Tracker tuning parameters.
     * @param sequences Supported blink templates.
     */
    BlinkProcessor(const ParamsGeneralized& params, const std::vector<std::vector<bool>>& sequences);
    /**
     * @brief Free resources.
     */
    ~BlinkProcessor();

    /**
     * @brief Recreate tracker instance with updated tuning parameters.
     */
    void reinit(const ParamsGeneralized& params);
    /**
     * @brief Replace supported signature sequences and reconfigure matcher.
     */
    void updateSequences(const std::vector<std::vector<bool>>& sequences);
    /**
     * @brief Process one timestamped detector frame and return uncertainty-rich tracks.
     */
    std::vector<TrackResult> processFrame(std::shared_ptr<const ImagePointsWithCovariancesStamped> image_points);

private:
    ParamsGeneralized passed_params_;
    std::unique_ptr<GeneralizedTracker> tracker_;
    std::vector<std::vector<bool>> sequences_;
};

} // namespace uvdar_core::tracking::generalized
