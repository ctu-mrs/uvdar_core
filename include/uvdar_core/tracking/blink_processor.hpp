#pragma once

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "uvdar_core/tracking/i_tracker.hpp"

namespace uvdar_core::tracking {

/**
 * @brief Shared lifecycle wrapper for blink trackers.
 *
 * Tracker backends own their association logic and result format. This wrapper
 * keeps the repeated setup, reinitialization, and frame-processing flow in one
 * parent implementation.
 */
template <typename Traits>
class BlinkProcessor {
public:
    using Params = typename Traits::Params;
    using Tracker = typename Traits::Tracker;
    static_assert(std::is_base_of_v<ITracker, Tracker>, "BlinkProcessor tracker must implement ITracker.");

    BlinkProcessor(const Params& params, const std::vector<std::vector<bool>>& sequences)
        : params_(params)
    {
        updateSequences(sequences);
    }

    /**
     * @brief Recreate the backend tracker with new runtime parameters.
     */
    void reinit(const Params& params)
    {
        params_ = params;
        tracker_ = std::make_unique<Tracker>(params_);
        tracker_->setupSequenceMatcher(sequences_);
    }

    /**
     * @brief Replace supported marker signatures and rebuild the matcher.
     */
    void updateSequences(const std::vector<std::vector<bool>>& sequences)
    {
        sequences_ = sequences;
        if (!tracker_) {
            tracker_ = std::make_unique<Tracker>(params_);
        }
        tracker_->setupSequenceMatcher(sequences_);
    }

    /**
     * @brief Process one timestamped detector frame and return backend results.
     */
    std::vector<TrackResult> processFrame(std::shared_ptr<const ImagePointsWithCovariancesStamped> image_points)
    {
        if (!image_points) {
            return {};
        }
        if (!tracker_) {
            throw std::runtime_error(std::string("[tracker] ") + Traits::name() + " processor is not initialized.");
        }

        insertMissingFrames(*image_points);
        updateFramerateEstimate(image_points->stamp);
        tracker_->processBuffer(*image_points);
        last_frame_stamp_ = image_points->stamp;
        have_last_frame_stamp_ = true;
        return tracker_->getResults();
    }

private:
    /**
     * @brief Estimate frame rate from batches of ten frame timestamps.
     */
    void updateFramerateEstimate(double stamp)
    {
        if (!have_diagnostic_stamp_) {
            last_diagnostic_stamp_ = stamp;
            have_diagnostic_stamp_ = true;
            sample_count_ = 0;
            return;
        }

        ++sample_count_;
        if (sample_count_ < 10) {
            return;
        }

        const double dt = stamp - last_diagnostic_stamp_;
        if (dt > 0.0) {
            framerate_estimate_ = 10.0 / dt;
        }
        last_diagnostic_stamp_ = stamp;
        sample_count_ = 0;
    }

    /**
     * @brief Insert empty frames when detector timestamps skip expected periods.
     *
     * This preserves blink sequence timing by feeding OFF observations to the
     * backend tracker for missed camera frames.
     */
    void insertMissingFrames(const ImagePointsWithCovariancesStamped& image_points)
    {
        if (!have_last_frame_stamp_ || framerate_estimate_ <= 0.0) {
            return;
        }

        const double dt = image_points.stamp - last_frame_stamp_;
        if (dt <= 1.5 / framerate_estimate_) {
            return;
        }

        const int missing_count = static_cast<int>(dt * framerate_estimate_ + 0.5) - 1;
        if (missing_count <= 0) {
            return;
        }

        ImagePointsWithCovariancesStamped missing_frame = image_points;
        missing_frame.points.clear();
        const double frame_period = 1.0 / framerate_estimate_;
        for (int i = 0; i < missing_count; ++i) {
            missing_frame.stamp = last_frame_stamp_ + static_cast<double>(i + 1) * frame_period;
            tracker_->processBuffer(missing_frame);
        }
    }

    Params params_;
    std::unique_ptr<Tracker> tracker_;
    std::vector<std::vector<bool>> sequences_;
    bool have_last_frame_stamp_ = false;
    double last_frame_stamp_ = 0.0;
    bool have_diagnostic_stamp_ = false;
    double last_diagnostic_stamp_ = 0.0;
    int sample_count_ = 0;
    double framerate_estimate_ = 72.0;
};

} // namespace uvdar_core::tracking
