#include "uvdar_core/tracking/generalized/blink_processor.h"

#include <stdexcept>

namespace uvdar_core::tracking::generalized {

BlinkProcessor::BlinkProcessor(const ParamsGeneralized& params, const std::vector<std::vector<bool>>& sequences)
    : passed_params_(params)
{
    updateSequences(sequences);
}

BlinkProcessor::~BlinkProcessor() = default;

void BlinkProcessor::reinit(const ParamsGeneralized& params)
{
    passed_params_ = params;
    tracker_ = std::make_unique<GeneralizedTracker>(passed_params_);
    tracker_->setupSequenceMatcher(sequences_);
}

void BlinkProcessor::updateSequences(const std::vector<std::vector<bool>>& sequences)
{
    sequences_ = sequences;
    if (!tracker_) {
        tracker_ = std::make_unique<GeneralizedTracker>(passed_params_);
    }
    tracker_->setupSequenceMatcher(sequences_);
}

std::vector<TrackResult> BlinkProcessor::processFrame(std::shared_ptr<const ImagePointsWithCovariancesStamped> image_points)
{
    std::vector<TrackResult> tracks;
    if (!image_points) {
        return tracks;
    }

    if (!tracker_) {
        throw std::runtime_error("[tracker] Generalized BlinkProcessor not initialized.");
    }

    tracker_->processBuffer(*image_points);
    return tracker_->getResults();
}

} // namespace uvdar_core::tracking::generalized
