#include "uvdar_core/tracking/ami/blink_processor.h"

#include <stdexcept>

namespace uvdar_core::tracking::ami {

/**
 * @brief Construct processor and initialize AMI with initial parameters.
 */
BlinkProcessor::BlinkProcessor(const ParamsAMI& params, const std::vector<std::vector<bool>>& sequences)
    : passed_params_(params)
{
    updateSequences(sequences);
}

/**
 * @brief Default destructor.
 */
BlinkProcessor::~BlinkProcessor() = default;

/**
 * @brief Reinitialize AMI with new runtime configuration.
 */
void BlinkProcessor::reinit(const ParamsAMI& params)
{
    passed_params_ = params;
    ami_ptr = std::make_unique<AMI>(passed_params_);
    ami_ptr->setupSequenceMatcher(sequences_);
}

/**
 * @brief Replace sequences and rebuild matcher for new signature set.
 */
void BlinkProcessor::updateSequences(const std::vector<std::vector<bool>>& sequences)
{
    sequences_ = sequences;
    if (!ami_ptr) {
        ami_ptr = std::make_unique<AMI>(passed_params_);
    }
    ami_ptr->setupSequenceMatcher(sequences_);
}

/**
 * @brief Convert detector frame to AMI state and return matched blinkers.
 */
std::vector<std::pair<PointState, int>> BlinkProcessor::processFrame(std::shared_ptr<const ImagePointsWithCovariancesStamped> image_points)
{
    std::vector<std::pair<PointState, int>> blinkers;
    if (!image_points) {
        return blinkers;
    }

    if (!ami_ptr) {
        throw std::runtime_error("[tracker] BlinkProcessor not initialized.");
    }

    ami_ptr->processBuffer(*image_points);
    const auto retrieved_signals = ami_ptr->getResults();
    for (const auto& blink : retrieved_signals) {
        blinkers.push_back(std::make_pair(blink.first.first, blink.first.second));
    }

    return blinkers;
}

} // namespace uvdar_core::tracking::ami
