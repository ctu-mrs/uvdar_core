#pragma once

#include <vector>

#include "uvdar_core/tracking/types.hpp"

namespace uvdar_core::tracking {

class ITracker {
public:
    virtual ~ITracker() = default;

    /**
     * @brief Load blinking templates and reconfigure marker-ID matching.
     */
    virtual void setupSequenceMatcher(std::vector<std::vector<bool>> sequences) = 0;
    /**
     * @brief Consume one timestamped detector frame.
     */
    virtual void processBuffer(const ImagePointsWithCovariancesStamped& points) = 0;
    /**
     * @brief Return currently matched tracks in the shared result format.
     */
    virtual std::vector<TrackResult> getResults() const = 0;
};

} // namespace uvdar_core::tracking
