#ifndef AMI_BLINK_PROCESSOR_H
#define AMI_BLINK_PROCESSOR_H

#include <iostream>
#include <memory>
#include <vector>

#include "uvdar_core/tracking/ami/ami.h"

namespace uvdar_core::tracking::ami {

/**
 * @brief Wrapper around AMI with lifecycle helpers similar to legacy blink processor.
 */
class BlinkProcessor {
private:
    ParamsAMI passed_params_;
    std::unique_ptr<AMI> ami_ptr;
    std::vector<std::vector<bool>> sequences_;

public:
    /**
     * @brief Construct processor and initialize underlying AMI.
     * @param params Tracker tuning parameters.
     * @param sequences Supported blink templates.
     */
    BlinkProcessor(const ParamsAMI& params, const std::vector<std::vector<bool>>& sequences);
    /**
     * @brief Free resources.
     */
    ~BlinkProcessor();

    /**
     * @brief Recreate AMI instance with updated tuning parameters.
     */
    void reinit(const ParamsAMI& params);
    /**
     * @brief Replace supported signature sequences and reconfigure matcher.
     */
    void updateSequences(const std::vector<std::vector<bool>>& sequences);
    /**
     * @brief Process one timestamped frame of detector points.
     */
    std::vector<std::pair<PointState, int>> processFrame(std::shared_ptr<const ImagePointsWithCovariancesStamped> image_points);
};

} // namespace uvdar_core::tracking::ami

#endif // AMI_BLINK_PROCESSOR_H
