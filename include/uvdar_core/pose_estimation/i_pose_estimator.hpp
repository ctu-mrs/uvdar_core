#pragma once

namespace uvdar_core::pose_estimation {

class IPoseEstimator {
public:
    /**
     * @brief Interface marker for pose estimation backends.
     */
    virtual ~IPoseEstimator() = default;
};

} // namespace uvdar_core::pose_estimation
