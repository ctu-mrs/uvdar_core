#pragma once

namespace uvdar_core::pose_estimation {

class IPoseEstimator {
public:
    virtual ~IPoseEstimator() = default;
};

} // namespace uvdar_core::pose_estimation