#pragma once

namespace uvdar_core::uncertainty {

class IUncertaintyEstimator {
public:
    /**
     * @brief Interface marker for uncertainty estimators.
     */
    virtual ~IUncertaintyEstimator() = default;
};

} // namespace uvdar_core::uncertainty
