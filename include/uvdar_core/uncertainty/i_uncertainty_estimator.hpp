#pragma once

namespace uvdar_core::uncertainty {

class IUncertaintyEstimator {
public:
    virtual ~IUncertaintyEstimator() = default;
};

} // namespace uvdar_core::uncertainty
