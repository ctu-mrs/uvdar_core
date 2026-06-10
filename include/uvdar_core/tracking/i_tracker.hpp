#pragma once

namespace uvdar_core::tracking {

class ITracker {
public:
    /**
     * @brief Interface marker for tracker backends.
     */
    virtual ~ITracker() = default;
};

} // namespace uvdar_core::tracking
