#pragma once

namespace uvdar_core::filtering {

class IFilter {
public:
    /**
     * @brief Interface marker for filter backends.
     */
    virtual ~IFilter() = default;
};

} // namespace uvdar_core::filtering
