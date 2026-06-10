#pragma once

namespace uvdar_core::calibration {

class ILensModel {
public:
    /**
     * @brief Interface marker for lens model implementations.
     */
    virtual ~ILensModel() = default;
};

} // namespace uvdar_core::calibration
