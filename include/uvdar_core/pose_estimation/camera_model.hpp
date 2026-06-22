#pragma once

#include "uvdar_core/calibration/i_lens_model.hpp"

namespace uvdar_core::pose_estimation {

/**
 * @brief Generic calibrated camera used by pose-estimation backends.
 *
 * Pose estimators depend only on the lens interface: project, back-project,
 * and their Jacobians. No solver should require a concrete lens model type.
 */
struct CameraModel {
    calibration::LensModelPtr lens;
    int image_width = 0;
    int image_height = 0;
};

} // namespace uvdar_core::pose_estimation
