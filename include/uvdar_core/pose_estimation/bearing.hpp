#pragma once

#include <optional>

#include <Eigen/Dense>

#include "uvdar_core/calibration/i_lens_model.hpp"

namespace uvdar_core::pose_estimation {

/**
 * @brief Unit camera ray and its Cartesian tangent-plane covariance.
 */
struct BearingMeasurement {
    Eigen::Vector3d vector = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
};

/**
 * @brief Back-project a Gaussian pixel observation into a unit bearing.
 *
 * Pixel covariance is symmetrized and projected to the positive-definite cone
 * before first-order propagation. The resulting 3D covariance is constrained
 * to the tangent plane of the unit sphere and is consequently rank at most two.
 *
 * @param lens Calibrated camera/lens model.
 * @param pixel Image point in [x, y] pixel coordinates.
 * @param pixel_covariance Pixel covariance in px^2.
 * @param covariance_floor_px2 Smallest accepted pixel-covariance eigenvalue.
 * @param fallback_pixel_variance_px2 Isotropic variance used for non-finite
 *        input covariance.
 * @return Bearing and covariance, or std::nullopt for invalid input/model
 * output.
 */
std::optional<BearingMeasurement> bearingFromPixel(const calibration::ILensModel& lens, const Eigen::Vector2d& pixel,
                                                   const Eigen::Matrix2d& pixel_covariance, double covariance_floor_px2,
                                                   double fallback_pixel_variance_px2);

} // namespace uvdar_core::pose_estimation
