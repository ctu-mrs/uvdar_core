#include "uvdar_core/pose_estimation/bearing.hpp"

#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>

namespace uvdar_core::pose_estimation {

namespace {

std::optional<Eigen::Matrix2d> sanitizePixelCovariance(const Eigen::Matrix2d& covariance,
                                                       const double covariance_floor_px2,
                                                       const double fallback_pixel_variance_px2)
{
    if (!std::isfinite(covariance_floor_px2) || covariance_floor_px2 < 0.0 ||
        !std::isfinite(fallback_pixel_variance_px2) || fallback_pixel_variance_px2 <= 0.0) {
        return std::nullopt;
    }

    Eigen::Matrix2d symmetric;
    if (covariance.allFinite()) {
        symmetric = 0.5 * (covariance + covariance.transpose());
    } else {
        symmetric = fallback_pixel_variance_px2 * Eigen::Matrix2d::Identity();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(symmetric);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
        return fallback_pixel_variance_px2 * Eigen::Matrix2d::Identity();
    }

    Eigen::Vector2d eigenvalues = solver.eigenvalues();
    eigenvalues = eigenvalues.cwiseMax(covariance_floor_px2);
    return solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
}

std::optional<Eigen::Matrix3d> sanitizeBearingCovariance(const Eigen::Matrix3d& covariance,
                                                         const Eigen::Vector3d& bearing)
{
    if (!covariance.allFinite() || !bearing.allFinite()) {
        return std::nullopt;
    }

    const Eigen::Matrix3d tangent_projection = Eigen::Matrix3d::Identity() - bearing * bearing.transpose();
    const Eigen::Matrix3d symmetric =
        tangent_projection * (0.5 * (covariance + covariance.transpose())) * tangent_projection;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
        return std::nullopt;
    }

    // First-order bearing uncertainty has one exactly unobservable radial
    // direction. Clamp only negative round-off, retaining that zero eigenvalue.
    const Eigen::Vector3d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
    return solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
}

} // namespace

std::optional<BearingMeasurement> bearingFromPixel(const calibration::ILensModel& lens, const Eigen::Vector2d& pixel,
                                                   const Eigen::Matrix2d& pixel_covariance,
                                                   const double covariance_floor_px2,
                                                   const double fallback_pixel_variance_px2)
{
    if (!pixel.allFinite()) {
        return std::nullopt;
    }

    const auto sanitized_pixel_covariance =
        sanitizePixelCovariance(pixel_covariance, covariance_floor_px2, fallback_pixel_variance_px2);
    if (!sanitized_pixel_covariance) {
        return std::nullopt;
    }

    const Eigen::Vector3d raw_bearing = lens.backProject(pixel);
    const double raw_norm = raw_bearing.norm();
    if (!raw_bearing.allFinite() || !std::isfinite(raw_norm) || raw_norm <= std::numeric_limits<double>::epsilon()) {
        return std::nullopt;
    }

    BearingMeasurement result;
    result.vector = raw_bearing / raw_norm;

    const Eigen::Matrix<double, 3, 2> raw_jacobian = lens.backProjectJacobian(pixel);
    if (!raw_jacobian.allFinite()) {
        return std::nullopt;
    }

    // The lens API already returns d(unit bearing)/d(pixel); differentiating
    // normalization again would incorrectly rescale a numerically non-unit ray.
    const Eigen::Matrix3d propagated = raw_jacobian * *sanitized_pixel_covariance * raw_jacobian.transpose();
    const auto sanitized_bearing_covariance = sanitizeBearingCovariance(propagated, result.vector);
    if (!sanitized_bearing_covariance) {
        return std::nullopt;
    }
    result.covariance = *sanitized_bearing_covariance;
    return result;
}

} // namespace uvdar_core::pose_estimation
