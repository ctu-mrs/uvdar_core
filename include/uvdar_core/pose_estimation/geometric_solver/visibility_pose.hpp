#pragma once

#include <vector>

#include <Eigen/Dense>

#include "uvdar_core/pose_estimation/types.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

/**
 * @brief Numerical integration controls for the analytic visibility manifold.
 *
 * Two bearings and their known marker separation leave two degrees of freedom:
 * position along the exact two-ray distance curve and rotation about the
 * marker baseline.  Visibility clips the latter to analytic circular arcs.
 * The remaining one-dimensional integrations use Gauss-Legendre quadrature.
 */
struct VisibilityPoseConfig {
    double visibility_half_angle_rad = 0.6981317007977318; // 40 degrees
    double minimum_bearing_separation_rad = 1.0e-4;
    double mode_gap_rad = 0.7853981633974483; // 45 degrees
    int depth_quadrature_order = 64;
    int spin_quadrature_order = 12;
};

/**
 * @brief Gaussian summary of one visibility-connected pose mode.
 */
struct VisibilityPoseEstimate {
    CameraPose pose;
    Eigen::Matrix<double, 6, 6> covariance =
        Eigen::Matrix<double, 6, 6>::Zero();
    /** Fraction of the total feasible arc-length/spin measure. */
    double probability = 0.0;
};

/**
 * @brief Estimate the visibility-constrained two-marker pose distribution.
 *
 * The returned distribution is uniform in baseline spin and in arc length
 * along the exact positive-depth two-ray distance curve.  That measure is
 * invariant to correspondence order.  LED normals and bearings are expressed
 * in the body and camera frames respectively; a normal is visible when its
 * angle to the direction from the marker to the camera does not exceed the
 * configured half-angle.
 */
class VisibilityPoseSolver {
public:
    static std::vector<VisibilityPoseEstimate> solve(
        const Eigen::Vector3d& first_body_point,
        const Eigen::Vector3d& second_body_point,
        const Eigen::Vector3d& first_body_normal,
        const Eigen::Vector3d& second_body_normal,
        const Eigen::Vector3d& first_camera_bearing,
        const Eigen::Vector3d& second_camera_bearing,
        const VisibilityPoseConfig& config = {});
};

} // namespace uvdar_core::pose_estimation::geometric_solver
