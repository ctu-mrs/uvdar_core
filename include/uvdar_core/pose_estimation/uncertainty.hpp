#pragma once

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "uvdar_core/pose_estimation/camera_model.hpp"

namespace uvdar_core::pose_estimation::uncertainty {

/**
 * @brief Body-to-camera pose used by local solver and uncertainty routines.
 *
 * It represents X_c = R X_b + t.
 */
struct CameraPose {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

/**
 * @brief Skew-symmetric matrix [v]x.
 *
 * Satisfies [v]x w = v x w and appears in SO(3) Jacobians.
 */
Eigen::Matrix3d skew(const Eigen::Vector3d& value);

/**
 * @brief Exponential map from an so(3) vector to SO(3).
 *
 * Uses Rodrigues' formula, with a first-order small-angle branch.
 */
Eigen::Matrix3d expSO3(const Eigen::Vector3d& omega);

/**
 * @brief Symmetrize and regularize a 2D covariance.
 *
 * Computes 0.5(P + P^T) + lambda I so pixel covariance can be inverted safely.
 */
Eigen::Matrix2d regularizedCovariance(const Eigen::Matrix2d& covariance, double regularization);

/**
 * @brief Convert a symmetric information matrix to covariance by pseudo-inverse.
 *
 * Eigenvalues below tolerance are treated as unobservable pose directions.
 */
Eigen::Matrix<double, 6, 6> covarianceFromInformation(const Eigen::Matrix<double, 6, 6>& information, double eps = 1.0e-12);

/**
 * @brief Projection Jacobian d(pixel residual)/d([translation, rotation]).
 *
 * The tangent order is position first, orientation second, matching the pose
 * covariance layout published by the particle-filter backend. The formula is
 * J = d pi(X_c)/d X_c * [I, -[X_c]x].
 */
Eigen::Matrix<double, 2, 6> imageProjectionJacobian(
    const CameraModel& camera,
    const CameraPose& pose,
    const Eigen::Vector3d& world_point);

/**
 * @brief Build a local pose information matrix from pixel Gaussian summaries.
 *
 * Linearizes all residuals and accumulates Lambda = sum(J_i^T R_i^-1 J_i).
 */
Eigen::Matrix<double, 6, 6> poseInformationMatrixFromPixelsLinearized(
    const CameraPose& pose,
    const std::vector<Eigen::Vector3d>& world_points,
    const std::vector<Eigen::Matrix2d>& pixel_covariances,
    const CameraModel& camera,
    double covariance_regularization);

/**
 * @brief Convert local pixel covariance summaries into 6D pose covariance.
 *
 * Computes the pseudo-inverse of the linearized information matrix.
 */
Eigen::Matrix<double, 6, 6> poseCovarianceFromPixelsLinearized(
    const CameraPose& pose,
    const std::vector<Eigen::Vector3d>& world_points,
    const std::vector<Eigen::Matrix2d>& pixel_covariances,
    const CameraModel& camera,
    double covariance_regularization,
    double eps = 1.0e-12);

} // namespace uvdar_core::pose_estimation::uncertainty
