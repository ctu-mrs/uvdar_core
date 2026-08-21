#pragma once

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "uvdar_core/pose_estimation/camera_model.hpp"
#include "uvdar_core/pose_estimation/types.hpp"
#include "uvdar_core/helpers/math.hpp"

namespace uvdar_core::pose_estimation::uncertainty {

using PoseTangent = Eigen::Matrix<double, 6, 1>;
using PoseCovariance = Eigen::Matrix<double, 6, 6>;

/**
 * @brief Express @p candidate relative to @p base in [translation, rotation] tangent order.
 */
PoseTangent relativePoseTangent(const CameraPose& base, const CameraPose& candidate);

/**
 * @brief Calculate covariance from tangent-space samples with caller-supplied scaling.
 */
PoseCovariance covarianceFromPoseSamples(const std::vector<PoseTangent>& samples, double scale);

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
PoseCovariance covarianceFromInformation(const PoseCovariance& information, double eps = 1.0e-12);

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
PoseCovariance poseInformationMatrixFromPixelsLinearized(
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
PoseCovariance poseCovarianceFromPixelsLinearized(
    const CameraPose& pose,
    const std::vector<Eigen::Vector3d>& world_points,
    const std::vector<Eigen::Matrix2d>& pixel_covariances,
    const CameraModel& camera,
    double covariance_regularization,
    double eps = 1.0e-12);

} // namespace uvdar_core::pose_estimation::uncertainty
