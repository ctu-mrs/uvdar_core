#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace uvdar_core::pose_estimation {

/**
 * @brief Return x^2 without repeating value * value at call sites.
 */
template <typename Scalar>
constexpr Scalar squared(Scalar value)
{
    return value * value;
}

/**
 * @brief Skew-symmetric matrix [v]x satisfying [v]x w = v x w.
 */
inline Eigen::Matrix3d skew(const Eigen::Vector3d& value)
{
    Eigen::Matrix3d output;
    output << 0.0, -value.z(), value.y(),
        value.z(), 0.0, -value.x(),
        -value.y(), value.x(), 0.0;
    return output;
}

/**
 * @brief Jacobian of unit-vector normalization d(v / ||v||) / dv.
 */
inline Eigen::Matrix3d normalizedVectorJacobian(const Eigen::Vector3d& vector, double epsilon = 1.0e-12)
{
    const double norm = vector.norm();
    if (norm < epsilon) {
        return Eigen::Matrix3d::Zero();
    }
    return Eigen::Matrix3d::Identity() / norm - (vector * vector.transpose()) / (norm * norm * norm);
}

/**
 * @brief Exponential map from an so(3) vector to SO(3) using Rodrigues' formula.
 */
inline Eigen::Matrix3d expSO3(const Eigen::Vector3d& omega, double epsilon = 1.0e-12)
{
    const double angle = omega.norm();
    if (angle < epsilon) {
        return Eigen::Matrix3d::Identity() + skew(omega);
    }
    return Eigen::AngleAxisd(angle, omega / angle).toRotationMatrix();
}

/**
 * @brief Minimal rotation that maps one nonzero vector direction to another.
 */
inline Eigen::Matrix3d rotationBetween(const Eigen::Vector3d& from, const Eigen::Vector3d& to, double epsilon = 1.0e-12)
{
    const double from_norm = from.norm();
    const double to_norm = to.norm();
    if (from_norm < epsilon || to_norm < epsilon) {
        return Eigen::Matrix3d::Identity();
    }

    const Eigen::Vector3d from_unit = from / from_norm;
    const Eigen::Vector3d to_unit = to / to_norm;
    const double cosine = std::clamp(from_unit.dot(to_unit), -1.0, 1.0);
    if (cosine > 1.0 - epsilon) {
        return Eigen::Matrix3d::Identity();
    }

    if (cosine < -1.0 + epsilon) {
        Eigen::Vector3d perpendicular = Eigen::Vector3d::UnitX();
        if (std::abs(from_unit.dot(perpendicular)) > 0.9) {
            perpendicular = Eigen::Vector3d::UnitY();
        }
        Eigen::Vector3d axis = from_unit.cross(perpendicular);
        const double axis_norm = axis.norm();
        if (axis_norm < epsilon) {
            return Eigen::Matrix3d::Identity();
        }
        axis /= axis_norm;
        return 2.0 * (axis * axis.transpose()) - Eigen::Matrix3d::Identity();
    }

    const Eigen::Vector3d axis_cross_sine = from_unit.cross(to_unit);
    const Eigen::Matrix3d cross_matrix = skew(axis_cross_sine);
    return Eigen::Matrix3d::Identity() + cross_matrix + (cross_matrix * cross_matrix) / (1.0 + cosine);
}

/**
 * @brief Rotation matrix for a yaw angle about the z axis.
 */
inline Eigen::Matrix3d rotationZ(double angle)
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    Eigen::Matrix3d rotation;
    rotation << cosine, -sine, 0.0,
        sine, cosine, 0.0,
        0.0, 0.0, 1.0;
    return rotation;
}

/**
 * @brief Derivative dRz(angle)/dangle.
 */
inline Eigen::Matrix3d rotationZDerivative(double angle)
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    Eigen::Matrix3d derivative;
    derivative << -sine, -cosine, 0.0,
        cosine, -sine, 0.0,
        0.0, 0.0, 0.0;
    return derivative;
}

/**
 * @brief Convert dR into a local angular velocity using vee(R^T dR).
 */
inline Eigen::Vector3d omegaFromRotationDerivative(const Eigen::Matrix3d& rotation, const Eigen::Matrix3d& rotation_derivative)
{
    const Eigen::Matrix3d skew_part = rotation.transpose() * rotation_derivative
        - rotation_derivative.transpose() * rotation;
    return 0.5 * Eigen::Vector3d(skew_part(2, 1), skew_part(0, 2), skew_part(1, 0));
}

/**
 * @brief Convert quaternion to fixed-axis roll-pitch-yaw angles.
 */
inline Eigen::Vector3d quaternionToRpy(const Eigen::Quaterniond& quaternion)
{
    const Eigen::Matrix3d matrix = quaternion.toRotationMatrix();
    return {
        std::atan2(matrix(2, 1), matrix(2, 2)),
        std::atan2(-matrix(2, 0), std::sqrt(matrix(2, 1) * matrix(2, 1) + matrix(2, 2) * matrix(2, 2))),
        std::atan2(matrix(1, 0), matrix(0, 0)),
    };
}

/**
 * @brief Convert fixed-axis roll-pitch-yaw angles to a quaternion.
 */
inline Eigen::Quaterniond rpyToQuaternion(const Eigen::Vector3d& rpy)
{
    return Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ());
}

/**
 * @brief Damped right pseudo-inverse J^T (J J^T + lambda I)^-1.
 */
template <int Rows, int Cols>
Eigen::Matrix<double, Cols, Rows> dampedRightPseudoInverse(
    const Eigen::Matrix<double, Rows, Cols>& matrix,
    double damping = 1.0e-12)
{
    Eigen::Matrix<double, Rows, Rows> normal = matrix * matrix.transpose();
    double lambda = damping;
    if (std::isfinite(normal.trace())) {
        lambda = std::max(1.0e-14, damping * normal.trace() / static_cast<double>(Rows));
    }
    normal.diagonal().array() += lambda;
    return matrix.transpose() * normal.ldlt().solve(Eigen::Matrix<double, Rows, Rows>::Identity());
}

} // namespace uvdar_core::pose_estimation
