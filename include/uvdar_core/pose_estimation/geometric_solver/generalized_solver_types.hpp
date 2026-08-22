#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "uvdar_core/helpers/math.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/solver_types.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

using GeneralizedPointMatrix = Eigen::Matrix<double, 3, Eigen::Dynamic>;

/**
 * @brief Common numerical controls for generalized absolute-pose solvers.
 */
struct GeneralizedSolverOptions {
    int max_iterations = 40;
    double damping = 1.0e-8;
    double step_tolerance = 1.0e-10;
    double residual_tolerance = 1.0e-10;
    double maximum_angular_error_rad = 0.05;
    int gp3p_depth_seed_levels = 7;
    int gp3p_depth_iterations = 60;
    double gp3p_root_tolerance = 1.0e-9;
};

/**
 * @brief Pose sensitivity to the entries of all generalized bearing vectors.
 */
struct GeneralizedPoseJacobian {
    PoseSolution sol;
    // 6x(3N): [omega(3), translation(3)] wrt raw bearing entries.
    Eigen::Matrix<double, 6, Eigen::Dynamic> dpose_directions;
};

/**
 * @brief Damped inverse sensitivity from a local pose tangent to bearings.
 */
struct GeneralizedBearingJacobian {
    PoseSolution sol;
    // (3N)x6: raw bearing entries wrt [omega(3), translation(3)].
    Eigen::Matrix<double, Eigen::Dynamic, 6> ddirections_dpose;
};

namespace generalized_detail {

inline constexpr double kEpsilon = 1.0e-12;

inline bool validInput(
    const GeneralizedPointMatrix& world_points,
    const GeneralizedPointMatrix& ray_origins,
    const GeneralizedPointMatrix& ray_directions,
    Eigen::Index minimum_count,
    Eigen::Index maximum_count = Eigen::Dynamic)
{
    const Eigen::Index count = world_points.cols();
    if (count < minimum_count
        || (maximum_count != Eigen::Dynamic && count > maximum_count)
        || ray_origins.cols() != count
        || ray_directions.cols() != count
        || !world_points.allFinite()
        || !ray_origins.allFinite()
        || !ray_directions.allFinite()) {
        return false;
    }
    for (Eigen::Index i = 0; i < count; ++i) {
        if (ray_directions.col(i).squaredNorm() <= kEpsilon) {
            return false;
        }
    }
    return true;
}

inline GeneralizedPointMatrix normalizedDirections(const GeneralizedPointMatrix& ray_directions)
{
    GeneralizedPointMatrix output = ray_directions;
    for (Eigen::Index i = 0; i < output.cols(); ++i) {
        output.col(i).normalize();
    }
    return output;
}

inline void applyLeftPoseIncrement(
    PoseSolution& pose,
    const Eigen::Vector3d& rotation_increment,
    const Eigen::Vector3d& translation_increment)
{
    pose.R = uvdar_core::helpers::expSO3(rotation_increment) * pose.R;
    pose.t += translation_increment;
}

inline std::optional<Eigen::VectorXd> angularResidualVector(
    const GeneralizedPointMatrix& world_points,
    const GeneralizedPointMatrix& ray_origins,
    const GeneralizedPointMatrix& ray_directions,
    const PoseSolution& pose)
{
    if (!validInput(world_points, ray_origins, ray_directions, 1)) {
        return std::nullopt;
    }
    Eigen::VectorXd residual(3 * world_points.cols());
    for (Eigen::Index i = 0; i < world_points.cols(); ++i) {
        const Eigen::Vector3d ray_to_point = pose.R * world_points.col(i) + pose.t - ray_origins.col(i);
        const double range = ray_to_point.norm();
        if (!std::isfinite(range) || range <= kEpsilon) {
            return std::nullopt;
        }
        residual.segment<3>(3 * i) = ray_to_point / range - ray_directions.col(i).normalized();
    }
    return residual.allFinite() ? std::optional<Eigen::VectorXd>(std::move(residual)) : std::nullopt;
}

inline std::optional<Eigen::Matrix<double, Eigen::Dynamic, 6>> angularResidualJacobianWrtPose(
    const GeneralizedPointMatrix& world_points,
    const GeneralizedPointMatrix& ray_origins,
    const GeneralizedPointMatrix& ray_directions,
    const PoseSolution& pose)
{
    if (!validInput(world_points, ray_origins, ray_directions, 1)) {
        return std::nullopt;
    }
    Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian(3 * world_points.cols(), 6);
    for (Eigen::Index i = 0; i < world_points.cols(); ++i) {
        const Eigen::Vector3d rotated_point = pose.R * world_points.col(i);
        const Eigen::Vector3d ray_to_point = rotated_point + pose.t - ray_origins.col(i);
        const double range = ray_to_point.norm();
        if (!std::isfinite(range) || range <= kEpsilon) {
            return std::nullopt;
        }
        const Eigen::Vector3d predicted_direction = ray_to_point / range;
        const Eigen::Matrix3d normalization_jacobian =
            (Eigen::Matrix3d::Identity() - predicted_direction * predicted_direction.transpose()) / range;
        jacobian.block<3, 3>(3 * i, 0) = normalization_jacobian * (-uvdar_core::helpers::skew(rotated_point));
        jacobian.block<3, 3>(3 * i, 3) = normalization_jacobian;
    }
    return jacobian.allFinite()
        ? std::optional<Eigen::Matrix<double, Eigen::Dynamic, 6>>(std::move(jacobian))
        : std::nullopt;
}

inline bool hasPositiveDepths(
    const GeneralizedPointMatrix& world_points,
    const GeneralizedPointMatrix& ray_origins,
    const GeneralizedPointMatrix& ray_directions,
    const PoseSolution& pose)
{
    for (Eigen::Index i = 0; i < world_points.cols(); ++i) {
        const Eigen::Vector3d point = pose.R * world_points.col(i) + pose.t;
        if (ray_directions.col(i).normalized().dot(point - ray_origins.col(i)) <= kEpsilon) {
            return false;
        }
    }
    return true;
}

inline double angularCost(
    const GeneralizedPointMatrix& world_points,
    const GeneralizedPointMatrix& ray_origins,
    const GeneralizedPointMatrix& ray_directions,
    const PoseSolution& pose)
{
    const auto residual = angularResidualVector(world_points, ray_origins, ray_directions, pose);
    return residual
        ? residual->squaredNorm() / static_cast<double>(world_points.cols())
        : std::numeric_limits<double>::infinity();
}

inline double maximumAngularError(
    const GeneralizedPointMatrix& world_points,
    const GeneralizedPointMatrix& ray_origins,
    const GeneralizedPointMatrix& ray_directions,
    const PoseSolution& pose)
{
    double maximum_error = 0.0;
    for (Eigen::Index i = 0; i < world_points.cols(); ++i) {
        const Eigen::Vector3d ray_to_point = pose.R * world_points.col(i) + pose.t - ray_origins.col(i);
        if (ray_to_point.squaredNorm() <= kEpsilon) {
            return std::numeric_limits<double>::infinity();
        }
        const double cosine = std::clamp(
            ray_to_point.normalized().dot(ray_directions.col(i).normalized()),
            -1.0,
            1.0);
        maximum_error = std::max(maximum_error, std::acos(cosine));
    }
    return maximum_error;
}

inline std::optional<PoseSolution> refinePose(
    const GeneralizedPointMatrix& world_points,
    const GeneralizedPointMatrix& ray_origins,
    const GeneralizedPointMatrix& ray_directions,
    const PoseSolution& seed,
    const GeneralizedSolverOptions& options)
{
    if (!validInput(world_points, ray_origins, ray_directions, 3)) {
        return std::nullopt;
    }
    PoseSolution pose = seed;
    double damping = std::max(options.damping, 1.0e-15);
    for (int iteration = 0; iteration < std::max(0, options.max_iterations); ++iteration) {
        const auto residual = angularResidualVector(world_points, ray_origins, ray_directions, pose);
        const auto jacobian = angularResidualJacobianWrtPose(world_points, ray_origins, ray_directions, pose);
        if (!residual || !jacobian) {
            return std::nullopt;
        }
        if (residual->norm() / static_cast<double>(world_points.cols())
            < std::max(0.0, options.residual_tolerance)) {
            break;
        }

        Eigen::Matrix<double, 6, 6> normal = jacobian->transpose() * *jacobian;
        normal.diagonal().array() += damping;
        const Eigen::Matrix<double, 6, 1> gradient = jacobian->transpose() * *residual;
        const Eigen::Matrix<double, 6, 1> step = -normal.ldlt().solve(gradient);
        if (!step.allFinite() || step.norm() < std::max(0.0, options.step_tolerance)) {
            break;
        }

        const double base_cost = residual->squaredNorm();
        bool accepted = false;
        double scale = 1.0;
        for (int line_search = 0; line_search < 10; ++line_search) {
            PoseSolution candidate = pose;
            applyLeftPoseIncrement(candidate, scale * step.head<3>(), scale * step.tail<3>());
            const auto candidate_residual = angularResidualVector(
                world_points, ray_origins, ray_directions, candidate);
            if (candidate_residual && candidate_residual->squaredNorm() < base_cost) {
                pose = candidate;
                damping = std::max(options.damping, 0.5 * damping);
                accepted = true;
                break;
            }
            scale *= 0.5;
        }
        if (!accepted) {
            damping *= 10.0;
            if (!std::isfinite(damping) || damping > 1.0e12) {
                break;
            }
        }
    }
    return pose.R.allFinite() && pose.t.allFinite() ? std::optional<PoseSolution>(pose) : std::nullopt;
}

inline std::optional<PoseSolution> absoluteOrientation(
    const GeneralizedPointMatrix& source_points,
    const GeneralizedPointMatrix& destination_points)
{
    if (source_points.cols() < 3
        || source_points.cols() != destination_points.cols()
        || !source_points.allFinite()
        || !destination_points.allFinite()) {
        return std::nullopt;
    }
    const Eigen::Vector3d source_centroid = source_points.rowwise().mean();
    const Eigen::Vector3d destination_centroid = destination_points.rowwise().mean();
    const GeneralizedPointMatrix source_centered = source_points.colwise() - source_centroid;
    const GeneralizedPointMatrix destination_centered = destination_points.colwise() - destination_centroid;
    const Eigen::Matrix3d covariance = source_centered * destination_centered.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if (svd.info() != Eigen::Success || svd.singularValues()(1) <= kEpsilon) {
        return std::nullopt;
    }
    Eigen::Matrix3d sign = Eigen::Matrix3d::Identity();
    if ((svd.matrixV() * svd.matrixU().transpose()).determinant() < 0.0) {
        sign(2, 2) = -1.0;
    }
    PoseSolution pose;
    pose.R = svd.matrixV() * sign * svd.matrixU().transpose();
    pose.t = destination_centroid - pose.R * source_centroid;
    return pose.R.allFinite() && pose.t.allFinite() ? std::optional<PoseSolution>(pose) : std::nullopt;
}

inline double poseDistance(const PoseSolution& first, const PoseSolution& second)
{
    const Eigen::AngleAxisd rotation_delta(first.R * second.R.transpose());
    return (first.t - second.t).norm() + std::abs(rotation_delta.angle());
}

inline void appendDeduplicated(
    std::vector<PoseSolution>& solutions,
    const PoseSolution& candidate,
    double tolerance = 1.0e-5)
{
    const bool duplicate = std::any_of(solutions.begin(), solutions.end(), [&](const PoseSolution& solution) {
        return poseDistance(solution, candidate) < tolerance;
    });
    if (!duplicate) {
        solutions.push_back(candidate);
    }
}

inline std::optional<GeneralizedPoseJacobian> poseJacobianWrtDirections(
    const GeneralizedPointMatrix& world_points,
    const GeneralizedPointMatrix& ray_origins,
    const GeneralizedPointMatrix& ray_directions,
    const PoseSolution& pose,
    double damping)
{
    const auto pose_jacobian = angularResidualJacobianWrtPose(
        world_points, ray_origins, ray_directions, pose);
    if (!pose_jacobian) {
        return std::nullopt;
    }
    Eigen::MatrixXd direction_jacobian = Eigen::MatrixXd::Zero(
        3 * world_points.cols(), 3 * world_points.cols());
    for (Eigen::Index i = 0; i < world_points.cols(); ++i) {
        direction_jacobian.block<3, 3>(3 * i, 3 * i) =
            -uvdar_core::helpers::normalizedVectorJacobian(ray_directions.col(i));
    }

    Eigen::Matrix<double, 6, 6> normal = pose_jacobian->transpose() * *pose_jacobian;
    normal.diagonal().array() += std::max(damping, 1.0e-15);
    GeneralizedPoseJacobian output;
    output.sol = pose;
    output.dpose_directions = -normal.ldlt().solve(pose_jacobian->transpose() * direction_jacobian);
    return output.dpose_directions.allFinite()
        ? std::optional<GeneralizedPoseJacobian>(std::move(output))
        : std::nullopt;
}

inline std::vector<GeneralizedPoseJacobian> poseJacobiansWrtDirections(
    const GeneralizedPointMatrix& world_points,
    const GeneralizedPointMatrix& ray_origins,
    const GeneralizedPointMatrix& ray_directions,
    const std::vector<PoseSolution>& solutions,
    double damping)
{
    std::vector<GeneralizedPoseJacobian> output;
    output.reserve(solutions.size());
    for (const PoseSolution& solution : solutions) {
        if (auto jacobian = poseJacobianWrtDirections(
                world_points, ray_origins, ray_directions, solution, damping)) {
            output.push_back(std::move(*jacobian));
        }
    }
    return output;
}

inline std::vector<GeneralizedBearingJacobian> bearingJacobiansWrtPose(
    const std::vector<GeneralizedPoseJacobian>& pose_jacobians,
    double damping)
{
    std::vector<GeneralizedBearingJacobian> output;
    output.reserve(pose_jacobians.size());
    for (const GeneralizedPoseJacobian& pose_jacobian : pose_jacobians) {
        GeneralizedBearingJacobian inverse;
        inverse.sol = pose_jacobian.sol;
        inverse.ddirections_dpose = uvdar_core::helpers::dampedRightPseudoInverse(
            pose_jacobian.dpose_directions,
            std::max(damping, 1.0e-15));
        if (inverse.ddirections_dpose.allFinite()) {
            output.push_back(std::move(inverse));
        }
    }
    return output;
}

} // namespace generalized_detail

} // namespace uvdar_core::pose_estimation::geometric_solver
