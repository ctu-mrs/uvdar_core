#pragma once

#include <Eigen/Dense>

#include "uvdar_core/pose_estimation/types.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

/**
 * @brief Body-to-camera solution returned by all minimal geometric solvers.
 *
 * The algebraic P2P, P3P, and P4P implementations retain R/t notation used
 * in their derivations; the application-facing representation is CameraPose.
 */
struct PoseSolution {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

/**
 * @brief Convert algebraic R/t notation to the shared pose-estimation type.
 */
inline CameraPose toCameraPose(const PoseSolution& solution)
{
    return {solution.R, solution.t};
}

} // namespace uvdar_core::pose_estimation::geometric_solver
