#pragma once
#ifndef GP4_5P_HPP
#define GP4_5P_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "uvdar_core/pose_estimation/geometric_solver/generalized_solver_types.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/gp3p.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

/**
 * @brief Generalized four/five-point pose by gP3P branch disambiguation.
 *
 * Four and five non-central point-ray correspondences are overdetermined but
 * below the six-correspondence threshold used by linear generalized PnP.
 * This solver enumerates gP3P candidates from all triplets and refines each on
 * every ray.  Every distinct branch satisfying the configured angular gate is
 * returned, ordered by residual, because application-level constraints such
 * as directional LED visibility can disambiguate exact algebraic branches
 * more reliably than a sub-pixel residual tie. This mirrors the hypothesize/
 * test use prescribed for gP3P without importing a generated UPnP template.
 */
class GP4_5P {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Solution = PoseSolution;
    using PointMatrix = GeneralizedPointMatrix;
    using Options = GeneralizedSolverOptions;
    using PoseJacobian = GeneralizedPoseJacobian;
    using BearingJacobian = GeneralizedBearingJacobian;

    static std::vector<Solution> solve(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Options& options = Options {})
    {
        if (!generalized_detail::validInput(
                world_points, ray_origins, ray_directions, 4, 5)) {
            return {};
        }

        std::vector<Solution> solutions;
        const Eigen::Index count = world_points.cols();
        for (Eigen::Index first = 0; first < count - 2; ++first) {
            for (Eigen::Index second = first + 1; second < count - 1; ++second) {
                for (Eigen::Index third = second + 1; third < count; ++third) {
                    const std::array<Eigen::Index, 3> indices {first, second, third};
                    PointMatrix subset_world(3, 3);
                    PointMatrix subset_origins(3, 3);
                    PointMatrix subset_directions(3, 3);
                    for (Eigen::Index i = 0; i < 3; ++i) {
                        subset_world.col(i) = world_points.col(indices[static_cast<std::size_t>(i)]);
                        subset_origins.col(i) = ray_origins.col(indices[static_cast<std::size_t>(i)]);
                        subset_directions.col(i) = ray_directions.col(indices[static_cast<std::size_t>(i)]);
                    }
                    for (const Solution& seed : GP3P::solve(
                             subset_world, subset_origins, subset_directions, options)) {
                        const auto refined = generalized_detail::refinePose(
                            world_points, ray_origins, ray_directions, seed, options);
                        if (!refined
                            || !generalized_detail::hasPositiveDepths(
                                world_points, ray_origins, ray_directions, *refined)
                            || generalized_detail::maximumAngularError(
                                   world_points, ray_origins, ray_directions, *refined)
                                > std::max(options.maximum_angular_error_rad, 1.0e-6)) {
                            continue;
                        }
                        generalized_detail::appendDeduplicated(solutions, *refined);
                    }
                }
            }
        }
        std::sort(solutions.begin(), solutions.end(), [&](const Solution& first, const Solution& second) {
            return generalized_detail::angularCost(
                       world_points, ray_origins, ray_directions, first)
                < generalized_detail::angularCost(
                       world_points, ray_origins, ray_directions, second);
        });
        return solutions;
    }

    static std::vector<PoseJacobian> jacobianPoseWrtBearings(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Options& options = Options {})
    {
        return generalized_detail::poseJacobiansWrtDirections(
            world_points,
            ray_origins,
            ray_directions,
            solve(world_points, ray_origins, ray_directions, options),
            options.damping);
    }

    static std::vector<BearingJacobian> jacobianBearingsWrtPose(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Options& options = Options {})
    {
        return generalized_detail::bearingJacobiansWrtPose(
            jacobianPoseWrtBearings(
                world_points, ray_origins, ray_directions, options),
            options.damping);
    }
};

} // namespace uvdar_core::pose_estimation::geometric_solver

#endif // GP4_5P_HPP
