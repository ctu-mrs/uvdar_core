#pragma once
#ifndef GPNP_HPP
#define GPNP_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "uvdar_core/pose_estimation/geometric_solver/generalized_solver_types.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/gp3p.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/gp4_5p.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/gp6p.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

/**
 * @brief Nonlinear generalized PnP for arbitrary calibrated ray origins.
 *
 * The solver dispatches to gP3P, gP4/5P, or the six-point generalized DLT
 * initializer according to correspondence count, then minimizes angular error
 * jointly over all rays. The residual and pose Jacobian are analytic; this is
 * the compact equivalent of OpenGV's non-central initialization plus nonlinear
 * optimization path.
 */
class GPnP {
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
                world_points, ray_origins, ray_directions, 3)) {
            return {};
        }
        if (world_points.cols() == 3) {
            return GP3P::solve(world_points, ray_origins, ray_directions, options);
        }
        if (world_points.cols() <= 5) {
            return GP4_5P::solve(world_points, ray_origins, ray_directions, options);
        }

        std::optional<Solution> seed;
        const std::vector<Solution> linear = GP6P::solve(
            world_points, ray_origins, ray_directions, options);
        if (!linear.empty()) {
            seed = linear.front();
        } else {
            seed = bestMinimalSeed(
                world_points, ray_origins, ray_directions, options);
        }
        if (!seed) {
            return {};
        }
        const auto refined = refine(
            world_points, ray_origins, ray_directions, *seed, options);
        if (!refined
            || !generalized_detail::hasPositiveDepths(
                world_points, ray_origins, ray_directions, *refined)
            || generalized_detail::maximumAngularError(
                   world_points, ray_origins, ray_directions, *refined)
                > std::max(options.maximum_angular_error_rad, 1.0e-6)) {
            return {};
        }
        return {*refined};
    }

    /**
     * @brief Refine a caller-provided body-to-rig pose against every ray.
     */
    static std::optional<Solution> refine(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Solution& seed,
        const Options& options = Options {})
    {
        return generalized_detail::refinePose(
            world_points, ray_origins, ray_directions, seed, options);
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

private:
    static std::optional<Solution> bestMinimalSeed(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Options& options)
    {
        std::optional<Solution> best;
        double best_cost = std::numeric_limits<double>::infinity();
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
                    for (const Solution& candidate : GP3P::solve(
                             subset_world, subset_origins, subset_directions, options)) {
                        const double cost = generalized_detail::angularCost(
                            world_points, ray_origins, ray_directions, candidate);
                        if (std::isfinite(cost) && cost < best_cost) {
                            best = candidate;
                            best_cost = cost;
                        }
                    }
                }
            }
        }
        return best;
    }
};

} // namespace uvdar_core::pose_estimation::geometric_solver

#endif // GPNP_HPP
