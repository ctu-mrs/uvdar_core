#pragma once
#ifndef GP3P_HPP
#define GP3P_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "uvdar_core/helpers/polynomial.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/generalized_solver_types.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/p3p.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

/**
 * @brief Minimal generalized P3P solver for three arbitrary calibrated rays.
 *
 * The formulation follows Nister and Stewenius, "A Minimal Solution to the
 * Generalised 3-Point Pose Problem" (JMIV 2007): solve the three pairwise
 * distance quadrics for the ray depths, then recover R,t by absolute
 * orientation. Two depth variables are eliminated with a quadratic resultant,
 * producing the paper's degree-eight polynomial without a generated Groebner
 * template. Real positive roots are polished against all three quadrics.
 */
class GP3P {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Solution = PoseSolution;
    using PointMatrix = GeneralizedPointMatrix;
    using Options = GeneralizedSolverOptions;
    using PoseJacobian = GeneralizedPoseJacobian;
    using BearingJacobian = GeneralizedBearingJacobian;

    /**
     * @brief Solve p_i + lambda_i d_i = R X_i + t for three rays.
     */
    static std::vector<Solution> solve(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Options& options = Options {})
    {
        if (!generalized_detail::validInput(
                world_points, ray_origins, ray_directions, 3, 3)
            || !hasObservableTriangle(world_points)) {
            return {};
        }
        const PointMatrix directions = generalized_detail::normalizedDirections(ray_directions);
        std::vector<Solution> solutions;

        // The pairwise distance constraints are three quadrics in the three
        // ray depths. Their resultant is an octic in the first depth.
        for (const Eigen::Vector3d& depths : algebraicDepthCandidates(
                 world_points, ray_origins, directions)) {
            tryDepthSeed(
                world_points, ray_origins, directions, depths, options, solutions);
        }

        // Retain a deterministic numerical fallback for nearly singular
        // resultants (parallel rays and roots clustered at machine precision).
        // A concurrent-ray P3P approximation supplies geometry-aware Newton
        // starts and is inexpensive compared with the full depth grid.
        const Eigen::Matrix3d central_world = world_points;
        const Eigen::Matrix3d central_directions = directions;
        for (const P3P::Solution& central : P3P::solve(
                 central_world, central_directions, 1)) {
            Eigen::Vector3d depths;
            for (int i = 0; i < 3; ++i) {
                depths(i) = directions.col(i).dot(
                    central.R * world_points.col(i) + central.t
                    - ray_origins.col(i));
            }
            tryDepthSeed(
                world_points, ray_origins, directions, depths, options, solutions);
        }
        const std::vector<double> levels = depthLevels(world_points, ray_origins, options);
        for (const double depth : levels) {
            tryDepthSeed(
                world_points,
                ray_origins,
                directions,
                Eigen::Vector3d::Constant(depth),
                options,
                solutions);
        }
        if (solutions.empty()) {
            for (double first : levels) {
                for (double second : levels) {
                    for (double third : levels) {
                        tryDepthSeed(
                            world_points,
                            ray_origins,
                            directions,
                            Eigen::Vector3d(first, second, third),
                            options,
                            solutions);
                    }
                }
            }
        }

        std::sort(solutions.begin(), solutions.end(), [&](const Solution& first, const Solution& second) {
            return generalized_detail::angularCost(
                       world_points, ray_origins, directions, first)
                < generalized_detail::angularCost(
                       world_points, ray_origins, directions, second);
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

private:
    using Polynomial = uvdar_core::helpers::Polynomial;

    static bool hasObservableTriangle(const PointMatrix& points)
    {
        const Eigen::Vector3d first = points.col(1) - points.col(0);
        const Eigen::Vector3d second = points.col(2) - points.col(0);
        const double scale = std::max({
            first.squaredNorm(),
            second.squaredNorm(),
            (points.col(2) - points.col(1)).squaredNorm(),
        });
        return scale > generalized_detail::kEpsilon
            && first.cross(second).norm() > 1.0e-8 * scale;
    }

    static std::vector<double> depthLevels(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const Options& options)
    {
        double scale = 1.0;
        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                scale = std::max(scale, (world_points.col(i) - world_points.col(j)).norm());
                scale = std::max(scale, (ray_origins.col(i) - ray_origins.col(j)).norm());
            }
        }
        const int count = std::clamp(options.gp3p_depth_seed_levels, 3, 9);
        std::vector<double> levels;
        levels.reserve(static_cast<std::size_t>(count));
        constexpr int shallow_side_levels = 2;
        for (int i = 0; i < count; ++i) {
            levels.push_back(scale * std::pow(2.0, static_cast<double>(i - shallow_side_levels)));
        }
        return levels;
    }

    static Polynomial ascending(
        std::initializer_list<Polynomial::Complex> coefficients)
    {
        return Polynomial::fromAscending(
            coefficients.begin(), coefficients.end());
    }

    static int permutationSign(const std::array<int, 4>& permutation)
    {
        int inversions = 0;
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                inversions += permutation[static_cast<std::size_t>(i)]
                    > permutation[static_cast<std::size_t>(j)];
            }
        }
        return inversions % 2 == 0 ? 1 : -1;
    }

    static Polynomial quadraticResultant(
        const Polynomial& first_linear,
        const Polynomial& first_constant,
        const Polynomial& second_quadratic,
        const Polynomial& second_linear,
        const Polynomial& second_constant)
    {
        const Polynomial zero {0.0};
        const Polynomial one {1.0};
        const std::array<std::array<Polynomial, 4>, 4> sylvester {{
            {{one, first_linear, first_constant, zero}},
            {{zero, one, first_linear, first_constant}},
            {{second_quadratic, second_linear, second_constant, zero}},
            {{zero, second_quadratic, second_linear, second_constant}},
        }};

        Polynomial determinant {0.0};
        std::array<int, 4> permutation {0, 1, 2, 3};
        do {
            Polynomial term {1.0};
            for (int row = 0; row < 4; ++row) {
                term *=
                    sylvester[static_cast<std::size_t>(row)]
                             [static_cast<std::size_t>(permutation[static_cast<std::size_t>(row)])];
            }
            determinant += term * static_cast<double>(
                permutationSign(permutation));
        } while (std::next_permutation(permutation.begin(), permutation.end()));
        return determinant;
    }

    static std::vector<double> realPolynomialRoots(
        const Polynomial& polynomial)
    {
        double scale = 0.0;
        for (const Polynomial::Complex coefficient
             : polynomial.coefficients()) {
            scale = std::max(scale, std::abs(coefficient));
        }
        return polynomial
            .stripLeadingZeros(std::max(1.0e-14, 1.0e-12 * scale))
            .realRoots(1.0e-7, true);
    }

    static std::vector<double> quadraticRoots(const double linear, const double constant)
    {
        return uvdar_core::helpers::realQuadraticRoots(
            1.0, linear, constant, 1.0e-10);
    }

    static std::vector<Eigen::Vector3d> algebraicDepthCandidates(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& directions)
    {
        const Eigen::Vector3d delta01 = ray_origins.col(0) - ray_origins.col(1);
        const Eigen::Vector3d delta02 = ray_origins.col(0) - ray_origins.col(2);
        const Eigen::Vector3d delta12 = ray_origins.col(1) - ray_origins.col(2);
        const double k01 = delta01.squaredNorm()
            - (world_points.col(0) - world_points.col(1)).squaredNorm();
        const double k02 = delta02.squaredNorm()
            - (world_points.col(0) - world_points.col(2)).squaredNorm();
        const double k12 = delta12.squaredNorm()
            - (world_points.col(1) - world_points.col(2)).squaredNorm();

        // F01 = b^2 + p(a)b + q(a), F02 = c^2 + r(a)c + s(a).
        const Polynomial p = ascending({
            -2.0 * delta01.dot(directions.col(1)),
            -2.0 * directions.col(0).dot(directions.col(1)),
        });
        const Polynomial q = ascending({
            k01,
            2.0 * delta01.dot(directions.col(0)),
            1.0,
        });
        const Polynomial r = ascending({
            -2.0 * delta02.dot(directions.col(2)),
            -2.0 * directions.col(0).dot(directions.col(2)),
        });
        const Polynomial s = ascending({
            k02,
            2.0 * delta02.dot(directions.col(0)),
            1.0,
        });

        // Reduce F12 with F01/F02 to A*b*c + B*b + C*c + D = 0.
        const double a_coefficient = -2.0 * directions.col(1).dot(directions.col(2));
        const Polynomial b = ascending({
            2.0 * delta12.dot(directions.col(1))}) - p;
        const Polynomial c = ascending({
            -2.0 * delta12.dot(directions.col(2))}) - r;
        const Polynomial d = ascending({k12}) - q - s;

        // Eliminating c from its quadratic and (A*b+C)c+(B*b+D)=0
        // produces G = g2(a)b^2 + g1(a)b + g0(a).
        const Polynomial g2 = b * b
            - r * b * a_coefficient
            + s * (a_coefficient * a_coefficient);
        const Polynomial g1 = b * d * 2.0
            - r * (b * c + d * a_coefficient)
            + s * c * (2.0 * a_coefficient);
        const Polynomial g0 = d * d - r * d * c + s * c * c;

        const Polynomial resultant = quadraticResultant(p, q, g2, g1, g0);
        std::vector<Eigen::Vector3d> candidates;
        for (const double first_depth : realPolynomialRoots(resultant)) {
            const std::vector<double> second_depths = quadraticRoots(
                p.evaluate(first_depth).real(),
                q.evaluate(first_depth).real());
            const std::vector<double> third_depths = quadraticRoots(
                r.evaluate(first_depth).real(),
                s.evaluate(first_depth).real());
            for (const double second_depth : second_depths) {
                for (const double third_depth : third_depths) {
                    const Eigen::Vector3d depths(
                        first_depth, second_depth, third_depth);
                    if (depths.allFinite()
                        && (depths.array() > generalized_detail::kEpsilon).all()) {
                        candidates.push_back(depths);
                    }
                }
            }
        }
        return candidates;
    }

    static Eigen::Vector3d depthResidual(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& directions,
        const Eigen::Vector3d& depths)
    {
        const std::array<std::pair<int, int>, 3> pairs {{{0, 1}, {0, 2}, {1, 2}}};
        Eigen::Vector3d residual;
        for (int row = 0; row < 3; ++row) {
            const auto [first, second] = pairs[static_cast<std::size_t>(row)];
            const Eigen::Vector3d rig_difference =
                ray_origins.col(first) + depths(first) * directions.col(first)
                - ray_origins.col(second) - depths(second) * directions.col(second);
            residual(row) = rig_difference.squaredNorm()
                - (world_points.col(first) - world_points.col(second)).squaredNorm();
        }
        return residual;
    }

    static Eigen::Matrix3d depthJacobian(
        const PointMatrix& ray_origins,
        const PointMatrix& directions,
        const Eigen::Vector3d& depths)
    {
        const std::array<std::pair<int, int>, 3> pairs {{{0, 1}, {0, 2}, {1, 2}}};
        Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
        for (int row = 0; row < 3; ++row) {
            const auto [first, second] = pairs[static_cast<std::size_t>(row)];
            const Eigen::Vector3d difference =
                ray_origins.col(first) + depths(first) * directions.col(first)
                - ray_origins.col(second) - depths(second) * directions.col(second);
            jacobian(row, first) = 2.0 * directions.col(first).dot(difference);
            jacobian(row, second) = -2.0 * directions.col(second).dot(difference);
        }
        return jacobian;
    }

    static void tryDepthSeed(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& directions,
        Eigen::Vector3d depths,
        const Options& options,
        std::vector<Solution>& solutions)
    {
        if (!depths.allFinite() || (depths.array() <= generalized_detail::kEpsilon).any()) {
            return;
        }
        double geometry_scale_squared = 1.0;
        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                geometry_scale_squared = std::max(
                    geometry_scale_squared,
                    (world_points.col(i) - world_points.col(j)).squaredNorm());
            }
        }
        const double root_tolerance = std::max(options.gp3p_root_tolerance, 1.0e-12)
            * geometry_scale_squared;

        uvdar_core::helpers::LevenbergMarquardtOptions lm_options;
        lm_options.max_iterations =
            std::max(1, options.gp3p_depth_iterations);
        lm_options.initial_damping = std::max(options.damping, 1.0e-12);
        lm_options.step_tolerance =
            std::max(options.step_tolerance, 1.0e-12);
        lm_options.residual_tolerance = root_tolerance / std::sqrt(3.0);
        const auto refined = uvdar_core::helpers::levenbergMarquardt(
            depths,
            Eigen::Index {3},
            [&](const Eigen::Vector3d& candidate, const bool) {
                if (!candidate.allFinite()
                    || !(candidate.array()
                            > generalized_detail::kEpsilon).all()) {
                    return std::optional<
                        uvdar_core::helpers::LeastSquaresLinearization> {};
                }
                uvdar_core::helpers::LeastSquaresLinearization output;
                output.residual = depthResidual(
                    world_points, ray_origins, directions, candidate);
                output.jacobian = depthJacobian(
                    ray_origins, directions, candidate);
                return std::optional<
                    uvdar_core::helpers::LeastSquaresLinearization>(
                        std::move(output));
            },
            [](const Eigen::Vector3d& state, const Eigen::VectorXd& step) {
                return state + step.head<3>();
            },
            lm_options);
        depths = refined.state;

        if (depthResidual(world_points, ray_origins, directions, depths).norm() >= root_tolerance
            || (depths.array() <= generalized_detail::kEpsilon).any()) {
            return;
        }
        PointMatrix rig_points(3, 3);
        for (int i = 0; i < 3; ++i) {
            rig_points.col(i) = ray_origins.col(i) + depths(i) * directions.col(i);
        }
        const auto pose = generalized_detail::absoluteOrientation(world_points, rig_points);
        if (!pose
            || !generalized_detail::hasPositiveDepths(
                world_points, ray_origins, directions, *pose)
            || generalized_detail::maximumAngularError(
                   world_points, ray_origins, directions, *pose)
                > std::max(options.maximum_angular_error_rad, 1.0e-6)) {
            return;
        }
        generalized_detail::appendDeduplicated(solutions, *pose);
    }
};

} // namespace uvdar_core::pose_estimation::geometric_solver

#endif // GP3P_HPP
