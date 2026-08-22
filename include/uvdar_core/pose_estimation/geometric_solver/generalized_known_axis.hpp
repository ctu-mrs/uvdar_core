#pragma once
#ifndef GENERALIZED_KNOWN_AXIS_HPP
#define GENERALIZED_KNOWN_AXIS_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "uvdar_core/helpers/math.hpp"
#include "uvdar_core/helpers/polynomial.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/generalized_solver_types.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

/**
 * @brief Generalized absolute pose with a known body-to-rig axis direction.
 *
 * Rays follow the non-central model
 *
 *   q_i + lambda_i p_i = R P_i + t,
 *
 * where q_i is the camera center in the rig/output frame.  The additional
 * constraint R v_model = v_output leaves translation and one rotation angle
 * as the four unknowns.  The two-ray initializers implement the generalized
 * Li and Sweeney equations; the non-minimal path also triangulates repeated
 * physical markers and jointly refines every ray.  Keeping this class separate
 * from P2P is intentional: P2P is the central-camera solver with q_i = 0.
 */
class GeneralizedKnownAxis {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Solution = PoseSolution;
    using PointMatrix = GeneralizedPointMatrix;

    enum class Method {
        Sweeney = 0,
        Li = 1,
    };

    struct PoseJacobian {
        Solution sol;
        // 6x(3N): [omega(3), translation(3)] wrt raw ray directions.
        Eigen::Matrix<double, 6, Eigen::Dynamic> dpose_directions;
    };

    struct BearingJacobian {
        Solution sol;
        // (3N)x6 damped inverse sensitivity.
        Eigen::Matrix<double, Eigen::Dynamic, 6> ddirections_dpose;
    };

    /** @brief Generalized minimal two-ray initializer from Li or Sweeney. */
    static std::vector<Solution> solveTwoPoint(
        const Eigen::Matrix<double, 3, 2>& world_points,
        const Eigen::Matrix<double, 3, 2>& ray_origins,
        const Eigen::Matrix<double, 3, 2>& ray_directions,
        const Eigen::Vector3d& output_axis,
        const Eigen::Vector3d& model_axis,
        Method method = Method::Li)
    {
        const Prealigned prep = prealign(
            world_points, ray_origins, ray_directions,
            output_axis, model_axis);
        if (!prep.ok) {
            return {};
        }
        return method == Method::Sweeney
            ? solveTwoPointSweeney(prep)
            : solveTwoPointLi(prep);
    }

    /**
     * @brief Close an otherwise underconstrained rig geometry with the known axis.
     *
     * The input may contain an arbitrary number of camera rays, but this path
     * is intended only for fewer than three non-collinear physical markers.
     * Distinct two-ray subsets provide algebraic seeds. Repeated observations
     * of the same markers additionally provide a triangulation seed; this is
     * what makes two LEDs seen in multiple cameras well constrained even when
     * every individual central P2P instance is at its horizon singularity.
     * Observable non-collinear geometries remain the responsibility of the
     * purely visual generalized solvers.
     */
    static std::vector<Solution> solve(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Eigen::Vector3d& output_axis,
        const Eigen::Vector3d& model_axis,
        const GeneralizedSolverOptions& options = GeneralizedSolverOptions {})
    {
        if (!generalized_detail::validInput(
                world_points, ray_origins, ray_directions, 2)
            || !validAxis(output_axis) || !validAxis(model_axis)
            || !hasYawObservableMarkerPair(world_points, model_axis)) {
            return {};
        }

        const PointMatrix directions =
            generalized_detail::normalizedDirections(ray_directions);
        std::vector<Solution> seeds;
        const auto triangulated_seed = triangulatedMarkerSeed(
            world_points, ray_origins, directions,
            output_axis, model_axis);
        if (triangulated_seed) {
            seeds.push_back(*triangulated_seed);
        } else {
            // A triangulation seed already incorporates every camera and is
            // preferable when repeated markers are available. Minimal subsets
            // are needed only when fewer than two markers can be triangulated.
            for (Eigen::Index first = 0;
                 first + 1 < world_points.cols(); ++first) {
                for (Eigen::Index second = first + 1;
                     second < world_points.cols(); ++second) {
                    if ((world_points.col(first)
                            - world_points.col(second)).norm()
                        <= kSamePointTolerance) {
                        continue;
                    }
                    Eigen::Matrix<double, 3, 2> pair_points;
                    Eigen::Matrix<double, 3, 2> pair_origins;
                    Eigen::Matrix<double, 3, 2> pair_directions;
                    pair_points << world_points.col(first),
                        world_points.col(second);
                    pair_origins << ray_origins.col(first),
                        ray_origins.col(second);
                    pair_directions << directions.col(first),
                        directions.col(second);
                    for (const Solution& seed : solveTwoPoint(
                             pair_points, pair_origins, pair_directions,
                             output_axis, model_axis, Method::Li)) {
                        generalized_detail::appendDeduplicated(
                            seeds, seed, 1.0e-7);
                    }
                }
            }
            if (world_points.cols() >= 3) {
                for (const Solution& seed : yawScanSeeds(
                         world_points,
                         ray_origins,
                         directions,
                         output_axis,
                         model_axis)) {
                    generalized_detail::appendDeduplicated(
                        seeds, seed, 1.0e-7);
                }
            }
        }

        std::vector<Solution> solutions;
        for (const Solution& seed : seeds) {
            const auto candidate = refine(
                world_points, ray_origins, directions, seed,
                output_axis, model_axis, options);
            if (!candidate
                || !generalized_detail::hasPositiveDepths(
                    world_points, ray_origins, directions, *candidate)
                || !isLocallyObservable(
                    world_points,
                    ray_origins,
                    directions,
                    *candidate,
                    output_axis.normalized())
                || generalized_detail::maximumAngularError(
                       world_points, ray_origins, directions, *candidate)
                    > std::max(options.maximum_angular_error_rad, 1.0e-6)) {
                continue;
            }
            generalized_detail::appendDeduplicated(solutions, *candidate, 1.0e-7);
        }
        std::sort(solutions.begin(), solutions.end(),
            [&](const Solution& first, const Solution& second) {
                return generalized_detail::angularCost(
                           world_points, ray_origins, directions, first)
                    < generalized_detail::angularCost(
                           world_points, ray_origins, directions, second);
            });
        return solutions;
    }

    /** @brief Refine a seed in the four-dimensional [translation,yaw] space. */
    static std::optional<Solution> refine(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Solution& seed,
        const Eigen::Vector3d& output_axis,
        const Eigen::Vector3d& model_axis,
        const GeneralizedSolverOptions& options = GeneralizedSolverOptions {})
    {
        if (!generalized_detail::validInput(
                world_points, ray_origins, ray_directions, 2)
            || !validAxis(output_axis) || !validAxis(model_axis)) {
            return std::nullopt;
        }
        const Eigen::Vector3d up = output_axis.normalized();
        const PointMatrix directions =
            generalized_detail::normalizedDirections(ray_directions);
        Solution pose = projectRotationToAxisConstraint(
            seed, up, model_axis.normalized());
        double damping = std::max(options.damping, 1.0e-15);

        for (int iteration = 0;
             iteration < std::max(0, options.max_iterations); ++iteration) {
            const auto linearization = constrainedLinearization(
                world_points, ray_origins, directions, pose, up);
            if (!linearization) {
                return std::nullopt;
            }
            if (linearization->residual.norm()
                    / static_cast<double>(world_points.cols())
                < std::max(0.0, options.residual_tolerance)) {
                break;
            }

            Eigen::Matrix4d normal =
                linearization->jacobian.transpose() * linearization->jacobian;
            normal.diagonal().array() += damping;
            const Eigen::Vector4d gradient =
                linearization->jacobian.transpose() * linearization->residual;
            const Eigen::Vector4d step = -normal.ldlt().solve(gradient);
            if (!step.allFinite()
                || step.norm() < std::max(0.0, options.step_tolerance)) {
                break;
            }

            const double base_cost = linearization->residual.squaredNorm();
            bool accepted = false;
            double scale = 1.0;
            for (int line_search = 0; line_search < 10; ++line_search) {
                Solution candidate = pose;
                candidate.t += scale * step.head<3>();
                candidate.R = Eigen::AngleAxisd(
                    scale * step(3), up).toRotationMatrix() * candidate.R;
                const auto candidate_residual =
                    generalized_detail::angularResidualVector(
                        world_points, ray_origins, directions, candidate);
                if (candidate_residual
                    && candidate_residual->squaredNorm() < base_cost) {
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
        return pose.R.allFinite() && pose.t.allFinite()
            ? std::optional<Solution>(pose) : std::nullopt;
    }

    /** @brief Implicit analytic sensitivity of the constrained optimum. */
    static std::vector<PoseJacobian> jacobianPoseWrtBearings(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Eigen::Vector3d& output_axis,
        const Eigen::Vector3d& model_axis,
        const GeneralizedSolverOptions& options = GeneralizedSolverOptions {})
    {
        std::vector<PoseJacobian> output;
        if (!validAxis(output_axis)) {
            return output;
        }
        const Eigen::Vector3d up = output_axis.normalized();
        for (const Solution& solution : solve(
                 world_points, ray_origins, ray_directions,
                 output_axis, model_axis, options)) {
            const auto linearization = constrainedLinearization(
                world_points, ray_origins, ray_directions, solution, up);
            if (!linearization) {
                continue;
            }
            Eigen::Matrix4d normal =
                linearization->jacobian.transpose() * linearization->jacobian;
            normal.diagonal().array() += std::max(options.damping, 1.0e-15);

            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
                residual_direction_jacobian = Eigen::MatrixXd::Zero(
                    3 * ray_directions.cols(), 3 * ray_directions.cols());
            for (Eigen::Index index = 0;
                 index < ray_directions.cols(); ++index) {
                const Eigen::Vector3d raw = ray_directions.col(index);
                const double norm = raw.norm();
                if (norm <= kEpsilon) {
                    continue;
                }
                const Eigen::Vector3d direction = raw / norm;
                residual_direction_jacobian.block<3, 3>(
                    3 * index, 3 * index) =
                    -(Eigen::Matrix3d::Identity()
                        - direction * direction.transpose()) / norm;
            }
            const Eigen::Matrix<double, 4, Eigen::Dynamic> state_jacobian =
                -normal.ldlt().solve(
                    linearization->jacobian.transpose()
                    * residual_direction_jacobian);
            Eigen::Matrix<double, 6, 4> lift =
                Eigen::Matrix<double, 6, 4>::Zero();
            lift.block<3, 1>(0, 3) = up;
            lift.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
            PoseJacobian jacobian;
            jacobian.sol = solution;
            jacobian.dpose_directions = lift * state_jacobian;
            if (jacobian.dpose_directions.allFinite()) {
                output.push_back(std::move(jacobian));
            }
        }
        return output;
    }

    static std::vector<BearingJacobian> jacobianBearingsWrtPose(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Eigen::Vector3d& output_axis,
        const Eigen::Vector3d& model_axis,
        const GeneralizedSolverOptions& options = GeneralizedSolverOptions {})
    {
        std::vector<BearingJacobian> output;
        for (const PoseJacobian& pose_jacobian : jacobianPoseWrtBearings(
                 world_points, ray_origins, ray_directions,
                 output_axis, model_axis, options)) {
            BearingJacobian bearing_jacobian;
            bearing_jacobian.sol = pose_jacobian.sol;
            bearing_jacobian.ddirections_dpose =
                uvdar_core::helpers::dampedRightPseudoInverse(
                    pose_jacobian.dpose_directions,
                    std::max(options.damping, 1.0e-15));
            output.push_back(std::move(bearing_jacobian));
        }
        return output;
    }

private:
    static constexpr double kEpsilon = 1.0e-12;
    static constexpr double kSamePointTolerance = 1.0e-9;

    struct Prealigned {
        bool ok = false;
        Eigen::Matrix3d output_to_aligned = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d model_to_aligned = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 3, 2> points;
        Eigen::Matrix<double, 3, 2> origins;
        Eigen::Matrix<double, 3, 2> directions;
    };

    struct Linearization {
        Eigen::VectorXd residual;
        Eigen::Matrix<double, Eigen::Dynamic, 4> jacobian;
    };

    struct TriangulatedMarker {
        Eigen::Vector3d model_point;
        Eigen::Vector3d output_point;
    };

    static bool validAxis(const Eigen::Vector3d& axis)
    {
        return axis.allFinite() && axis.squaredNorm() > kEpsilon;
    }

    static bool hasYawObservableMarkerPair(
        const PointMatrix& world_points,
        const Eigen::Vector3d& model_axis)
    {
        const Eigen::Vector3d axis = model_axis.normalized();
        for (Eigen::Index first = 0;
             first + 1 < world_points.cols(); ++first) {
            for (Eigen::Index second = first + 1;
                 second < world_points.cols(); ++second) {
                const Eigen::Vector3d delta =
                    world_points.col(first) - world_points.col(second);
                if (delta.cross(axis).norm()
                    > 1.0e-9 * std::max(1.0, delta.norm())) {
                    return true;
                }
            }
        }
        return false;
    }

    static Prealigned prealign(
        const Eigen::Matrix<double, 3, 2>& world_points,
        const Eigen::Matrix<double, 3, 2>& ray_origins,
        const Eigen::Matrix<double, 3, 2>& ray_directions,
        const Eigen::Vector3d& output_axis,
        const Eigen::Vector3d& model_axis)
    {
        Prealigned output;
        if (!world_points.allFinite() || !ray_origins.allFinite()
            || !ray_directions.allFinite()
            || !validAxis(output_axis) || !validAxis(model_axis)) {
            return output;
        }
        output.output_to_aligned = uvdar_core::helpers::rotationBetween(
            output_axis.normalized(), Eigen::Vector3d::UnitZ());
        output.model_to_aligned = uvdar_core::helpers::rotationBetween(
            model_axis.normalized(), Eigen::Vector3d::UnitZ());
        output.points = output.model_to_aligned * world_points;
        output.origins = output.output_to_aligned * ray_origins;
        output.directions = output.output_to_aligned * ray_directions;
        for (int index = 0; index < 2; ++index) {
            if (output.directions.col(index).squaredNorm() <= kEpsilon) {
                return output;
            }
            output.directions.col(index).normalize();
        }
        const Eigen::Vector3d point_delta =
            output.points.col(0) - output.points.col(1);
        output.ok = point_delta.head<2>().norm()
            > 1.0e-9 * std::max(1.0, point_delta.norm());
        return output;
    }

    static std::vector<double> realQuadraticRoots(
        const double quadratic,
        const double linear,
        const double constant)
    {
        return uvdar_core::helpers::realQuadraticRoots(
            quadratic, linear, constant);
    }

    static Solution fromAlignedPose(
        const Prealigned& prep,
        const Eigen::Matrix3d& aligned_rotation,
        const Eigen::Vector3d& aligned_translation)
    {
        return {
            prep.output_to_aligned.transpose() * aligned_rotation
                * prep.model_to_aligned,
            prep.output_to_aligned.transpose() * aligned_translation};
    }

    static std::vector<Solution> solveTwoPointLi(const Prealigned& prep)
    {
        const Eigen::Vector3d point_delta =
            prep.points.col(0) - prep.points.col(1);
        const Eigen::Vector3d origin_delta =
            prep.origins.col(0) - prep.origins.col(1);
        const Eigen::Vector3d p1 = prep.directions.col(0);
        const Eigen::Vector3d p2 = prep.directions.col(1);
        const Eigen::Vector3d cross = p1.cross(p2);
        if (cross.squaredNorm() <= kEpsilon * kEpsilon) {
            return {};
        }
        const double origin_dot = origin_delta.dot(cross);
        const double a = -point_delta.x() * cross.x()
            - point_delta.y() * cross.y()
            + point_delta.z() * cross.z() - origin_dot;
        const double b = 2.0 * point_delta.x() * cross.y()
            - 2.0 * point_delta.y() * cross.x();
        const double c = point_delta.x() * cross.x()
            + point_delta.y() * cross.y()
            + point_delta.z() * cross.z() - origin_dot;

        std::vector<Solution> output;
        for (const double tangent_half_angle : realQuadraticRoots(a, b, c)) {
            const Eigen::Matrix3d rotation =
                uvdar_core::helpers::rotationZ(
                    2.0 * std::atan(tangent_half_angle));
            const Eigen::Vector3d rotated_delta =
                rotation * point_delta - origin_delta;
            const Eigen::Vector3d lhs = p2.cross(p1);
            const double lambda1 = p2.cross(rotated_delta).dot(lhs)
                / lhs.squaredNorm();
            const double lambda2 =
                (lambda1 * p1 - rotated_delta).dot(p2);
            if (lambda1 <= kEpsilon || lambda2 <= kEpsilon) {
                continue;
            }
            const Eigen::Vector3d translation = prep.origins.col(0)
                + lambda1 * p1 - rotation * prep.points.col(0);
            output.push_back(fromAlignedPose(
                prep, rotation, translation));
        }
        return output;
    }

    static std::vector<Solution> solveTwoPointSweeney(
        const Prealigned& original)
    {
        Prealigned prep = original;
        if (std::abs(prep.directions(2, 1))
            > std::abs(prep.directions(2, 0))) {
            prep.points.col(0).swap(prep.points.col(1));
            prep.origins.col(0).swap(prep.origins.col(1));
            prep.directions.col(0).swap(prep.directions.col(1));
        }
        const Eigen::Vector3d p1 = prep.directions.col(0);
        const Eigen::Vector3d p2 = prep.directions.col(1);
        if (std::abs(p1.z()) <= kEpsilon) {
            return {};
        }
        const Eigen::Vector3d point_delta =
            prep.points.col(0) - prep.points.col(1);
        const Eigen::Vector3d origin_delta =
            prep.origins.col(0) - prep.origins.col(1);
        const double m = (point_delta.z() - origin_delta.z()) / p1.z();
        const double n = p2.z() / p1.z();
        const Eigen::Vector3d constant = origin_delta + m * p1;
        const Eigen::Vector3d slope = n * p1 - p2;

        std::vector<Solution> output;
        for (const double lambda2 : realQuadraticRoots(
                 slope.squaredNorm(),
                 2.0 * constant.dot(slope),
                 constant.squaredNorm() - point_delta.squaredNorm())) {
            const double lambda1 = m + n * lambda2;
            if (lambda1 <= kEpsilon || lambda2 <= kEpsilon) {
                continue;
            }
            const Eigen::Vector3d observed_delta = origin_delta
                + lambda1 * p1 - lambda2 * p2;
            const Eigen::Vector2d model_xy = point_delta.head<2>();
            const Eigen::Vector2d output_xy = observed_delta.head<2>();
            if (model_xy.norm() <= kEpsilon
                || output_xy.norm() <= kEpsilon) {
                continue;
            }
            const double yaw = std::atan2(
                model_xy.x() * output_xy.y()
                    - model_xy.y() * output_xy.x(),
                model_xy.dot(output_xy));
            const Eigen::Matrix3d rotation =
                uvdar_core::helpers::rotationZ(yaw);
            const Eigen::Vector3d translation = prep.origins.col(0)
                + lambda1 * p1 - rotation * prep.points.col(0);
            output.push_back(fromAlignedPose(
                prep, rotation, translation));
        }
        return output;
    }

    static std::optional<Linearization> constrainedLinearization(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Solution& pose,
        const Eigen::Vector3d& output_axis)
    {
        if (!generalized_detail::validInput(
                world_points, ray_origins, ray_directions, 2)) {
            return std::nullopt;
        }
        Linearization output;
        output.residual.resize(3 * world_points.cols());
        output.jacobian.resize(3 * world_points.cols(), 4);
        for (Eigen::Index index = 0;
             index < world_points.cols(); ++index) {
            const Eigen::Vector3d rotated = pose.R * world_points.col(index);
            const Eigen::Vector3d ray_to_point =
                rotated + pose.t - ray_origins.col(index);
            const double range = ray_to_point.norm();
            if (!std::isfinite(range) || range <= kEpsilon) {
                return std::nullopt;
            }
            const Eigen::Vector3d predicted = ray_to_point / range;
            const Eigen::Matrix3d normalize =
                (Eigen::Matrix3d::Identity()
                    - predicted * predicted.transpose()) / range;
            output.residual.segment<3>(3 * index) =
                predicted - ray_directions.col(index).normalized();
            output.jacobian.block<3, 3>(3 * index, 0) = normalize;
            output.jacobian.block<3, 1>(3 * index, 3) =
                normalize * output_axis.cross(rotated);
        }
        return output.residual.allFinite() && output.jacobian.allFinite()
            ? std::optional<Linearization>(std::move(output))
            : std::nullopt;
    }

    static Solution projectRotationToAxisConstraint(
        const Solution& seed,
        const Eigen::Vector3d& output_axis,
        const Eigen::Vector3d& model_axis)
    {
        const Eigen::Matrix3d output_to_aligned =
            uvdar_core::helpers::rotationBetween(
                output_axis, Eigen::Vector3d::UnitZ());
        const Eigen::Matrix3d model_to_aligned =
            uvdar_core::helpers::rotationBetween(
                model_axis, Eigen::Vector3d::UnitZ());
        const Eigen::Matrix3d aligned = output_to_aligned * seed.R
            * model_to_aligned.transpose();
        const double yaw = std::atan2(
            aligned(1, 0) - aligned(0, 1),
            aligned(0, 0) + aligned(1, 1));
        Solution output = seed;
        output.R = output_to_aligned.transpose()
            * uvdar_core::helpers::rotationZ(yaw)
            * model_to_aligned;
        return output;
    }

    static bool isLocallyObservable(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Solution& pose,
        const Eigen::Vector3d& output_axis)
    {
        const auto linearization = constrainedLinearization(
            world_points, ray_origins, ray_directions, pose, output_axis);
        if (!linearization) {
            return false;
        }
        const Eigen::Matrix4d information =
            linearization->jacobian.transpose() * linearization->jacobian;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen(information);
        return eigen.info() == Eigen::Success
            && eigen.eigenvalues().maxCoeff() > kEpsilon
            && eigen.eigenvalues().minCoeff()
                > 1.0e-10 * eigen.eigenvalues().maxCoeff();
    }

    static std::vector<Solution> yawScanSeeds(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Eigen::Vector3d& output_axis,
        const Eigen::Vector3d& model_axis)
    {
        const Eigen::Vector3d output_up = output_axis.normalized();
        const Eigen::Matrix3d base_rotation =
            uvdar_core::helpers::rotationBetween(
                model_axis.normalized(), output_up);
        Eigen::Matrix3d translation_normal = Eigen::Matrix3d::Zero();
        for (Eigen::Index index = 0;
             index < ray_directions.cols(); ++index) {
            const Eigen::Vector3d direction =
                ray_directions.col(index).normalized();
            translation_normal += Eigen::Matrix3d::Identity()
                - direction * direction.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> translation_eigen(
            translation_normal);
        if (translation_eigen.info() != Eigen::Success
            || translation_eigen.eigenvalues().minCoeff()
                <= 1.0e-10 * translation_eigen.eigenvalues().maxCoeff()) {
            return {};
        }

        struct ScoredSeed {
            Solution pose;
            double cost = std::numeric_limits<double>::infinity();
        };
        std::vector<ScoredSeed> scored;
        constexpr int yaw_samples = 72;
        constexpr double pi = 3.14159265358979323846;
        scored.reserve(yaw_samples);
        for (int sample = 0; sample < yaw_samples; ++sample) {
            const double yaw = -pi + 2.0 * pi
                * static_cast<double>(sample)
                / static_cast<double>(yaw_samples);
            Solution seed;
            seed.R = Eigen::AngleAxisd(yaw, output_up).toRotationMatrix()
                * base_rotation;
            Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
            for (Eigen::Index index = 0;
                 index < ray_directions.cols(); ++index) {
                const Eigen::Vector3d direction =
                    ray_directions.col(index).normalized();
                const Eigen::Matrix3d perpendicular =
                    Eigen::Matrix3d::Identity()
                    - direction * direction.transpose();
                rhs += perpendicular * (
                    ray_origins.col(index)
                    - seed.R * world_points.col(index));
            }
            seed.t = translation_normal.ldlt().solve(rhs);
            const double cost = generalized_detail::angularCost(
                world_points, ray_origins, ray_directions, seed);
            if (seed.t.allFinite() && std::isfinite(cost)) {
                scored.push_back({seed, cost});
            }
        }
        std::sort(scored.begin(), scored.end(),
            [](const ScoredSeed& first, const ScoredSeed& second) {
                return first.cost < second.cost;
            });
        std::vector<Solution> output;
        for (std::size_t index = 0U;
             index < std::min<std::size_t>(4U, scored.size()); ++index) {
            output.push_back(scored[index].pose);
        }
        return output;
    }

    static std::optional<Solution> triangulatedMarkerSeed(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Eigen::Vector3d& output_axis,
        const Eigen::Vector3d& model_axis)
    {
        std::vector<std::vector<Eigen::Index>> groups;
        for (Eigen::Index index = 0; index < world_points.cols(); ++index) {
            auto group = std::find_if(
                groups.begin(), groups.end(), [&](const auto& candidate) {
                    return (world_points.col(candidate.front())
                            - world_points.col(index)).norm()
                        <= kSamePointTolerance;
                });
            if (group == groups.end()) {
                groups.push_back({index});
            } else {
                group->push_back(index);
            }
        }

        std::vector<TriangulatedMarker> markers;
        for (const auto& group : groups) {
            if (group.size() < 2U) {
                continue;
            }
            Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
            Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
            for (const Eigen::Index index : group) {
                const Eigen::Vector3d direction =
                    ray_directions.col(index).normalized();
                const Eigen::Matrix3d perpendicular =
                    Eigen::Matrix3d::Identity()
                    - direction * direction.transpose();
                normal += perpendicular;
                rhs += perpendicular * ray_origins.col(index);
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(normal);
            if (eigen.info() != Eigen::Success
                || eigen.eigenvalues().minCoeff()
                    <= 1.0e-8 * eigen.eigenvalues().maxCoeff()) {
                continue;
            }
            const Eigen::Vector3d point = normal.ldlt().solve(rhs);
            if (point.allFinite()) {
                markers.push_back(
                    {world_points.col(group.front()), point});
            }
        }
        if (markers.size() < 2U) {
            return std::nullopt;
        }

        const Eigen::Vector3d model_up = model_axis.normalized();
        const Eigen::Vector3d output_up = output_axis.normalized();
        const Eigen::Matrix3d base_rotation =
            uvdar_core::helpers::rotationBetween(model_up, output_up);
        std::optional<std::pair<std::size_t, std::size_t>> best_pair;
        double best_score = 0.0;
        for (std::size_t first = 0; first + 1U < markers.size(); ++first) {
            for (std::size_t second = first + 1U;
                 second < markers.size(); ++second) {
                const Eigen::Vector3d model_delta = base_rotation
                    * (markers[first].model_point
                        - markers[second].model_point);
                const Eigen::Vector3d output_delta =
                    markers[first].output_point
                    - markers[second].output_point;
                const Eigen::Vector3d model_horizontal = model_delta
                    - output_up * output_up.dot(model_delta);
                const Eigen::Vector3d output_horizontal = output_delta
                    - output_up * output_up.dot(output_delta);
                const double score = model_horizontal.norm()
                    * output_horizontal.norm();
                if (score > best_score) {
                    best_score = score;
                    best_pair = {first, second};
                }
            }
        }
        if (!best_pair || best_score <= kEpsilon) {
            return std::nullopt;
        }
        const auto& first = markers[best_pair->first];
        const auto& second = markers[best_pair->second];
        Eigen::Vector3d model_horizontal = base_rotation
            * (first.model_point - second.model_point);
        Eigen::Vector3d output_horizontal =
            first.output_point - second.output_point;
        model_horizontal -= output_up * output_up.dot(model_horizontal);
        output_horizontal -= output_up * output_up.dot(output_horizontal);
        const double yaw = std::atan2(
            output_up.dot(model_horizontal.cross(output_horizontal)),
            model_horizontal.dot(output_horizontal));

        Solution output;
        output.R = Eigen::AngleAxisd(yaw, output_up).toRotationMatrix()
            * base_rotation;
        output.t.setZero();
        for (const TriangulatedMarker& marker : markers) {
            output.t += marker.output_point - output.R * marker.model_point;
        }
        output.t /= static_cast<double>(markers.size());
        return output.R.allFinite() && output.t.allFinite()
            ? std::optional<Solution>(output) : std::nullopt;
    }
};

} // namespace uvdar_core::pose_estimation::geometric_solver

#endif // GENERALIZED_KNOWN_AXIS_HPP
