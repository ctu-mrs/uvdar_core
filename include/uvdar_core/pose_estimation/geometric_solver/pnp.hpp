#pragma once
#ifndef PNP_HPP
#define PNP_HPP

#include <cmath>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "uvdar_core/helpers/math.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/p4p.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/solver_types.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

/**
 * @brief Iterative perspective-n-point solver for four or more correspondences.
 *
 * PnP minimizes the difference between measured unit bearings and the bearings
 * induced by a body-to-camera pose. It seeds the optimization from the P4P
 * solution of the first four correspondences, then applies finite-difference
 * Levenberg-Marquardt iterations over every correspondence.
 */
class PnP {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Solution = PoseSolution;
    using PointMatrix = Eigen::Matrix<double, 3, Eigen::Dynamic>;

    /**
     * @brief Parameters of the bearing-space Levenberg-Marquardt iteration.
     */
    struct Options {
        constexpr Options()
            : max_iterations(40)
            , damping(1.0e-8)
            , finite_difference_epsilon(1.0e-6)
            , step_tolerance(1.0e-10)
            , residual_tolerance(1.0e-10)
            , p4p_reprojection_threshold_rad(0.01)
        {
        }

        int max_iterations;
        double damping;
        double finite_difference_epsilon;
        double step_tolerance;
        double residual_tolerance;
        double p4p_reprojection_threshold_rad;
    };

    /**
     * @brief Pose sensitivity with respect to all input bearing-vector entries.
     */
    struct PoseJacobian {
        Solution sol;
        // 6x(3N) Jacobian: [omega(3), t(3)] wrt Pi entries (column-major).
        Eigen::Matrix<double, 6, Eigen::Dynamic> dpose_dpi;
    };

    /**
     * @brief Bearing sensitivity with respect to the local pose tangent.
     */
    struct BearingJacobian {
        Solution sol;
        // (3N)x6 Jacobian: Pi entries (column-major) wrt [omega(3), t(3)].
        Eigen::Matrix<double, Eigen::Dynamic, 6> dpi_dpose;
    };

    /**
     * @brief Solve PnP from corresponding body points and camera bearings.
     *
     * @param Pw 3xN matrix of body/model points (columns), N >= 4.
     * @param Pi 3xN matrix of camera bearing vectors (columns), need not be unit.
     * @return a single locally optimized solution, or no solution on invalid input.
     */
    static std::vector<Solution> solve(
        const PointMatrix& Pw,
        const PointMatrix& Pi,
        const Options& options = Options {})
    {
        if (!validInput(Pw, Pi)) {
            return {};
        }

        const double finite_difference_epsilon = sanePositive(options.finite_difference_epsilon, 1.0e-6);
        const double damping = saneNonNegative(options.damping, 1.0e-8);
        const double step_tolerance = saneNonNegative(options.step_tolerance, 1.0e-10);
        const double residual_tolerance = saneNonNegative(options.residual_tolerance, 1.0e-10);
        const double p4p_reprojection_threshold_rad = sanePositive(options.p4p_reprojection_threshold_rad, 0.01);

        const std::vector<Solution> p4p_solutions = P4P::solve(
            Pw.leftCols<4>(),
            Pi.leftCols<4>(),
            p4p_reprojection_threshold_rad);
        std::optional<Solution> seed;
        if (!p4p_solutions.empty()) {
            // P4P orders its deduplicated candidates by the algebraic
            // reprojection residual. Use that single preferred P4P branch as
            // the deterministic PnP initialization.
            seed = p4p_solutions.front();
        }

        Solution pose;
        if (seed) {
            pose = *seed;
        } else {
            pose.R = Eigen::Matrix3d::Identity();
            pose.t = Eigen::Vector3d(0.0, 0.0, 3.0);
        }

        const int max_iterations = std::max(0, options.max_iterations);
        for (int iteration = 0; iteration < max_iterations; ++iteration) {
            const auto residual = bearingResidualVector(Pw, Pi, pose);
            if (!residual) {
                return {};
            }
            if (residual->norm() / static_cast<double>(Pw.cols()) < residual_tolerance) {
                break;
            }

            Eigen::Matrix<double, Eigen::Dynamic, 6> jacobian(residual->size(), 6);
            for (int parameter = 0; parameter < 6; ++parameter) {
                Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
                delta(parameter) = finite_difference_epsilon;

                Solution perturbed = pose;
                if (parameter < 3) {
                    applyLeftPoseIncrement(perturbed, Eigen::Vector3d::Zero(), delta.head<3>());
                } else {
                    applyLeftPoseIncrement(perturbed, delta.tail<3>(), Eigen::Vector3d::Zero());
                }

                const auto perturbed_residual = bearingResidualVector(Pw, Pi, perturbed);
                jacobian.col(parameter) = ((perturbed_residual ? *perturbed_residual : *residual) - *residual)
                    / finite_difference_epsilon;
            }

            Eigen::Matrix<double, 6, 6> hessian = jacobian.transpose() * jacobian;
            hessian.diagonal().array() += damping;
            const Eigen::Matrix<double, 6, 1> gradient = jacobian.transpose() * *residual;
            const Eigen::Matrix<double, 6, 1> step = -hessian.ldlt().solve(gradient);
            if (!step.allFinite() || step.norm() < step_tolerance) {
                break;
            }

            applyLeftPoseIncrement(pose, step.tail<3>(), step.head<3>());
        }

        return {{pose}};
    }

    /**
     * @brief Differentiate the optimized pose with respect to input bearings.
     *
     * The derivative is central finite-difference based and re-solves PnP for
     * every bearing entry, so it tracks the exact public solve() path.
     */
    static std::vector<PoseJacobian> jacobianPoseWrtBearings(
        const PointMatrix& Pw,
        const PointMatrix& Pi,
        const Options& options = Options {})
    {
        if (!validInput(Pw, Pi)) {
            return {};
        }

        const double epsilon = sanePositive(options.finite_difference_epsilon, 1.0e-6);
        const std::vector<Solution> base_solutions = solve(Pw, Pi, options);
        std::vector<PoseJacobian> output;
        output.reserve(base_solutions.size());

        for (const Solution& base : base_solutions) {
            PoseJacobian pose_jacobian;
            pose_jacobian.sol = base;
            pose_jacobian.dpose_dpi = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, 3 * Pi.cols());

            for (int parameter = 0; parameter < 3 * Pi.cols(); ++parameter) {
                const int column = parameter / 3;
                const int row = parameter % 3;

                PointMatrix plus = Pi;
                PointMatrix minus = Pi;
                plus(row, column) += epsilon;
                minus(row, column) -= epsilon;

                const auto plus_solution = firstSolution(solve(Pw, plus, options));
                const auto minus_solution = firstSolution(solve(Pw, minus, options));
                if (plus_solution && minus_solution) {
                    const double scale = 1.0 / (2.0 * epsilon);
                    pose_jacobian.dpose_dpi.block<3, 1>(0, parameter) =
                        uvdar_core::helpers::omegaFromRotationDerivative(
                            base.R,
                            (plus_solution->R - minus_solution->R) * scale);
                    pose_jacobian.dpose_dpi.block<3, 1>(3, parameter) =
                        (plus_solution->t - minus_solution->t) * scale;
                } else if (plus_solution) {
                    const double scale = 1.0 / epsilon;
                    pose_jacobian.dpose_dpi.block<3, 1>(0, parameter) =
                        uvdar_core::helpers::omegaFromRotationDerivative(
                            base.R,
                            (plus_solution->R - base.R) * scale);
                    pose_jacobian.dpose_dpi.block<3, 1>(3, parameter) =
                        (plus_solution->t - base.t) * scale;
                } else if (minus_solution) {
                    const double scale = 1.0 / epsilon;
                    pose_jacobian.dpose_dpi.block<3, 1>(0, parameter) =
                        uvdar_core::helpers::omegaFromRotationDerivative(
                            base.R,
                            (base.R - minus_solution->R) * scale);
                    pose_jacobian.dpose_dpi.block<3, 1>(3, parameter) =
                        (base.t - minus_solution->t) * scale;
                }
            }

            output.push_back(std::move(pose_jacobian));
        }
        return output;
    }

    /**
     * @brief Approximate inverse sensitivity from pose tangent to bearings.
     */
    static std::vector<BearingJacobian> jacobianBearingsWrtPose(
        const PointMatrix& Pw,
        const PointMatrix& Pi,
        const Options& options = Options {})
    {
        const std::vector<PoseJacobian> pose_jacobians = jacobianPoseWrtBearings(Pw, Pi, options);
        std::vector<BearingJacobian> output;
        output.reserve(pose_jacobians.size());

        for (const PoseJacobian& pose_jacobian : pose_jacobians) {
            BearingJacobian bearing_jacobian;
            bearing_jacobian.sol = pose_jacobian.sol;
            bearing_jacobian.dpi_dpose = uvdar_core::helpers::dampedRightPseudoInverse(
                pose_jacobian.dpose_dpi,
                std::max(saneNonNegative(options.damping, 1.0e-8), 1.0e-15));
            if (bearing_jacobian.dpi_dpose.allFinite()) {
                output.push_back(std::move(bearing_jacobian));
            }
        }
        return output;
    }

private:
    static constexpr double kEpsilon = 1.0e-12;

    static bool validInput(const PointMatrix& Pw, const PointMatrix& Pi)
    {
        if (Pw.cols() < 4 || Pw.cols() != Pi.cols() || !Pw.allFinite() || !Pi.allFinite()) {
            return false;
        }
        for (Eigen::Index i = 0; i < Pi.cols(); ++i) {
            if (Pi.col(i).norm() < kEpsilon) {
                return false;
            }
        }
        return true;
    }

    static double sanePositive(double value, double fallback)
    {
        return std::isfinite(value) && value > 0.0 ? value : fallback;
    }

    static double saneNonNegative(double value, double fallback)
    {
        return std::isfinite(value) && value >= 0.0 ? value : fallback;
    }

    static void applyLeftPoseIncrement(
        Solution& pose,
        const Eigen::Vector3d& translation_increment,
        const Eigen::Vector3d& rotation_increment)
    {
        pose.t += translation_increment;
        pose.R = uvdar_core::helpers::expSO3(rotation_increment) * pose.R;
    }

    static std::optional<Eigen::VectorXd> bearingResidualVector(
        const PointMatrix& Pw,
        const PointMatrix& Pi,
        const Solution& pose)
    {
        Eigen::VectorXd residual(3 * Pw.cols());
        for (Eigen::Index i = 0; i < Pw.cols(); ++i) {
            const Eigen::Vector3d predicted_raw = pose.R * Pw.col(i) + pose.t;
            const double predicted_norm = predicted_raw.norm();
            if (!std::isfinite(predicted_norm) || predicted_norm < kEpsilon) {
                return std::nullopt;
            }
            residual.segment<3>(3 * i) = predicted_raw / predicted_norm - Pi.col(i).normalized();
        }
        return residual.allFinite() ? std::optional<Eigen::VectorXd> {std::move(residual)} : std::nullopt;
    }

    static std::optional<Solution> firstSolution(const std::vector<Solution>& solutions)
    {
        return solutions.empty() ? std::nullopt : std::optional<Solution> {solutions.front()};
    }
};

} // namespace uvdar_core::pose_estimation::geometric_solver

#endif // PNP_HPP
