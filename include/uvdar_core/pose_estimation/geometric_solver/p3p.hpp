#pragma once
#ifndef P3P_HPP
#define P3P_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <limits>
#include <vector>

#include <Eigen/Dense>

#include "uvdar_core/helpers/poly_quartic.hpp"
#include "uvdar_core/helpers/math.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

/**
 * @brief Perspective-3-point solver based on the Smith depth-ratio quartic.
 *
 * The implementation follows arXiv:2508.01312v4: normalize the three bearing
 * vectors, reindex them so m13 <= m12 <= m23, solve the compact quartic in
 * x=d1/d3, recover y=d2/d3 and d3 from the third cosine equation, optionally
 * refine the three depths by Gauss-Newton, then recover R,t from the
 * depth-scaled bearing points.
 */
class P3P {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Body-to-camera rigid transform X_c = R X_w + t.
     */
    struct Solution {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t = Eigen::Vector3d::Zero();
    };

    /**
     * @brief Pose sensitivity with respect to three bearing vectors.
     */
    struct PoseJacobian {
        Solution sol;
        // 6x9 Jacobian: [omega(3), t(3)] wrt Pi entries (column-major).
        Eigen::Matrix<double, 6, 9> dpose_dpi = Eigen::Matrix<double, 6, 9>::Zero();
    };

    /**
     * @brief Bearing sensitivity with respect to the local pose tangent.
     */
    struct BearingJacobian {
        Solution sol;
        // 9x6 Jacobian: Pi entries (column-major) wrt [omega(3), t(3)].
        Eigen::Matrix<double, 9, 6> dpi_dpose = Eigen::Matrix<double, 9, 6>::Zero();
    };

    /**
     * @brief Solve P3P by quartic root finding and optional depth polishing.
     */
    static std::vector<Solution> solve(
        const Eigen::Matrix3d& Pw_in,
        const Eigen::Matrix3d& Pi_in,
        int polishing = 1)
    {
        if (!Pw_in.allFinite() || !Pi_in.allFinite()) {
            return {};
        }

        Eigen::Matrix3d Pi_normalized;
        for (int i = 0; i < 3; ++i) {
            const double norm = Pi_in.col(i).norm();
            if (norm < kEpsilon) {
                return {};
            }
            Pi_normalized.col(i) = Pi_in.col(i) / norm;
        }

        const IndexedInput indexed = reindexByBearingCosines(Pw_in, Pi_normalized);
        const Eigen::Matrix3d& Pw = indexed.Pw;
        const Eigen::Matrix3d& Pi = indexed.Pi;

        const double m12 = Pi.col(0).dot(Pi.col(1));
        const double m13 = Pi.col(0).dot(Pi.col(2));
        const double m23 = Pi.col(1).dot(Pi.col(2));
        const double s12 = (Pw.col(0) - Pw.col(1)).squaredNorm();
        const double s13 = (Pw.col(0) - Pw.col(2)).squaredNorm();
        const double s23 = (Pw.col(1) - Pw.col(2)).squaredNorm();
        if (s12 < kEpsilon || s13 < kEpsilon || s23 < kEpsilon) {
            return {};
        }

        // Smith quartic from arXiv:2508.01312v4, Eq. (c4)..(c0), stored as c0..c4.
        std::array<std::complex<double>, 5> coefficients;
        coefficients[4] = -s12*s12 + 2.0*s12*s13 + 2.0*s12*s23 - s13*s13
            + 4.0*s13*s23*m12*m12 - 2.0*s13*s23 - s23*s23;
        coefficients[3] = 4.0*s12*s12*m13 - 4.0*s12*s13*m12*m23 - 4.0*s12*s13*m13
            - 8.0*s12*s23*m13 + 4.0*s13*s13*m12*m23
            - 8.0*s13*s23*m12*m12*m13 - 4.0*s13*s23*m12*m23
            + 4.0*s13*s23*m13 + 4.0*s23*s23*m13;
        coefficients[2] = -4.0*s12*s12*m13*m13 - 2.0*s12*s12
            + 8.0*s12*s13*m12*m13*m23 + 4.0*s12*s13*m23*m23
            + 8.0*s12*s23*m13*m13 + 4.0*s12*s23
            - 4.0*s13*s13*m12*m12 - 4.0*s13*s13*m23*m23 + 2.0*s13*s13
            + 4.0*s13*s23*m12*m12 + 8.0*s13*s23*m12*m13*m23
            - 4.0*s23*s23*m13*m13 - 2.0*s23*s23;
        coefficients[1] = 4.0*s12*s12*m13 - 4.0*s12*s13*m12*m23
            - 8.0*s12*s13*m13*m23*m23 + 4.0*s12*s13*m13
            - 8.0*s12*s23*m13 + 4.0*s13*s13*m12*m23
            - 4.0*s13*s23*m12*m23 - 4.0*s13*s23*m13
            + 4.0*s23*s23*m13;
        coefficients[0] = -s12*s12 + 4.0*s12*s13*m23*m23 - 2.0*s12*s13
            + 2.0*s12*s23 - s13*s13 + 2.0*s13*s23 - s23*s23;

        if (std::abs(coefficients[4]) < kEpsilon) {
            return {};
        }

        std::vector<Solution> output;
        const auto roots = uvdar_core::helpers::poly_quartic::roots(coefficients);
        for (const auto& root : roots) {
            if (std::abs(root.imag()) > 1.0e-8 * std::max(1.0, std::abs(root.real()))) {
                continue;
            }

            const double x = root.real();
            if (x <= 0.0) {
                continue;
            }

            Eigen::Vector3d depths;
            if (!depthsFromRoot(x, s12, s13, s23, m12, m13, m23, depths)) {
                continue;
            }
            if (polishing > 0) {
                refineDepths(s12, s13, s23, m12, m13, m23, polishing, depths);
            }

            Solution solution;
            if (recoverPose(Pw, Pi, depths, solution)) {
                output.push_back(solution);
            }
        }

        return output;
    }

    /**
     * @brief Differentiate P3P pose solutions with respect to bearing entries.
     *
     * The derivative is finite-difference based and matches the current solve()
     * path, including reindexing, depth recovery, and depth polishing.
     */
    static std::vector<PoseJacobian> jacobianPoseWrtBearings(
        const Eigen::Matrix3d& Pw,
        const Eigen::Matrix3d& Pi,
        int polishing = 1)
    {
        constexpr double eps = 1.0e-6;
        const std::vector<Solution> base_solutions = solve(Pw, Pi, polishing);

        std::vector<PoseJacobian> output;
        output.reserve(base_solutions.size());
        for (const Solution& base : base_solutions) {
            PoseJacobian pose_jacobian;
            pose_jacobian.sol = base;

            for (int parameter = 0; parameter < 9; ++parameter) {
                const int col = parameter / 3;
                const int row = parameter % 3;

                Eigen::Matrix3d plus = Pi;
                Eigen::Matrix3d minus = Pi;
                plus(row, col) += eps;
                minus(row, col) -= eps;

                const auto plus_solution = closestSolution(base, solve(Pw, plus, polishing));
                const auto minus_solution = closestSolution(base, solve(Pw, minus, polishing));
                if (plus_solution.first && minus_solution.first) {
                    const double scale = 1.0 / (2.0 * eps);
                    pose_jacobian.dpose_dpi.block<3, 1>(0, parameter) =
                        uvdar_core::helpers::omegaFromRotationDerivative(base.R, (plus_solution.second.R - minus_solution.second.R) * scale);
                    pose_jacobian.dpose_dpi.block<3, 1>(3, parameter) =
                        (plus_solution.second.t - minus_solution.second.t) * scale;
                } else if (plus_solution.first) {
                    const double scale = 1.0 / eps;
                    pose_jacobian.dpose_dpi.block<3, 1>(0, parameter) =
                        uvdar_core::helpers::omegaFromRotationDerivative(base.R, (plus_solution.second.R - base.R) * scale);
                    pose_jacobian.dpose_dpi.block<3, 1>(3, parameter) =
                        (plus_solution.second.t - base.t) * scale;
                } else if (minus_solution.first) {
                    const double scale = 1.0 / eps;
                    pose_jacobian.dpose_dpi.block<3, 1>(0, parameter) =
                        uvdar_core::helpers::omegaFromRotationDerivative(base.R, (base.R - minus_solution.second.R) * scale);
                    pose_jacobian.dpose_dpi.block<3, 1>(3, parameter) =
                        (base.t - minus_solution.second.t) * scale;
                }
            }

            output.push_back(pose_jacobian);
        }
        return output;
    }

    /**
     * @brief Approximate inverse sensitivity from pose tangent to bearing entries.
     */
    static std::vector<BearingJacobian> jacobianBearingsWrtPose(
        const Eigen::Matrix3d& Pw,
        const Eigen::Matrix3d& Pi,
        int polishing = 1)
    {
        const std::vector<PoseJacobian> pose_jacobians = jacobianPoseWrtBearings(Pw, Pi, polishing);

        std::vector<BearingJacobian> output;
        output.reserve(pose_jacobians.size());
        for (const PoseJacobian& pose_jacobian : pose_jacobians) {
            BearingJacobian bearing_jacobian;
            bearing_jacobian.sol = pose_jacobian.sol;
            bearing_jacobian.dpi_dpose = uvdar_core::helpers::dampedRightPseudoInverse(pose_jacobian.dpose_dpi);
            output.push_back(bearing_jacobian);
        }
        return output;
    }

private:
    static constexpr double kEpsilon = 1.0e-12;

    struct IndexedInput {
        Eigen::Matrix3d Pw = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d Pi = Eigen::Matrix3d::Zero();
    };

    /**
     * @brief Reindex correspondences so m13 <= m12 <= m23 as recommended by the paper.
     */
    static IndexedInput reindexByBearingCosines(const Eigen::Matrix3d& Pw, const Eigen::Matrix3d& Pi)
    {
        const std::array<std::array<int, 3>, 6> permutations {{
            {{0, 1, 2}},
            {{0, 2, 1}},
            {{1, 0, 2}},
            {{1, 2, 0}},
            {{2, 0, 1}},
            {{2, 1, 0}},
        }};

        std::array<int, 3> best = permutations.front();
        double best_violation = std::numeric_limits<double>::infinity();
        for (const auto& permutation : permutations) {
            const double m12 = Pi.col(permutation[0]).dot(Pi.col(permutation[1]));
            const double m13 = Pi.col(permutation[0]).dot(Pi.col(permutation[2]));
            const double m23 = Pi.col(permutation[1]).dot(Pi.col(permutation[2]));
            const double violation = std::max(0.0, m13 - m12) + std::max(0.0, m12 - m23);
            if (violation < best_violation) {
                best_violation = violation;
                best = permutation;
            }
            if (violation <= kEpsilon) {
                break;
            }
        }

        IndexedInput output;
        for (int i = 0; i < 3; ++i) {
            output.Pw.col(i) = Pw.col(best[static_cast<std::size_t>(i)]);
            output.Pi.col(i) = Pi.col(best[static_cast<std::size_t>(i)]);
        }
        return output;
    }

    /**
     * @brief Recover positive depths from one positive x=d1/d3 root.
     */
    static bool depthsFromRoot(
        double x,
        double s12,
        double s13,
        double s23,
        double m12,
        double m13,
        double m23,
        Eigen::Vector3d& depths)
    {
        const double a = -s12 + s23 + s13;
        const double b = 2.0 * (s12 - s23) * m13;
        const double c = -s12 + s23 - s13;
        const double y_denominator = 2.0 * s13 * (m12 * x - m23);
        if (std::abs(y_denominator) < kEpsilon) {
            return false;
        }

        const double y = (a * x * x + b * x + c) / y_denominator;
        if (!std::isfinite(y) || y <= 0.0) {
            return false;
        }

        const double d3_denominator = y*y - 2.0*y*m23 + 1.0;
        if (d3_denominator <= kEpsilon) {
            return false;
        }

        const double d3_squared = s23 / d3_denominator;
        if (!std::isfinite(d3_squared) || d3_squared <= 0.0) {
            return false;
        }

        const double d3 = std::sqrt(d3_squared);
        depths << x * d3, y * d3, d3;
        return depths.allFinite() && (depths.array() > 0.0).all();
    }

    /**
     * @brief Refine depths by Gauss-Newton on the three law-of-cosines residuals.
     */
    static void refineDepths(
        double s12,
        double s13,
        double s23,
        double m12,
        double m13,
        double m23,
        int iterations,
        Eigen::Vector3d& depths)
    {
        for (int iteration = 0; iteration < iterations; ++iteration) {
            const double d1 = depths(0);
            const double d2 = depths(1);
            const double d3 = depths(2);

            Eigen::Vector3d residual;
            residual << d1*d1 + d2*d2 - 2.0*d1*d2*m12 - s12,
                d1*d1 + d3*d3 - 2.0*d1*d3*m13 - s13,
                d2*d2 + d3*d3 - 2.0*d2*d3*m23 - s23;

            Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
            jacobian(0, 0) = 2.0*d1 - 2.0*d2*m12;
            jacobian(0, 1) = 2.0*d2 - 2.0*d1*m12;
            jacobian(1, 0) = 2.0*d1 - 2.0*d3*m13;
            jacobian(1, 2) = 2.0*d3 - 2.0*d1*m13;
            jacobian(2, 1) = 2.0*d2 - 2.0*d3*m23;
            jacobian(2, 2) = 2.0*d3 - 2.0*d2*m23;

            const Eigen::Matrix3d normal = jacobian.transpose() * jacobian;
            const Eigen::Vector3d gradient = jacobian.transpose() * residual;
            const Eigen::Vector3d step = -normal.ldlt().solve(gradient);
            if (!step.allFinite() || step.norm() < 1.0e-12) {
                break;
            }

            double scale = 1.0;
            Eigen::Vector3d candidate = depths + step;
            while ((!candidate.allFinite() || !(candidate.array() > 0.0).all()) && scale > 1.0e-4) {
                scale *= 0.5;
                candidate = depths + scale * step;
            }
            if (!candidate.allFinite() || !(candidate.array() > 0.0).all()) {
                break;
            }
            depths = candidate;
        }
    }

    /**
     * @brief Recover R,t from refined depths using Eq. (R) and Eq. (t).
     */
    static bool recoverPose(
        const Eigen::Matrix3d& Pw,
        const Eigen::Matrix3d& Pi,
        const Eigen::Vector3d& depths,
        Solution& solution)
    {
        const Eigen::Vector3d y1 = depths(0) * Pi.col(0) - depths(1) * Pi.col(1);
        const Eigen::Vector3d y2 = depths(0) * Pi.col(0) - depths(2) * Pi.col(2);
        const Eigen::Vector3d x1 = Pw.col(0) - Pw.col(1);
        const Eigen::Vector3d x2 = Pw.col(0) - Pw.col(2);

        Eigen::Matrix3d y_matrix;
        y_matrix.col(0) = y1;
        y_matrix.col(1) = y2;
        y_matrix.col(2) = y1.cross(y2);

        Eigen::Matrix3d x_matrix;
        x_matrix.col(0) = x1;
        x_matrix.col(1) = x2;
        x_matrix.col(2) = x1.cross(x2);
        if (std::abs(x_matrix.determinant()) < kEpsilon) {
            return false;
        }

        solution.R = y_matrix * x_matrix.inverse();
        solution.t = depths(0) * Pi.col(0) - solution.R * Pw.col(0);
        return solution.R.allFinite() && solution.t.allFinite() && std::isfinite(solution.R.determinant());
    }

    static double poseDistance(const Solution& left, const Solution& right)
    {
        return (left.R - right.R).norm() + (left.t - right.t).norm();
    }

    static std::pair<bool, Solution> closestSolution(const Solution& reference, const std::vector<Solution>& candidates)
    {
        if (candidates.empty()) {
            return {false, {}};
        }

        double best_distance = std::numeric_limits<double>::infinity();
        Solution best;
        for (const Solution& candidate : candidates) {
            const double distance = poseDistance(reference, candidate);
            if (distance < best_distance) {
                best_distance = distance;
                best = candidate;
            }
        }
        return {std::isfinite(best_distance), best};
    }
};

} // namespace uvdar_core::pose_estimation::geometric_solver

#endif // P3P_HPP
