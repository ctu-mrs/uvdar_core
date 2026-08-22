#pragma once
#ifndef P4P_HPP
#define P4P_HPP

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <limits>

#include <Eigen/Dense>

#include "uvdar_core/helpers/math.hpp"
#include "uvdar_core/helpers/polynomial.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/solver_types.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

// ============================================================================
// P4P solver — Lehavi & Osserman algebraic formula
//
// D. Lehavi and B. Osserman, "A polynomial formula for the perspective four
// points problem," arXiv:2501.13058v2, 2025.
//
// Direct algebraic formula.  Given four 3-D world
// points and four bearing vectors through the camera centre, compute depths
// z_i by:
//   1.  Map inputs to invariant coordinates (a, b, c, d).
//   2.  Evaluate pre-derived polynomial coefficients X_{i,j} to form four
//       quadratic polynomials Q_i(z_i^2) = 0.
//   3.  Solve each quadratic, enumerate all sign combinations, and select
//       the one minimising the constraint residuals.
//   4.  Recover original-canvas depths and compute R, t via Davenport
//       quaternion (Horn's method — fully algebraic).
// ============================================================================

/**
 * @brief Perspective-4-point solver using the Lehavi-Osserman algebraic formula.
 *
 * The solver computes invariant coordinates, evaluates the closed-form
 * quadratic depth constraints, enumerates valid sign choices, and recovers
 * R,t with Davenport/Horn absolute orientation.
 */
class P4P {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Solution = PoseSolution;

    /**
     * @brief Pose sensitivity with respect to four bearing vectors.
     */
    struct PoseJacobian {
        Solution sol;
        // 6x12 Jacobian: [omega(3), t(3)] wrt Pi entries (column-major).
        Eigen::Matrix<double, 6, 12> dpose_dpi;
    };

    /**
     * @brief Bearing sensitivity with respect to the local pose tangent.
     */
    struct BearingJacobian {
        Solution sol;
        // 12x6 Jacobian: Pi entries (column-major) wrt [omega(3), t(3)].
        Eigen::Matrix<double, 12, 6> dpi_dpose;
    };

    /// Solve the perspective-4-point problem.
    /// @param Pw 3x4 matrix of world points (columns).
    /// @param Pi 3x4 matrix of bearing vectors (columns), need not be unit.
    /// @param reproj_thresh  max angular reprojection error (radians).
    /// @return vector of solutions (typically 0 or 1).
    static std::vector<Solution> solve(
        const Eigen::Matrix<double,3,4>& Pw,
        const Eigen::Matrix<double,3,4>& Pi,
        double reproj_thresh = 0.01)
    {
        std::vector<Solution> results;

        // Validate inputs
        if (!Pw.allFinite() || !Pi.allFinite()) return results;

        // Normalize bearing vectors
        Eigen::Matrix<double,3,4> pi;
        for (int i = 0; i < 4; i++) {
            double n = Pi.col(i).norm();
            if (n < 1e-12) return results;
            pi.col(i) = Pi.col(i) / n;
        }

        // Check world points are not all collinear
        {
            Eigen::Vector3d v01 = Pw.col(1) - Pw.col(0);
            Eigen::Vector3d v02 = Pw.col(2) - Pw.col(0);
            Eigen::Vector3d v03 = Pw.col(3) - Pw.col(0);
            double max_cross = std::max({
                v01.cross(v02).norm(),
                v01.cross(v03).norm(),
                v02.cross(v03).norm()
            });
            double max_len = std::max({v01.norm(), v02.norm(), v03.norm(), 1.0});
            if (max_cross < 1e-8 * max_len * max_len) return results;
        }

        // ---- Step 1: invariant coordinates ----
        double a[3], c[3];
        for (int i = 0; i < 3; i++) {
            int j = (i + 1) % 3, k = (i + 2) % 3;
            a[i] = (Pw.col(j) - Pw.col(k)).squaredNorm();
            c[i] = (Pw.col(i) - Pw.col(3)).squaredNorm();
        }

        double p3_dot_p3 = pi.col(3).dot(pi.col(3));   // = 1.0 for unit vectors
        double dots_p3[4];
        for (int i = 0; i < 4; i++)
            dots_p3[i] = pi.col(i).dot(pi.col(3));

        for (int i = 0; i < 3; i++)
            if (std::abs(dots_p3[i]) < 1e-14) return results;

        double b[3], d[3];
        for (int i = 0; i < 3; i++) {
            int j = (i + 1) % 3, k = (i + 2) % 3;
            b[i] = pi.col(i).dot(pi.col(i)) * p3_dot_p3 / (dots_p3[i] * dots_p3[i]);
            d[i] = pi.col(j).dot(pi.col(k)) * p3_dot_p3 / (dots_p3[j] * dots_p3[k]);
        }

        double v[12] = {a[0],a[1],a[2], c[0],c[1],c[2], b[0],b[1],b[2], d[0],d[1],d[2]};

        // ---- Step 2: evaluate polynomial coefficients ----
        double X00val = evalPoly(kX00, kNX00, v);
        double X01val = evalPoly(kX01, kNX01, v);
        double X02val = evalPoly(kX02, kNX02, v);

        double v1[12];
        permuteVars(v, v1, 0, 1);
        double X10val = evalPoly(kX00, kNX00, v1);
        double X11val = evalPoly(kX01, kNX01, v1);
        double X12val = evalPoly(kX02, kNX02, v1);

        double v2[12];
        permuteVars(v, v2, 0, 2);
        double X20val = evalPoly(kX00, kNX00, v2);
        double X21val = evalPoly(kX01, kNX01, v2);
        double X22val = evalPoly(kX02, kNX02, v2);

        double X30val = evalPoly(kX30, kNX30, v);
        double X31val = evalPoly(kX31, kNX31, v);
        double X32val = evalPoly(kX32, kNX32, v);

        // ---- Step 3: solve each quadratic Q_i(z_i^2) = 0 ----
        double z2_0[2], z2_1[2], z2_2[2], z2_3[2];
        int n0 = solveQuadratic(X02val, X01val, X00val, z2_0);
        int n1 = solveQuadratic(X12val, X11val, X10val, z2_1);
        int n2 = solveQuadratic(X22val, X21val, X20val, z2_2);
        int n3 = solveQuadratic(X32val, X31val, X30val, z2_3);

        if (n0 == 0 || n1 == 0 || n2 == 0 || n3 == 0) return results;

        // ---- Step 4: enumerate sign combinations, pick best combo ----
        double signs[4];
        for (int i = 0; i < 3; i++)
            signs[i] = (dots_p3[i] > 0.0) ? 1.0 : -1.0;
        signs[3] = 1.0;

        double* z2_all[4] = {z2_0, z2_1, z2_2, z2_3};
        int nall[4] = {n0, n1, n2, n3};

        // Collect all combos sorted by residual
        struct Candidate {
            double z[4];
            double err;
        };
        std::vector<Candidate> candidates;

        for (int i0 = 0; i0 < nall[0]; i0++) {
            for (int i1 = 0; i1 < nall[1]; i1++) {
                for (int i2 = 0; i2 < nall[2]; i2++) {
                    for (int i3 = 0; i3 < nall[3]; i3++) {
                        double combo[4] = {z2_all[0][i0], z2_all[1][i1],
                                           z2_all[2][i2], z2_all[3][i3]};
                        Candidate cand;
                        for (int ii = 0; ii < 4; ii++)
                            cand.z[ii] = signs[ii] * std::sqrt(std::max(combo[ii], 0.0));

                        // Constraint residual
                        cand.err = 0.0;
                        for (int ii = 0; ii < 3; ii++) {
                            int jj = (ii + 1) % 3, kk = (ii + 2) % 3;
                            double r_a = b[jj]*cand.z[jj]*cand.z[jj]
                                       + b[kk]*cand.z[kk]*cand.z[kk]
                                       - 2.0*d[ii]*cand.z[jj]*cand.z[kk] - a[ii];
                            double r_c = cand.z[3]*cand.z[3]
                                       + b[ii]*cand.z[ii]*cand.z[ii]
                                       - 2.0*cand.z[ii]*cand.z[3] - c[ii];
                            cand.err += r_a*r_a + r_c*r_c;
                        }
                        candidates.push_back(cand);
                    }
                }
            }
        }

        // Sort by residual (smallest first)
        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate& ca, const Candidate& cb) { return ca.err < cb.err; });

        // ---- Step 5: recover pose for each candidate ----
        double norm_p3 = pi.col(3).norm();  // = 1.0

        for (const auto& cand : candidates) {
            Eigen::Vector4d lambdas;
            bool ok = true;
            for (int i = 0; i < 4; i++) {
                lambdas(i) = norm_p3 / dots_p3[i] * cand.z[i];
                if (lambdas(i) <= 1e-12 || !std::isfinite(lambdas(i))) { ok = false; break; }
            }
            if (!ok) continue;

            Solution sol;
            if (poseFromDepths(Pw, pi, lambdas, reproj_thresh, sol)) {
                // Deduplicate
                bool dup = false;
                for (const auto& prev : results) {
                    if ((prev.R - sol.R).norm() < 1e-6 &&
                        (prev.t - sol.t).norm() < 1e-6) {
                        dup = true;
                        break;
                    }
                }
                if (!dup) results.push_back(sol);
            }
        }

        return results;
    }

    // Analytic Jacobian of pose wrt 4 bearing vectors Pi (column-major ordering).
    // Derivation: implicit-function chain rule on reprojection orthogonality
    // residuals r_i = pi_i x (R*Pw_i + t), with pi_i = Pi_i / ||Pi_i||.
    /**
     * @brief Differentiate P4P pose solutions with respect to bearing entries.
     */
    static std::vector<PoseJacobian> jacobianPoseWrtBearings(
        const Eigen::Matrix<double,3,4>& Pw,
        const Eigen::Matrix<double,3,4>& Pi,
        double reproj_thresh = 0.01,
        double damping = 1e-12)
    {
        std::vector<PoseJacobian> out;

        // Reuse the P4P closed-form solver to obtain consistent solution branches.
        const auto sols = solve(Pw, Pi, reproj_thresh);
        out.reserve(sols.size());
        if (sols.empty()) return out;

        // Precompute normalized bearings and normalization Jacobians.
        Eigen::Matrix<double,3,4> pi;
        Eigen::Matrix<double,3,3> Jnorm[4];
        for (int i = 0; i < 4; ++i) {
            const Eigen::Vector3d p = Pi.col(i);
            const double n = p.norm();
            if (!(n > 1e-12) || !std::isfinite(n)) {
                return out;
            }
            const Eigen::Vector3d u = p / n;
            pi.col(i) = u;
            Jnorm[i] = (Eigen::Matrix3d::Identity() - u * u.transpose()) / n;
        }

        for (const auto& sol : sols) {
            Eigen::Matrix<double,12,6> A;
            Eigen::Matrix<double,12,12> B;
            A.setZero();
            B.setZero();

            for (int i = 0; i < 4; ++i) {
                const Eigen::Vector3d pi_i = pi.col(i);
                const Eigen::Vector3d Rpw = sol.R * Pw.col(i);
                const Eigen::Vector3d pc = Rpw + sol.t;

                const Eigen::Matrix3d S_pi = uvdar_core::helpers::skew(pi_i);
                const Eigen::Matrix3d S_pc = uvdar_core::helpers::skew(pc);
                const Eigen::Matrix3d S_Rpw = uvdar_core::helpers::skew(Rpw);

                const int r0 = 3 * i;
                const int c0 = 3 * i;

                // dr_i/domega = [pi_i]_x * d(RPw_i)/domega, with
                // d(RPw_i)/domega = -[RPw_i]_x in right-multiplicative local coords.
                A.block<3,3>(r0, 0) = S_pi * (-S_Rpw);

                // dr_i/dt = [pi_i]_x
                A.block<3,3>(r0, 3) = S_pi;

                // dr_i/dPi_i = dr_i/dpi_i * dpi_i/dPi_i = (-[pc_i]_x) * Jnorm_i
                B.block<3,3>(r0, c0) = (-S_pc) * Jnorm[i];
            }

            Eigen::Matrix<double,6,6> AtA = A.transpose() * A;
            AtA.diagonal().array() += std::max(damping, 1e-15);
            const Eigen::Matrix<double,6,12> J = -AtA.ldlt().solve(A.transpose() * B);

            if (!J.allFinite()) {
                continue;
            }

            PoseJacobian pj;
            pj.sol = sol;
            pj.dpose_dpi = J;
            out.emplace_back(pj);
        }

        return out;
    }

    // Analytic Jacobian of bearings wrt pose from inverse-function chain rule.
    /**
     * @brief Approximate inverse sensitivity from pose tangent to bearing entries.
     */
    static std::vector<BearingJacobian> jacobianBearingsWrtPose(
        const Eigen::Matrix<double,3,4>& Pw,
        const Eigen::Matrix<double,3,4>& Pi,
        double reproj_thresh = 0.01,
        double damping = 1e-12)
    {
        std::vector<BearingJacobian> out;
        const auto pose_jacs = jacobianPoseWrtBearings(Pw, Pi, reproj_thresh, damping);
        out.reserve(pose_jacs.size());

        for (const auto& pj : pose_jacs) {
            BearingJacobian bj;
            bj.sol = pj.sol;
            bj.dpi_dpose = uvdar_core::helpers::dampedRightPseudoInverse(pj.dpose_dpi, std::max(damping, 1.0e-15));

            if (!bj.dpi_dpose.allFinite()) {
                continue;
            }
            out.emplace_back(bj);
        }

        return out;
    }

private:
    // =====================================================================
    // Polynomial term structure
    // =====================================================================
    struct PolyTerm {
        int coeff;
        int exps[12];   // a0,a1,a2, c0,c1,c2, b0,b1,b2, d0,d1,d2
    };

    // =====================================================================
    // Pre-compiled polynomial coefficients (Lehavi & Osserman, Appendix B)
    //
    // Variable ordering:
    //   a0=0, a1=1, a2=2, c0=3, c1=4, c2=5,
    //   b0=6, b1=7, b2=8, d0=9, d1=10, d2=11
    // =====================================================================

    static constexpr PolyTerm kX00[] = {
        {1, {1,0,0,2,0,0,0,1,0,1,1,0}},
        {-2, {1,0,0,1,1,0,0,1,0,1,1,0}},
        {1, {1,0,0,0,2,0,0,1,0,1,1,0}},
        {1, {0,1,0,2,0,0,0,1,0,1,1,0}},
        {-2, {0,1,0,1,1,0,0,1,0,1,1,0}},
        {1, {0,1,0,0,2,0,0,1,0,1,1,0}},
        {-1, {0,0,1,2,0,0,0,1,0,1,1,0}},
        {2, {0,0,1,1,1,0,0,1,0,1,1,0}},
        {-1, {0,0,1,0,2,0,0,1,0,1,1,0}},
        {2, {1,0,1,1,0,0,0,1,0,1,1,0}},
        {-2, {1,0,1,0,1,0,0,1,0,1,1,0}},
        {2, {0,1,1,1,0,0,0,1,0,1,1,0}},
        {-2, {0,1,1,0,1,0,0,1,0,1,1,0}},
        {-2, {0,0,2,1,0,0,0,1,0,1,1,0}},
        {2, {0,0,2,0,1,0,0,1,0,1,1,0}},
        {1, {1,0,2,0,0,0,0,1,0,1,1,0}},
        {1, {0,1,2,0,0,0,0,1,0,1,1,0}},
        {-1, {0,0,3,0,0,0,0,1,0,1,1,0}},
        {-1, {1,0,0,2,0,0,0,1,1,0,0,1}},
        {2, {1,0,0,1,1,0,0,1,1,0,0,1}},
        {-1, {1,0,0,0,2,0,0,1,1,0,0,1}},
        {-1, {2,0,0,1,0,0,0,1,1,0,0,1}},
        {1, {2,0,0,0,1,0,0,1,1,0,0,1}},
        {1, {0,1,0,2,0,0,0,1,1,0,0,1}},
        {-2, {0,1,0,1,1,0,0,1,1,0,0,1}},
        {1, {0,1,0,0,2,0,0,1,1,0,0,1}},
        {2, {1,1,0,1,0,0,0,1,1,0,0,1}},
        {-2, {1,1,0,0,1,0,0,1,1,0,0,1}},
        {-1, {0,2,0,1,0,0,0,1,1,0,0,1}},
        {1, {0,2,0,0,1,0,0,1,1,0,0,1}},
        {1, {0,0,1,2,0,0,0,1,1,0,0,1}},
        {-2, {0,0,1,1,1,0,0,1,1,0,0,1}},
        {1, {0,0,1,0,2,0,0,1,1,0,0,1}},
        {-1, {2,0,1,0,0,0,0,1,1,0,0,1}},
        {2, {1,1,1,0,0,0,0,1,1,0,0,1}},
        {-1, {0,2,1,0,0,0,0,1,1,0,0,1}},
        {1, {0,0,2,1,0,0,0,1,1,0,0,1}},
        {-1, {0,0,2,0,1,0,0,1,1,0,0,1}},
        {1, {1,0,2,0,0,0,0,1,1,0,0,1}},
        {-1, {0,1,2,0,0,0,0,1,1,0,0,1}},
        {-2, {0,1,0,2,0,0,0,0,0,2,0,1}},
        {4, {0,1,0,1,1,0,0,0,0,2,0,1}},
        {-2, {0,1,0,0,2,0,0,0,0,2,0,1}},
        {2, {0,1,2,0,0,0,0,0,0,2,0,1}},
        {-1, {2,0,0,1,0,0,0,1,1,0,0,0}},
        {-1, {2,0,0,0,1,0,0,1,1,0,0,0}},
        {2, {1,1,0,1,0,0,0,1,1,0,0,0}},
        {2, {1,1,0,0,1,0,0,1,1,0,0,0}},
        {-1, {0,2,0,1,0,0,0,1,1,0,0,0}},
        {-1, {0,2,0,0,1,0,0,1,1,0,0,0}},
        {2, {1,0,1,1,0,0,0,1,1,0,0,0}},
        {2, {1,0,1,0,1,0,0,1,1,0,0,0}},
        {1, {2,0,1,0,0,0,0,1,1,0,0,0}},
        {-2, {0,1,1,1,0,0,0,1,1,0,0,0}},
        {-2, {0,1,1,0,1,0,0,1,1,0,0,0}},
        {-2, {1,1,1,0,0,0,0,1,1,0,0,0}},
        {1, {0,2,1,0,0,0,0,1,1,0,0,0}},
        {-1, {0,0,2,1,0,0,0,1,1,0,0,0}},
        {-1, {0,0,2,0,1,0,0,1,1,0,0,0}},
        {-2, {1,0,2,0,0,0,0,1,1,0,0,0}},
        {2, {0,1,2,0,0,0,0,1,1,0,0,0}},
        {1, {0,0,3,0,0,0,0,1,1,0,0,0}},
        {4, {0,1,1,1,0,0,0,0,0,2,0,0}},
        {4, {0,1,1,0,1,0,0,0,0,2,0,0}},
        {-4, {0,1,2,0,0,0,0,0,0,2,0,0}},
        {-4, {1,0,1,1,0,0,0,0,0,1,1,0}},
        {-4, {0,1,1,1,0,0,0,0,0,1,1,0}},
        {4, {0,0,2,1,0,0,0,0,0,1,1,0}},
        {2, {2,0,0,1,0,0,0,0,1,0,0,1}},
        {-4, {1,1,0,1,0,0,0,0,1,0,0,1}},
        {2, {0,2,0,1,0,0,0,0,1,0,0,1}},
        {-2, {0,0,2,1,0,0,0,0,1,0,0,1}},
    };
    static constexpr int kNX00 = sizeof(kX00) / sizeof(PolyTerm);

    static constexpr PolyTerm kX01[] = {
        {-4, {1,0,0,1,0,0,1,1,0,1,1,0}},
        {4, {1,0,0,0,1,0,1,1,0,1,1,0}},
        {-4, {0,1,0,1,0,0,1,1,0,1,1,0}},
        {4, {0,1,0,0,1,0,1,1,0,1,1,0}},
        {4, {0,0,1,1,0,0,1,1,0,1,1,0}},
        {-4, {0,0,1,0,1,0,1,1,0,1,1,0}},
        {-4, {1,0,1,0,0,0,1,1,0,1,1,0}},
        {-4, {0,1,1,0,0,0,1,1,0,1,1,0}},
        {4, {0,0,2,0,0,0,1,1,0,1,1,0}},
        {-2, {0,0,0,2,0,0,1,1,1,0,0,1}},
        {4, {0,0,0,1,1,0,1,1,1,0,0,1}},
        {-2, {0,0,0,0,2,0,1,1,1,0,0,1}},
        {2, {2,0,0,0,0,0,1,1,1,0,0,1}},
        {-4, {1,1,0,0,0,0,1,1,1,0,0,1}},
        {2, {0,2,0,0,0,0,1,1,1,0,0,1}},
        {-4, {0,0,1,1,0,0,1,1,1,0,0,1}},
        {4, {0,0,1,0,1,0,1,1,1,0,0,1}},
        {-4, {1,0,1,0,0,0,1,1,1,0,0,1}},
        {4, {0,1,1,0,0,0,1,1,1,0,0,1}},
        {2, {0,0,0,2,0,0,1,0,0,2,0,1}},
        {-4, {0,0,0,1,1,0,1,0,0,2,0,1}},
        {2, {0,0,0,0,2,0,1,0,0,2,0,1}},
        {4, {0,1,0,1,0,0,1,0,0,2,0,1}},
        {-4, {0,1,0,0,1,0,1,0,0,2,0,1}},
        {-4, {0,1,1,0,0,0,1,0,0,2,0,1}},
        {-2, {0,0,2,0,0,0,1,0,0,2,0,1}},
        {2, {0,0,0,2,0,0,0,1,0,0,2,1}},
        {-4, {0,0,0,1,1,0,0,1,0,0,2,1}},
        {2, {0,0,0,0,2,0,0,1,0,0,2,1}},
        {4, {1,0,0,1,0,0,0,1,0,0,2,1}},
        {-4, {1,0,0,0,1,0,0,1,0,0,2,1}},
        {4, {1,0,1,0,0,0,0,1,0,0,2,1}},
        {-2, {0,0,2,0,0,0,0,1,0,0,2,1}},
        {-4, {0,0,0,2,0,0,0,0,0,1,1,2}},
        {8, {0,0,0,1,1,0,0,0,0,1,1,2}},
        {-4, {0,0,0,0,2,0,0,0,0,1,1,2}},
        {4, {1,0,1,0,0,0,0,0,0,1,1,2}},
        {4, {0,1,1,0,0,0,0,0,0,1,1,2}},
        {2, {0,0,0,2,0,0,0,0,1,0,0,3}},
        {-4, {0,0,0,1,1,0,0,0,1,0,0,3}},
        {2, {0,0,0,0,2,0,0,0,1,0,0,3}},
        {-2, {2,0,0,0,0,0,0,0,1,0,0,3}},
        {4, {1,1,0,0,0,0,0,0,1,0,0,3}},
        {-2, {0,2,0,0,0,0,0,0,1,0,0,3}},
        {-4, {1,0,0,1,0,0,1,1,1,0,0,0}},
        {-4, {1,0,0,0,1,0,1,1,1,0,0,0}},
        {4, {0,1,0,1,0,0,1,1,1,0,0,0}},
        {4, {0,1,0,0,1,0,1,1,1,0,0,0}},
        {4, {0,0,1,1,0,0,1,1,1,0,0,0}},
        {4, {0,0,1,0,1,0,1,1,1,0,0,0}},
        {4, {1,0,1,0,0,0,1,1,1,0,0,0}},
        {-4, {0,1,1,0,0,0,1,1,1,0,0,0}},
        {-4, {0,0,2,0,0,0,1,1,1,0,0,0}},
        {-4, {0,1,0,1,0,0,1,0,0,2,0,0}},
        {-4, {0,1,0,0,1,0,1,0,0,2,0,0}},
        {-4, {0,0,1,1,0,0,1,0,0,2,0,0}},
        {-4, {0,0,1,0,1,0,1,0,0,2,0,0}},
        {4, {0,1,1,0,0,0,1,0,0,2,0,0}},
        {4, {0,0,2,0,0,0,1,0,0,2,0,0}},
        {4, {1,0,0,1,0,0,1,0,0,1,1,0}},
        {4, {0,1,0,1,0,0,1,0,0,1,1,0}},
        {-4, {0,0,1,1,0,0,1,0,0,1,1,0}},
        {4, {1,0,1,0,0,0,1,0,0,1,1,0}},
        {4, {0,1,1,0,0,0,1,0,0,1,1,0}},
        {-4, {0,0,2,0,0,0,1,0,0,1,1,0}},
        {-4, {1,0,0,0,1,0,0,1,0,1,1,0}},
        {-4, {0,1,0,0,1,0,0,1,0,1,1,0}},
        {4, {0,0,1,0,1,0,0,1,0,1,1,0}},
        {4, {1,0,1,0,0,0,0,1,0,1,1,0}},
        {4, {0,1,1,0,0,0,0,1,0,1,1,0}},
        {-4, {0,0,2,0,0,0,0,1,0,1,1,0}},
        {4, {1,0,0,1,0,0,0,1,0,0,2,0}},
        {4, {1,0,0,0,1,0,0,1,0,0,2,0}},
        {-4, {0,0,1,1,0,0,0,1,0,0,2,0}},
        {-4, {0,0,1,0,1,0,0,1,0,0,2,0}},
        {-4, {1,0,1,0,0,0,0,1,0,0,2,0}},
        {4, {0,0,2,0,0,0,0,1,0,0,2,0}},
        {4, {1,0,0,1,0,0,1,0,1,0,0,1}},
        {-2, {2,0,0,0,0,0,1,0,1,0,0,1}},
        {-4, {0,1,0,1,0,0,1,0,1,0,0,1}},
        {4, {1,1,0,0,0,0,1,0,1,0,0,1}},
        {-2, {0,2,0,0,0,0,1,0,1,0,0,1}},
        {4, {0,0,1,1,0,0,1,0,1,0,0,1}},
        {2, {0,0,2,0,0,0,1,0,1,0,0,1}},
        {4, {1,0,0,0,1,0,0,1,1,0,0,1}},
        {-2, {2,0,0,0,0,0,0,1,1,0,0,1}},
        {-4, {0,1,0,0,1,0,0,1,1,0,0,1}},
        {4, {1,1,0,0,0,0,0,1,1,0,0,1}},
        {-2, {0,2,0,0,0,0,0,1,1,0,0,1}},
        {-4, {0,0,1,0,1,0,0,1,1,0,0,1}},
        {2, {0,0,2,0,0,0,0,1,1,0,0,1}},
        {8, {0,1,0,0,1,0,0,0,0,2,0,1}},
        {8, {0,0,1,1,0,0,0,0,0,1,1,1}},
        {8, {0,0,1,0,1,0,0,0,0,1,1,1}},
        {-8, {1,0,1,0,0,0,0,0,0,1,1,1}},
        {-8, {0,1,1,0,0,0,0,0,0,1,1,1}},
        {-8, {1,0,0,1,0,0,0,0,0,0,2,1}},
        {4, {2,0,0,0,0,0,0,0,1,0,0,2}},
        {-8, {1,1,0,0,0,0,0,0,1,0,0,2}},
        {4, {0,2,0,0,0,0,0,0,1,0,0,2}},
        {-4, {0,0,1,1,0,0,0,0,1,0,0,2}},
        {-4, {0,0,1,0,1,0,0,0,1,0,0,2}},
    };
    static constexpr int kNX01 = sizeof(kX01) / sizeof(PolyTerm);

    static constexpr PolyTerm kX02[] = {
        {4, {1,0,0,0,0,0,2,1,0,1,1,0}},
        {4, {0,1,0,0,0,0,2,1,0,1,1,0}},
        {-4, {0,0,1,0,0,0,2,1,0,1,1,0}},
        {4, {0,0,0,1,0,0,2,1,1,0,0,1}},
        {-4, {0,0,0,0,1,0,2,1,1,0,0,1}},
        {4, {1,0,0,0,0,0,2,1,1,0,0,1}},
        {-4, {0,1,0,0,0,0,2,1,1,0,0,1}},
        {-4, {0,0,0,1,0,0,2,0,0,2,0,1}},
        {4, {0,0,0,0,1,0,2,0,0,2,0,1}},
        {4, {0,0,1,0,0,0,2,0,0,2,0,1}},
        {-4, {0,0,0,1,0,0,1,1,0,0,2,1}},
        {4, {0,0,0,0,1,0,1,1,0,0,2,1}},
        {-8, {1,0,0,0,0,0,1,1,0,0,2,1}},
        {4, {0,0,1,0,0,0,1,1,0,0,2,1}},
        {8, {0,0,0,1,0,0,1,0,0,1,1,2}},
        {-8, {0,0,0,0,1,0,1,0,0,1,1,2}},
        {-4, {1,0,0,0,0,0,1,0,0,1,1,2}},
        {-4, {0,1,0,0,0,0,1,0,0,1,1,2}},
        {-4, {0,0,1,0,0,0,1,0,0,1,1,2}},
        {-4, {0,0,0,1,0,0,1,0,1,0,0,3}},
        {4, {0,0,0,0,1,0,1,0,1,0,0,3}},
        {-4, {1,0,0,0,0,0,1,0,1,0,0,3}},
        {4, {0,1,0,0,0,0,1,0,1,0,0,3}},
        {8, {1,0,0,0,0,0,0,0,0,0,2,3}},
        {-4, {0,0,0,1,0,0,2,1,1,0,0,0}},
        {-4, {0,0,0,0,1,0,2,1,1,0,0,0}},
        {4, {0,0,1,0,0,0,2,1,1,0,0,0}},
        {4, {0,0,0,1,0,0,2,0,0,2,0,0}},
        {4, {0,0,0,0,1,0,2,0,0,2,0,0}},
        {-4, {0,0,1,0,0,0,2,0,0,2,0,0}},
        {-4, {1,0,0,0,0,0,2,0,0,1,1,0}},
        {-4, {0,1,0,0,0,0,2,0,0,1,1,0}},
        {4, {0,0,1,0,0,0,2,0,0,1,1,0}},
        {-4, {1,0,0,0,0,0,1,1,0,1,1,0}},
        {-4, {0,1,0,0,0,0,1,1,0,1,1,0}},
        {4, {0,0,1,0,0,0,1,1,0,1,1,0}},
        {4, {0,0,0,1,0,0,1,1,0,0,2,0}},
        {4, {0,0,0,0,1,0,1,1,0,0,2,0}},
        {-4, {0,0,1,0,0,0,1,1,0,0,2,0}},
        {-4, {1,0,0,0,0,0,2,0,1,0,0,1}},
        {4, {0,1,0,0,0,0,2,0,1,0,0,1}},
        {-4, {0,0,1,0,0,0,2,0,1,0,0,1}},
        {8, {0,0,0,0,1,0,1,1,1,0,0,1}},
        {-4, {1,0,0,0,0,0,1,1,1,0,0,1}},
        {4, {0,1,0,0,0,0,1,1,1,0,0,1}},
        {-4, {0,0,1,0,0,0,1,1,1,0,0,1}},
        {-8, {0,0,0,0,1,0,1,0,0,2,0,1}},
        {-8, {0,0,0,1,0,0,1,0,0,1,1,1}},
        {-8, {0,0,0,0,1,0,1,0,0,1,1,1}},
        {8, {1,0,0,0,0,0,1,0,0,1,1,1}},
        {8, {0,1,0,0,0,0,1,0,0,1,1,1}},
        {8, {1,0,0,0,0,0,1,0,0,0,2,1}},
        {-8, {0,0,0,0,1,0,0,1,0,0,2,1}},
        {8, {1,0,0,0,0,0,0,1,0,0,2,1}},
        {4, {0,0,0,1,0,0,1,0,1,0,0,2}},
        {4, {0,0,0,0,1,0,1,0,1,0,0,2}},
        {8, {1,0,0,0,0,0,1,0,1,0,0,2}},
        {-8, {0,1,0,0,0,0,1,0,1,0,0,2}},
        {4, {0,0,1,0,0,0,1,0,1,0,0,2}},
        {16, {0,0,0,0,1,0,0,0,0,1,1,2}},
        {-16, {1,0,0,0,0,0,0,0,0,0,2,2}},
        {-8, {0,0,0,0,1,0,0,0,1,0,0,3}},
    };
    static constexpr int kNX02 = sizeof(kX02) / sizeof(PolyTerm);

    static constexpr PolyTerm kX30[] = {
        {1, {0,0,0,2,1,0,1,1,1,0,0,0}},
        {1, {0,0,0,1,2,0,1,1,1,0,0,0}},
        {-1, {0,0,0,2,0,1,1,1,1,0,0,0}},
        {1, {0,0,0,0,2,1,1,1,1,0,0,0}},
        {-1, {0,0,0,1,0,2,1,1,1,0,0,0}},
        {-1, {0,0,0,0,1,2,1,1,1,0,0,0}},
        {1, {0,1,0,2,0,0,1,1,1,0,0,0}},
        {-1, {0,1,0,0,2,0,1,1,1,0,0,0}},
        {2, {0,1,0,1,0,1,1,1,1,0,0,0}},
        {2, {0,1,0,0,1,1,1,1,1,0,0,0}},
        {-1, {0,2,0,1,0,0,1,1,1,0,0,0}},
        {-1, {0,2,0,0,1,0,1,1,1,0,0,0}},
        {-1, {0,0,1,2,0,0,1,1,1,0,0,0}},
        {-2, {0,0,1,1,1,0,1,1,1,0,0,0}},
        {-2, {0,0,1,0,1,1,1,1,1,0,0,0}},
        {1, {0,0,1,0,0,2,1,1,1,0,0,0}},
        {2, {0,1,1,0,1,0,1,1,1,0,0,0}},
        {-2, {0,1,1,0,0,1,1,1,1,0,0,0}},
        {1, {0,2,1,0,0,0,1,1,1,0,0,0}},
        {1, {0,0,2,1,0,0,1,1,1,0,0,0}},
        {1, {0,0,2,0,0,1,1,1,1,0,0,0}},
        {-1, {0,1,2,0,0,0,1,1,1,0,0,0}},
        {-1, {0,0,0,3,0,0,1,1,0,0,1,0}},
        {-2, {0,0,0,2,1,0,1,1,0,0,1,0}},
        {-1, {0,0,0,1,2,0,1,1,0,0,1,0}},
        {1, {0,0,0,2,0,1,1,1,0,0,1,0}},
        {2, {0,0,0,1,1,1,1,1,0,0,1,0}},
        {1, {0,0,0,0,2,1,1,1,0,0,1,0}},
        {1, {0,1,0,2,0,0,1,1,0,0,1,0}},
        {2, {0,1,0,1,1,0,1,1,0,0,1,0}},
        {1, {0,1,0,0,2,0,1,1,0,0,1,0}},
        {2, {0,0,1,2,0,0,1,1,0,0,1,0}},
        {2, {0,0,1,1,1,0,1,1,0,0,1,0}},
        {-2, {0,0,1,1,0,1,1,1,0,0,1,0}},
        {-2, {0,0,1,0,1,1,1,1,0,0,1,0}},
        {-2, {0,1,1,1,0,0,1,1,0,0,1,0}},
        {-2, {0,1,1,0,1,0,1,1,0,0,1,0}},
        {-1, {0,0,2,1,0,0,1,1,0,0,1,0}},
        {1, {0,0,2,0,0,1,1,1,0,0,1,0}},
        {1, {0,1,2,0,0,0,1,1,0,0,1,0}},
        {2, {0,0,0,2,0,1,0,1,0,0,2,0}},
        {-2, {0,0,0,0,2,1,0,1,0,0,2,0}},
        {4, {0,0,1,0,1,1,0,1,0,0,2,0}},
        {-2, {0,0,2,0,0,1,0,1,0,0,2,0}},
        {1, {0,0,0,3,0,0,1,0,1,0,0,1}},
        {-1, {0,0,0,2,1,0,1,0,1,0,0,1}},
        {2, {0,0,0,2,0,1,1,0,1,0,0,1}},
        {-2, {0,0,0,1,1,1,1,0,1,0,0,1}},
        {1, {0,0,0,1,0,2,1,0,1,0,0,1}},
        {-1, {0,0,0,0,1,2,1,0,1,0,0,1}},
        {-2, {0,1,0,2,0,0,1,0,1,0,0,1}},
        {2, {0,1,0,1,1,0,1,0,1,0,0,1}},
        {-2, {0,1,0,1,0,1,1,0,1,0,0,1}},
        {2, {0,1,0,0,1,1,1,0,1,0,0,1}},
        {1, {0,2,0,1,0,0,1,0,1,0,0,1}},
        {-1, {0,2,0,0,1,0,1,0,1,0,0,1}},
        {-1, {0,0,1,2,0,0,1,0,1,0,0,1}},
        {-2, {0,0,1,1,0,1,1,0,1,0,0,1}},
        {-1, {0,0,1,0,0,2,1,0,1,0,0,1}},
        {2, {0,1,1,1,0,0,1,0,1,0,0,1}},
        {2, {0,1,1,0,0,1,1,0,1,0,0,1}},
        {-1, {0,2,1,0,0,0,1,0,1,0,0,1}},
        {-4, {0,0,0,2,0,1,0,0,0,0,2,1}},
        {4, {0,0,0,1,1,1,0,0,0,0,2,1}},
        {4, {0,0,1,1,0,1,0,0,0,0,2,1}},
        {-2, {0,0,0,2,1,0,0,0,1,0,0,2}},
        {2, {0,0,0,0,1,2,0,0,1,0,0,2}},
        {-4, {0,1,0,0,1,1,0,0,1,0,0,2}},
        {2, {0,2,0,0,1,0,0,0,1,0,0,2}},
        {4, {0,0,0,2,1,0,0,0,0,0,1,2}},
        {-4, {0,0,0,1,1,1,0,0,0,0,1,2}},
        {-4, {0,1,0,1,1,0,0,0,0,0,1,2}},
    };
    static constexpr int kNX30 = sizeof(kX30) / sizeof(PolyTerm);

    static constexpr PolyTerm kX31[] = {
        {-4, {0,0,0,1,1,0,1,1,1,0,0,0}},
        {-2, {0,0,0,0,2,0,1,1,1,0,0,0}},
        {4, {0,0,0,1,0,1,1,1,1,0,0,0}},
        {2, {0,0,0,0,0,2,1,1,1,0,0,0}},
        {-4, {0,1,0,1,0,0,1,1,1,0,0,0}},
        {-4, {0,1,0,0,0,1,1,1,1,0,0,0}},
        {2, {0,2,0,0,0,0,1,1,1,0,0,0}},
        {4, {0,0,1,1,0,0,1,1,1,0,0,0}},
        {4, {0,0,1,0,1,0,1,1,1,0,0,0}},
        {-2, {0,0,2,0,0,0,1,1,1,0,0,0}},
        {4, {0,0,0,2,0,0,1,1,0,0,1,0}},
        {4, {0,0,0,1,1,0,1,1,0,0,1,0}},
        {-4, {0,0,0,1,0,1,1,1,0,0,1,0}},
        {-4, {0,0,0,0,1,1,1,1,0,0,1,0}},
        {-4, {0,1,0,1,0,0,1,1,0,0,1,0}},
        {-4, {0,1,0,0,1,0,1,1,0,0,1,0}},
        {-4, {0,0,1,1,0,0,1,1,0,0,1,0}},
        {4, {0,0,1,0,0,1,1,1,0,0,1,0}},
        {4, {0,1,1,0,0,0,1,1,0,0,1,0}},
        {-2, {0,0,0,2,0,0,0,1,0,0,2,0}},
        {2, {0,0,0,0,2,0,0,1,0,0,2,0}},
        {-4, {0,0,0,1,0,1,0,1,0,0,2,0}},
        {4, {0,0,0,0,1,1,0,1,0,0,2,0}},
        {-4, {0,0,1,0,1,0,0,1,0,0,2,0}},
        {-4, {0,0,1,0,0,1,0,1,0,0,2,0}},
        {2, {0,0,2,0,0,0,0,1,0,0,2,0}},
        {-4, {0,0,0,2,0,0,1,0,1,0,0,1}},
        {4, {0,0,0,1,1,0,1,0,1,0,0,1}},
        {-4, {0,0,0,1,0,1,1,0,1,0,0,1}},
        {4, {0,0,0,0,1,1,1,0,1,0,0,1}},
        {4, {0,1,0,1,0,0,1,0,1,0,0,1}},
        {-4, {0,1,0,0,1,0,1,0,1,0,0,1}},
        {4, {0,0,1,1,0,0,1,0,1,0,0,1}},
        {4, {0,0,1,0,0,1,1,0,1,0,0,1}},
        {-4, {0,1,1,0,0,0,1,0,1,0,0,1}},
        {4, {0,0,0,2,0,0,0,0,0,0,2,1}},
        {-4, {0,0,0,1,1,0,0,0,0,0,2,1}},
        {4, {0,0,0,1,0,1,0,0,0,0,2,1}},
        {-4, {0,0,0,0,1,1,0,0,0,0,2,1}},
        {-4, {0,0,1,1,0,0,0,0,0,0,2,1}},
        {-4, {0,0,1,0,0,1,0,0,0,0,2,1}},
        {2, {0,0,0,2,0,0,0,0,1,0,0,2}},
        {4, {0,0,0,1,1,0,0,0,1,0,0,2}},
        {-4, {0,0,0,0,1,1,0,0,1,0,0,2}},
        {-2, {0,0,0,0,0,2,0,0,1,0,0,2}},
        {4, {0,1,0,0,1,0,0,0,1,0,0,2}},
        {4, {0,1,0,0,0,1,0,0,1,0,0,2}},
        {-2, {0,2,0,0,0,0,0,0,1,0,0,2}},
        {-4, {0,0,0,2,0,0,0,0,0,0,1,2}},
        {-4, {0,0,0,1,1,0,0,0,0,0,1,2}},
        {4, {0,0,0,1,0,1,0,0,0,0,1,2}},
        {4, {0,0,0,0,1,1,0,0,0,0,1,2}},
        {4, {0,1,0,1,0,0,0,0,0,0,1,2}},
        {4, {0,1,0,0,1,0,0,0,0,0,1,2}},
        {-2, {0,0,0,2,0,0,1,1,0,0,0,0}},
        {2, {0,0,0,0,2,0,1,1,0,0,0,0}},
        {4, {0,1,0,1,0,0,1,1,0,0,0,0}},
        {4, {0,1,0,0,1,0,1,1,0,0,0,0}},
        {-4, {0,0,1,0,1,0,1,1,0,0,0,0}},
        {-4, {0,1,1,0,0,0,1,1,0,0,0,0}},
        {2, {0,0,2,0,0,0,1,1,0,0,0,0}},
        {2, {0,0,0,2,0,0,1,0,1,0,0,0}},
        {-2, {0,0,0,0,0,2,1,0,1,0,0,0}},
        {4, {0,1,0,0,0,1,1,0,1,0,0,0}},
        {-2, {0,2,0,0,0,0,1,0,1,0,0,0}},
        {-4, {0,0,1,1,0,0,1,0,1,0,0,0}},
        {-4, {0,0,1,0,0,1,1,0,1,0,0,0}},
        {4, {0,1,1,0,0,0,1,0,1,0,0,0}},
        {2, {0,0,0,0,2,0,0,1,1,0,0,0}},
        {-2, {0,0,0,0,0,2,0,1,1,0,0,0}},
        {4, {0,1,0,0,0,1,0,1,1,0,0,0}},
        {-2, {0,2,0,0,0,0,0,1,1,0,0,0}},
        {-4, {0,0,1,0,1,0,0,1,1,0,0,0}},
        {2, {0,0,2,0,0,0,0,1,1,0,0,0}},
        {-4, {0,0,0,2,0,0,1,0,0,0,1,0}},
        {4, {0,0,0,1,0,1,1,0,0,0,1,0}},
        {4, {0,1,0,1,0,0,1,0,0,0,1,0}},
        {4, {0,0,1,1,0,0,1,0,0,0,1,0}},
        {-4, {0,0,1,0,0,1,1,0,0,0,1,0}},
        {-4, {0,1,1,0,0,0,1,0,0,0,1,0}},
        {-4, {0,0,0,0,2,0,0,1,0,0,1,0}},
        {4, {0,0,0,1,0,1,0,1,0,0,1,0}},
        {4, {0,1,0,1,0,0,0,1,0,0,1,0}},
        {8, {0,0,1,0,1,0,0,1,0,0,1,0}},
        {-4, {0,0,2,0,0,0,0,1,0,0,1,0}},
        {8, {0,0,1,0,0,1,0,0,0,0,2,0}},
        {4, {0,0,0,2,0,0,1,0,0,0,0,1}},
        {-4, {0,0,0,1,1,0,1,0,0,0,0,1}},
        {-4, {0,1,0,1,0,0,1,0,0,0,0,1}},
        {4, {0,1,0,0,1,0,1,0,0,0,0,1}},
        {-4, {0,0,1,1,0,0,1,0,0,0,0,1}},
        {4, {0,1,1,0,0,0,1,0,0,0,0,1}},
        {-4, {0,0,0,1,1,0,0,0,1,0,0,1}},
        {4, {0,0,0,0,0,2,0,0,1,0,0,1}},
        {-8, {0,1,0,0,0,1,0,0,1,0,0,1}},
        {4, {0,2,0,0,0,0,0,0,1,0,0,1}},
        {-4, {0,0,1,1,0,0,0,0,1,0,0,1}},
        {8, {0,0,0,1,1,0,0,0,0,0,1,1}},
        {-8, {0,0,0,1,0,1,0,0,0,0,1,1}},
        {-8, {0,1,0,1,0,0,0,0,0,0,1,1}},
        {8, {0,0,1,1,0,0,0,0,0,0,1,1}},
        {-8, {0,1,0,0,1,0,0,0,0,0,0,2}},
    };
    static constexpr int kNX31 = sizeof(kX31) / sizeof(PolyTerm);

    static constexpr PolyTerm kX32[] = {
        {4, {0,0,0,0,1,0,1,1,1,0,0,0}},
        {-4, {0,0,0,0,0,1,1,1,1,0,0,0}},
        {4, {0,1,0,0,0,0,1,1,1,0,0,0}},
        {-4, {0,0,1,0,0,0,1,1,1,0,0,0}},
        {-4, {0,0,0,1,0,0,1,1,0,0,1,0}},
        {4, {0,0,0,0,0,1,1,1,0,0,1,0}},
        {4, {0,1,0,0,0,0,1,1,0,0,1,0}},
        {4, {0,0,0,1,0,0,0,1,0,0,2,0}},
        {-4, {0,0,0,0,1,0,0,1,0,0,2,0}},
        {4, {0,0,1,0,0,0,0,1,0,0,2,0}},
        {4, {0,0,0,1,0,0,1,0,1,0,0,1}},
        {-4, {0,0,0,0,1,0,1,0,1,0,0,1}},
        {-4, {0,0,1,0,0,0,1,0,1,0,0,1}},
        {-4, {0,0,0,1,0,0,0,0,0,0,2,1}},
        {4, {0,0,0,0,1,0,0,0,0,0,2,1}},
        {4, {0,0,1,0,0,0,0,0,0,0,2,1}},
        {-4, {0,0,0,1,0,0,0,0,1,0,0,2}},
        {4, {0,0,0,0,0,1,0,0,1,0,0,2}},
        {-4, {0,1,0,0,0,0,0,0,1,0,0,2}},
        {4, {0,0,0,1,0,0,0,0,0,0,1,2}},
        {-4, {0,0,0,0,0,1,0,0,0,0,1,2}},
        {-4, {0,1,0,0,0,0,0,0,0,0,1,2}},
        {4, {0,0,0,1,0,0,1,1,0,0,0,0}},
        {-4, {0,0,0,0,1,0,1,1,0,0,0,0}},
        {-8, {0,1,0,0,0,0,1,1,0,0,0,0}},
        {4, {0,0,1,0,0,0,1,1,0,0,0,0}},
        {-4, {0,0,0,1,0,0,1,0,1,0,0,0}},
        {4, {0,0,0,0,0,1,1,0,1,0,0,0}},
        {-4, {0,1,0,0,0,0,1,0,1,0,0,0}},
        {8, {0,0,1,0,0,0,1,0,1,0,0,0}},
        {-4, {0,0,0,0,1,0,0,1,1,0,0,0}},
        {4, {0,0,0,0,0,1,0,1,1,0,0,0}},
        {-4, {0,1,0,0,0,0,0,1,1,0,0,0}},
        {4, {0,0,1,0,0,0,0,1,1,0,0,0}},
        {4, {0,0,0,1,0,0,1,0,0,0,1,0}},
        {-4, {0,0,0,0,0,1,1,0,0,0,1,0}},
        {-4, {0,1,0,0,0,0,1,0,0,0,1,0}},
        {-4, {0,0,0,1,0,0,0,1,0,0,1,0}},
        {8, {0,0,0,0,1,0,0,1,0,0,1,0}},
        {-4, {0,0,0,0,0,1,0,1,0,0,1,0}},
        {-4, {0,1,0,0,0,0,0,1,0,0,1,0}},
        {-8, {0,0,1,0,0,0,0,1,0,0,1,0}},
        {-8, {0,0,1,0,0,0,0,0,0,0,2,0}},
        {-4, {0,0,0,1,0,0,1,0,0,0,0,1}},
        {4, {0,0,0,0,1,0,1,0,0,0,0,1}},
        {4, {0,0,1,0,0,0,1,0,0,0,0,1}},
        {4, {0,0,0,1,0,0,0,0,1,0,0,1}},
        {4, {0,0,0,0,1,0,0,0,1,0,0,1}},
        {-8, {0,0,0,0,0,1,0,0,1,0,0,1}},
        {8, {0,1,0,0,0,0,0,0,1,0,0,1}},
        {4, {0,0,1,0,0,0,0,0,1,0,0,1}},
        {-8, {0,0,0,0,1,0,0,0,0,0,1,1}},
        {8, {0,0,0,0,0,1,0,0,0,0,1,1}},
        {8, {0,1,0,0,0,0,0,0,0,0,1,1}},
        {-8, {0,0,1,0,0,0,0,0,0,0,1,1}},
        {8, {0,1,0,0,0,0,0,0,0,0,0,2}},
        {8, {0,1,0,0,0,0,1,0,0,0,0,0}},
        {-8, {0,0,1,0,0,0,1,0,0,0,0,0}},
        {8, {0,1,0,0,0,0,0,1,0,0,0,0}},
        {-8, {0,0,1,0,0,0,0,0,1,0,0,0}},
        {16, {0,0,1,0,0,0,0,0,0,0,1,0}},
        {-16, {0,1,0,0,0,0,0,0,0,0,0,1}},
    };
    static constexpr int kNX32 = sizeof(kX32) / sizeof(PolyTerm);

    // =====================================================================
    // Sparse polynomial evaluation
    // =====================================================================
    static double evalPoly(const PolyTerm* terms, int n, const double v[12]) {
        double total = 0.0;
        for (int i = 0; i < n; i++) {
            double prod = static_cast<double>(terms[i].coeff);
            for (int j = 0; j < 12; j++) {
                switch (terms[i].exps[j]) {
                    case 0: break;
                    case 1: prod *= v[j]; break;
                    case 2: prod *= v[j] * v[j]; break;
                    case 3: prod *= v[j] * v[j] * v[j]; break;
                    default: {
                        double p = 1.0;
                        for (int k = 0; k < terms[i].exps[j]; k++) p *= v[j];
                        prod *= p;
                    }
                }
            }
            total += prod;
        }
        return total;
    }

    // =====================================================================
    // Permute variables: swap indices (i,j) within each 3-element group
    // Groups: (a0,a1,a2), (c0,c1,c2), (b0,b1,b2), (d0,d1,d2)
    // =====================================================================
    static void permuteVars(const double v_in[12], double v_out[12], int i, int j) {
        for (int k = 0; k < 12; k++) v_out[k] = v_in[k];
        for (int base = 0; base < 12; base += 3) {
            double tmp = v_out[base + i];
            v_out[base + i] = v_out[base + j];
            v_out[base + j] = tmp;
        }
    }

    // =====================================================================
    // Solve c2*x^2 + c1*x + c0 = 0 for non-negative roots.
    // Returns number of roots found (0, 1, or 2).
    // =====================================================================
    static int solveQuadratic(double c2, double c1, double c0, double roots[2]) {
        int count = 0;
        for (const double root :
             uvdar_core::helpers::realQuadraticRoots(
                 c2, c1, c0, 1.0e-12)) {
            if (root >= -1.0e-12) {
                roots[count++] = std::max(root, 0.0);
            }
        }
        return count;
    }

    // =====================================================================
    // Quaternion (q0,q1,q2,q3) -> 3x3 rotation matrix
    // =====================================================================
    static Eigen::Matrix3d quatToRotation(const Eigen::Vector4d& q_in) {
        Eigen::Vector4d q = q_in.normalized();
        double q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);
        Eigen::Matrix3d R;
        R(0,0) = 1.0 - 2.0*(q2*q2 + q3*q3);
        R(0,1) = 2.0*(q1*q2 - q0*q3);
        R(0,2) = 2.0*(q1*q3 + q0*q2);
        R(1,0) = 2.0*(q1*q2 + q0*q3);
        R(1,1) = 1.0 - 2.0*(q1*q1 + q3*q3);
        R(1,2) = 2.0*(q2*q3 - q0*q1);
        R(2,0) = 2.0*(q1*q3 - q0*q2);
        R(2,1) = 2.0*(q2*q3 + q0*q1);
        R(2,2) = 1.0 - 2.0*(q1*q1 + q2*q2);
        return R;
    }

    // =====================================================================
    // Davenport / Horn: find the dominant eigenvector of the 4x4 K matrix
    // built from B = Qw * Qc^T (uses symmetric eigendecomposition).
    // =====================================================================
    static bool davenportQuaternion(const Eigen::Matrix3d& B, Eigen::Vector4d& q) {
        double sigma = B.trace();
        Eigen::Matrix3d S = B + B.transpose();
        Eigen::Vector3d z(B(1,2) - B(2,1), B(2,0) - B(0,2), B(0,1) - B(1,0));

        Eigen::Matrix4d K;
        K(0,0) = sigma;
        K(0,1) = z(0); K(0,2) = z(1); K(0,3) = z(2);
        K(1,0) = z(0); K(1,1) = S(0,0) - sigma; K(1,2) = S(0,1); K(1,3) = S(0,2);
        K(2,0) = z(1); K(2,1) = S(1,0); K(2,2) = S(1,1) - sigma; K(2,3) = S(1,2);
        K(3,0) = z(2); K(3,1) = S(2,0); K(3,2) = S(2,1); K(3,3) = S(2,2) - sigma;

        // Symmetric eigendecomposition
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(K);
        if (eig.info() != Eigen::Success) return false;

        // Eigenvalues sorted ascending; dominant = last column
        q = eig.eigenvectors().col(3);
        if (q(0) < 0.0) q = -q;
        return q.norm() > 1e-12;
    }

    // =====================================================================
    // Recover R, t from depths lambda_i using Horn's quaternion method.
    //   p_cam_i = lambda_i * pi_i   (camera-frame 3-D points)
    //   R * Pw_i + t = p_cam_i
    // =====================================================================
    static bool poseFromDepths(const Eigen::Matrix<double,3,4>& Pw,
                               const Eigen::Matrix<double,3,4>& Pi,
                               const Eigen::Vector4d& lambdas,
                               double reproj_thresh,
                               Solution& sol) {
        // Camera-frame points
        Eigen::Matrix<double,3,4> Pc;
        for (int i = 0; i < 4; i++)
            Pc.col(i) = Pi.col(i) * lambdas(i);

        // Centroids
        Eigen::Vector3d w_cent = Pw.rowwise().mean();
        Eigen::Vector3d c_cent = Pc.rowwise().mean();

        // Centered
        Eigen::Matrix<double,3,4> Qw, Qc;
        for (int i = 0; i < 4; i++) {
            Qw.col(i) = Pw.col(i) - w_cent;
            Qc.col(i) = Pc.col(i) - c_cent;
        }

        // Cross-covariance
        Eigen::Matrix3d B = Qw * Qc.transpose();

        // Davenport quaternion
        Eigen::Vector4d q;
        if (!davenportQuaternion(B, q)) return false;

        Eigen::Matrix3d R = quatToRotation(q);
        Eigen::Vector3d t = c_cent - R * w_cent;

        if (!R.allFinite() || !t.allFinite()) return false;

        // Reprojection check (angular error)
        for (int i = 0; i < 4; i++) {
            Eigen::Vector3d p_cam = R * Pw.col(i) + t;
            double depth = p_cam.norm();
            if (depth < 1e-12) return false;
            Eigen::Vector3d dir = p_cam / depth;
            double dot = std::min(1.0, std::max(-1.0, dir.dot(Pi.col(i))));
            double angle = std::acos(dot);
            if (angle > reproj_thresh || !std::isfinite(angle)) return false;
        }

        sol.R = R;
        sol.t = t;
        return true;
    }
};

} // namespace uvdar_core::pose_estimation::geometric_solver

#endif // P4P_HPP
