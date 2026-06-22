#pragma once
#ifndef P3P_HPP
#define P3P_HPP

#include <vector>
#include <array>
#include <complex>
#include <cmath>

#include <Eigen/Dense>

#include "uvdar_core/pose_estimation/geometric_solver/poly_quartic.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

/**
 * @brief Perspective-3-point solver using quartic depth constraints.
 *
 * The method normalizes three bearing vectors, constructs a body-plane basis,
 * derives quartic equations from inter-point distances and bearing cosines,
 * solves the real positive depth candidates, and recovers R,t by absolute
 * orientation. Optional polishing improves the depth roots.
 */
class P3P {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Body-to-camera rigid transform X_c = R X_w + t.
     */
    struct Solution {
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
    };

    /**
     * @brief Pose sensitivity with respect to three bearing vectors.
     */
    struct PoseJacobian {
        Solution sol;
        // 6x9 Jacobian: [omega(3), t(3)] wrt Pi entries (column-major).
        Eigen::Matrix<double, 6, 9> dpose_dpi;
    };

    /**
     * @brief Bearing sensitivity with respect to the local pose tangent.
     */
    struct BearingJacobian {
        Solution sol;
        // 9x6 Jacobian: Pi entries (column-major) wrt [omega(3), t(3)].
        Eigen::Matrix<double, 9, 6> dpi_dpose;
    };

    /**
     * @brief Solve P3P by quartic root finding and optional polishing.
     */
    static std::vector<Solution> solve(const Eigen::Matrix3d& Pw_in,
                                             const Eigen::Matrix3d& Pi_in,
                                             int polishing = 1)
    {
        // 1) Reorder to maximize baseline
        Eigen::Vector3d dists;
        dists(0) = (Pw_in.col(1) - Pw_in.col(0)).norm();
        dists(1) = (Pw_in.col(2) - Pw_in.col(1)).norm();
        dists(2) = (Pw_in.col(0) - Pw_in.col(2)).norm();
        Eigen::Index idx;
        dists.maxCoeff(&idx);

        Eigen::Matrix3d Pw = Pw_in;
        Eigen::Matrix3d Pi = Pi_in;
        if (idx == 1) {
            Pw = (Eigen::Matrix3d() << Pw_in.col(1), Pw_in.col(2), Pw_in.col(0)).finished();
            Pi = (Eigen::Matrix3d() << Pi_in.col(1), Pi_in.col(2), Pi_in.col(0)).finished();
        } else if (idx == 2) {
            Pw = (Eigen::Matrix3d() << Pw_in.col(0), Pw_in.col(2), Pw_in.col(1)).finished();
            Pi = (Eigen::Matrix3d() << Pi_in.col(0), Pi_in.col(2), Pi_in.col(1)).finished();
        }

        // 2) Plane frame N
        const Eigen::Vector3d X21 = Pw.col(1) - Pw.col(0);
        const Eigen::Vector3d X31 = Pw.col(2) - Pw.col(0);
        const Eigen::Vector3d nx = X21.normalized();
        Eigen::Vector3d nz = nx.cross(X31);
        nz /= nz.norm();
        const Eigen::Vector3d ny = nz.cross(nx);
        const Eigen::Matrix3d N = (Eigen::Matrix3d() << nx, ny, nz).finished();

        // 3) Scalars
        const double a = nx.dot(X21);
        const double b = nx.dot(X31);
        const double c = ny.dot(X31);

        const double M12 = Pi.col(0).dot(Pi.col(1));
        const double M13 = Pi.col(0).dot(Pi.col(2));
        const double M23 = Pi.col(1).dot(Pi.col(2));
        const double p = b / a;
        const double q = (b * b + c * c) / (a * a);

        // 4) Build f and g
        double f[6], g[6];
        f[0] = p;        
        f[1] = -M23;    
        f[2] = 0.0;
        f[3] = -M12 * (2.0 * p - 1.0);
        f[4] = M13;
        f[5] = p - 1.0;

        g[0] = q;        
        g[1] = 0.0;     
        g[2] = -1.0;
        g[3] = -2.0 * M12 * q;
        g[4] = 2.0 * M13;
        g[5] = q - 1.0;

        // 5) Quartic coefficients
        std::array<std::complex<double>, 5> h;
        h[0] = -f[0]*f[0] + g[0]*f[1]*f[1];
        h[1] = f[1]*f[1]*g[3] - 2.0*f[0]*f[3] - 2.0*f[0]*f[1]*f[4] + 2.0*f[1]*f[4]*g[0];
        h[2] = f[4]*f[4]*g[0] - 2.0*f[0]*f[4]*f[4] - 2.0*f[0]*f[5] + f[1]*f[1]*g[5] - f[3]*f[3] - 2.0*f[1]*f[3]*f[4] + 2.0*f[1]*f[4]*g[3];
        h[3] = f[4]*f[4]*g[3] - 2.0*f[3]*f[4]*f[4] - 2.0*f[3]*f[5] - 2.0*f[1]*f[4]*f[5] + 2.0*f[1]*f[4]*g[5];
        h[4] = -2.0*f[4]*f[4]*f[5] + g[5]*f[4]*f[4] - f[5]*f[5];

        // 6) Solve quartic and filter positive real roots
        std::vector<Solution> sols;
        std::vector<std::pair<double,double>> xy_candidates;
        {
            auto rts = poly_quartic::roots(h);

            for (const auto& rt : rts) {
                if (std::abs(rt.imag()) < 1e-8) {
                    double x = rt.real();
                    if (x > 0.0) {
                        const double denom = f[4] + f[1] * x;
                        const double y = -(((f[0]*x + f[3]) * x) + f[5]) / denom;
                        xy_candidates.emplace_back(x, y);
                    }
                }
            }
        }

        // 7) Root polishing (Gauss-Newton)
        if (!xy_candidates.empty() && polishing > 0) {
            auto polish_once = [&](double& x, double& y) {
                const double x2 = x * x;
                const double xy = x * y;
                const double fv = f[0]*x2 + f[1]*xy + f[3]*x + f[4]*y + f[5];
                const double gv = g[0]*x2 - y*y   + g[3]*x + g[4]*y + g[5];
                if (std::abs(fv) < 1e-15 && std::abs(gv) < 1e-15) return;
                const double dfdx = 2.0*f[0]*x + f[1]*y + f[3];
                const double dfdy = f[1]*x + f[4];
                const double dgdx = 2.0*g[0]*x + g[3];
                const double dgdy = -2.0*y + g[4];
                const double inv_detJ = 1.0 / (dfdx*dgdy - dfdy*dgdx);
                const double dx = ( dgdy*fv - dfdy*gv) * inv_detJ;
                const double dy = (-dgdx*fv + dfdx*gv) * inv_detJ;
                x -= dx;
                y -= dy;
            };

            for (auto& xy : xy_candidates) {
                for (int it = 0; it < polishing; ++it) polish_once(xy.first, xy.second);
            }
        }

        // 8) Recover motion for each (x,y)
        for (const auto& xy : xy_candidates) {
            const double x = xy.first;
            const double y = xy.second;

            const Eigen::Vector3d A_lam = -Pi.col(0) + Pi.col(1) * x;
            const double s = A_lam.norm() / a;
            const double d0 = 1.0 / s;
            const double d1 = x / s;
            const double d2 = y / s;

            const Eigen::Vector3d Ad = (-Pi.col(0) * d0 + Pi.col(1) * d1);
            const Eigen::Vector3d r1 = Ad / a;

            const Eigen::Vector3d Bd = (-Pi.col(0) * d0 + Pi.col(2) * d2);
            const Eigen::Vector3d r2 = (Bd - p * Ad) / c;

            const Eigen::Vector3d r3 = r1.cross(r2);

            Eigen::Matrix3d Rc;
            Rc.col(0) = r1;
            Rc.col(1) = r2;
            Rc.col(2) = r3;

            const Eigen::Vector3d tc = d0 * Pi.col(0);
            const Eigen::Matrix3d R = Rc * N.transpose();
            const Eigen::Vector3d t = tc - R * Pw.col(0);

            sols.emplace_back(Solution{R, t});
        }

        return sols;
    }

    // Analytic Jacobian of pose wrt 3 bearing vectors Pi (column-major ordering).
    // Returns up to 4 solutions with 6x9 Jacobians in se(3) small-angle form.
    /**
     * @brief Differentiate P3P pose solutions with respect to bearing entries.
     *
     * The derivative follows the same algebraic depth equations used by solve().
     */
    static std::vector<PoseJacobian> jacobianPoseWrtBearings(
        const Eigen::Matrix3d& Pw_in,
        const Eigen::Matrix3d& Pi_in,
        int polishing = 1)
    {
        // 1) Reorder to maximize baseline
        Eigen::Vector3d dists;
        dists(0) = (Pw_in.col(1) - Pw_in.col(0)).norm();
        dists(1) = (Pw_in.col(2) - Pw_in.col(1)).norm();
        dists(2) = (Pw_in.col(0) - Pw_in.col(2)).norm();
        Eigen::Index idx;
        dists.maxCoeff(&idx);

        Eigen::Matrix3d Pw = Pw_in;
        Eigen::Matrix3d Pi = Pi_in;
        std::array<int, 3> perm = {0, 1, 2};
        if (idx == 1) {
            Pw = (Eigen::Matrix3d() << Pw_in.col(1), Pw_in.col(2), Pw_in.col(0)).finished();
            Pi = (Eigen::Matrix3d() << Pi_in.col(1), Pi_in.col(2), Pi_in.col(0)).finished();
            perm = {1, 2, 0};
        } else if (idx == 2) {
            Pw = (Eigen::Matrix3d() << Pw_in.col(0), Pw_in.col(2), Pw_in.col(1)).finished();
            Pi = (Eigen::Matrix3d() << Pi_in.col(0), Pi_in.col(2), Pi_in.col(1)).finished();
            perm = {0, 2, 1};
        }

        // 2) Plane frame N
        const Eigen::Vector3d X21 = Pw.col(1) - Pw.col(0);
        const Eigen::Vector3d X31 = Pw.col(2) - Pw.col(0);
        const Eigen::Vector3d nx = X21.normalized();
        Eigen::Vector3d nz = nx.cross(X31);
        nz /= nz.norm();
        const Eigen::Vector3d ny = nz.cross(nx);
        const Eigen::Matrix3d N = (Eigen::Matrix3d() << nx, ny, nz).finished();

        // 3) Scalars
        const double a = nx.dot(X21);
        const double b = nx.dot(X31);
        const double c = ny.dot(X31);

        const double M12 = Pi.col(0).dot(Pi.col(1));
        const double M13 = Pi.col(0).dot(Pi.col(2));
        const double M23 = Pi.col(1).dot(Pi.col(2));
        const double p = b / a;
        const double q = (b * b + c * c) / (a * a);

        // 4) Build f and g
        double f[6], g[6];
        f[0] = p;        
        f[1] = -M23;    
        f[2] = 0.0;
        f[3] = -M12 * (2.0 * p - 1.0);
        f[4] = M13;
        f[5] = p - 1.0;

        g[0] = q;        
        g[1] = 0.0;     
        g[2] = -1.0;
        g[3] = -2.0 * M12 * q;
        g[4] = 2.0 * M13;
        g[5] = q - 1.0;

        // 5) Quartic coefficients
        std::array<std::complex<double>, 5> h;
        h[0] = -f[0]*f[0] + g[0]*f[1]*f[1];
        h[1] = f[1]*f[1]*g[3] - 2.0*f[0]*f[3] - 2.0*f[0]*f[1]*f[4] + 2.0*f[1]*f[4]*g[0];
        h[2] = f[4]*f[4]*g[0] - 2.0*f[0]*f[4]*f[4] - 2.0*f[0]*f[5] + f[1]*f[1]*g[5] - f[3]*f[3] - 2.0*f[1]*f[3]*f[4] + 2.0*f[1]*f[4]*g[3];
        h[3] = f[4]*f[4]*g[3] - 2.0*f[3]*f[4]*f[4] - 2.0*f[3]*f[5] - 2.0*f[1]*f[4]*f[5] + 2.0*f[1]*f[4]*g[5];
        h[4] = -2.0*f[4]*f[4]*f[5] + g[5]*f[4]*f[4] - f[5]*f[5];

        auto rts = poly_quartic::roots(h);
        const auto Jr = poly_quartic::jacobian_roots_wrt_coeffs(h, rts);

        std::vector<PoseJacobian> out;
        out.reserve(4);

        auto permute_cols = [&](const Eigen::Matrix<double, 6, 9>& Jin) {
            Eigen::Matrix<double, 6, 9> Jout;
            Jout.setZero();
            for (int j = 0; j < 3; ++j) {
                Jout.block<6, 3>(0, 3 * perm[j]) = Jin.block<6, 3>(0, 3 * j);
            }
            return Jout;
        };

        // Helper to compute omega from dR
        auto omega_from_dR = [](const Eigen::Matrix3d& R, const Eigen::Matrix3d& dR) {
            Eigen::Matrix3d S = R.transpose() * dR - dR.transpose() * R;
            Eigen::Vector3d w;
            w << S(2,1), S(0,2), S(1,0);
            return (0.5 * w).eval();
        };

        for (int root_idx = 0; root_idx < 4; ++root_idx) {
            const auto& rt = rts[root_idx];
            if (std::abs(rt.imag()) >= 1e-8) {
                continue;
            }
            double x = rt.real();
            if (x <= 0.0) {
                continue;
            }
            const double denom = f[4] + f[1] * x;
            double y = -(((f[0]*x + f[3]) * x) + f[5]) / denom;

            // Optional polishing
            if (polishing > 0) {
                auto polish_once = [&](double& px, double& py) {
                    const double x2 = px * px;
                    const double xy = px * py;
                    const double fv = f[0]*x2 + f[1]*xy + f[3]*px + f[4]*py + f[5];
                    const double gv = g[0]*x2 - py*py   + g[3]*px + g[4]*py + g[5];
                    if (std::abs(fv) < 1e-15 && std::abs(gv) < 1e-15) return;
                    const double dfdx = 2.0*f[0]*px + f[1]*py + f[3];
                    const double dfdy = f[1]*px + f[4];
                    const double dgdx = 2.0*g[0]*px + g[3];
                    const double dgdy = -2.0*py + g[4];
                    const double inv_detJ = 1.0 / (dfdx*dgdy - dfdy*dgdx);
                    const double dx = ( dgdy*fv - dfdy*gv) * inv_detJ;
                    const double dy = (-dgdx*fv + dfdx*gv) * inv_detJ;
                    px -= dx;
                    py -= dy;
                };
                for (int it = 0; it < polishing; ++it) {
                    polish_once(x, y);
                }
            }

            Eigen::Matrix<double, 6, 9> J;
            J.setZero();

            for (int k = 0; k < 9; ++k) {
                const int col = k / 3;
                const int row = k % 3;
                Eigen::Vector3d e = Eigen::Vector3d::Zero();
                e(row) = 1.0;
                Eigen::Vector3d dPi0 = (col == 0) ? e : Eigen::Vector3d::Zero();
                Eigen::Vector3d dPi1 = (col == 1) ? e : Eigen::Vector3d::Zero();
                Eigen::Vector3d dPi2 = (col == 2) ? e : Eigen::Vector3d::Zero();

                // dM
                const double dM12 = dPi0.dot(Pi.col(1)) + Pi.col(0).dot(dPi1);
                const double dM13 = dPi0.dot(Pi.col(2)) + Pi.col(0).dot(dPi2);
                const double dM23 = dPi1.dot(Pi.col(2)) + Pi.col(1).dot(dPi2);

                // df/dg components
                const double df1 = -dM23;
                const double df3 = -(2.0 * p - 1.0) * dM12;
                const double df4 = dM13;
                const double dg3 = -2.0 * q * dM12;
                const double dg4 = 2.0 * dM13;

                // dh via chain rule
                const double dh0 = 2.0 * g[0] * f[1] * df1;

                const double dh1 = (2.0 * f[1] * g[3] - 2.0 * f[0] * f[4] + 2.0 * f[4] * g[0]) * df1
                                 + (-2.0 * f[0]) * df3
                                 + (-2.0 * f[0] * f[1] + 2.0 * f[1] * g[0]) * df4
                                 + (f[1] * f[1]) * dg3;

                const double dh2 = (2.0 * f[1] * g[5] - 2.0 * f[3] * f[4] + 2.0 * f[4] * g[3]) * df1
                                 + (-2.0 * f[3] - 2.0 * f[1] * f[4]) * df3
                                 + (2.0 * f[4] * g[0] - 4.0 * f[0] * f[4] - 2.0 * f[1] * f[3] + 2.0 * f[1] * g[3]) * df4
                                 + (2.0 * f[1] * f[4]) * dg3;

                const double dh3 = (-2.0 * f[4] * f[5] + 2.0 * f[4] * g[5]) * df1
                                 + (-2.0 * f[4] * f[4] - 2.0 * f[5]) * df3
                                 + (2.0 * f[4] * g[3] - 4.0 * f[3] * f[4] - 2.0 * f[1] * f[5] + 2.0 * f[1] * g[5]) * df4
                                 + (f[4] * f[4]) * dg3;

                const double dh4 = 2.0 * f[4] * (g[5] - 2.0 * f[5]) * df4;

                // dx from quartic root Jacobian dr/dh
                std::complex<double> dx_c = Jr(root_idx, 0) * dh4
                                          + Jr(root_idx, 1) * dh3
                                          + Jr(root_idx, 2) * dh2
                                          + Jr(root_idx, 3) * dh1
                                          + Jr(root_idx, 4) * dh0;
                double dx = dx_c.real();

                // dy from explicit formula (pre-polish)
                const double num = (f[0] * x + f[3]) * x + f[5];
                const double dnum = 2.0 * f[0] * x * dx + df3 * x + f[3] * dx;
                const double dden = df4 + df1 * x + f[1] * dx;
                double dy = -((dnum * denom - num * dden) / (denom * denom));

                // If polished, prefer implicit differentiation at final (x,y)
                if (polishing > 0) {
                    const double dfdx = 2.0 * f[0] * x + f[1] * y + f[3];
                    const double dfdy = f[1] * x + f[4];
                    const double dgdx = 2.0 * g[0] * x + g[3];
                    const double dgdy = -2.0 * y + g[4];
                    const double det = dfdx * dgdy - dfdy * dgdx;
                    if (std::abs(det) > 1e-15) {
                        const double df_pi = df1 * x * y + df3 * x + df4 * y;
                        const double dg_pi = dg3 * x + dg4 * y;
                        dx = (-df_pi * dgdy + dfdy * dg_pi) / det;
                        dy = (dgdx * df_pi - dfdx * dg_pi) / det;
                    }
                }

                // Recover motion differentials
                const Eigen::Vector3d A_lam = -Pi.col(0) + Pi.col(1) * x;
                const Eigen::Vector3d dA = -dPi0 + dPi1 * x + Pi.col(1) * dx;
                const double A_norm = A_lam.norm();
                const double dA_norm = A_lam.dot(dA) / A_norm;

                const double s = A_norm / a;
                const double ds = dA_norm / a;
                const double d0 = 1.0 / s;
                const double dd0 = -d0 * d0 * ds;
                const double d1 = x * d0;
                const double dd1 = dx * d0 + x * dd0;
                const double d2 = y * d0;
                const double dd2 = dy * d0 + y * dd0;

                const Eigen::Vector3d Ad = (-Pi.col(0) * d0 + Pi.col(1) * d1);
                const Eigen::Vector3d dAd = -dPi0 * d0 - Pi.col(0) * dd0 + dPi1 * d1 + Pi.col(1) * dd1;
                const Eigen::Vector3d r1 = Ad / a;
                const Eigen::Vector3d dr1 = dAd / a;

                const Eigen::Vector3d Bd = (-Pi.col(0) * d0 + Pi.col(2) * d2);
                const Eigen::Vector3d dBd = -dPi0 * d0 - Pi.col(0) * dd0 + dPi2 * d2 + Pi.col(2) * dd2;
                const Eigen::Vector3d r2 = (Bd - p * Ad) / c;
                const Eigen::Vector3d dr2 = (dBd - p * dAd) / c;

                const Eigen::Vector3d r3 = r1.cross(r2);
                const Eigen::Vector3d dr3 = dr1.cross(r2) + r1.cross(dr2);

                Eigen::Matrix3d Rc;
                Rc.col(0) = r1;
                Rc.col(1) = r2;
                Rc.col(2) = r3;
                Eigen::Matrix3d dRc;
                dRc.col(0) = dr1;
                dRc.col(1) = dr2;
                dRc.col(2) = dr3;

                const Eigen::Vector3d dtc = dd0 * Pi.col(0) + d0 * dPi0;
                const Eigen::Matrix3d R = Rc * N.transpose();
                const Eigen::Matrix3d dR = dRc * N.transpose();
                const Eigen::Vector3d dt = dtc - dR * Pw.col(0);

                const Eigen::Vector3d domega = omega_from_dR(R, dR);

                J.block<3,1>(0, k) = domega;
                J.block<3,1>(3, k) = dt;
            }

            const Eigen::Vector3d A_lam = -Pi.col(0) + Pi.col(1) * x;
            const double s = A_lam.norm() / a;
            const double d0 = 1.0 / s;
            const double d1 = x / s;
            const double d2 = y / s;

            const Eigen::Vector3d Ad = (-Pi.col(0) * d0 + Pi.col(1) * d1);
            const Eigen::Vector3d r1 = Ad / a;

            const Eigen::Vector3d Bd = (-Pi.col(0) * d0 + Pi.col(2) * d2);
            const Eigen::Vector3d r2 = (Bd - p * Ad) / c;

            const Eigen::Vector3d r3 = r1.cross(r2);

            Eigen::Matrix3d Rc;
            Rc.col(0) = r1;
            Rc.col(1) = r2;
            Rc.col(2) = r3;

            const Eigen::Vector3d tc = d0 * Pi.col(0);
            const Eigen::Matrix3d R = Rc * N.transpose();
            const Eigen::Vector3d t = tc - R * Pw.col(0);

            PoseJacobian pj;
            pj.sol = Solution{R, t};
            pj.dpose_dpi = permute_cols(J);
            out.emplace_back(pj);
        }

        return out;
    }

    // Analytic Jacobian of bearings wrt pose from inverse-function chain rule.
    /**
     * @brief Approximate inverse sensitivity from pose tangent to bearing entries.
     */
    static std::vector<BearingJacobian> jacobianBearingsWrtPose(
        const Eigen::Matrix3d& Pw_in,
        const Eigen::Matrix3d& Pi_in,
        int polishing = 1)
    {
        std::vector<BearingJacobian> out;
        auto pose_jacs = jacobianPoseWrtBearings(Pw_in, Pi_in, polishing);
        out.reserve(pose_jacs.size());

        Eigen::Vector3d dists;
        dists(0) = (Pw_in.col(1) - Pw_in.col(0)).norm();
        dists(1) = (Pw_in.col(2) - Pw_in.col(1)).norm();
        dists(2) = (Pw_in.col(0) - Pw_in.col(2)).norm();
        Eigen::Index idx;
        dists.maxCoeff(&idx);

        Eigen::Matrix3d Pw = Pw_in;
        Eigen::Matrix3d Pi = Pi_in;
        if (idx == 1) {
            Pw = (Eigen::Matrix3d() << Pw_in.col(1), Pw_in.col(2), Pw_in.col(0)).finished();
            Pi = (Eigen::Matrix3d() << Pi_in.col(1), Pi_in.col(2), Pi_in.col(0)).finished();
        } else if (idx == 2) {
            Pw = (Eigen::Matrix3d() << Pw_in.col(0), Pw_in.col(2), Pw_in.col(1)).finished();
            Pi = (Eigen::Matrix3d() << Pi_in.col(0), Pi_in.col(2), Pi_in.col(1)).finished();
        }

        const Eigen::Vector3d X21 = Pw.col(1) - Pw.col(0);
        const Eigen::Vector3d X31 = Pw.col(2) - Pw.col(0);
        const Eigen::Vector3d nx = X21.normalized();
        Eigen::Vector3d nz = nx.cross(X31);
        nz /= nz.norm();
        const Eigen::Vector3d ny = nz.cross(nx);

        const double a = nx.dot(X21);
        const double b = nx.dot(X31);
        const double c = ny.dot(X31);

        const double M12 = Pi.col(0).dot(Pi.col(1));
        const double M13 = Pi.col(0).dot(Pi.col(2));
        const double M23 = Pi.col(1).dot(Pi.col(2));
        const double p = b / a;
        const double q = (b * b + c * c) / (a * a);

        double f[6], g[6];
        f[0] = p;
        f[1] = -M23;
        f[2] = 0.0;
        f[3] = -M12 * (2.0 * p - 1.0);
        f[4] = M13;
        f[5] = p - 1.0;

        g[0] = q;
        g[1] = 0.0;
        g[2] = -1.0;
        g[3] = -2.0 * M12 * q;
        g[4] = 2.0 * M13;
        g[5] = q - 1.0;

        std::array<std::complex<double>, 5> h;
        h[0] = -f[0]*f[0] + g[0]*f[1]*f[1];
        h[1] = f[1]*f[1]*g[3] - 2.0*f[0]*f[3] - 2.0*f[0]*f[1]*f[4] + 2.0*f[1]*f[4]*g[0];
        h[2] = f[4]*f[4]*g[0] - 2.0*f[0]*f[4]*f[4] - 2.0*f[0]*f[5] + f[1]*f[1]*g[5] - f[3]*f[3] - 2.0*f[1]*f[3]*f[4] + 2.0*f[1]*f[4]*g[3];
        h[3] = f[4]*f[4]*g[3] - 2.0*f[3]*f[4]*f[4] - 2.0*f[3]*f[5] - 2.0*f[1]*f[4]*f[5] + 2.0*f[1]*f[4]*g[5];
        h[4] = -2.0*f[4]*f[4]*f[5] + g[5]*f[4]*f[4] - f[5]*f[5];

        const auto rts = poly_quartic::roots(h);
        const auto Jc = poly_quartic::jacobian_coeffs_wrt_roots(h, rts);

        std::vector<int> valid_root_indices;
        valid_root_indices.reserve(4);
        for (int root_idx = 0; root_idx < 4; ++root_idx) {
            const auto& rt = rts[root_idx];
            if (std::abs(rt.imag()) >= 1e-8) {
                continue;
            }
            double x = rt.real();
            if (x <= 0.0) {
                continue;
            }
            double y = -(((f[0]*x + f[3]) * x) + f[5]) / (f[4] + f[1] * x);
            if (polishing > 0) {
                auto polish_once = [&](double& px, double& py) {
                    const double x2 = px * px;
                    const double xy = px * py;
                    const double fv = f[0]*x2 + f[1]*xy + f[3]*px + f[4]*py + f[5];
                    const double gv = g[0]*x2 - py*py   + g[3]*px + g[4]*py + g[5];
                    if (std::abs(fv) < 1e-15 && std::abs(gv) < 1e-15) return;
                    const double dfdx = 2.0*f[0]*px + f[1]*py + f[3];
                    const double dfdy = f[1]*px + f[4];
                    const double dgdx = 2.0*g[0]*px + g[3];
                    const double dgdy = -2.0*py + g[4];
                    const double inv_detJ = 1.0 / (dfdx*dgdy - dfdy*dgdx);
                    const double dx = ( dgdy*fv - dfdy*gv) * inv_detJ;
                    const double dy = (-dgdx*fv + dfdx*gv) * inv_detJ;
                    px -= dx;
                    py -= dy;
                };
                for (int it = 0; it < polishing; ++it) {
                    polish_once(x, y);
                }
            }
            if (x > 0.0) {
                valid_root_indices.push_back(root_idx);
            }
        }

        for (std::size_t i = 0; i < pose_jacs.size(); ++i) {
            const auto& pj = pose_jacs[i];
            BearingJacobian bj;
            bj.sol = pj.sol;
            const Eigen::Matrix<double, 6, 9>& Jpose_pi = pj.dpose_dpi;
            Eigen::Matrix<double, 6, 6> JJt = Jpose_pi * Jpose_pi.transpose();

            double lambda = 1e-12;
            if (i < valid_root_indices.size()) {
                const int ridx = valid_root_indices[i];
                const double root_scale = Jc.col(ridx).norm();
                if (std::isfinite(root_scale) && root_scale > 1e-15) {
                    lambda = std::max(1e-14, 1e-10 / root_scale);
                }
            }

            JJt.diagonal().array() += lambda;
            bj.dpi_dpose = Jpose_pi.transpose() * JJt.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

            out.emplace_back(bj);
        }

        return out;
    }
};

} // namespace uvdar_core::pose_estimation::geometric_solver

#endif // P3P_HPP
