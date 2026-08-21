#pragma once
#ifndef P2P_HPP
#define P2P_HPP

#include <vector>
#include <array>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include "uvdar_core/helpers/math.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/solver_types.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

/**
 * @brief Perspective-2-point solver with an additional normal constraint.
 *
 * Two 2D-3D correspondences leave a one-degree ambiguity. The supplied camera
 * and world normals close that ambiguity by constraining the plane spanned by
 * the two rays and the corresponding body-frame baseline. The implementation
 * exposes both Li-style and Sweeney-style variants plus finite analytic
 * Jacobian helpers used for future covariance propagation.
 */
class P2P {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 * @brief Select the algebraic P2P formulation.
	 */
	enum class Method {
		Sweeney = 0,
		Li = 1,
	};

	using Solution = PoseSolution;

	/**
	 * @brief Pose sensitivity with respect to two input bearing vectors.
	 */
	struct PoseJacobian {
		Solution sol;
		// 6x6 Jacobian: [omega(3), t(3)] wrt Pi entries (column-major, 2 bearings).
		Eigen::Matrix<double, 6, 6> dpose_dpi;
	};

	/**
	 * @brief Bearing sensitivity with respect to the local pose tangent.
	 */
	struct BearingJacobian {
		Solution sol;
		// 6x6 Jacobian: Pi entries (column-major, 2 bearings) wrt [omega(3), t(3)].
		Eigen::Matrix<double, 6, 6> dpi_dpose;
	};

	/**
	 * @brief Solve P2P from world points, camera bearings, and plane normals.
	 */
	static std::vector<Solution> solve(
		const Eigen::Matrix<double, 3, 2>& Pw,
		const Eigen::Matrix<double, 3, 2>& Pi,
		const Eigen::Vector3d& v_cam,
		const Eigen::Vector3d& v_world,
		Method method = Method::Li)
	{
		if (method == Method::Sweeney) {
			return solveSweeney(Pw, Pi, v_cam, v_world);
		}
		return solveLi(Pw, Pi, v_cam, v_world);
	}

	/**
	 * @brief Differentiate the returned poses with respect to bearing entries.
	 */
	static std::vector<PoseJacobian> jacobianPoseWrtBearings(
		const Eigen::Matrix<double, 3, 2>& Pw,
		const Eigen::Matrix<double, 3, 2>& Pi,
		const Eigen::Vector3d& v_cam,
		const Eigen::Vector3d& v_world,
		Method method = Method::Li)
	{
		if (method == Method::Sweeney) {
			return jacobianPoseWrtBearingsSweeney(Pw, Pi, v_cam, v_world);
		}
		return jacobianPoseWrtBearingsLi(Pw, Pi, v_cam, v_world);
	}

	/**
	 * @brief Differentiate bearing residuals with respect to pose tangent.
	 */
	static std::vector<BearingJacobian> jacobianBearingsWrtPose(
		const Eigen::Matrix<double, 3, 2>& Pw,
		const Eigen::Matrix<double, 3, 2>& Pi,
		const Eigen::Vector3d& v_cam,
		const Eigen::Vector3d& v_world,
		Method method = Method::Li)
	{
		std::vector<BearingJacobian> out;
		const auto pose_jacs = jacobianPoseWrtBearings(Pw, Pi, v_cam, v_world, method);
		out.reserve(pose_jacs.size());

		for (const auto& pj : pose_jacs) {
			BearingJacobian bj;
			bj.sol = pj.sol;
			bj.dpi_dpose = uvdar_core::helpers::dampedRightPseudoInverse(pj.dpose_dpi);
			out.emplace_back(bj);
		}

		return out;
	}

	static std::vector<Solution> solveSweeney(
		const Eigen::Matrix<double, 3, 2>& Pw,
		const Eigen::Matrix<double, 3, 2>& Pi,
		const Eigen::Vector3d& v_cam,
		const Eigen::Vector3d& v_world)
	{
		std::vector<Solution> sols;
		const auto prep = prealign(Pw, Pi, v_cam, v_world);
		if (!prep.ok) {
			return sols;
		}

		const Eigen::Vector3d p1 = prep.p1;
		const Eigen::Vector3d p2 = prep.p2;
		const Eigen::Vector3d Pw1 = prep.Pw1;
		const Eigen::Vector3d Pw2 = prep.Pw2;
		const Eigen::Vector3d dw_vec = Pw1 - Pw2;

		const double D_sq = dw_vec.squaredNorm();
		const double p1v = p1.z();
		const double p2v = p2.z();
		const double dw = dw_vec.z();

		if (std::abs(p1v) < 1e-15) {
			return sols;
		}

		const double m = dw / p1v;
		const double n = p2v / p1v;
		const double cos12 = p1.dot(p2);

		const double A = n * n - 2.0 * n * cos12 + 1.0;
		const double B = 2.0 * m * (n - cos12);
		const double C = m * m - D_sq;

		std::vector<double> lam2_vals;
		if (std::abs(A) < 1e-15) {
			if (std::abs(B) > 1e-15) {
				lam2_vals.emplace_back(-C / B);
			}
		} else {
			const double disc = B * B - 4.0 * A * C;
			if (disc < -1e-10) {
				return sols;
			}
			const double sd = std::sqrt(std::max(0.0, disc));
			lam2_vals.emplace_back((-B + sd) / (2.0 * A));
			lam2_vals.emplace_back((-B - sd) / (2.0 * A));
		}

		sols.reserve(lam2_vals.size());

		for (double lam2 : lam2_vals) {
			const double lam1 = m + n * lam2;
			if (lam1 <= 1e-12 || lam2 <= 1e-12) {
				continue;
			}

			const Eigen::Vector3d Pc1 = lam1 * p1;
			const Eigen::Vector3d Pc2 = lam2 * p2;
			const Eigen::Vector3d dc_vec = Pc1 - Pc2;

			const Eigen::Vector3d dw_proj(dw_vec.x(), dw_vec.y(), 0.0);
			const Eigen::Vector3d dc_proj(dc_vec.x(), dc_vec.y(), 0.0);
			const double dw_pn = dw_proj.norm();
			const double dc_pn = dc_proj.norm();

			double alpha = 0.0;
			if (dw_pn >= 1e-14 && dc_pn >= 1e-14) {
				const double cos_a = dw_proj.dot(dc_proj) / (dw_pn * dc_pn);
				const double sin_a = (dw_proj.x() * dc_proj.y() - dw_proj.y() * dc_proj.x()) / (dw_pn * dc_pn);
				alpha = std::atan2(sin_a, cos_a);
			}

			const Eigen::Matrix3d Rp = uvdar_core::helpers::rotationZ(alpha);

			const Eigen::Matrix3d R = prep.Rc.transpose() * Rp * prep.Rw;
			const Eigen::Vector3d t = prep.Rc.transpose() * (Pc1 - Rp * Pw1);
			sols.emplace_back(Solution{R, t});
		}

		return sols;
	}

	static std::vector<Solution> solveLi(
		const Eigen::Matrix<double, 3, 2>& Pw,
		const Eigen::Matrix<double, 3, 2>& Pi,
		const Eigen::Vector3d& v_cam,
		const Eigen::Vector3d& v_world)
	{
		std::vector<Solution> sols;
		const auto prep = prealign(Pw, Pi, v_cam, v_world);
		if (!prep.ok) {
			return sols;
		}

		const Eigen::Vector3d p1 = prep.p1;
		const Eigen::Vector3d p2 = prep.p2;
		const Eigen::Vector3d Pw1 = prep.Pw1;
		const Eigen::Vector3d delta = prep.Pw1 - prep.Pw2;
		const Eigen::Vector3d p_cross = p1.cross(p2);

		const double dx = delta.x();
		const double dy = delta.y();
		const double dz = delta.z();
		const double px = p_cross.x();
		const double py = p_cross.y();
		const double pz = p_cross.z();

		const double a1 = -dx * px - dy * py + dz * pz;
		const double a2 = 2.0 * dx * py - 2.0 * dy * px;
		const double a3 = dx * px + dy * py + dz * pz;

		std::vector<double> s_vals;
		if (std::abs(a1) < 1e-15) {
			if (std::abs(a2) > 1e-15) {
				s_vals.emplace_back(-a3 / a2);
			}
		} else {
			const double disc = a2 * a2 - 4.0 * a1 * a3;
			if (disc < -1e-10) {
				return sols;
			}
			const double sd = std::sqrt(std::max(0.0, disc));
			s_vals.emplace_back((-a2 + sd) / (2.0 * a1));
			s_vals.emplace_back((-a2 - sd) / (2.0 * a1));
		}

		sols.reserve(s_vals.size());
		for (double s : s_vals) {
			const double alpha = 2.0 * std::atan(s);
			const Eigen::Matrix3d Rp = uvdar_core::helpers::rotationZ(alpha);

			const Eigen::Vector3d R_delta = Rp * delta;
			const Eigen::Vector3d lhs = p2.cross(p1);
			const Eigen::Vector3d rhs = p2.cross(R_delta);

			const double lhs_sq = lhs.squaredNorm();
			if (lhs_sq < 1e-30) {
				continue;
			}

			const double lam1 = rhs.dot(lhs) / lhs_sq;
			if (lam1 <= 1e-12) {
				continue;
			}

			const double lam2 = (lam1 * p1 - R_delta).dot(p2);
			if (lam2 <= 1e-12) {
				continue;
			}

			const Eigen::Vector3d tp = lam1 * p1 - Rp * Pw1;
			const Eigen::Matrix3d R = prep.Rc.transpose() * Rp * prep.Rw;
			const Eigen::Vector3d t = prep.Rc.transpose() * tp;

			sols.emplace_back(Solution{R, t});
		}

		return sols;
	}

private:
	struct PrealignData {
		bool ok = false;
		Eigen::Matrix3d Rc;
		Eigen::Matrix3d Rw;
		Eigen::Vector3d p1;
		Eigen::Vector3d p2;
		Eigen::Vector3d Pw1;
		Eigen::Vector3d Pw2;
	};

	static PrealignData prealign(
		const Eigen::Matrix<double, 3, 2>& Pw,
		const Eigen::Matrix<double, 3, 2>& Pi,
		const Eigen::Vector3d& v_cam,
		const Eigen::Vector3d& v_world)
	{
		PrealignData out;
		const double nvc = v_cam.norm();
		const double nvw = v_world.norm();
		if (nvc < 1e-15 || nvw < 1e-15) {
			return out;
		}

		const Eigen::Vector3d z(0.0, 0.0, 1.0);
		out.Rc = uvdar_core::helpers::rotationBetween(v_cam / nvc, z);
		out.Rw = uvdar_core::helpers::rotationBetween(v_world / nvw, z);

		out.p1 = out.Rc * Pi.col(0);
		out.p2 = out.Rc * Pi.col(1);
		out.Pw1 = out.Rw * Pw.col(0);
		out.Pw2 = out.Rw * Pw.col(1);
		out.ok = true;
		return out;
	}

	static std::vector<PoseJacobian> jacobianPoseWrtBearingsLi(
		const Eigen::Matrix<double, 3, 2>& Pw,
		const Eigen::Matrix<double, 3, 2>& Pi,
		const Eigen::Vector3d& v_cam,
		const Eigen::Vector3d& v_world)
	{
		std::vector<PoseJacobian> out;
		const auto prep = prealign(Pw, Pi, v_cam, v_world);
		if (!prep.ok) {
			return out;
		}

		const Eigen::Vector3d p1 = prep.p1;
		const Eigen::Vector3d p2 = prep.p2;
		const Eigen::Vector3d Pw1 = prep.Pw1;
		const Eigen::Vector3d delta = prep.Pw1 - prep.Pw2;
		const Eigen::Vector3d p_cross = p1.cross(p2);

		const double dx = delta.x();
		const double dy = delta.y();
		const double dz = delta.z();
		const double a1 = -dx * p_cross.x() - dy * p_cross.y() + dz * p_cross.z();
		const double a2 =  2.0 * dx * p_cross.y() - 2.0 * dy * p_cross.x();
		const double a3 =  dx * p_cross.x() + dy * p_cross.y() + dz * p_cross.z();

		std::vector<double> s_vals;
		if (std::abs(a1) < 1e-15) {
			if (std::abs(a2) > 1e-15) {
				s_vals.emplace_back(-a3 / a2);
			}
		} else {
			const double disc = a2 * a2 - 4.0 * a1 * a3;
			if (disc < -1e-10) {
				return out;
			}
			const double sd = std::sqrt(std::max(0.0, disc));
			s_vals.emplace_back((-a2 + sd) / (2.0 * a1));
			s_vals.emplace_back((-a2 - sd) / (2.0 * a1));
		}

		const Eigen::Vector3d c1(-dx, -dy, dz);
		const Eigen::Vector3d c2(-2.0 * dy, 2.0 * dx, 0.0);
		const Eigen::Vector3d c3(dx, dy, dz);

		for (double s : s_vals) {
			const double alpha = 2.0 * std::atan(s);
			const Eigen::Matrix3d Rp = uvdar_core::helpers::rotationZ(alpha);
			const Eigen::Matrix3d dRp_da = uvdar_core::helpers::rotationZDerivative(alpha);
			const Eigen::Vector3d R_delta = Rp * delta;

			const Eigen::Vector3d lhs = p2.cross(p1);
			const Eigen::Vector3d rhs = p2.cross(R_delta);
			const double lhs_sq = lhs.squaredNorm();
			if (lhs_sq < 1e-30) {
				continue;
			}

			const double lam1 = rhs.dot(lhs) / lhs_sq;
			if (lam1 <= 1e-12) {
				continue;
			}

			const double lam2 = (lam1 * p1 - R_delta).dot(p2);
			if (lam2 <= 1e-12) {
				continue;
			}

			const Eigen::Matrix3d R = prep.Rc.transpose() * Rp * prep.Rw;
			const Eigen::Vector3d tp = lam1 * p1 - Rp * Pw1;
			const Eigen::Vector3d t = prep.Rc.transpose() * tp;

			Eigen::Matrix<double, 6, 6> J;
			J.setZero();

			for (int k = 0; k < 6; ++k) {
				const int col = k / 3;
				const int row = k % 3;
				Eigen::Vector3d e = Eigen::Vector3d::Zero();
				e(row) = 1.0;

				const Eigen::Vector3d dPi0 = (col == 0) ? e : Eigen::Vector3d::Zero();
				const Eigen::Vector3d dPi1 = (col == 1) ? e : Eigen::Vector3d::Zero();

				const Eigen::Vector3d dp1 = prep.Rc * dPi0;
				const Eigen::Vector3d dp2 = prep.Rc * dPi1;

				const Eigen::Vector3d dpc = dp1.cross(p2) + p1.cross(dp2);
				const double da1 = c1.dot(dpc);
				const double da2 = c2.dot(dpc);
				const double da3 = c3.dot(dpc);

				const double dFds = 2.0 * a1 * s + a2;
				if (std::abs(dFds) < 1e-15) {
					continue;
				}

				const double ds = -(da1 * s * s + da2 * s + da3) / dFds;
				const double dalpha = 2.0 * ds / (1.0 + s * s);

				const Eigen::Matrix3d dRp = dRp_da * dalpha;
				const Eigen::Vector3d dR_delta = dRp * delta;

				const Eigen::Vector3d dlhs = dp2.cross(p1) + p2.cross(dp1);
				const Eigen::Vector3d drhs = dp2.cross(R_delta) + p2.cross(dR_delta);

				const double num = rhs.dot(lhs);
				const double dnum = drhs.dot(lhs) + rhs.dot(dlhs);
				const double den = lhs_sq;
				const double dden = 2.0 * lhs.dot(dlhs);
				const double dlam1 = (dnum * den - num * dden) / (den * den);

				const Eigen::Vector3d u = lam1 * p1 - R_delta;
				const Eigen::Vector3d du = dlam1 * p1 + lam1 * dp1 - dR_delta;
				const double dlam2 = du.dot(p2) + u.dot(dp2);
				(void)dlam2;

				const Eigen::Vector3d dtp = dlam1 * p1 + lam1 * dp1 - dRp * Pw1;
				const Eigen::Matrix3d dR = prep.Rc.transpose() * dRp * prep.Rw;
				const Eigen::Vector3d dt = prep.Rc.transpose() * dtp;

				const Eigen::Vector3d domega = uvdar_core::helpers::omegaFromRotationDerivative(R, dR);
				J.block<3, 1>(0, k) = domega;
				J.block<3, 1>(3, k) = dt;
			}

			PoseJacobian pj;
			pj.sol = Solution{R, t};
			pj.dpose_dpi = J;
			out.emplace_back(pj);
		}

		return out;
	}

	static std::vector<PoseJacobian> jacobianPoseWrtBearingsSweeney(
		const Eigen::Matrix<double, 3, 2>& Pw,
		const Eigen::Matrix<double, 3, 2>& Pi,
		const Eigen::Vector3d& v_cam,
		const Eigen::Vector3d& v_world)
	{
		std::vector<PoseJacobian> out;
		const auto prep = prealign(Pw, Pi, v_cam, v_world);
		if (!prep.ok) {
			return out;
		}

		const Eigen::Vector3d p1 = prep.p1;
		const Eigen::Vector3d p2 = prep.p2;
		const Eigen::Vector3d Pw1 = prep.Pw1;
		const Eigen::Vector3d Pw2 = prep.Pw2;
		const Eigen::Vector3d dw_vec = Pw1 - Pw2;
		const Eigen::Vector3d dw_proj(dw_vec.x(), dw_vec.y(), 0.0);

		const double D_sq = dw_vec.squaredNorm();
		const double p1v = p1.z();
		const double p2v = p2.z();
		const double dw = dw_vec.z();
		if (std::abs(p1v) < 1e-15) {
			return out;
		}

		const double m = dw / p1v;
		const double n = p2v / p1v;
		const double cos12 = p1.dot(p2);

		const double A = n * n - 2.0 * n * cos12 + 1.0;
		const double B = 2.0 * m * (n - cos12);
		const double C = m * m - D_sq;

		std::vector<double> lam2_vals;
		if (std::abs(A) < 1e-15) {
			if (std::abs(B) > 1e-15) {
				lam2_vals.emplace_back(-C / B);
			}
		} else {
			const double disc = B * B - 4.0 * A * C;
			if (disc < -1e-10) {
				return out;
			}
			const double sd = std::sqrt(std::max(0.0, disc));
			lam2_vals.emplace_back((-B + sd) / (2.0 * A));
			lam2_vals.emplace_back((-B - sd) / (2.0 * A));
		}

		const double dw_pn = dw_proj.norm();

		for (double lam2 : lam2_vals) {
			const double lam1 = m + n * lam2;
			if (lam1 <= 1e-12 || lam2 <= 1e-12) {
				continue;
			}

			const Eigen::Vector3d dc_vec = lam1 * p1 - lam2 * p2;
			const Eigen::Vector3d dc_proj(dc_vec.x(), dc_vec.y(), 0.0);
			const double dc_pn = dc_proj.norm();

			double alpha = 0.0;
			if (dw_pn >= 1e-14 && dc_pn >= 1e-14) {
				const double cos_a = dw_proj.dot(dc_proj) / (dw_pn * dc_pn);
				const double sin_a = (dw_proj.x() * dc_proj.y() - dw_proj.y() * dc_proj.x()) / (dw_pn * dc_pn);
				alpha = std::atan2(sin_a, cos_a);
			}

			const Eigen::Matrix3d Rp = uvdar_core::helpers::rotationZ(alpha);
			const Eigen::Matrix3d dRp_da = uvdar_core::helpers::rotationZDerivative(alpha);

			const Eigen::Matrix3d R = prep.Rc.transpose() * Rp * prep.Rw;
			const Eigen::Vector3d Pc1 = lam1 * p1;
			const Eigen::Vector3d tp = Pc1 - Rp * Pw1;
			const Eigen::Vector3d t = prep.Rc.transpose() * tp;

			Eigen::Matrix<double, 6, 6> J;
			J.setZero();

			for (int k = 0; k < 6; ++k) {
				const int col = k / 3;
				const int row = k % 3;
				Eigen::Vector3d e = Eigen::Vector3d::Zero();
				e(row) = 1.0;

				const Eigen::Vector3d dPi0 = (col == 0) ? e : Eigen::Vector3d::Zero();
				const Eigen::Vector3d dPi1 = (col == 1) ? e : Eigen::Vector3d::Zero();

				const Eigen::Vector3d dp1 = prep.Rc * dPi0;
				const Eigen::Vector3d dp2 = prep.Rc * dPi1;

				const double dp1v = dp1.z();
				const double dp2v = dp2.z();
				const double dcos12 = dp1.dot(p2) + p1.dot(dp2);

				const double dm = -(dw / (p1v * p1v)) * dp1v;
				const double dn = (dp2v * p1v - p2v * dp1v) / (p1v * p1v);

				const double dA = 2.0 * n * dn - 2.0 * (dn * cos12 + n * dcos12);
				const double dB = 2.0 * (dm * (n - cos12) + m * (dn - dcos12));
				const double dC = 2.0 * m * dm;

				const double dFdl = 2.0 * A * lam2 + B;
				if (std::abs(dFdl) < 1e-15) {
					continue;
				}

				const double dlam2 = -(dA * lam2 * lam2 + dB * lam2 + dC) / dFdl;
				const double dlam1 = dm + dn * lam2 + n * dlam2;

				const Eigen::Vector3d ddc = dlam1 * p1 + lam1 * dp1 - dlam2 * p2 - lam2 * dp2;
				const Eigen::Vector3d ddc_proj(ddc.x(), ddc.y(), 0.0);

				double dalpha = 0.0;
				if (dw_pn >= 1e-14 && dc_pn >= 1e-14) {
					const Eigen::Vector3d u = dw_proj / dw_pn;
					const Eigen::Vector3d v(-u.y(), u.x(), 0.0);

					const double q = dc_pn;
					const double dq = (dc_proj.dot(ddc_proj)) / q;

					const double num_c = u.dot(dc_proj);
					const double num_s = v.dot(dc_proj);
					const double cos_a = num_c / q;
					const double sin_a = num_s / q;

					const double dcos_a = (u.dot(ddc_proj) * q - num_c * dq) / (q * q);
					const double dsin_a = (v.dot(ddc_proj) * q - num_s * dq) / (q * q);

					const double denom = cos_a * cos_a + sin_a * sin_a;
					if (denom > 1e-15) {
						dalpha = (cos_a * dsin_a - sin_a * dcos_a) / denom;
					}
				}

				const Eigen::Matrix3d dRp = dRp_da * dalpha;
				const Eigen::Vector3d dPc1 = dlam1 * p1 + lam1 * dp1;
				const Eigen::Vector3d dtp = dPc1 - dRp * Pw1;

				const Eigen::Matrix3d dR = prep.Rc.transpose() * dRp * prep.Rw;
				const Eigen::Vector3d dt = prep.Rc.transpose() * dtp;

				const Eigen::Vector3d domega = uvdar_core::helpers::omegaFromRotationDerivative(R, dR);
				J.block<3, 1>(0, k) = domega;
				J.block<3, 1>(3, k) = dt;
			}

			PoseJacobian pj;
			pj.sol = Solution{R, t};
			pj.dpose_dpi = J;
			out.emplace_back(pj);
		}

		return out;
	}
};

} // namespace uvdar_core::pose_estimation::geometric_solver

#endif // P2P_HPP
