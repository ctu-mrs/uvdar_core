#pragma once
#ifndef POLY_QUARTIC_HPP
#define POLY_QUARTIC_HPP

#include <utility>
#include <vector>
#include <cmath>
#include <cstdint>
#include <tuple>
#include <complex>
#include <algorithm>
#include <limits>
#include <Eigen/Dense>

namespace uvdar_core::helpers::poly_quartic {

/**
 * @brief Small algebraic helpers for quartic equations used by P3P/P4P.
 *
 * Coefficients are stored in ascending order, c0 + c1 x + ... + c4 x^4. The
 * root Jacobians follow implicit differentiation of f(root, coeffs) = 0.
 */
    /**
     * @brief Closed-form Ferrari roots of a quartic polynomial.
     */
    template <typename T>
    std::array<std::complex<T>, 4> roots(const std::array<std::complex<T>, 5>& coeffs) {
        auto c3 = coeffs[3] / coeffs[4]; 
        auto c2 = coeffs[2] / coeffs[4]; 
        auto c1 = coeffs[1] / coeffs[4]; 
        auto c0 = coeffs[0] / coeffs[4];
        auto c4_q = 0.25 * c3;
        auto c4_q2 = c4_q * c4_q;
        auto p = 3.0 * c4_q2 - 0.5 * c2;
        auto q = c3 * c4_q2 - c2 * c4_q + 0.5 * c1;
        auto r = 3.0 * c4_q2 * c4_q2 - c2 * c4_q2 + c1 * c4_q - c0;
        auto o = p * r - 0.5 * q * q;
        auto p2 = (3.0 * r - p * p) / 3.0;
        auto q2 = (2.0 * p * p * p - 9.0 * p * r + 27.0 * o) / 27.0;
        auto q2_h = -0.5 * q2;
        auto d = std::pow(q2_h * q2_h + (p2 / 3.0) * (p2 / 3.0) * (p2 / 3.0), 0.5);
        auto u = std::pow(q2_h + d, 1.0/3.0);
        auto v = (std::abs(p2) > 1e-15) ? (-p2 / (3.0 * u)) : std::pow(q2_h - d, 1.0/3.0);
        auto z0 = u + v - p / 3.0;
        auto s = std::pow(2.0 * p + 2.0 * z0, 0.5);
        auto t = (std::abs(s) < 1e-15) ? (z0 * z0 + r) : (-q / s);
        auto s_h = 0.5 * s;
        auto d1 = std::pow(s_h * s_h - z0 - t, 0.5);
        auto d2 = std::pow(s_h * s_h - z0 + t, 0.5);
        return {-s_h - d1 - c4_q, -s_h + d1 - c4_q, s_h - d2 - c4_q, s_h + d2 - c4_q};
    }

    /**
     * @brief Differentiate roots with respect to polynomial coefficients.
     *
     * Uses dx/dc_i = -x^i / f'(x), valid for simple roots.
     */
    template <typename T>
    Eigen::Matrix<std::complex<T>, 4, 5> jacobian_roots_wrt_coeffs(
        const std::array<std::complex<T>, 5>& coeffs,
        const std::array<std::complex<T>, 4>& roots) {
        Eigen::Matrix<std::complex<T>, 4, 5> J;
        for (int k = 0; k < 4; ++k) {
            const std::complex<T> x = roots[k];
            const std::complex<T> fp = T(4) * coeffs[4] * x * x * x + T(3) * coeffs[3] * x * x + T(2) * coeffs[2] * x + coeffs[1];
            if (std::abs(fp) < std::numeric_limits<T>::epsilon()) {
                J.row(k).setConstant(std::complex<T>(std::numeric_limits<T>::quiet_NaN(), T(0)));
                continue;
            }
            const std::complex<T> inv = -T(1) / fp;
            J(k, 0) = inv * x * x * x * x;
            J(k, 1) = inv * x * x * x;
            J(k, 2) = inv * x * x;
            J(k, 3) = inv * x;
            J(k, 4) = inv;
        }
        return J;
    }

    /**
     * @brief Differentiate polynomial coefficients with respect to roots.
     */
    template <typename T>
    Eigen::Matrix<std::complex<T>, 5, 4> jacobian_coeffs_wrt_roots(
        const std::array<std::complex<T>, 5>& coeffs,
        const std::array<std::complex<T>, 4>& roots) {
        Eigen::Matrix<std::complex<T>, 5, 4> J;
        J.row(0).setZero();
        for (int k = 0; k < 4; ++k) {
            std::complex<T> sum1 = T(0);
            std::complex<T> sum2 = T(0);
            std::complex<T> prod3 = T(1);
            for (int i = 0; i < 4; ++i) {
                if (i == k) continue;
                const std::complex<T> xi = roots[i];
                sum1 += xi;
                prod3 *= xi;
            }
            for (int i = 0; i < 4; ++i) {
                if (i == k) continue;
                for (int j = i + 1; j < 4; ++j) {
                    if (j == k) continue;
                    sum2 += roots[i] * roots[j];
                }
            }

            J(1, k) = -coeffs[4];
            J(2, k) = coeffs[4] * sum1;
            J(3, k) = -coeffs[4] * sum2;
            J(4, k) = coeffs[4] * prod3;
        }
        return J;
    }
} // namespace uvdar_core::helpers::poly_quartic

#endif // POLY_QUARTIC_HPP
