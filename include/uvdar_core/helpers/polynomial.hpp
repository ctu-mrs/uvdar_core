#pragma once
#ifndef UVDAR_CORE_HELPERS_POLYNOMIAL_HPP
#define UVDAR_CORE_HELPERS_POLYNOMIAL_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace uvdar_core::helpers {

inline constexpr double polynomial_epsilon = 1.0e-15;

/** Evaluate coefficients stored as c0, c1, ..., cn. */
template <typename BidirectionalIterator, typename X>
auto evaluatePolynomialAscending(
    BidirectionalIterator first,
    BidirectionalIterator last,
    const X& x)
{
    using Coefficient =
        typename std::iterator_traits<BidirectionalIterator>::value_type;
    using Result = std::decay_t<
        decltype(std::declval<Coefficient>() * std::declval<X>())>;
    Result value {};
    while (last != first) {
        value = value * x + *--last;
    }
    return value;
}

/** Evaluate an ascending-order polynomial and its derivative together. */
template <typename BidirectionalIterator, typename X>
auto evaluatePolynomialAndDerivativeAscending(
    BidirectionalIterator first,
    BidirectionalIterator last,
    const X& x)
{
    using Coefficient =
        typename std::iterator_traits<BidirectionalIterator>::value_type;
    using Result = std::decay_t<
        decltype(std::declval<Coefficient>() * std::declval<X>())>;
    Result value {};
    Result derivative {};
    while (last != first) {
        derivative = derivative * x + value;
        value = value * x + *--last;
    }
    return std::pair<Result, Result> {value, derivative};
}

/** Stable real roots of ax^2 + bx + c, including the linear case. */
template <typename T>
std::vector<T> realQuadraticRoots(
    T a,
    T b,
    T c,
    T relative_tolerance =
        T(64) * std::numeric_limits<T>::epsilon())
{
    static_assert(std::is_floating_point_v<T>);
    const T scale = std::max({std::abs(a), std::abs(b), std::abs(c), T(1)});
    const T tolerance = relative_tolerance * scale;
    if (std::abs(a) <= tolerance) {
        return std::abs(b) <= tolerance
            ? std::vector<T> {}
            : std::vector<T> {-c / b};
    }

    T discriminant = b * b - T(4) * a * c;
    const T discriminant_scale =
        std::max({b * b, std::abs(T(4) * a * c), T(1)});
    if (discriminant < -relative_tolerance * discriminant_scale) {
        return {};
    }
    discriminant = std::max(discriminant, T(0));
    const T square_root = std::sqrt(discriminant);
    const T sign = b == T(0) ? T(1) : b;
    const T q = -T(0.5) * (b + std::copysign(square_root, sign));
    if (std::abs(q) <= tolerance) {
        return {-b / (T(2) * a)};
    }
    const T first = q / a;
    const T second = c / q;
    if (std::abs(first - second)
        <= relative_tolerance
            * std::max({std::abs(first), std::abs(second), T(1)})) {
        return {first};
    }
    return {first, second};
}

/**
 * Jacobian of roots with respect to descending coefficients.
 *
 * For p(x)=c0*x^n+...+cn, dr_k/dc_j=-r_k^(n-j)/p'(r_k).
 * The n by n+1 result is NaN on rows belonging to repeated roots.
 */
template <typename T>
Eigen::Matrix<std::complex<T>, Eigen::Dynamic, Eigen::Dynamic>
jacobianRootsWrtCoefficients(
    const std::vector<std::complex<T>>& coefficients,
    const std::vector<std::complex<T>>& roots,
    T tolerance = T(64) * std::numeric_limits<T>::epsilon())
{
    using Complex = std::complex<T>;
    using Matrix =
        Eigen::Matrix<Complex, Eigen::Dynamic, Eigen::Dynamic>;
    const std::size_t degree =
        coefficients.empty() ? 0U : coefficients.size() - 1U;
    if (roots.size() != degree) {
        throw std::invalid_argument(
            "Root count must equal polynomial degree");
    }

    Matrix jacobian(
        static_cast<Eigen::Index>(degree),
        static_cast<Eigen::Index>(degree + 1U));
    for (std::size_t k = 0U; k < degree; ++k) {
        const Complex root = roots[k];
        Complex value = coefficients.front();
        Complex derivative {};
        for (std::size_t j = 1U; j < coefficients.size(); ++j) {
            derivative = derivative * root + value;
            value = value * root + coefficients[j];
        }

        T scale = T(0);
        T root_power = T(1);
        for (std::size_t power = 1U; power <= degree; ++power) {
            scale += T(power)
                * std::abs(coefficients[degree - power]) * root_power;
            root_power *= std::max(T(1), std::abs(root));
        }
        if (std::abs(derivative)
            <= tolerance * std::max(T(1), scale)) {
            jacobian.row(static_cast<Eigen::Index>(k)).setConstant(
                Complex(std::numeric_limits<T>::quiet_NaN(), T(0)));
            continue;
        }

        Complex power(1, 0);
        for (std::size_t reverse = 0U; reverse <= degree; ++reverse) {
            const std::size_t j = degree - reverse;
            jacobian(
                static_cast<Eigen::Index>(k),
                static_cast<Eigen::Index>(j)) = -power / derivative;
            power *= root;
        }
    }
    return jacobian;
}

/**
 * Jacobian of descending coefficients with respect to roots.
 *
 * The leading coefficient is fixed. The (n+1) by n result follows
 * dp(x)/dr_k=-c0*product_{j!=k}(x-r_j), so its first row is zero.
 */
template <typename T>
Eigen::Matrix<std::complex<T>, Eigen::Dynamic, Eigen::Dynamic>
jacobianCoefficientsWrtRoots(
    const std::complex<T>& leading_coefficient,
    const std::vector<std::complex<T>>& roots)
{
    using Complex = std::complex<T>;
    using Matrix =
        Eigen::Matrix<Complex, Eigen::Dynamic, Eigen::Dynamic>;
    const std::size_t degree = roots.size();
    Matrix jacobian(
        static_cast<Eigen::Index>(degree + 1U),
        static_cast<Eigen::Index>(degree));
    jacobian.setZero();

    std::vector<Complex> monic {Complex(1, 0)};
    for (const Complex& root : roots) {
        monic.push_back(Complex(0, 0));
        for (std::size_t j = monic.size() - 1U; j > 0U; --j) {
            monic[j] -= root * monic[j - 1U];
        }
    }

    for (std::size_t k = 0U; k < degree; ++k) {
        std::vector<Complex> quotient(degree);
        quotient[0] = monic[0];
        for (std::size_t j = 1U; j < degree; ++j) {
            quotient[j] = monic[j] + roots[k] * quotient[j - 1U];
        }
        for (std::size_t j = 0U; j < degree; ++j) {
            jacobian(
                static_cast<Eigen::Index>(j + 1U),
                static_cast<Eigen::Index>(k)) =
                -leading_coefficient * quotient[j];
        }
    }
    return jacobian;
}

class Term {
public:
    using Complex = std::complex<double>;

    Term() = default;
    Term(Complex coefficient, int degree)
        : coefficient_(coefficient), degree_(degree)
    {
    }

    Complex coefficient() const { return coefficient_; }
    int degree() const { return degree_; }
    void setCoefficient(Complex coefficient) { coefficient_ = coefficient; }
    void setDegree(int degree) { degree_ = degree; }

    Complex evaluate(Complex x) const
    {
        return coefficient_ * std::pow(x, degree_);
    }
    Term operator-() const { return {-coefficient_, degree_}; }
    Term operator*(Complex scalar) const
    {
        return {coefficient_ * scalar, degree_};
    }
    friend Term operator*(Complex scalar, const Term& term)
    {
        return term * scalar;
    }

private:
    Complex coefficient_ {};
    int degree_ = 0;
};

/**
 * General complex polynomial with coefficients ordered highest degree first.
 *
 * The class provides evaluation, arithmetic, calculus, root solving, and
 * root/coefficient Jacobians through one coefficient representation.
 */
class Polynomial {
public:
    using Complex = std::complex<double>;
    using Coefficients = std::vector<Complex>;

    Polynomial() : coefficients_(1U, Complex {}) {}

    Polynomial(int length, const Complex* coefficients = nullptr)
        : coefficients_(
            length > 0 ? static_cast<std::size_t>(length) : 1U,
            Complex {})
    {
        if (coefficients != nullptr && length > 0) {
            std::copy(
                coefficients,
                coefficients + length,
                coefficients_.begin());
        }
        strip();
    }

    explicit Polynomial(Coefficients coefficients)
        : coefficients_(std::move(coefficients))
    {
        strip();
    }

    Polynomial(std::initializer_list<Complex> coefficients)
        : coefficients_(coefficients)
    {
        strip();
    }

    template <typename BidirectionalIterator>
    static Polynomial fromAscending(
        BidirectionalIterator first,
        BidirectionalIterator last)
    {
        Coefficients coefficients;
        coefficients.reserve(
            static_cast<std::size_t>(std::distance(first, last)));
        while (last != first) {
            coefficients.emplace_back(*--last);
        }
        return Polynomial(std::move(coefficients));
    }

    static Polynomial fromTerm(const Term& term)
    {
        Coefficients coefficients(
            static_cast<std::size_t>(std::max(0, term.degree())) + 1U,
            Complex {});
        coefficients.front() = term.coefficient();
        return Polynomial(std::move(coefficients));
    }

    int size() const { return static_cast<int>(coefficients_.size()); }
    int degree() const { return size() - 1; }
    int getDegree() const { return degree(); }
    const Coefficients& coefficients() const { return coefficients_; }

    const Complex& operator[](int index) const
    {
        return coefficients_[static_cast<std::size_t>(index)];
    }
    Complex& operator[](int index)
    {
        return coefficients_[static_cast<std::size_t>(index)];
    }

    Complex getCoefficientForPower(int power) const
    {
        return power < 0 || power > degree()
            ? Complex {}
            : coefficients_[
                static_cast<std::size_t>(degree() - power)];
    }

    void setCoefficientForPower(int power, Complex value)
    {
        if (power >= 0 && power <= degree()) {
            coefficients_[
                static_cast<std::size_t>(degree() - power)] = value;
        }
    }

    int getPowerAtIndex(int index) const
    {
        if (index < 0 || index >= size()) {
            throw std::out_of_range("Polynomial index out of range");
        }
        return degree() - index;
    }

    Complex evaluate(Complex x) const
    {
        Complex value {};
        for (const Complex coefficient : coefficients_) {
            value = value * x + coefficient;
        }
        return value;
    }

    std::pair<Complex, Complex> evaluateWithDerivative(Complex x) const
    {
        Complex value {};
        Complex derivative {};
        for (const Complex coefficient : coefficients_) {
            derivative = derivative * x + value;
            value = value * x + coefficient;
        }
        return {value, derivative};
    }

    Polynomial derivative() const
    {
        if (degree() <= 0) {
            return {};
        }
        Coefficients result(static_cast<std::size_t>(degree()));
        for (int index = 0; index < degree(); ++index) {
            result[static_cast<std::size_t>(index)] =
                coefficients_[static_cast<std::size_t>(index)]
                * static_cast<double>(degree() - index);
        }
        return Polynomial(std::move(result));
    }

    Polynomial operator+() const { return *this; }

    Polynomial operator-() const
    {
        Coefficients result = coefficients_;
        for (Complex& coefficient : result) {
            coefficient = -coefficient;
        }
        return Polynomial(std::move(result));
    }

    Polynomial operator+(const Polynomial& other) const
    {
        const std::size_t result_size =
            std::max(coefficients_.size(), other.coefficients_.size());
        Coefficients result(result_size, Complex {});
        const std::size_t first_offset =
            result_size - coefficients_.size();
        const std::size_t second_offset =
            result_size - other.coefficients_.size();
        for (std::size_t index = 0U;
             index < coefficients_.size(); ++index) {
            result[first_offset + index] += coefficients_[index];
        }
        for (std::size_t index = 0U;
             index < other.coefficients_.size(); ++index) {
            result[second_offset + index] += other.coefficients_[index];
        }
        return Polynomial(std::move(result));
    }

    Polynomial operator-(const Polynomial& other) const
    {
        return *this + (-other);
    }

    Polynomial operator*(const Polynomial& other) const
    {
        Coefficients result(
            coefficients_.size() + other.coefficients_.size() - 1U,
            Complex {});
        for (std::size_t first = 0U;
             first < coefficients_.size(); ++first) {
            for (std::size_t second = 0U;
                 second < other.coefficients_.size(); ++second) {
                result[first + second] +=
                    coefficients_[first] * other.coefficients_[second];
            }
        }
        return Polynomial(std::move(result));
    }

    Polynomial operator*(Complex scalar) const
    {
        Coefficients result = coefficients_;
        for (Complex& coefficient : result) {
            coefficient *= scalar;
        }
        return Polynomial(std::move(result));
    }

    Polynomial operator*(double scalar) const
    {
        return *this * Complex(scalar, 0.0);
    }

    friend Polynomial operator*(Complex scalar, const Polynomial& polynomial)
    {
        return polynomial * scalar;
    }

    friend Polynomial operator*(double scalar, const Polynomial& polynomial)
    {
        return polynomial * scalar;
    }

    std::pair<Polynomial, Polynomial> divmod(
        const Polynomial& divisor) const
    {
        if (divisor.isZero()) {
            throw std::invalid_argument("Division by the zero polynomial");
        }
        if (degree() < divisor.degree()) {
            return {Polynomial {}, *this};
        }

        Coefficients remainder = coefficients_;
        Coefficients quotient(
            static_cast<std::size_t>(degree() - divisor.degree() + 1),
            Complex {});
        for (std::size_t row = 0U; row < quotient.size(); ++row) {
            quotient[row] = remainder[row] / divisor.coefficients_[0];
            for (std::size_t column = 0U;
                 column < divisor.coefficients_.size(); ++column) {
                remainder[row + column] -=
                    quotient[row] * divisor.coefficients_[column];
            }
        }

        Coefficients tail(
            remainder.begin()
                + static_cast<std::ptrdiff_t>(quotient.size()),
            remainder.end());
        return {
            Polynomial(std::move(quotient)),
            tail.empty()
                ? Polynomial {}
                : Polynomial(std::move(tail)),
        };
    }

    Polynomial operator/(const Polynomial& other) const
    {
        return divmod(other).first;
    }
    Polynomial operator%(const Polynomial& other) const
    {
        return divmod(other).second;
    }

    Polynomial& operator+=(const Polynomial& other)
    {
        return *this = *this + other;
    }
    Polynomial& operator-=(const Polynomial& other)
    {
        return *this = *this - other;
    }
    Polynomial& operator*=(const Polynomial& other)
    {
        return *this = *this * other;
    }
    Polynomial& operator*=(Complex scalar)
    {
        return *this = *this * scalar;
    }
    Polynomial& operator*=(double scalar)
    {
        return *this = *this * scalar;
    }

    Polynomial normalize() const
    {
        return isZero()
            ? *this
            : *this * (Complex(1, 0) / coefficients_.front());
    }

    bool hasRealCoefficients(double tolerance = polynomial_epsilon) const
    {
        return std::all_of(
            coefficients_.begin(),
            coefficients_.end(),
            [&](Complex coefficient) {
                return std::abs(coefficient.imag()) <= tolerance;
            });
    }

    double cauchyBound() const
    {
        if (!hasRealCoefficients() || degree() <= 0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        double ratio = 0.0;
        for (std::size_t index = 1U;
             index < coefficients_.size(); ++index) {
            ratio = std::max(
                ratio,
                std::abs(coefficients_[index])
                    / std::abs(coefficients_.front()));
        }
        return 1.0 + ratio;
    }

    /** Closed-form roots of ax^2+bx+c. */
    std::array<Complex, 2> solveQuadratic() const
    {
        requireDegree(2);
        const Complex p = coefficients_[1] / coefficients_[0];
        const Complex q = coefficients_[2] / coefficients_[0];
        const Complex center = -0.5 * p;
        const Complex delta = std::sqrt(center * center - q);
        return algebraicOrCompanion<2>({center - delta, center + delta});
    }

    /** Cardano roots after reducing to t^3+p*t+q. */
    std::array<Complex, 3> solveCubic() const
    {
        requireDegree(3);
        const Complex a = coefficients_[0];
        const Complex b = coefficients_[1];
        const Complex c = coefficients_[2];
        const Complex d = coefficients_[3];
        const Complex p =
            (3.0 * a * c - b * b) / (3.0 * a * a);
        const Complex q =
            (2.0 * b * b * b - 9.0 * a * b * c
                + 27.0 * a * a * d)
            / (27.0 * a * a * a);
        const Complex discriminant =
            q * q / 4.0 + p * p * p / 27.0;
        Complex u = std::pow(
            -q / 2.0 + std::sqrt(discriminant), 1.0 / 3.0);
        const Complex v = std::abs(u) > polynomial_epsilon
            ? -p / (3.0 * u)
            : std::pow(
                -q / 2.0 - std::sqrt(discriminant), 1.0 / 3.0);
        const Complex omega(-0.5, std::sqrt(3.0) / 2.0);
        const Complex offset = -b / (3.0 * a);
        return algebraicOrCompanion<3>({
            u + v + offset,
            omega * u + std::conj(omega) * v + offset,
            std::conj(omega) * u + omega * v + offset,
        });
    }

    /** Ferrari roots using one root of the resolvent cubic. */
    std::array<Complex, 4> solveQuartic() const
    {
        requireDegree(4);
        const Complex a = coefficients_[1] / coefficients_[0];
        const Complex b = coefficients_[2] / coefficients_[0];
        const Complex c = coefficients_[3] / coefficients_[0];
        const Complex d = coefficients_[4] / coefficients_[0];
        const Complex quarter_a = a / 4.0;
        const Complex quarter_a_squared = quarter_a * quarter_a;
        const Complex p = 3.0 * quarter_a_squared - b / 2.0;
        const Complex q =
            a * quarter_a_squared - b * quarter_a + c / 2.0;
        const Complex r =
            3.0 * quarter_a_squared * quarter_a_squared
            - b * quarter_a_squared + c * quarter_a - d;

        const auto resolvent =
            Polynomial({1.0, p, r, p * r - q * q / 2.0})
                .solveCubic();
        const Complex z = *std::max_element(
            resolvent.begin(), resolvent.end(),
            [&](Complex first, Complex second) {
                return std::abs(2.0 * (p + first))
                    < std::abs(2.0 * (p + second));
            });
        const Complex s = std::sqrt(2.0 * (p + z));
        const Complex t = std::abs(s) > polynomial_epsilon
            ? -q / s
            : std::sqrt(z * z + r);
        const auto first = Polynomial({1.0, s, z + t}).solveQuadratic();
        const auto second =
            Polynomial({1.0, -s, z - t}).solveQuadratic();
        return algebraicOrCompanion<4>({
            first[0] - quarter_a,
            first[1] - quarter_a,
            second[0] - quarter_a,
            second[1] - quarter_a,
        });
    }

    /** Deterministic fallback roots via a companion matrix. */
    std::vector<Complex> solveAllCompanion() const
    {
        if (degree() <= 0) {
            return {};
        }
        Eigen::MatrixXcd companion =
            Eigen::MatrixXcd::Zero(degree(), degree());
        for (int row = 1; row < degree(); ++row) {
            companion(row, row - 1) = 1.0;
        }
        for (int row = 0; row < degree(); ++row) {
            companion(row, degree() - 1) =
                -coefficients_[
                    static_cast<std::size_t>(degree() - row)]
                / coefficients_[0];
        }
        Eigen::ComplexEigenSolver<Eigen::MatrixXcd> solver(
            companion, false);
        if (solver.info() != Eigen::Success) {
            return {};
        }
        const Eigen::VectorXcd eigenvalues = solver.eigenvalues();
        return {eigenvalues.data(), eigenvalues.data() + eigenvalues.size()};
    }

    /** Initial complex shift on the Cauchy root bound. */
    Complex getInitialS() const
    {
        return initialShift(0);
    }

    /**
     * Three-stage Jenkins-Traub single-root iteration.
     *
     * Deterministic shifts make repeated solves reproducible. The companion
     * solver is used only when every shifted iteration fails.
     */
    Complex solveJenkinsTraubSingle(
        double tolerance = 1.0e-10,
        int max_iterations = 100,
        int max_outer_iterations = 50,
        int max_inner_iterations = 20) const
    {
        if (degree() <= 0) {
            throw std::invalid_argument(
                "A non-constant polynomial is required");
        }
        const Polynomial polynomial = normalize();
        if (std::abs(polynomial.coefficients_.back()) <= tolerance) {
            return {};
        }

        Polynomial h = polynomial.derivative();
        for (int iteration = 0; iteration < 5; ++iteration) {
            const Complex value = polynomial.evaluate(0.0);
            if (std::abs(value) <= polynomial_epsilon) {
                return {};
            }
            h = shiftedH(polynomial, h, Complex {}, value);
        }
        const Polynomial initial_h = h;

        for (int attempt = 0; attempt < max_outer_iterations; ++attempt) {
            Complex shift = polynomial.initialShift(attempt);
            h = initial_h;
            Complex previous_step(
                std::numeric_limits<double>::infinity(), 0.0);

            for (int iteration = 0;
                 iteration < max_inner_iterations; ++iteration) {
                const Complex value = polynomial.evaluate(shift);
                if (polynomial.relativeResidual(shift) <= tolerance) {
                    return shift;
                }
                h = shiftedH(polynomial, h, shift, value);
                const Complex slope = h.evaluate(shift);
                if (std::abs(slope) <= polynomial_epsilon) {
                    break;
                }
                const Complex step = value / slope;
                if (iteration > 0
                    && std::abs(step - previous_step)
                    <= 0.5 * std::max(1.0, std::abs(previous_step))) {
                    break;
                }
                previous_step = step;
            }

            for (int iteration = 0;
                 iteration < max_iterations; ++iteration) {
                const Complex value = polynomial.evaluate(shift);
                if (polynomial.relativeResidual(shift) <= tolerance) {
                    return shift;
                }
                h = shiftedH(polynomial, h, shift, value);
                const Complex slope = h.evaluate(shift);
                if (std::abs(slope) <= polynomial_epsilon) {
                    break;
                }
                const Complex next = shift - value / slope;
                if (!finite(next)) {
                    break;
                }
                if (std::abs(next - shift)
                    <= tolerance * (1.0 + std::abs(next))) {
                    shift = next;
                    if (polynomial.relativeResidual(shift)
                        <= 100.0 * tolerance) {
                        return shift;
                    }
                    break;
                }
                shift = next;
            }
        }

        const auto fallback = solveAllCompanion();
        if (fallback.empty()) {
            return solveNewtonComplex(tolerance);
        }
        return *std::min_element(
            fallback.begin(), fallback.end(),
            [&](Complex first, Complex second) {
                return relativeResidual(first)
                    < relativeResidual(second);
            });
    }

    /** Find and deflate all roots through the Jenkins-Traub entry point. */
    std::vector<Complex> solveAllJenkinsTraub(
        double tolerance = 1.0e-10) const
    {
        Polynomial current = *this;
        std::vector<Complex> result;
        result.reserve(static_cast<std::size_t>(degree()));
        while (current.degree() > 0) {
            const Complex root =
                current.solveJenkinsTraubSingle(tolerance);
            if (!finite(root)) {
                return solveAllCompanion();
            }
            result.push_back(root);
            current = current / Polynomial({1.0, -root});
        }

        for (Complex& root : result) {
            for (int iteration = 0; iteration < 4; ++iteration) {
                const auto [value, slope] = evaluateWithDerivative(root);
                if (std::abs(slope) <= polynomial_epsilon) {
                    break;
                }
                root -= value / slope;
            }
            if (!finite(root)
                || relativeResidual(root) > 1000.0 * tolerance) {
                return solveAllCompanion();
            }
        }
        return result;
    }

    std::vector<Complex> roots(
        double tolerance = 1.0e-10,
        bool force_numerical = false) const
    {
        if (degree() <= 0) {
            return {};
        }
        if (force_numerical || degree() > 4) {
            return solveAllJenkinsTraub(tolerance);
        }
        if (degree() == 1) {
            return {-coefficients_[1] / coefficients_[0]};
        }
        if (degree() == 2) {
            const auto result = solveQuadratic();
            return {result.begin(), result.end()};
        }
        if (degree() == 3) {
            const auto result = solveCubic();
            return {result.begin(), result.end()};
        }
        const auto result = solveQuartic();
        return {result.begin(), result.end()};
    }

    std::vector<double> realRoots(
        double tolerance = 1.0e-8,
        bool unique = false,
        bool force_numerical = false) const
    {
        std::vector<double> result;
        for (const Complex root : roots(tolerance, force_numerical)) {
            if (std::isfinite(root.real())
                && std::isfinite(root.imag())
                && std::abs(root.imag())
                    <= tolerance * (1.0 + std::abs(root.real()))) {
                result.push_back(root.real());
            }
        }
        std::sort(result.begin(), result.end());
        if (unique) {
            result.erase(
                std::unique(
                    result.begin(), result.end(),
                    [&](double first, double second) {
                        return std::abs(first - second)
                            <= tolerance
                                * (1.0 + std::max(
                                    std::abs(first),
                                    std::abs(second)));
                    }),
                result.end());
        }
        return result;
    }

    Eigen::MatrixXcd jacobianRootsWrtCoefficients(
        const std::vector<Complex>& roots,
        double tolerance =
            64.0 * std::numeric_limits<double>::epsilon()) const
    {
        return uvdar_core::helpers::jacobianRootsWrtCoefficients(
            coefficients_, roots, tolerance);
    }

    Eigen::MatrixXcd jacobianCoefficientsWrtRoots(
        const std::vector<Complex>& roots) const
    {
        if (roots.size() != static_cast<std::size_t>(degree())) {
            throw std::invalid_argument(
                "Root count must equal polynomial degree");
        }
        return uvdar_core::helpers::jacobianCoefficientsWrtRoots(
            coefficients_.front(), roots);
    }

    Polynomial stripLeadingZeros(
        double tolerance = polynomial_epsilon) const
    {
        Polynomial result = *this;
        result.strip(tolerance);
        return result;
    }

    Polynomial scalarMultiply(Complex scalar) const
    {
        return *this * scalar;
    }
    Polynomial copy() const { return *this; }

    Polynomial getCauchyPolynomial() const
    {
        Coefficients result = coefficients_;
        const Complex leading = coefficients_.front();
        if (std::abs(leading) > polynomial_epsilon) {
            for (Complex& coefficient : result) {
                coefficient = std::abs(coefficient / leading);
            }
        }
        return Polynomial(std::move(result));
    }

    int countRealRootsSturm(double lower, double upper) const
    {
        if (!hasRealCoefficients() || degree() <= 0) {
            return 0;
        }
        std::vector<Polynomial> chain {*this, derivative()};
        while (!chain.back().isZero() && chain.back().degree() > 0) {
            Polynomial remainder =
                -(chain[chain.size() - 2U] % chain.back());
            if (remainder.isZero()) {
                break;
            }
            chain.push_back(std::move(remainder));
        }

        const auto variations = [&](double x) {
            int count = 0;
            int previous = 0;
            for (const Polynomial& polynomial : chain) {
                const double value = polynomial.evaluate(x).real();
                const int sign = value > polynomial_epsilon
                    ? 1
                    : value < -polynomial_epsilon ? -1 : 0;
                if (sign != 0) {
                    count += previous != 0 && sign != previous;
                    previous = sign;
                }
            }
            return count;
        };
        return std::abs(variations(lower) - variations(upper));
    }

    double solveNewtonReal(double tolerance = 1.0e-10) const
    {
        if (!hasRealCoefficients() || degree() <= 0) {
            throw std::invalid_argument(
                "A non-constant real polynomial is required");
        }
        const double bound = cauchyBound();
        double x = 0.0;
        for (int iteration = 0; iteration < 1000; ++iteration) {
            const auto [value, derivative] =
                evaluateWithDerivative(x);
            if (std::abs(value) <= tolerance) {
                return x;
            }
            if (std::abs(derivative) <= polynomial_epsilon) {
                x = -bound
                    + 2.0 * bound
                        * static_cast<double>((iteration % 97) + 1)
                        / 98.0;
            } else {
                x -= value.real() / derivative.real();
            }
        }
        return x;
    }

    Complex solveNewtonComplex(double tolerance = 1.0e-10) const
    {
        if (degree() <= 0) {
            throw std::invalid_argument(
                "A non-constant polynomial is required");
        }
        Complex x(0.4, 0.9);
        for (int iteration = 0; iteration < 1000; ++iteration) {
            const auto [value, derivative] =
                evaluateWithDerivative(x);
            if (std::abs(value) <= tolerance) {
                return x;
            }
            x = std::abs(derivative) > polynomial_epsilon
                ? x - value / derivative
                : x + Complex(0.31, 0.17);
        }
        return x;
    }

    void print(std::ostream& output) const
    {
        if (isZero()) {
            output << '0';
            return;
        }
        bool first = true;
        for (int index = 0; index < size(); ++index) {
            const Complex coefficient =
                coefficients_[static_cast<std::size_t>(index)];
            if (std::abs(coefficient) <= polynomial_epsilon) {
                continue;
            }
            if (!first) {
                output << " + ";
            }
            output << '(' << coefficient << ')';
            const int power = degree() - index;
            if (power > 0) {
                output << "*x";
            }
            if (power > 1) {
                output << '^' << power;
            }
            first = false;
        }
    }

    friend std::ostream& operator<<(
        std::ostream& output,
        const Polynomial& polynomial)
    {
        polynomial.print(output);
        return output;
    }

private:
    Coefficients coefficients_;

    void strip(double tolerance = polynomial_epsilon)
    {
        if (coefficients_.empty()) {
            coefficients_.push_back(Complex {});
            return;
        }
        const auto first = std::find_if(
            coefficients_.begin(),
            coefficients_.end() - 1,
            [&](Complex coefficient) {
                return std::abs(coefficient) > tolerance;
            });
        coefficients_.erase(coefficients_.begin(), first);
    }

    bool isZero() const
    {
        return coefficients_.size() == 1U
            && std::abs(coefficients_.front())
                <= polynomial_epsilon;
    }

    static bool finite(Complex value)
    {
        return std::isfinite(value.real())
            && std::isfinite(value.imag());
    }

    Complex initialShift(int attempt) const
    {
        double bound = 1.0;
        for (std::size_t index = 1U;
             index < coefficients_.size(); ++index) {
            bound = std::max(
                bound,
                1.0 + std::abs(
                    coefficients_[index] / coefficients_.front()));
        }
        constexpr double golden_angle =
            2.39996322972865332223;
        const double angle = golden_angle * (attempt + 1);
        return bound * Complex(std::cos(angle), std::sin(angle));
    }

    double relativeResidual(Complex x) const
    {
        double scale = 0.0;
        for (const Complex coefficient : coefficients_) {
            scale = scale * std::abs(x) + std::abs(coefficient);
        }
        return std::abs(evaluate(x))
            / std::max(scale, std::numeric_limits<double>::min());
    }

    template <std::size_t Count>
    std::array<Complex, Count> algebraicOrCompanion(
        std::array<Complex, Count> roots) const
    {
        const bool accurate = std::all_of(
            roots.begin(), roots.end(), [&](Complex root) {
                return finite(root) && relativeResidual(root) <= 1.0e-10;
            });
        if (accurate) {
            return roots;
        }
        const auto fallback = solveAllCompanion();
        if (fallback.size() == Count) {
            std::copy(fallback.begin(), fallback.end(), roots.begin());
        }
        return roots;
    }

    static Polynomial shiftedH(
        const Polynomial& polynomial,
        const Polynomial& h,
        Complex shift,
        Complex value)
    {
        if (std::abs(value) <= polynomial_epsilon) {
            return h;
        }
        const Complex factor = -h.evaluate(shift) / value;
        return (h + polynomial * factor)
            / Polynomial({1.0, -shift});
    }

    void requireDegree(int expected) const
    {
        if (degree() != expected) {
            throw std::invalid_argument(
                "Polynomial has the wrong degree");
        }
    }
};

} // namespace uvdar_core::helpers

#endif // UVDAR_CORE_HELPERS_POLYNOMIAL_HPP
