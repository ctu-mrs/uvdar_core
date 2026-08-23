#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include <Eigen/Dense>

namespace uvdar_core::helpers {

/**
 * @brief Residual vector and its exact local Jacobian for one nonlinear state.
 *
 * The Jacobian columns use the local increment accepted by the retraction
 * passed to levenbergMarquardt().  This lets callers optimize manifold states
 * (for example SO(3) poses) without flattening them into an unsafe additive
 * representation.
 */
struct LeastSquaresLinearization {
    Eigen::VectorXd residual;
    Eigen::MatrixXd jacobian;
};

/** @brief Shared numerical controls for analytic Levenberg-Marquardt solves. */
struct LevenbergMarquardtOptions {
    int max_iterations = 50;
    double initial_damping = 1.0e-6;
    double minimum_damping = 1.0e-15;
    double maximum_damping = 1.0e15;
    double gradient_tolerance = 1.0e-10;
    double step_tolerance = 1.0e-10;
    double residual_tolerance = 1.0e-10;
    double relative_cost_tolerance = 1.0e-12;
};

/** @brief Termination reason returned by the shared LM implementation. */
enum class LevenbergMarquardtStatus {
    Converged,
    MaximumIterations,
    InvalidLinearization,
    NumericalFailure,
    DampingLimit,
};

/** @brief Per-trial diagnostics suitable for logs and live visualizations. */
struct LevenbergMarquardtIteration {
    int iteration = 0;
    int accepted_iterations = 0;
    bool accepted = false;
    double cost = std::numeric_limits<double>::infinity();
    double candidate_cost = std::numeric_limits<double>::infinity();
    double damping = 0.0;
    double gain_ratio = -std::numeric_limits<double>::infinity();
    double gradient_inf_norm = std::numeric_limits<double>::infinity();
    double step_norm = std::numeric_limits<double>::infinity();
};

/** @brief State and diagnostics produced by levenbergMarquardt(). */
template <typename State>
struct LevenbergMarquardtResult {
    State state;
    LevenbergMarquardtStatus status =
        LevenbergMarquardtStatus::InvalidLinearization;
    int iterations = 0;
    int accepted_iterations = 0;
    double initial_cost = std::numeric_limits<double>::infinity();
    double final_cost = std::numeric_limits<double>::infinity();

    bool converged() const
    {
        return status == LevenbergMarquardtStatus::Converged;
    }
};

inline const char* toString(const LevenbergMarquardtStatus status)
{
    switch (status) {
        case LevenbergMarquardtStatus::Converged:
            return "converged";
        case LevenbergMarquardtStatus::MaximumIterations:
            return "maximum iterations";
        case LevenbergMarquardtStatus::InvalidLinearization:
            return "invalid linearization";
        case LevenbergMarquardtStatus::NumericalFailure:
            return "numerical failure";
        case LevenbergMarquardtStatus::DampingLimit:
            return "damping limit";
    }
    return "unknown";
}

/**
 * @brief Minimize 0.5 ||r(state)||^2 with exact Jacobians and a local retraction.
 *
 * @tparam State Arbitrary copyable optimization state.
 * @param initial_state Starting state.
 * @param local_parameter_count Number of columns in the local Jacobian.
 * @param evaluate Callable `(state, with_jacobian)` returning an optional
 *        LeastSquaresLinearization.  Candidate evaluations may leave the
 *        Jacobian empty when `with_jacobian` is false.
 * @param retract Callable `(state, local_delta)` returning the candidate state.
 * @param options Solver controls.
 * @param progress Optional callback invoked after each attempted step.
 *
 * The diagonal-scaled LM system and gain-ratio update make the same helper
 * usable for mixed units such as pixels, radians, metres, and focal lengths.
 * No numerical differentiation is performed here: callers must supply the
 * analytic (or exact automatic-differentiation) Jacobian explicitly.
 */
template <typename State, typename Evaluate, typename Retract>
LevenbergMarquardtResult<State> levenbergMarquardt(
    State initial_state,
    const Eigen::Index local_parameter_count,
    Evaluate&& evaluate,
    Retract&& retract,
    const LevenbergMarquardtOptions& options = {},
    const std::function<void(const LevenbergMarquardtIteration&)>& progress = {})
{
    LevenbergMarquardtResult<State> result {std::move(initial_state)};
    if (local_parameter_count <= 0) {
        result.status = LevenbergMarquardtStatus::InvalidLinearization;
        return result;
    }

    double damping = std::clamp(
        options.initial_damping,
        std::max(options.minimum_damping, 0.0),
        std::max(options.maximum_damping, options.minimum_damping));
    double damping_multiplier = 2.0;

    auto linearization = evaluate(result.state, true);
    auto valid_linearization = [local_parameter_count](
                                   const std::optional<LeastSquaresLinearization>& value,
                                   const bool require_jacobian) {
        if (!value || value->residual.size() == 0
            || !value->residual.allFinite()) {
            return false;
        }
        return !require_jacobian
            || (value->jacobian.rows() == value->residual.size()
                && value->jacobian.cols() == local_parameter_count
                && value->jacobian.allFinite());
    };
    if (!valid_linearization(linearization, true)) {
        result.status = LevenbergMarquardtStatus::InvalidLinearization;
        return result;
    }

    double cost = 0.5 * linearization->residual.squaredNorm();
    result.initial_cost = cost;
    result.final_cost = cost;
    if (!std::isfinite(cost)) {
        result.status = LevenbergMarquardtStatus::NumericalFailure;
        return result;
    }

    const int maximum_iterations = std::max(0, options.max_iterations);
    for (int iteration = 0; iteration < maximum_iterations; ++iteration) {
        result.iterations = iteration + 1;
        const Eigen::VectorXd gradient =
            linearization->jacobian.transpose() * linearization->residual;
        const double gradient_inf_norm = gradient.lpNorm<Eigen::Infinity>();
        if (!gradient.allFinite() || !std::isfinite(gradient_inf_norm)) {
            result.status = LevenbergMarquardtStatus::NumericalFailure;
            return result;
        }
        if (gradient_inf_norm <= std::max(0.0, options.gradient_tolerance)
            || std::sqrt(2.0 * cost / static_cast<double>(
                                      linearization->residual.size()))
                <= std::max(0.0, options.residual_tolerance)) {
            result.status = LevenbergMarquardtStatus::Converged;
            return result;
        }

        const Eigen::MatrixXd normal =
            linearization->jacobian.transpose() * linearization->jacobian;
        Eigen::VectorXd diagonal = normal.diagonal().cwiseAbs();
        diagonal = diagonal.cwiseMax(1.0e-12);
        Eigen::MatrixXd damped_normal = normal;
        damped_normal.diagonal().array() += damping * diagonal.array();
        const Eigen::VectorXd step = -damped_normal.ldlt().solve(gradient);
        const double step_norm = step.norm();
        if (!step.allFinite() || !std::isfinite(step_norm)) {
            result.status = LevenbergMarquardtStatus::NumericalFailure;
            return result;
        }
        if (step_norm <= std::max(0.0, options.step_tolerance)) {
            result.status = LevenbergMarquardtStatus::Converged;
            return result;
        }

        State candidate_state = retract(result.state, step);
        auto candidate = evaluate(candidate_state, false);
        const double candidate_cost = valid_linearization(candidate, false)
            ? 0.5 * candidate->residual.squaredNorm()
            : std::numeric_limits<double>::infinity();
        const double predicted_reduction =
            0.5 * step.dot(damping * diagonal.cwiseProduct(step) - gradient);
        const double gain_ratio = predicted_reduction > 0.0
            ? (cost - candidate_cost) / predicted_reduction
            : -std::numeric_limits<double>::infinity();
        const bool accepted = std::isfinite(candidate_cost)
            && std::isfinite(gain_ratio) && gain_ratio > 0.0;

        LevenbergMarquardtIteration iteration_info;
        iteration_info.iteration = iteration;
        iteration_info.accepted_iterations = result.accepted_iterations;
        iteration_info.accepted = accepted;
        iteration_info.cost = cost;
        iteration_info.candidate_cost = candidate_cost;
        iteration_info.damping = damping;
        iteration_info.gain_ratio = gain_ratio;
        iteration_info.gradient_inf_norm = gradient_inf_norm;
        iteration_info.step_norm = step_norm;

        if (accepted) {
            const double previous_cost = cost;
            result.state = std::move(candidate_state);
            cost = candidate_cost;
            result.final_cost = cost;
            ++result.accepted_iterations;
            iteration_info.accepted_iterations = result.accepted_iterations;
            const double update = std::max(
                1.0 / 3.0,
                1.0 - std::pow(2.0 * gain_ratio - 1.0, 3.0));
            damping = std::max(options.minimum_damping, damping * update);
            damping_multiplier = 2.0;
            if (progress) {
                progress(iteration_info);
            }

            linearization = evaluate(result.state, true);
            if (!valid_linearization(linearization, true)) {
                result.status =
                    LevenbergMarquardtStatus::InvalidLinearization;
                return result;
            }
            const double relative_change = std::abs(previous_cost - cost)
                / std::max(1.0, previous_cost);
            if (relative_change
                <= std::max(0.0, options.relative_cost_tolerance)) {
                result.status = LevenbergMarquardtStatus::Converged;
                return result;
            }
        } else {
            damping *= damping_multiplier;
            damping_multiplier *= 2.0;
            if (progress) {
                progress(iteration_info);
            }
            if (!std::isfinite(damping)
                || damping > options.maximum_damping) {
                result.status = LevenbergMarquardtStatus::DampingLimit;
                return result;
            }
        }
    }

    result.status = LevenbergMarquardtStatus::MaximumIterations;
    return result;
}

} // namespace uvdar_core::helpers
