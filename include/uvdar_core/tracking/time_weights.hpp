#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace uvdar_core::tracking {

/**
 * @brief Build normalized exponential weights where newer timestamps matter more.
 *
 * Formula: w_i = exp(-decay_factor * (t_last - t_i)) / sum_j w_j.
 */
inline std::vector<double> normalizedExponentialWeights(
    const std::vector<double>& times,
    double decay_factor,
    bool empty_returns_unit = false)
{
    if (times.empty()) {
        return empty_returns_unit ? std::vector<double> { 1.0 } : std::vector<double> {};
    }

    std::vector<double> weights;
    weights.reserve(times.size());
    const double reference_time = times.back();
    double sum = 0.0;

    for (const double time : times) {
        const double time_distance = std::max(0.0, reference_time - time);
        const double weight = std::exp(-decay_factor * time_distance);
        weights.push_back(weight);
        sum += weight;
    }

    if (sum <= 0.0) {
        const double uniform = 1.0 / static_cast<double>(weights.size());
        std::fill(weights.begin(), weights.end(), uniform);
        return weights;
    }

    for (double& weight : weights) {
        weight /= sum;
    }
    return weights;
}

} // namespace uvdar_core::tracking
