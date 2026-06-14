#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "uvdar_core/detection/i_detector.hpp"

namespace uvdar_core::detection::fimd {

struct WeightedPoint {
    cv::Point2f point;
    float weight;
};

inline constexpr std::uint32_t packPackedPoint(std::uint32_t x, std::uint32_t y, std::uint32_t value)
{
    return ((x & 0x0FFFu) << 20u) | ((y & 0x0FFFu) << 8u) | (value & 0xFFu);
}

inline WeightedPoint unpackPackedPoint(std::uint32_t packed)
{
    return WeightedPoint {
        cv::Point2f(
            static_cast<float>((packed >> 20u) & 0x0FFFu),
            static_cast<float>((packed >> 8u) & 0x0FFFu)),
        static_cast<float>(packed & 0xFFu),
    };
}

inline std::vector<uvdar_core::detection::DetectorPoint> collapseRawPointsInternal(
    const std::vector<WeightedPoint>& raw_points,
    unsigned distance_px);

/**
 * @brief Merge neighboring raw detections with running mean + Welford covariance.
 * Assumes each pixel contributes uniform localization uncertainty in its area
 * (variance 1/12 px^2 per axis).
 */
inline std::vector<uvdar_core::detection::DetectorPoint> collapseRawPoints(
    const std::vector<cv::Point2i>& raw_points,
    unsigned distance_px)
{
    std::vector<WeightedPoint> weighted_points;
    weighted_points.reserve(raw_points.size());
    for (const auto& point : raw_points) {
        weighted_points.push_back(WeightedPoint {
            cv::Point2f(static_cast<float>(point.x), static_cast<float>(point.y)),
            1.0F,
        });
    }
    return collapseRawPointsInternal(weighted_points, distance_px);
}

/**
 * @brief Merge neighboring raw detections with weighted running mean + weighted
 * Welford covariance. Pixel-wise intensity can be used as weight.
 * Assumes each pixel contributes uniform localization uncertainty in its area
 * (variance 1/12 px^2 per axis).
 */
inline std::vector<uvdar_core::detection::DetectorPoint> collapseRawPoints(
    const std::vector<WeightedPoint>& raw_points,
    unsigned distance_px)
{
    return collapseRawPointsInternal(raw_points, distance_px);
}

inline std::vector<uvdar_core::detection::DetectorPoint> collapseRawPointsInternal(
    const std::vector<WeightedPoint>& raw_points,
    unsigned distance_px)
{
    constexpr float pixel_discretization_variance = 1.0F / 12.0F;

    if (raw_points.empty()) {
        return {};
    }

    const double max_distance_squared = static_cast<double>(distance_px) * static_cast<double>(distance_px);
    if (distance_px == 0U) {
        std::vector<uvdar_core::detection::DetectorPoint> output;
        output.reserve(raw_points.size());
        for (const auto& raw_point : raw_points) {
            output.push_back(uvdar_core::detection::DetectorPoint {
                raw_point.point,
                pixel_discretization_variance,
                0.0F,
                0.0F,
                pixel_discretization_variance,
                1});
        }
        return output;
    }

    std::vector<WeightedPoint> sorted_points = raw_points;
    std::sort(sorted_points.begin(), sorted_points.end(), [](const WeightedPoint& lhs, const WeightedPoint& rhs) {
        if (lhs.point.y == rhs.point.y) {
            return lhs.point.x < rhs.point.x;
        }
        return lhs.point.y < rhs.point.y;
    });

    struct Accumulator {
        cv::Point2f mean;
        double sum_weights = 0.0;
        double m00 = 0.0;
        double m01 = 0.0;
        double m10 = 0.0;
        double m11 = 0.0;
        std::size_t count = 0;
    };

    std::vector<Accumulator> accumulators;
    accumulators.reserve(sorted_points.size());

    auto addSample = [](Accumulator& accumulator, const cv::Point2f& point, float weight) {
        const double w = static_cast<double>(weight);
        if (accumulator.sum_weights <= 0.0) {
            accumulator.mean = point;
            accumulator.sum_weights = w;
            accumulator.m00 = 0.0;
            accumulator.m01 = 0.0;
            accumulator.m10 = 0.0;
            accumulator.m11 = 0.0;
            return;
        }

        const double next_sum = accumulator.sum_weights + w;
        const double weight_ratio = w / next_sum;

        const double delta_x = static_cast<double>(point.x) - static_cast<double>(accumulator.mean.x);
        const double delta_y = static_cast<double>(point.y) - static_cast<double>(accumulator.mean.y);

        const double updated_x = static_cast<double>(accumulator.mean.x) + weight_ratio * delta_x;
        const double updated_y = static_cast<double>(accumulator.mean.y) + weight_ratio * delta_y;

        const double delta2_x = static_cast<double>(point.x) - updated_x;
        const double delta2_y = static_cast<double>(point.y) - updated_y;

        accumulator.m00 += w * delta_x * delta2_x;
        accumulator.m01 += w * delta_x * delta2_y;
        accumulator.m10 += w * delta_y * delta2_x;
        accumulator.m11 += w * delta_y * delta2_y;

        accumulator.mean.x = static_cast<float>(updated_x);
        accumulator.mean.y = static_cast<float>(updated_y);
        accumulator.sum_weights = next_sum;
    };

    std::size_t min_index = 0;
    for (const auto& point : sorted_points) {
        const double p_x = static_cast<double>(point.point.x);
        const double p_y = static_cast<double>(point.point.y);

        double best_distance = max_distance_squared;
        long best_index       = -1;

        for (std::size_t index = min_index; index < accumulators.size(); ++index) {
            const auto& accumulator = accumulators[index];
            if (p_y > static_cast<double>(accumulator.mean.y) + static_cast<double>(distance_px)) {
                min_index = index + 1;
                continue;
            }

            const double dx = static_cast<double>(accumulator.mean.x) - p_x;
            const double dy = static_cast<double>(accumulator.mean.y) - p_y;
            const double distance_squared = dx * dx + dy * dy;
            if (distance_squared < best_distance) {
                best_distance = distance_squared;
                best_index    = static_cast<long>(index);
            }
        }

        if (best_index >= 0) {
            auto& accumulator = accumulators[static_cast<std::size_t>(best_index)];

            addSample(accumulator, point.point, point.weight);
            ++accumulator.count;
        } else {
            Accumulator accumulator;
            accumulator.mean = point.point;
            accumulator.count = 1;
            accumulator.sum_weights = static_cast<double>(point.weight);
            accumulators.push_back(accumulator);
        }
    }

    std::vector<uvdar_core::detection::DetectorPoint> collapsed;
    collapsed.reserve(accumulators.size());

    for (const auto& accumulator : accumulators) {
        uvdar_core::detection::DetectorPoint point {
            accumulator.mean,
            pixel_discretization_variance,
            0.0F,
            0.0F,
            pixel_discretization_variance,
            accumulator.count,
        };

        if (accumulator.count > 1 && accumulator.sum_weights > 1.0) {
            const double inv_sum = 1.0 / (accumulator.sum_weights - 1.0);
            point.covariance_00 = static_cast<float>(accumulator.m00 * inv_sum) + pixel_discretization_variance;
            point.covariance_01 = static_cast<float>(accumulator.m01 * inv_sum);
            point.covariance_10 = static_cast<float>(accumulator.m10 * inv_sum);
            point.covariance_11 = static_cast<float>(accumulator.m11 * inv_sum) + pixel_discretization_variance;
        }

        collapsed.push_back(point);
    }

    return collapsed;
}

/**
 * @brief Remove detected markers that are too close to sun points.
 * @param output Detection output to filter in-place.
 * @param min_sun_marker_distance Minimum Euclidean distance in pixels.
 */
inline void filterMarkersNearSunPoints(uvdar_core::detection::DetectorOutput& output, unsigned min_sun_marker_distance)
{
    if (min_sun_marker_distance == 0 || output.sun_points.empty() || output.detected_points.empty()) {
        return;
    }

    const double minimum_distance_squared = static_cast<double>(min_sun_marker_distance) * static_cast<double>(min_sun_marker_distance);

    std::vector<uvdar_core::detection::DetectorPoint> filtered_markers;
    filtered_markers.reserve(output.detected_points.size());

    for (const auto& marker : output.detected_points) {
        bool keep_marker = true;
        for (const auto& sun_point : output.sun_points) {
            const double dx = static_cast<double>(marker.point.x) - static_cast<double>(sun_point.point.x);
            const double dy = static_cast<double>(marker.point.y) - static_cast<double>(sun_point.point.y);
            if ((dx * dx + dy * dy) < minimum_distance_squared) {
                keep_marker = false;
                break;
            }
        }

        if (keep_marker) {
            filtered_markers.push_back(marker);
        }
    }

    output.detected_points = std::move(filtered_markers);
}

} // namespace uvdar_core::detection::fimd
