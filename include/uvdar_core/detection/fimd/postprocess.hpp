#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

#include "uvdar_core/detection/i_detector.hpp"

namespace uvdar_core::detection::fimd {

/**
 * @brief Remove detected markers that are too close to sun points.
 * @param output Detection output to filter in-place.
 * @param min_sun_marker_distance Minimum Euclidean distance in pixels.
 */
inline void filterMarkersNearSunPoints(DetectorOutput& output, unsigned min_sun_marker_distance)
{
    if (min_sun_marker_distance == 0 || output.sun_points.empty() || output.detected_points.empty()) {
        return;
    }

    const std::int64_t minimum_distance_squared = static_cast<std::int64_t>(min_sun_marker_distance) * static_cast<std::int64_t>(min_sun_marker_distance);

    std::vector<cv::Point2i> filtered_markers;
    filtered_markers.reserve(output.detected_points.size());

    for (const auto& marker : output.detected_points) {
        bool keep_marker = true;
        for (const auto& sun_point : output.sun_points) {
            const std::int64_t dx = static_cast<std::int64_t>(marker.x) - static_cast<std::int64_t>(sun_point.x);
            const std::int64_t dy = static_cast<std::int64_t>(marker.y) - static_cast<std::int64_t>(sun_point.y);
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
