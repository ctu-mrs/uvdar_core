#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
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

struct SunMaskWorkspace {
    unsigned dilation_distance = 0U;
    std::vector<int> dilation_half_widths;
    std::vector<std::vector<int>> dilation_row_offsets;
    std::vector<std::uint64_t> raw_sun_bits;
    std::vector<std::uint64_t> dilated_sun_bits;
    std::vector<std::uint64_t> row_bits_a;
    std::vector<std::uint64_t> row_bits_b;
};

inline SunMaskWorkspace& sunMaskWorkspace()
{
    thread_local SunMaskWorkspace workspace;
    return workspace;
}

/**
 * @brief Return row half-widths for an integer disk matching strict distance.
 */
inline std::vector<int> strictDiskHalfWidths(unsigned distance)
{
    const std::uint64_t diameter_64 =
        2U * static_cast<std::uint64_t>(distance) - 1U;
    if (diameter_64 > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
        throw std::length_error("minimum sun-marker distance is too large to rasterize");
    }

    const int diameter = static_cast<int>(diameter_64);
    const int center = static_cast<int>(distance - 1U);
    const std::uint64_t distance_squared =
        static_cast<std::uint64_t>(distance) * distance;
    std::vector<int> half_widths(static_cast<std::size_t>(diameter));
    for (int y = 0; y < diameter; ++y) {
        const std::int64_t dy = static_cast<std::int64_t>(y) - center;
        int half_width = center;
        while (half_width > 0 &&
               static_cast<std::uint64_t>(
                   static_cast<std::int64_t>(half_width) * half_width + dy * dy) >=
                   distance_squared) {
            --half_width;
        }
        half_widths[static_cast<std::size_t>(y)] = half_width;
    }
    return half_widths;
}

inline std::size_t packedSunMaskWordsPerRow(unsigned image_width)
{
    return (static_cast<std::size_t>(image_width) + 63U) / 64U;
}

/** Grow a packed row by one pixel without allowing bits to wrap across rows. */
inline void expandPackedSunRowOnePixel(
    const std::vector<std::uint64_t>& source,
    std::size_t words_per_row,
    std::uint64_t last_word_mask,
    std::vector<std::uint64_t>& destination)
{
    destination.resize(words_per_row);
    for (std::size_t word = 0; word < words_per_row; ++word) {
        std::uint64_t expanded = source[word] |
            (source[word] << 1U) | (source[word] >> 1U);
        if (word > 0U) {
            expanded |= source[word - 1U] >> 63U;
        }
        if (word + 1U < words_per_row) {
            expanded |= source[word + 1U] << 63U;
        }
        destination[word] = expanded;
    }
    destination.back() &= last_word_mask;
}

/**
 * @brief Dilate a row-packed binary sun mask with an exact integer disk.
 */
inline const std::vector<std::uint64_t>& dilatePackedSunMask(
    const std::vector<std::uint64_t>& raw_sun_bits,
    unsigned min_sun_marker_distance,
    unsigned image_width,
    unsigned image_height)
{
    SunMaskWorkspace& workspace = sunMaskWorkspace();
    const std::size_t words_per_row = packedSunMaskWordsPerRow(image_width);
    const std::size_t expected_words =
        words_per_row * static_cast<std::size_t>(image_height);
    if (raw_sun_bits.size() != expected_words) {
        throw std::invalid_argument("packed sun mask dimensions do not match input image");
    }

    workspace.dilated_sun_bits.assign(expected_words, 0U);
    if (workspace.dilation_distance != min_sun_marker_distance) {
        workspace.dilation_half_widths =
            strictDiskHalfWidths(min_sun_marker_distance);
        const int center = static_cast<int>(min_sun_marker_distance - 1U);
        const int maximum_half_width = *std::max_element(
            workspace.dilation_half_widths.begin(),
            workspace.dilation_half_widths.end());
        workspace.dilation_row_offsets.assign(
            static_cast<std::size_t>(maximum_half_width) + 1U, {});
        for (std::size_t disk_row = 0;
             disk_row < workspace.dilation_half_widths.size();
             ++disk_row) {
            const auto half_width = static_cast<std::size_t>(
                workspace.dilation_half_widths[disk_row]);
            workspace.dilation_row_offsets[half_width].push_back(
                static_cast<int>(disk_row) - center);
        }
        workspace.dilation_distance = min_sun_marker_distance;
    }

    const unsigned valid_tail_bits = image_width % 64U;
    const std::uint64_t last_word_mask = valid_tail_bits == 0U
        ? std::numeric_limits<std::uint64_t>::max()
        : (std::uint64_t {1U} << valid_tail_bits) - 1U;
    for (unsigned source_y = 0U; source_y < image_height; ++source_y) {
        const std::uint64_t* const source_row =
            raw_sun_bits.data() + static_cast<std::size_t>(source_y) * words_per_row;
        bool row_has_sun = false;
        for (std::size_t word = 0; word < words_per_row; ++word) {
            if (source_row[word] != 0U) {
                row_has_sun = true;
                break;
            }
        }
        if (!row_has_sun) {
            continue;
        }

        workspace.row_bits_a.assign(source_row, source_row + words_per_row);
        workspace.row_bits_a.back() &= last_word_mask;
        for (std::size_t half_width = 0;
             half_width < workspace.dilation_row_offsets.size();
             ++half_width) {
            if (half_width > 0U) {
                expandPackedSunRowOnePixel(
                    workspace.row_bits_a,
                    words_per_row,
                    last_word_mask,
                    workspace.row_bits_b);
                workspace.row_bits_a.swap(workspace.row_bits_b);
            }
            for (const int row_offset : workspace.dilation_row_offsets[half_width]) {
                const int target_y = static_cast<int>(source_y) + row_offset;
                if (target_y < 0 || target_y >= static_cast<int>(image_height)) {
                    continue;
                }
                std::uint64_t* const target_row =
                    workspace.dilated_sun_bits.data() +
                    static_cast<std::size_t>(target_y) * words_per_row;
                for (std::size_t word = 0; word < words_per_row; ++word) {
                    target_row[word] |= workspace.row_bits_a[word];
                }
            }
        }
    }

    return workspace.dilated_sun_bits;
}

/**
 * @brief Filter raw markers using an already row-packed binary sun mask.
 */
inline void filterRawMarkersNearPackedSunMask(
    std::vector<WeightedPoint>& raw_markers,
    const std::vector<std::uint64_t>& raw_sun_bits,
    unsigned min_sun_marker_distance,
    unsigned image_width,
    unsigned image_height)
{
    if (min_sun_marker_distance == 0U || raw_markers.empty()) {
        return;
    }
    if (image_width == 0U || image_height == 0U) {
        throw std::invalid_argument("sun mask requires valid input image dimensions");
    }

    const std::uint64_t maximum_dx = image_width - 1U;
    const std::uint64_t maximum_dy = image_height - 1U;
    const std::uint64_t maximum_image_distance_squared =
        maximum_dx * maximum_dx + maximum_dy * maximum_dy;
    const std::uint64_t minimum_distance_squared =
        static_cast<std::uint64_t>(min_sun_marker_distance) * min_sun_marker_distance;
    if (minimum_distance_squared > maximum_image_distance_squared) {
        raw_markers.clear();
        return;
    }

    const auto& dilated = dilatePackedSunMask(
        raw_sun_bits,
        min_sun_marker_distance,
        image_width,
        image_height);
    const std::size_t words_per_row = packedSunMaskWordsPerRow(image_width);
    raw_markers.erase(
        std::remove_if(
            raw_markers.begin(),
            raw_markers.end(),
            [&](const auto& marker) {
                const unsigned x = static_cast<unsigned>(std::clamp(
                    static_cast<int>(std::lround(marker.point.x)),
                    0,
                    static_cast<int>(image_width) - 1));
                const unsigned y = static_cast<unsigned>(std::clamp(
                    static_cast<int>(std::lround(marker.point.y)),
                    0,
                    static_cast<int>(image_height) - 1));
                const std::size_t word = static_cast<std::size_t>(y) * words_per_row + x / 64U;
                return (dilated[word] & (std::uint64_t {1U} << (x % 64U))) != 0U;
            }),
        raw_markers.end());
}

/**
 * @brief Remove raw marker samples selected by a dilated boolean sun mask.
 *
 * Filtering before collapse prevents rejected sun-adjacent samples from moving
 * a surviving cluster centroid or inflating its covariance. It also avoids
 * spending clustering work on samples that cannot appear in the result.
 *
 * @param raw_markers Raw marker samples to filter in-place.
 * @param raw_sun_points Raw sun samples used to construct the exclusion mask.
 * @param min_sun_marker_distance Minimum Euclidean distance in pixels.
 * @param image_width Width of the source image in pixels.
 * @param image_height Height of the source image in pixels.
 */
inline void filterRawMarkersNearSunPoints(
    std::vector<WeightedPoint>& raw_markers,
    const std::vector<WeightedPoint>& raw_sun_points,
    unsigned min_sun_marker_distance,
    unsigned image_width,
    unsigned image_height)
{
    if (min_sun_marker_distance == 0U || raw_sun_points.empty() || raw_markers.empty()) {
        return;
    }
    if (image_width == 0U || image_height == 0U ||
        image_width > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
        image_height > static_cast<unsigned>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("sun mask requires valid input image dimensions");
    }

    SunMaskWorkspace& workspace = sunMaskWorkspace();
    const std::size_t words_per_row = packedSunMaskWordsPerRow(image_width);
    workspace.raw_sun_bits.assign(
        words_per_row * static_cast<std::size_t>(image_height), 0U);
    for (const auto& sun : raw_sun_points) {
        const int x = static_cast<int>(sun.point.x);
        const int y = static_cast<int>(sun.point.y);
        if (x < 0 || x >= static_cast<int>(image_width) ||
            y < 0 || y >= static_cast<int>(image_height)) {
            throw std::out_of_range("sun result lies outside the input image");
        }
        const std::size_t word = static_cast<std::size_t>(y) * words_per_row +
            static_cast<unsigned>(x) / 64U;
        workspace.raw_sun_bits[word] |=
            std::uint64_t {1U} << (static_cast<unsigned>(x) % 64U);
    }
    filterRawMarkersNearPackedSunMask(
        raw_markers,
        workspace.raw_sun_bits,
        min_sun_marker_distance,
        image_width,
        image_height);
}

} // namespace uvdar_core::detection::fimd
