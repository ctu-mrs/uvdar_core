#pragma once

#include <algorithm>
#include <vector>

namespace uvdar_core::tracking {

/**
 * @brief Append a point and keep only the newest samples when a history limit is set.
 */
template <typename PointState>
void appendBounded(std::vector<PointState>& sequence, const PointState& point, int max_size)
{
    sequence.push_back(point);
    if (max_size > 0 && static_cast<int>(sequence.size()) > max_size) {
        sequence.erase(sequence.begin());
    }
}

/**
 * @brief Return the latest samples matching the configured blink-sequence length.
 */
template <typename PointState>
std::vector<PointState> trailingSamples(const std::vector<PointState>& sequence, std::size_t desired_size)
{
    if (sequence.size() <= desired_size) {
        return sequence;
    }

    return std::vector<PointState>(sequence.end() - static_cast<std::ptrdiff_t>(desired_size), sequence.end());
}

/**
 * @brief Count the number of OFF states at the end of a candidate track.
 */
template <typename PointState>
int countTrailingOffStates(const std::vector<PointState>& sequence)
{
    int count = 0;
    for (auto point = sequence.rbegin(); point != sequence.rend(); ++point) {
        if (point->led_state) {
            break;
        }
        ++count;
    }
    return count;
}

/**
 * @brief Check whether a track contains a run of OFF states longer than allowed.
 */
template <typename PointState>
bool hasOffRunLongerThan(const std::vector<PointState>& sequence, int max_allowed_run)
{
    if (max_allowed_run < 0) {
        return false;
    }

    int count = 0;
    return std::any_of(sequence.begin(), sequence.end(), [&](const PointState& point) {
        if (!point.led_state) {
            ++count;
            return count > max_allowed_run;
        }
        count = 0;
        return false;
    });
}

} // namespace uvdar_core::tracking
