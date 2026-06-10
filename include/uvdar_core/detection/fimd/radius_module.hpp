#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace uvdar_core::detection::fimd {

struct RuntimeCirclePoint {
    int y = 0;
    int x = 0;
};

class RuntimeFimdRadiusModule {
public:
    RuntimeFimdRadiusModule(unsigned radius, unsigned image_width, unsigned image_height)
        : radius_(radius)
        , image_width_(image_width)
        , image_height_(image_height)
    {
        if (radius_ == 0) {
            throw std::runtime_error("FIMD CPU radius must be positive.");
        }
        rebuild();
    }

    unsigned radius() const { return radius_; }
    unsigned image_width() const { return image_width_; }
    unsigned image_height() const { return image_height_; }
    unsigned offset() const { return offset_; }
    const std::vector<int>& boundary_offsets() const { return boundary_offsets_; }
    const std::vector<int>& interior_offsets() const { return interior_offsets_; }

    static std::uint64_t key(unsigned radius, unsigned image_width, unsigned image_height)
    {
        return (static_cast<std::uint64_t>(radius) << 48U) | (static_cast<std::uint64_t>(image_width) << 24U) | static_cast<std::uint64_t>(image_height);
    }

private:
    static RuntimeCirclePoint makePoint(int y, int x)
    {
        return RuntimeCirclePoint { y, x };
    }

    static int coord2to1(const RuntimeCirclePoint& point, unsigned image_width)
    {
        return (point.y * static_cast<int>(image_width)) + point.x;
    }

    static std::vector<RuntimeCirclePoint> generateBoundary(int radius)
    {
        std::vector<RuntimeCirclePoint> boundary;
        int x        = 0;
        int y        = radius;
        int decision = 3 - 2 * radius;

        while (x <= y) {
            boundary.push_back(makePoint(+y, -x));
            if (radius == 0) {
                break;
            }
            boundary.push_back(makePoint(-y, +x));
            if (x < y) {
                boundary.push_back(makePoint(+x, -y));
                boundary.push_back(makePoint(-x, +y));
            }

            if (x > 0) {
                boundary.push_back(makePoint(+y, +x));
                boundary.push_back(makePoint(-y, -x));
                if (x < y) {
                    boundary.push_back(makePoint(+x, +y));
                    boundary.push_back(makePoint(-x, -y));
                }
            }

            if (decision < 0) {
                x += 1;
                decision += 4 * x + 6;
            } else {
                x += 1;
                y -= 1;
                decision += 4 * (x - y) + 10;
            }
        }

        return boundary;
    }

    static std::vector<RuntimeCirclePoint> generateInterior(int radius)
    {
        std::vector<RuntimeCirclePoint> interior;
        int x        = 0;
        int y        = radius;
        int decision = 3 - 2 * radius;

        while (x <= y) {
            for (int y_i = x; y_i < y; ++y_i) {
                interior.push_back(makePoint(+y_i, +x));
                if (y_i > x) {
                    interior.push_back(makePoint(+x, +y_i));
                    interior.push_back(makePoint(+x, -y_i));
                }
                if (y_i > 0) {
                    interior.push_back(makePoint(-y_i, +x));
                }
                if (x > 0) {
                    interior.push_back(makePoint(+y_i, -x));
                    interior.push_back(makePoint(-y_i, -x));
                    if (y_i > x) {
                        interior.push_back(makePoint(+x, -y_i));
                        interior.push_back(makePoint(-x, -y_i));
                    }
                }
            }

            if (decision < 0) {
                x += 1;
                decision += 4 * x + 6;
            } else {
                x += 1;
                y -= 1;
                decision += 4 * (x - y) + 10;
            }
        }

        return interior;
    }

    static std::vector<RuntimeCirclePoint> orderBoundaryEvaluation(const std::vector<RuntimeCirclePoint>& boundary)
    {
        std::vector<RuntimeCirclePoint> quadrant;
        quadrant.reserve(boundary.size());
        int radius = 0;
        for (const auto& point : boundary) {
            if (point.y >= 0 && point.x >= 0) {
                quadrant.push_back(point);
            }
            radius = std::max(radius, point.y);
        }

        std::vector<int> distances;
        distances.reserve(quadrant.size());
        for (const auto& point : quadrant) {
            distances.push_back(std::max(point.y, radius - point.x));
        }

        std::vector<RuntimeCirclePoint> ordered;
        ordered.reserve(boundary.size());
        std::size_t next_index = 0;
        while (ordered.size() < boundary.size()) {
            const auto point = quadrant[next_index];
            ordered.push_back(makePoint(+point.y, +point.x));
            ordered.push_back(makePoint(-point.y, -point.x));
            if (point.x == 0) {
                ordered.push_back(makePoint(0, +point.y));
                ordered.push_back(makePoint(0, -point.y));
            } else {
                ordered.push_back(makePoint(+point.x, -point.y));
                ordered.push_back(makePoint(-point.x, +point.y));
            }
            distances[next_index] = 0;

            int selected_distance      = 0;
            int selected_y             = 0;
            std::size_t selected_index = 0;
            for (std::size_t index = 0; index < quadrant.size(); ++index) {
                const auto candidate = quadrant[index];
                const int distance   = std::max(std::abs(candidate.y - point.y), std::abs(candidate.x - point.x));
                if (distances[index] > distance) {
                    distances[index] = distance;
                }

                if (distances[index] > selected_distance || (distances[index] == selected_distance && candidate.y > selected_y)) {
                    selected_distance = distances[index];
                    selected_y        = candidate.y;
                    selected_index    = index;
                }
            }
            next_index = selected_index;
        }

        return ordered;
    }

    void rebuild()
    {
        offset_ = (image_width_ * radius_) + radius_;

        const auto boundary = orderBoundaryEvaluation(generateBoundary(static_cast<int>(radius_)));
        auto interior       = generateInterior(static_cast<int>(radius_));
        interior.erase(
            std::remove_if(interior.begin(), interior.end(), [](const RuntimeCirclePoint& point) {
                return point.y < 0 || (point.y == 0 && point.x < 0);
            }),
            interior.end());
        std::sort(interior.begin(), interior.end(), [](const RuntimeCirclePoint& lhs, const RuntimeCirclePoint& rhs) {
            if (lhs.y == rhs.y) {
                return lhs.x < rhs.x;
            }
            return lhs.y < rhs.y;
        });

        boundary_offsets_.clear();
        boundary_offsets_.reserve(boundary.size());
        for (const auto& point : boundary) {
            boundary_offsets_.push_back(coord2to1(point, image_width_));
        }

        interior_offsets_.clear();
        interior_offsets_.reserve(interior.size());
        for (const auto& point : interior) {
            interior_offsets_.push_back(coord2to1(point, image_width_));
        }
    }

    unsigned radius_;
    unsigned image_width_;
    unsigned image_height_;
    unsigned offset_ = 0;
    std::vector<int> boundary_offsets_;
    std::vector<int> interior_offsets_;
};

} // namespace uvdar_core::detection::fimd