#include "uvdar_core/calibration/grid_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <utility>

namespace uvdar_core::calibration {

namespace {

constexpr double pi = 3.14159265358979323846;

bool pointsEqual(const cv::Point2f& first, const cv::Point2f& second)
{
    return static_cast<int>(first.x) == static_cast<int>(second.x)
        && static_cast<int>(first.y) == static_cast<int>(second.y);
}

float distance(const cv::Point2f& first, const cv::Point2f& second)
{
    return cv::norm(first - second);
}

double directedAngle(
    const cv::Point2f& first,
    const cv::Point2f& center,
    const cv::Point2f& last)
{
    const cv::Point2f a = first - center;
    const cv::Point2f b = last - center;
    const double sine = static_cast<double>(a.x) * b.y
        - static_cast<double>(a.y) * b.x;
    const double cosine = static_cast<double>(a.x) * b.x
        + static_cast<double>(a.y) * b.y;
    double angle = -std::atan2(sine, cosine);
    if (angle < 0.0) {
        angle += 2.0 * pi;
    }
    return angle;
}

float maximumNeighborDistance(
    const cv::Point2f& current,
    const std::vector<cv::Point2f>& points,
    const std::size_t maximum_samples = 7U)
{
    std::vector<float> distances;
    distances.reserve(points.size());
    for (const cv::Point2f& point : points) {
        const float value = distance(point, current);
        if (value > 1.0e-6F) {
            distances.push_back(value);
        }
    }
    if (distances.empty()) {
        return 1.0F;
    }
    std::sort(distances.begin(), distances.end());
    const std::size_t count = std::min(maximum_samples, distances.size());
    return 1.5F * std::accumulate(
        distances.begin(), distances.begin() + count, 0.0F)
        / static_cast<float>(count);
}

std::vector<cv::Point2f> uniqueIntegerPoints(
    const std::vector<cv::Point2f>& points)
{
    std::map<std::pair<int, int>, bool> seen;
    std::vector<cv::Point2f> output;
    output.reserve(points.size());
    for (const cv::Point2f& point : points) {
        const std::pair<int, int> key {
            static_cast<int>(point.x), static_cast<int>(point.y)};
        if (seen.emplace(key, true).second) {
            output.push_back(point);
        }
    }
    return output;
}

struct HullNode {
    cv::Point2f point;
    int left = -1;
    int right = -1;
    double angle = -1.0;
};

struct Hull {
    std::vector<HullNode> nodes;
    int start = -1;
};

std::optional<Hull> buildHull(
    const std::vector<cv::Point2f>& points,
    const GridExtractorOptions& options)
{
    if (points.empty()) {
        return std::nullopt;
    }
    std::vector<bool> marked(points.size(), false);
    std::size_t left_index = 0U;
    for (std::size_t index = 1U; index < points.size(); ++index) {
        if (static_cast<int>(points[index].x)
            < static_cast<int>(points[left_index].x)) {
            left_index = index;
        }
    }

    Hull hull;
    hull.nodes.reserve(points.size());
    hull.nodes.push_back(HullNode {points[left_index]});
    hull.start = 0;
    int current_node = hull.start;
    marked[left_index] = true;
    cv::Point2f previous = points[left_index];
    previous.y += 10.0F;

    // Bound the hull walk so malformed or heavily cluttered candidate sets
    // fail deterministically instead of cycling indefinitely.
    for (std::size_t walk = 0U; walk <= points.size(); ++walk) {
        const cv::Point2f current = hull.nodes[static_cast<std::size_t>(current_node)].point;
        const float maximum_distance = maximumNeighborDistance(current, points);
        double best_angle = 0.0;
        float best_distance = 9.0e9F;
        int best_index = -1;
        for (std::size_t index = 0U; index < points.size(); ++index) {
            if (pointsEqual(points[index], current)) {
                continue;
            }
            if (marked[index]
                && !pointsEqual(points[index], hull.nodes[0].point)) {
                continue;
            }
            const float candidate_distance = distance(current, points[index]);
            if (candidate_distance > maximum_distance) {
                continue;
            }
            const double candidate_angle = directedAngle(
                previous, current, points[index]);
            if (candidate_angle > pi + options.maximum_concave_angle) {
                continue;
            }
            if ((std::abs(candidate_angle - best_angle)
                        < options.similar_angle
                    && candidate_distance < best_distance)
                || candidate_angle > best_angle + options.similar_angle) {
                best_angle = candidate_angle;
                best_distance = candidate_distance;
                best_index = static_cast<int>(index);
            }
        }
        if (best_index < 0) {
            return std::nullopt;
        }

        const bool done = pointsEqual(
            points[static_cast<std::size_t>(best_index)], hull.nodes[0].point);
        const int next_node = done ? hull.start
            : static_cast<int>(hull.nodes.size());
        if (!done) {
            hull.nodes.push_back(HullNode {
                points[static_cast<std::size_t>(best_index)]});
        }
        hull.nodes[static_cast<std::size_t>(current_node)].right = next_node;
        hull.nodes[static_cast<std::size_t>(current_node)].angle = best_angle;
        hull.nodes[static_cast<std::size_t>(next_node)].left = current_node;
        marked[static_cast<std::size_t>(best_index)] = true;
        if (done) {
            return hull.nodes.size() >= 3U
                ? std::optional<Hull>(std::move(hull)) : std::nullopt;
        }
        previous = current;
        current_node = next_node;
    }
    return std::nullopt;
}

int sharpestHullPoint(const Hull& hull)
{
    int sharpest = hull.start;
    double best = 2.0 * pi;
    int current = hull.start;
    do {
        if (hull.nodes[static_cast<std::size_t>(current)].angle < best) {
            best = hull.nodes[static_cast<std::size_t>(current)].angle;
            sharpest = current;
        }
        current = hull.nodes[static_cast<std::size_t>(current)].right;
    } while (current >= 0 && current != hull.start);
    return sharpest;
}

int walkRight(const Hull& hull, int node, const int count)
{
    for (int index = 0; index < count && node >= 0; ++index) {
        node = hull.nodes[static_cast<std::size_t>(node)].right;
    }
    return node;
}

struct GridNode {
    cv::Point2f point;
    int left = -1;
    int right = -1;
    int above = -1;
    int below = -1;
};

struct Grid {
    std::vector<GridNode> nodes;
    int root = -1;
    bool x_axis_first = true;
};

int addGridNode(Grid& grid, const cv::Point2f& point)
{
    grid.nodes.push_back(GridNode {point});
    return static_cast<int>(grid.nodes.size() - 1U);
}

std::optional<Grid> buildGrid(
    const Hull& hull,
    const int corner,
    const std::vector<cv::Point2f>& points,
    const int width,
    const int height)
{
    if (corner < 0
        || hull.nodes[static_cast<std::size_t>(corner)].right < 0
        || hull.nodes[static_cast<std::size_t>(corner)].left < 0) {
        return std::nullopt;
    }
    std::vector<bool> marked(points.size(), false);
    int hull_current = corner;
    Grid grid;
    grid.nodes.reserve(static_cast<std::size_t>(width * height));
    grid.root = addGridNode(
        grid, hull.nodes[static_cast<std::size_t>(hull_current)].point);
    int grid_current = grid.root;
    int grid_first_in_axis = grid.root;
    int hull_first_in_axis = corner;

    const int right = addGridNode(grid,
        hull.nodes[static_cast<std::size_t>(
            hull.nodes[static_cast<std::size_t>(hull_current)].right)].point);
    const int below = addGridNode(grid,
        hull.nodes[static_cast<std::size_t>(
            hull.nodes[static_cast<std::size_t>(hull_current)].left)].point);
    grid.nodes[static_cast<std::size_t>(grid_current)].right = right;
    grid.nodes[static_cast<std::size_t>(grid_current)].below = below;
    grid.nodes[static_cast<std::size_t>(right)].left = grid_current;
    grid.nodes[static_cast<std::size_t>(below)].above = grid_current;

    int hull_end_a = walkRight(hull, hull_current, width - 1);
    int hull_end_b = walkRight(hull, hull_current, height - 1);
    const int hull_last = walkRight(hull, hull_current, height + width - 2);
    if (hull_end_a < 0 || hull_end_b < 0 || hull_last < 0) {
        return std::nullopt;
    }

    bool first_line = true;
    bool x_axis_first = true;
    const std::size_t maximum_steps =
        std::max<std::size_t>(64U, points.size() * points.size() * 2U);
    std::size_t steps = 0U;
    while (steps++ < maximum_steps) {
        while (steps++ < maximum_steps) {
            const int grid_right =
                grid.nodes[static_cast<std::size_t>(grid_current)].right;
            const int grid_below =
                grid.nodes[static_cast<std::size_t>(grid_current)].below;
            if (grid_right < 0 || grid_below < 0) {
                break;
            }
            const cv::Point2f origin =
                grid.nodes[static_cast<std::size_t>(grid_current)].point;
            const cv::Point2f x_vector =
                grid.nodes[static_cast<std::size_t>(grid_right)].point - origin;
            const cv::Point2f y_vector =
                grid.nodes[static_cast<std::size_t>(grid_below)].point - origin;
            const float determinant = x_vector.x * y_vector.y
                - y_vector.x * x_vector.y;
            bool found = false;
            float best_distance = 9.0e9F;
            int best_index = -1;
            if (std::abs(determinant) > 1.0e-12F) {
                for (std::size_t index = 0U; index < points.size(); ++index) {
                    if (marked[index]) {
                        continue;
                    }
                    const cv::Point2f delta = points[index] - origin;
                    const cv::Point2f transformed(
                        (y_vector.y * delta.x - y_vector.x * delta.y)
                            / determinant,
                        (-x_vector.y * delta.x + x_vector.x * delta.y)
                            / determinant);
                    if (transformed.x > 0.5F && transformed.x < 1.5F
                        && transformed.y > 0.5F && transformed.y < 1.5F) {
                        const float candidate_distance = cv::norm(
                            transformed - cv::Point2f(1.0F, 1.0F));
                        if (!found || candidate_distance < best_distance) {
                            found = true;
                            best_distance = candidate_distance;
                            best_index = static_cast<int>(index);
                        }
                    }
                }
            }

            if (found && best_index >= 0) {
                const int diagonal = addGridNode(
                    grid, points[static_cast<std::size_t>(best_index)]);
                grid.nodes[static_cast<std::size_t>(grid_right)].below = diagonal;
                grid.nodes[static_cast<std::size_t>(grid_below)].right = diagonal;
                grid.nodes[static_cast<std::size_t>(diagonal)].above = grid_right;
                grid.nodes[static_cast<std::size_t>(diagonal)].left = grid_below;
                marked[static_cast<std::size_t>(best_index)] = true;
                if (pointsEqual(
                        points[static_cast<std::size_t>(best_index)],
                        hull.nodes[static_cast<std::size_t>(hull_last)].point)) {
                    grid.x_axis_first = x_axis_first;
                    return grid;
                }
            } else {
                if (hull_end_a >= 0 && pointsEqual(
                        grid.nodes[static_cast<std::size_t>(grid_current)].point,
                        hull.nodes[static_cast<std::size_t>(hull_end_a)].point)) {
                    x_axis_first = true;
                    hull_end_a = hull.nodes[static_cast<std::size_t>(hull_end_a)].right;
                    hull_end_b = -1;
                    grid.nodes[static_cast<std::size_t>(grid_current)].right = -1;
                    break;
                }
                if (hull_end_b >= 0 && pointsEqual(
                        grid.nodes[static_cast<std::size_t>(grid_current)].point,
                        hull.nodes[static_cast<std::size_t>(hull_end_b)].point)) {
                    x_axis_first = false;
                    hull_end_b = hull.nodes[static_cast<std::size_t>(hull_end_b)].right;
                    hull_end_a = -1;
                    grid.nodes[static_cast<std::size_t>(grid_current)].right = -1;
                    break;
                }
            }

            hull_current = hull.nodes[static_cast<std::size_t>(hull_current)].right;
            grid_current =
                grid.nodes[static_cast<std::size_t>(grid_current)].right;
            if (hull_current < 0 || grid_current < 0) {
                return std::nullopt;
            }
            const int hull_next =
                hull.nodes[static_cast<std::size_t>(hull_current)].right;
            if (first_line && hull_next >= 0) {
                const int next = addGridNode(
                    grid, hull.nodes[static_cast<std::size_t>(hull_next)].point);
                grid.nodes[static_cast<std::size_t>(grid_current)].right = next;
                grid.nodes[static_cast<std::size_t>(next)].left = grid_current;
            }
            if (grid.nodes[static_cast<std::size_t>(grid_current)].right < 0) {
                if (x_axis_first && hull_end_a >= 0) {
                    hull_end_a =
                        hull.nodes[static_cast<std::size_t>(hull_end_a)].right;
                } else if (!x_axis_first && hull_end_b >= 0) {
                    hull_end_b =
                        hull.nodes[static_cast<std::size_t>(hull_end_b)].right;
                }
                break;
            }
        }

        first_line = false;
        if (hull_current == hull_last) {
            break;
        }
        hull_current =
            hull.nodes[static_cast<std::size_t>(hull_first_in_axis)].left;
        const int next_grid_line =
            grid.nodes[static_cast<std::size_t>(grid_first_in_axis)].below;
        if (hull_current < 0 || next_grid_line < 0) {
            grid.x_axis_first = x_axis_first;
            return grid;
        }
        grid_current = next_grid_line;
        const int hull_below =
            hull.nodes[static_cast<std::size_t>(hull_current)].left;
        if (hull_below < 0) {
            grid.x_axis_first = x_axis_first;
            return grid;
        }
        const int next_below = addGridNode(
            grid, hull.nodes[static_cast<std::size_t>(hull_below)].point);
        grid.nodes[static_cast<std::size_t>(grid_current)].below = next_below;
        grid.nodes[static_cast<std::size_t>(next_below)].above = grid_current;
        hull_first_in_axis = hull_current;
        grid_first_in_axis = grid_current;
    }

    grid.x_axis_first = x_axis_first;
    return grid;
}

std::optional<std::vector<cv::Point2f>> traverseGrid(
    const Grid& grid, const int columns, const int rows)
{
    if (grid.root < 0) {
        return std::nullopt;
    }
    // Preserve the grid's geometric row traversal while returning the
    // column-major order expected by the LED pattern object points.
    std::vector<std::vector<cv::Point2f>> by_column(
        static_cast<std::size_t>(columns),
        std::vector<cv::Point2f>(static_cast<std::size_t>(rows)));
    int line_first = grid.root;
    for (int row = 0; row < rows; ++row) {
        int current = line_first;
        for (int column = 0; column < columns; ++column) {
            if (current < 0) {
                return std::nullopt;
            }
            by_column[static_cast<std::size_t>(column)]
                     [static_cast<std::size_t>(row)] =
                grid.nodes[static_cast<std::size_t>(current)].point;
            current = grid.x_axis_first
                ? grid.nodes[static_cast<std::size_t>(current)].right
                : grid.nodes[static_cast<std::size_t>(current)].below;
        }
        line_first = grid.x_axis_first
            ? grid.nodes[static_cast<std::size_t>(line_first)].below
            : grid.nodes[static_cast<std::size_t>(line_first)].right;
        if (line_first < 0 && row != rows - 1) {
            return std::nullopt;
        }
    }
    std::vector<cv::Point2f> ordered;
    ordered.reserve(static_cast<std::size_t>(columns * rows));
    for (int column = 0; column < columns; ++column) {
        for (int row = 0; row < rows; ++row) {
            ordered.push_back(by_column[static_cast<std::size_t>(column)]
                                       [static_cast<std::size_t>(row)]);
        }
    }
    return ordered;
}

} // namespace

GridExtractionResult extractGridFromFimdPoints(
    const std::vector<cv::Point2f>& input_points,
    const int columns,
    const int rows,
    const GridExtractorOptions& options)
{
    GridExtractionResult result;
    if (columns < 2 || rows < 2) {
        result.detail = "Grid dimensions must both be at least two";
        return result;
    }
    const std::vector<cv::Point2f> points =
        uniqueIntegerPoints(input_points);
    if (points.size() < static_cast<std::size_t>(columns * rows)) {
        result.detail = "Not enough unique integer-pixel candidates";
        return result;
    }
    if (options.maximum_concave_angle < 0.0
        || options.similar_angle < 0.0) {
        result.detail = "Hull angle thresholds must be non-negative";
        return result;
    }

    const auto hull = buildHull(points, options);
    if (!hull) {
        result.detail = "Concave hull walk did not close";
        return result;
    }
    int current = hull->start;
    do {
        result.hull_points.push_back(
            hull->nodes[static_cast<std::size_t>(current)].point);
        current = hull->nodes[static_cast<std::size_t>(current)].right;
    } while (current >= 0 && current != hull->start);

    const int corner = sharpestHullPoint(*hull);
    const auto grid = buildGrid(*hull, corner, points, columns, rows);
    if (!grid) {
        result.detail = "Hull could not seed a complete projective grid";
        return result;
    }
    const auto ordered = traverseGrid(*grid, columns, rows);
    if (!ordered
        || ordered->size() != static_cast<std::size_t>(columns * rows)) {
        result.detail = "Grid graph traversal is incomplete";
        return result;
    }
    result.success = true;
    result.x_axis_first = grid->x_axis_first;
    result.image_points = *ordered;
    result.detail = "FIMD hull/grid extractor ordered the target";
    return result;
}

} // namespace uvdar_core::calibration
