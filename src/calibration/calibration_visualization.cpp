#include "uvdar_core/calibration/calibration_visualization.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>

#include <opencv2/imgproc.hpp>

namespace uvdar_core::calibration {

namespace {

constexpr int panel_width = 520;
const cv::Scalar white(242, 245, 248);
const cv::Scalar muted(155, 164, 178);
const cv::Scalar cyan(235, 190, 45);
const cv::Scalar green(100, 220, 95);
const cv::Scalar orange(40, 155, 250);
const cv::Scalar magenta(220, 90, 220);
const cv::Scalar red(80, 80, 240);

void text(
    cv::Mat& image,
    const std::string& value,
    const cv::Point origin,
    const double scale = 0.5,
    const cv::Scalar& color = white,
    const int thickness = 1)
{
    cv::putText(
        image,
        value,
        origin,
        cv::FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv::LINE_AA);
}

std::string compactNumber(const double value, const int precision = 3)
{
    if (!std::isfinite(value)) {
        return "--";
    }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

cv::Mat toBgr(const cv::Mat& input)
{
    if (input.empty()) {
        return cv::Mat::zeros(480, 752, CV_8UC3);
    }
    cv::Mat output;
    if (input.type() == CV_8UC3) {
        output = input.clone();
    } else if (input.channels() == 1) {
        cv::cvtColor(input, output, cv::COLOR_GRAY2BGR);
    } else if (input.channels() == 4) {
        cv::cvtColor(input, output, cv::COLOR_BGRA2BGR);
    } else {
        input.convertTo(output, CV_8UC3);
    }
    return output;
}

void drawPattern(
    cv::Mat& image,
    const CalibrationVisualizationState& state)
{
    for (const cv::Point2f& candidate : state.candidates) {
        cv::circle(image, candidate, 2, orange, -1, cv::LINE_AA);
    }
    if (state.hull_points.size() >= 3U) {
        for (std::size_t index = 0U; index < state.hull_points.size(); ++index) {
            cv::line(image,
                state.hull_points[index],
                state.hull_points[(index + 1U) % state.hull_points.size()],
                orange, 1, cv::LINE_AA);
        }
    }
    if (state.detected_points.empty()) {
        return;
    }
    if (state.pattern_rows > 0 && state.pattern_columns > 0
        && state.detected_points.size()
            == static_cast<std::size_t>(
                state.pattern_rows * state.pattern_columns)) {
        for (int row = 0; row < state.pattern_rows; ++row) {
            for (int column = 0; column + 1 < state.pattern_columns;
                 ++column) {
                const int first = state.pattern_column_major
                    ? column * state.pattern_rows + row
                    : row * state.pattern_columns + column;
                const int second = state.pattern_column_major
                    ? (column + 1) * state.pattern_rows + row
                    : first + 1;
                cv::line(image,
                    state.detected_points[static_cast<std::size_t>(first)],
                    state.detected_points[static_cast<std::size_t>(second)],
                    cyan, 1, cv::LINE_AA);
            }
        }
        for (int column = 0; column < state.pattern_columns; ++column) {
            for (int row = 0; row + 1 < state.pattern_rows; ++row) {
                const int first = state.pattern_column_major
                    ? column * state.pattern_rows + row
                    : row * state.pattern_columns + column;
                const int second = state.pattern_column_major
                    ? first + 1
                    : first + state.pattern_columns;
                cv::line(image,
                    state.detected_points[static_cast<std::size_t>(first)],
                    state.detected_points[static_cast<std::size_t>(second)],
                    cyan, 1, cv::LINE_AA);
            }
        }
    }
    for (std::size_t index = 0U; index < state.detected_points.size(); ++index) {
        cv::circle(image, state.detected_points[index],
            index == 0U ? 6 : 3,
            index == 0U ? orange : green,
            -1, cv::LINE_AA);
    }
}

void drawReprojection(
    cv::Mat& image,
    const CalibrationVisualizationState& state)
{
    const std::size_t count = std::min(
        state.measured_points.size(), state.projected_points.size());
    for (std::size_t index = 0U; index < count; ++index) {
        cv::line(image, state.measured_points[index],
            state.projected_points[index], red, 1, cv::LINE_AA);
        cv::circle(image, state.measured_points[index], 3, green, 1,
            cv::LINE_AA);
        cv::drawMarker(image, state.projected_points[index], magenta,
            cv::MARKER_CROSS, 7, 1, cv::LINE_AA);
    }
}

void drawProgressBar(
    cv::Mat& image,
    const cv::Rect& area,
    const double fraction,
    const cv::Scalar& color)
{
    cv::rectangle(image, area, cv::Scalar(61, 67, 78), -1, cv::LINE_AA);
    const cv::Rect filled(
        area.x,
        area.y,
        static_cast<int>(std::round(
            area.width * std::clamp(fraction, 0.0, 1.0))),
        area.height);
    if (filled.width > 0) {
        cv::rectangle(image, filled, color, -1, cv::LINE_AA);
    }
}

void drawCoverage(
    cv::Mat& image,
    const cv::Rect& area,
    const CalibrationVisualizationState& state)
{
    constexpr int columns = 5;
    constexpr int rows = 4;
    std::array<int, columns * rows> counts {};
    int maximum = 1;
    for (const cv::Point2f& point : state.accepted_centers_normalized) {
        const int column = std::clamp(
            static_cast<int>(point.x * columns), 0, columns - 1);
        const int row = std::clamp(
            static_cast<int>(point.y * rows), 0, rows - 1);
        maximum = std::max(maximum, ++counts[static_cast<std::size_t>(
                                      row * columns + column)]);
    }
    for (int row = 0; row < rows; ++row) {
        for (int column = 0; column < columns; ++column) {
            const cv::Rect cell(
                area.x + column * area.width / columns,
                area.y + row * area.height / rows,
                area.width / columns - 2,
                area.height / rows - 2);
            const double strength = static_cast<double>(
                counts[static_cast<std::size_t>(row * columns + column)])
                / maximum;
            const cv::Scalar color(
                57 + 30 * strength,
                62 + 115 * strength,
                70 + 75 * strength);
            cv::rectangle(image, cell, color, -1);
        }
    }
}

void drawCostPlot(
    cv::Mat& image,
    const cv::Rect& area,
    const std::vector<double>& history)
{
    cv::rectangle(image, area, cv::Scalar(38, 42, 50), -1);
    if (history.size() < 2U) {
        text(image, "waiting for optimizer", area.tl() + cv::Point(8, 22),
            0.4, muted);
        return;
    }
    std::vector<double> values;
    values.reserve(history.size());
    for (const double value : history) {
        values.push_back(std::log10(std::max(value, 1.0e-12)));
    }
    const auto [minimum, maximum] = std::minmax_element(
        values.begin(), values.end());
    const double range = std::max(*maximum - *minimum, 1.0e-6);
    const std::size_t first = values.size() > 100U
        ? values.size() - 100U : 0U;
    cv::Point previous;
    bool have_previous = false;
    for (std::size_t index = first; index < values.size(); ++index) {
        const double x_fraction = static_cast<double>(index - first)
            / static_cast<double>(std::max<std::size_t>(
                1U, values.size() - first - 1U));
        const double y_fraction = (values[index] - *minimum) / range;
        const cv::Point current(
            area.x + 3 + static_cast<int>(x_fraction * (area.width - 6)),
            area.y + area.height - 3
                - static_cast<int>(y_fraction * (area.height - 6)));
        if (have_previous) {
            cv::line(image, previous, current, cyan, 2, cv::LINE_AA);
        }
        previous = current;
        have_previous = true;
    }
}

void drawRmsPlot(
    cv::Mat& image,
    const cv::Rect& area,
    const CalibrationVisualizationState& state)
{
    cv::rectangle(image, area, cv::Scalar(38, 42, 50), -1);
    if (state.per_view_rms_px.empty()) {
        text(image, "no per-view errors", area.tl() + cv::Point(8, 22),
            0.4, muted);
        return;
    }

    double maximum_rms = std::max(1.0e-6, state.rms_px);
    for (const double rms : state.per_view_rms_px) {
        if (std::isfinite(rms)) {
            maximum_rms = std::max(maximum_rms, rms);
        }
    }
    maximum_rms *= 1.12;
    const int count = static_cast<int>(state.per_view_rms_px.size());
    const int baseline = area.y + area.height - 18;
    const double column_width = static_cast<double>(area.width - 10)
        / std::max(1, count);
    for (int index = 0; index < count; ++index) {
        const double rms = state.per_view_rms_px[static_cast<std::size_t>(index)];
        if (!std::isfinite(rms) || rms < 0.0) {
            continue;
        }
        const bool retained = index >= static_cast<int>(
            state.retained_view_mask.size())
            || state.retained_view_mask[static_cast<std::size_t>(index)];
        const int bar_height = static_cast<int>(std::round(
            rms / maximum_rms * (area.height - 29)));
        const int left = area.x + 5
            + static_cast<int>(std::floor(index * column_width));
        const int right = area.x + 5
            + static_cast<int>(std::floor((index + 1) * column_width));
        cv::rectangle(
            image,
            cv::Rect(left, baseline - bar_height,
                std::max(1, right - left - 1), bar_height),
            retained ? cyan : red,
            -1,
            cv::LINE_AA);
    }

    const int global_y = baseline - static_cast<int>(std::round(
        state.rms_px / maximum_rms * (area.height - 29)));
    cv::line(image,
        cv::Point(area.x + 4, global_y),
        cv::Point(area.x + area.width - 4, global_y),
        green, 1, cv::LINE_AA);
    text(image,
        "global " + compactNumber(state.rms_px) + " px",
        area.tl() + cv::Point(7, 15), 0.36, green);
    text(image, "view 1", cv::Point(area.x + 5, area.y + area.height - 4),
        0.3, muted);
    text(image, "view " + std::to_string(count),
        cv::Point(area.x + area.width - 58, area.y + area.height - 4),
        0.3, muted);
}

void drawProjectionPlot(
    cv::Mat& image,
    const cv::Rect& area,
    const std::vector<cv::Point2f>& curve)
{
    cv::rectangle(image, area, cv::Scalar(38, 42, 50), -1);
    if (curve.size() < 2U) {
        text(image, "projection samples unavailable",
            area.tl() + cv::Point(8, 22), 0.4, muted);
        return;
    }

    double maximum_angle = 1.0;
    double maximum_radius = 1.0;
    for (const cv::Point2f& point : curve) {
        maximum_angle = std::max(maximum_angle, static_cast<double>(point.x));
        maximum_radius = std::max(maximum_radius, static_cast<double>(point.y));
    }
    const cv::Rect graph(
        area.x + 8, area.y + 7, area.width - 16, area.height - 24);
    cv::Point previous;
    bool have_previous = false;
    for (const cv::Point2f& sample : curve) {
        const cv::Point current(
            graph.x + static_cast<int>(std::round(
                sample.x / maximum_angle * graph.width)),
            graph.y + graph.height - static_cast<int>(std::round(
                sample.y / maximum_radius * graph.height)));
        if (have_previous) {
            cv::line(image, previous, current, orange, 2, cv::LINE_AA);
        }
        previous = current;
        have_previous = true;
    }
    text(image, "0 deg", cv::Point(area.x + 5, area.y + area.height - 4),
        0.3, muted);
    text(image, compactNumber(maximum_angle, 1) + " deg",
        cv::Point(area.x + area.width - 66, area.y + area.height - 4),
        0.3, muted);
    text(image, compactNumber(maximum_radius, 1) + " px",
        area.tl() + cv::Point(7, 15), 0.34, orange);
}

std::string parameterNumber(const double value)
{
    std::ostringstream stream;
    const double magnitude = std::abs(value);
    if (magnitude > 0.0 && (magnitude < 1.0e-3 || magnitude >= 1.0e4)) {
        stream << std::scientific << std::setprecision(2) << value;
    } else {
        stream << std::fixed << std::setprecision(4) << value;
    }
    return stream.str();
}

std::string shortenedFromLeft(
    const std::string& value,
    const std::size_t maximum_characters)
{
    if (value.size() <= maximum_characters) {
        return value;
    }
    return "..." + value.substr(value.size() - maximum_characters + 3U);
}

std::vector<std::string> parameterLines(
    const std::string& label,
    const std::vector<double>& values,
    const std::size_t values_per_line = 4U)
{
    std::vector<std::string> lines;
    for (std::size_t first = 0U; first < values.size();
         first += values_per_line) {
        std::ostringstream line;
        line << (first == 0U ? label + " [" : "  ");
        const std::size_t last = std::min(
            values.size(), first + values_per_line);
        for (std::size_t index = first; index < last; ++index) {
            if (index > first) {
                line << ", ";
            }
            line << parameterNumber(values[index]);
        }
        line << (last == values.size() ? "]" : ",");
        lines.push_back(line.str());
    }
    return lines;
}

void drawResultParameters(
    cv::Mat& image,
    const int x,
    int& y,
    const int bottom,
    const CalibrationVisualizationState& state)
{
    text(image, "MODEL PARAMETERS", cv::Point(x, y), 0.42, muted, 1);
    y += 18;
    std::vector<std::string> lines;
    if (!state.intrinsics.empty()) {
        const auto intrinsics = parameterLines(
            "K(fx,fy,cx,cy)", state.intrinsics);
        const auto distortion = parameterLines(
            state.model == "pinhole" ? "D(k1,k2,p1,p2,k3)"
                                      : "D(k1,k2,k3,k4)",
            state.distortion);
        lines.insert(lines.end(), intrinsics.begin(), intrinsics.end());
        lines.insert(lines.end(), distortion.begin(), distortion.end());
    } else {
        lines.push_back(
            "center(x,y) [" + parameterNumber(state.center.x) + ", "
            + parameterNumber(state.center.y) + "]");
        lines.push_back(
            "stretch [[" + parameterNumber(state.stretch_matrix(0, 0)) + ", "
            + parameterNumber(state.stretch_matrix(0, 1)) + "],");
        lines.push_back(
            "         [" + parameterNumber(state.stretch_matrix(1, 0)) + ", "
            + parameterNumber(state.stretch_matrix(1, 1)) + "]]");
        const auto direct = parameterLines("direct", state.direct_polynomial);
        const auto inverse = parameterLines("inverse", state.inverse_polynomial);
        lines.insert(lines.end(), direct.begin(), direct.end());
        lines.insert(lines.end(), inverse.begin(), inverse.end());
    }
    for (const std::string& line : lines) {
        if (y > bottom) {
            break;
        }
        text(image, line, cv::Point(x, y), 0.34, white);
        y += 17;
    }
}

double cross2d(const cv::Point2f& first, const cv::Point2f& second)
{
    return static_cast<double>(first.x) * second.y
        - static_cast<double>(first.y) * second.x;
}

struct RingLabelPlacement {
    cv::Point2f position;
    cv::Point2f tangent;
};

std::optional<RingLabelPlacement> findRingLabelPlacement(
    const AngularProjectionRing& ring,
    const cv::Point2f& center,
    const cv::Point2f& direction,
    const cv::Rect& bounds)
{
    double best_distance = std::numeric_limits<double>::infinity();
    std::optional<RingLabelPlacement> best;
    for (std::size_t index = 1U; index < ring.points.size(); ++index) {
        const cv::Point2f first = ring.points[index - 1U];
        const cv::Point2f second = ring.points[index];
        if (!std::isfinite(first.x) || !std::isfinite(first.y)
            || !std::isfinite(second.x) || !std::isfinite(second.y)) {
            continue;
        }
        const cv::Point2f segment = second - first;
        const double denominator = cross2d(direction, segment);
        if (std::abs(denominator) < 1.0e-9) {
            continue;
        }
        const cv::Point2f center_to_segment = first - center;
        const double ray_distance =
            cross2d(center_to_segment, segment) / denominator;
        const double segment_fraction =
            cross2d(center_to_segment, direction) / denominator;
        if (ray_distance < 0.0 || segment_fraction < 0.0
            || segment_fraction > 1.0 || ray_distance >= best_distance) {
            continue;
        }
        const cv::Point2f intersection = center
            + static_cast<float>(ray_distance) * direction;
        const cv::Point rounded(
            static_cast<int>(std::lround(intersection.x)),
            static_cast<int>(std::lround(intersection.y)));
        if (!bounds.contains(rounded) || rounded.y <= 84) {
            continue;
        }
        best_distance = ray_distance;
        best = RingLabelPlacement {intersection, segment};
    }
    return best;
}

void applyTextMask(
    cv::Mat& image,
    const cv::Mat& mask,
    const cv::Point2f& center,
    const cv::Scalar& color)
{
    const cv::Point top_left(
        static_cast<int>(std::lround(center.x - 0.5F * mask.cols)),
        static_cast<int>(std::lround(center.y - 0.5F * mask.rows)));
    const cv::Rect destination = cv::Rect(
        top_left.x, top_left.y, mask.cols, mask.rows)
        & cv::Rect(0, 0, image.cols, image.rows);
    if (destination.empty()) {
        return;
    }
    const cv::Rect source(
        destination.x - top_left.x,
        destination.y - top_left.y,
        destination.width,
        destination.height);
    image(destination).setTo(color, mask(source));
}

void rotatedText(
    cv::Mat& image,
    const std::string& value,
    const cv::Point2f& center,
    cv::Point2f tangent,
    const cv::Scalar& color)
{
    constexpr double scale = 0.36;
    constexpr int supersampling = 3;
    constexpr int foreground_thickness = supersampling;
    constexpr int outline_thickness = 3 * supersampling;
    int baseline = 0;
    const cv::Size text_size = cv::getTextSize(
        value,
        cv::FONT_HERSHEY_SIMPLEX,
        scale * supersampling,
        foreground_thickness,
        &baseline);
    const int render_side = std::max(
        24 * supersampling,
        static_cast<int>(std::ceil(std::hypot(
            text_size.width + 10.0 * supersampling,
            text_size.height + baseline + 10.0 * supersampling))));
    const cv::Point origin(
        (render_side - text_size.width) / 2,
        (render_side + text_size.height - baseline) / 2);
    cv::Mat outline = cv::Mat::zeros(
        render_side, render_side, CV_8UC1);
    cv::Mat foreground = cv::Mat::zeros(
        render_side, render_side, CV_8UC1);
    cv::putText(outline, value, origin, cv::FONT_HERSHEY_SIMPLEX,
        scale * supersampling, cv::Scalar(255), outline_thickness, cv::LINE_AA);
    cv::putText(foreground, value, origin, cv::FONT_HERSHEY_SIMPLEX,
        scale * supersampling, cv::Scalar(255), foreground_thickness, cv::LINE_AA);

    if (tangent.x < 0.0F
        || (std::abs(tangent.x) < 1.0e-6F && tangent.y < 0.0F)) {
        tangent *= -1.0F;
    }
    const double image_angle = std::atan2(tangent.y, tangent.x)
        * 180.0 / M_PI;
    const cv::Point2f patch_center(
        0.5F * static_cast<float>(render_side - 1),
        0.5F * static_cast<float>(render_side - 1));
    const cv::Mat rotation = cv::getRotationMatrix2D(
        patch_center, -image_angle, 1.0);
    cv::Mat rotated_outline;
    cv::Mat rotated_foreground;
    cv::warpAffine(outline, rotated_outline, rotation, outline.size(),
        cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::warpAffine(foreground, rotated_foreground, rotation, foreground.size(),
        cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
    const int output_side = std::max(24, render_side / supersampling);
    cv::resize(rotated_outline, rotated_outline,
        cv::Size(output_side, output_side), 0.0, 0.0, cv::INTER_AREA);
    cv::resize(rotated_foreground, rotated_foreground,
        cv::Size(output_side, output_side), 0.0, 0.0, cv::INTER_AREA);
    applyTextMask(image, rotated_outline, center, cv::Scalar(15, 18, 23));
    applyTextMask(image, rotated_foreground, center, color);
}

void drawCalibratedFieldOfView(
    cv::Mat& camera,
    const CalibrationVisualizationState& state)
{
    if (camera.empty() || state.angular_projection_rings.empty()) {
        return;
    }
    const cv::Rect bounds(0, 0, camera.cols, camera.rows);
    const cv::Point2f center =
        std::isfinite(state.calibrated_center.x)
            && std::isfinite(state.calibrated_center.y)
        ? state.calibrated_center
        : cv::Point2f(0.5F * camera.cols, 0.5F * camera.rows);
    cv::Point2f diagonal(
        static_cast<float>(camera.cols - 1) - center.x,
        -center.y);
    const float diagonal_norm = std::hypot(diagonal.x, diagonal.y);
    if (diagonal_norm > 1.0e-6F) {
        diagonal *= 1.0F / diagonal_norm;
    } else {
        diagonal = cv::Point2f(1.0F, -1.0F) * static_cast<float>(M_SQRT1_2);
    }

    for (const AngularProjectionRing& ring : state.angular_projection_rings) {
        const cv::Scalar color = ring.limit ? orange : cyan;
        const int thickness = ring.limit ? 3 : 1;
        for (std::size_t index = 1U; index < ring.points.size(); ++index) {
            const cv::Point2f& first_float = ring.points[index - 1U];
            const cv::Point2f& second_float = ring.points[index];
            if (!std::isfinite(first_float.x) || !std::isfinite(first_float.y)
                || !std::isfinite(second_float.x)
                || !std::isfinite(second_float.y)) {
                continue;
            }
            cv::Point first(
                static_cast<int>(std::lround(first_float.x)),
                static_cast<int>(std::lround(first_float.y)));
            cv::Point second(
                static_cast<int>(std::lround(second_float.x)),
                static_cast<int>(std::lround(second_float.y)));
            cv::Point clipped_first = first;
            cv::Point clipped_second = second;
            if (cv::clipLine(bounds, clipped_first, clipped_second)) {
                cv::line(camera, clipped_first, clipped_second,
                    color, thickness, cv::LINE_AA);
            }
        }
    }

    for (const AngularProjectionRing& ring : state.angular_projection_rings) {
        const std::optional<RingLabelPlacement> placement =
            findRingLabelPlacement(ring, center, diagonal, bounds);
        if (!placement) {
            continue;
        }
        const std::string label = compactNumber(ring.angle_degrees, 1)
            + (ring.limit ? " deg limit" : " deg");
        rotatedText(
            camera,
            label,
            placement->position - 3.0F * diagonal,
            placement->tangent,
            ring.limit ? orange : cyan);
    }

    if (std::isfinite(state.calibrated_center.x)
        && std::isfinite(state.calibrated_center.y)) {
        const cv::Point center(
            static_cast<int>(std::lround(state.calibrated_center.x)),
            static_cast<int>(std::lround(state.calibrated_center.y)));
        if (bounds.contains(center)) {
            cv::drawMarker(camera, center, cv::Scalar(15, 18, 23),
                cv::MARKER_CROSS, 31, 5, cv::LINE_AA);
            cv::drawMarker(camera, center, green,
                cv::MARKER_CROSS, 31, 2, cv::LINE_AA);
            cv::circle(camera, center, 5, green, 2, cv::LINE_AA);
            const std::string center_label = "calibrated center  "
                + compactNumber(state.calibrated_center.x, 1) + ", "
                + compactNumber(state.calibrated_center.y, 1);
            const cv::Point origin(
                std::clamp(center.x + 12, 4, camera.cols - 250),
                std::clamp(center.y - 12, 90, camera.rows - 5));
            text(camera, center_label, origin, 0.4,
                cv::Scalar(15, 18, 23), 3);
            text(camera, center_label, origin, 0.4, green, 1);
        }
    }
}

} // namespace

cv::Mat renderCalibrationDetectionOverlay(
    const cv::Mat& image,
    const CalibrationVisualizationState& state)
{
    cv::Mat output = toBgr(image);
    drawPattern(output, state);
    return output;
}

cv::Mat renderCalibrationVisualization(
    const cv::Mat& image,
    const CalibrationVisualizationState& state)
{
    const cv::Mat camera = toBgr(image);
    const bool terminal = state.successful || state.failed;
    const int output_rows = terminal ? std::max(camera.rows, 940) : camera.rows;
    cv::Mat output(output_rows, camera.cols + panel_width, CV_8UC3,
        cv::Scalar(29, 32, 38));
    camera.copyTo(output(cv::Rect(0, 0, camera.cols, camera.rows)));
    cv::Mat camera_area = output(cv::Rect(0, 0, camera.cols, camera.rows));
    if (terminal) {
        drawCalibratedFieldOfView(camera_area, state);
    } else {
        drawPattern(camera_area, state);
        drawReprojection(camera_area, state);
    }

    cv::Mat shade = camera_area.clone();
    cv::rectangle(shade, cv::Rect(0, 0, camera.cols, 72),
        cv::Scalar(15, 18, 23), -1);
    cv::addWeighted(shade, 0.72, camera_area, 0.28, 0.0, camera_area);
    text(camera_area, "UVDAR CAMERA CALIBRATION", cv::Point(20, 30),
        0.66, white, 2);
    text(camera_area, state.stage + "  /  " + state.detail,
        cv::Point(20, 56), 0.48,
        state.failed ? red : state.successful ? green : cyan);

    const int x = camera.cols + 22;
    int y = 32;
    text(output, "CALIBRATOR", cv::Point(x, y), 0.68, white, 2);
    y += 31;
    text(output, state.model + "  |  " + state.pattern,
        cv::Point(x, y), 0.46, muted);
    y += 28;

    static const std::array<const char*, 6> stages {
        "Collect frames", "Initialize", "Optimize",
        "Reject / refine", "Validate / save", "Complete"};
    for (int index = 0; index < static_cast<int>(stages.size()); ++index) {
        const bool done = index < state.stage_index;
        const bool active = index == state.stage_index;
        const cv::Scalar stage_color = done || (active && state.successful)
            ? green
            : active && state.failed ? red
            : active ? cyan : cv::Scalar(75, 80, 90);
        cv::circle(output, cv::Point(x + 7, y - 4), 5,
            stage_color, -1, cv::LINE_AA);
        text(output, stages[static_cast<std::size_t>(index)],
            cv::Point(x + 22, y), 0.43,
            done || (active && state.successful) ? green
                : active && state.failed ? red
                : active ? white : muted,
            active ? 2 : 1);
        y += 20;
    }

    y += 5;
    const bool collecting = state.stage_index == 0;
    const double fraction = collecting
        ? static_cast<double>(state.accepted_frames)
            / std::max(1, state.required_frames)
        : state.maximum_iterations > 0
        ? static_cast<double>(state.iteration)
            / state.maximum_iterations
        : state.stage_index / 5.0;
    drawProgressBar(output, cv::Rect(x, y, panel_width - 44, 8),
        fraction, state.failed ? red : green);
    y += 20;

    if (terminal) {
        text(output,
            state.result_message.empty() ? state.detail : state.result_message,
            cv::Point(x, y), 0.46,
            state.successful ? green : red, 1);
        y += 22;
        text(output,
            "RMS " + compactNumber(state.rms_px, 4) + " px   views "
                + std::to_string(state.retained_views) + "/"
                + std::to_string(state.total_views),
            cv::Point(x, y), 0.48, white, 1);
        y += 20;
        text(output,
            "image " + std::to_string(state.image_width) + "x"
                + std::to_string(state.image_height) + "   iterations "
                + std::to_string(state.result_iterations),
            cv::Point(x, y), 0.4, muted, 1);
        y += 18;
        text(output,
            "detected FoV " + compactNumber(state.detected_fov_degrees, 1)
                + " deg   expected "
                + (state.expected_fov_degrees > 0.0
                    ? compactNumber(state.expected_fov_degrees, 1) + " deg"
                    : std::string("automatic")),
            cv::Point(x, y), 0.38, muted, 1);
        y += 18;
        text(output,
            "outer ring "
                + compactNumber(0.5 * state.visualized_fov_degrees, 1)
                + " deg off-axis",
            cv::Point(x, y), 0.38, orange, 1);
        y += 18;
        text(output,
            "total " + compactNumber(state.elapsed_seconds, 3)
                + " s   initialize "
                + compactNumber(state.initialization_seconds, 3) + " s",
            cv::Point(x, y), 0.38, muted, 1);
        y += 18;
        text(output,
            "optimize " + compactNumber(state.optimization_seconds, 3)
                + " s   refine "
                + compactNumber(state.refinement_seconds, 3)
                + " s   validate "
                + compactNumber(state.validation_seconds, 3) + " s",
            cv::Point(x, y), 0.35, muted, 1);
        y += 23;

        text(output, "PER-VIEW RMS ERROR", cv::Point(x, y), 0.42, muted, 1);
        y += 7;
        drawRmsPlot(output, cv::Rect(x, y, panel_width - 44, 92), state);
        y += 111;
        text(output, "MODEL RADIAL PROJECTION", cv::Point(x, y),
            0.42, muted, 1);
        y += 7;
        drawProjectionPlot(output,
            cv::Rect(x, y, panel_width - 44, 92),
            state.model_projection_curve);
        y += 111;
        text(output, "OPTIMIZATION COST (log)", cv::Point(x, y),
            0.42, muted, 1);
        y += 7;
        drawCostPlot(output,
            cv::Rect(x, y, panel_width - 44, 82),
            state.cost_history);
        y += 103;
        drawResultParameters(output, x, y, output.rows - 48, state);

        text(output,
            "output: " + shortenedFromLeft(state.output_path, 72U),
            cv::Point(x, output.rows - 22), 0.32, muted);
        return output;
    }

    if (collecting) {
        text(output,
            std::to_string(state.accepted_frames) + " / "
                + std::to_string(state.required_frames) + " diverse frames",
            cv::Point(x, y), 0.5, white, 1);
        y += 21;
        text(output,
            std::to_string(state.detected_candidates) + " candidates  |  "
                + (state.pattern_found ? "pattern locked" : "searching"),
            cv::Point(x, y), 0.42,
            state.pattern_found ? green : muted);
    } else {
        text(output,
            "iteration " + std::to_string(state.iteration) + " / "
                + std::to_string(state.maximum_iterations),
            cv::Point(x, y), 0.5, white);
        y += 21;
        text(output,
            "RMS  " + compactNumber(state.rms_px) + " px     lambda  "
                + compactNumber(state.damping, 2),
            cv::Point(x, y), 0.42,
            state.rms_px > 0.0 ? cyan : muted);
        y += 21;
        text(output,
            "elapsed  " + compactNumber(state.elapsed_seconds, 3) + " s",
            cv::Point(x, y), 0.42, muted);
    }
    y += 20;

    text(output, "FRAME COVERAGE", cv::Point(x, y), 0.42, muted, 1);
    y += 8;
    drawCoverage(output, cv::Rect(x, y, panel_width - 44, 64), state);
    y += 80;
    text(output, "OPTIMIZATION COST (log)", cv::Point(x, y), 0.42, muted, 1);
    y += 8;
    const int plot_height = std::max(35, std::min(90, output.rows - y - 42));
    drawCostPlot(output, cv::Rect(x, y, panel_width - 44, plot_height),
        state.cost_history);

    const std::string filename = state.output_path.empty()
        ? "output path not configured"
        : std::filesystem::path(state.output_path).filename().string();
    text(output, filename, cv::Point(x, output.rows - 24), 0.4, muted);
    return output;
}

} // namespace uvdar_core::calibration
