#include "uvdar_core/calibration/calibration_visualization.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include <opencv2/imgproc.hpp>

namespace uvdar_core::calibration {

namespace {

constexpr int panel_width = 390;
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

} // namespace

cv::Mat renderCalibrationVisualization(
    const cv::Mat& image,
    const CalibrationVisualizationState& state)
{
    const cv::Mat camera = toBgr(image);
    cv::Mat output(camera.rows, camera.cols + panel_width, CV_8UC3,
        cv::Scalar(29, 32, 38));
    camera.copyTo(output(cv::Rect(0, 0, camera.cols, camera.rows)));
    cv::Mat camera_area = output(cv::Rect(0, 0, camera.cols, camera.rows));
    drawPattern(camera_area, state);
    drawReprojection(camera_area, state);

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
        cv::circle(output, cv::Point(x + 7, y - 4), 5,
            done ? green : active ? cyan : cv::Scalar(75, 80, 90), -1,
            cv::LINE_AA);
        text(output, stages[static_cast<std::size_t>(index)],
            cv::Point(x + 22, y), 0.43,
            active ? white : done ? green : muted,
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
