#include "uvdar_core/app/visualization.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>
#include <sstream>
#include <utility>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <CvPlot/cvplot.h>

namespace uvdar_core::app::visualization {

namespace {

const cv::Scalar kCanvasColor(255, 255, 255);
const cv::Scalar kPanelColor(255, 255, 255);
const cv::Scalar kBorderColor(190, 190, 190);
const cv::Scalar kGridColor(230, 230, 230);
const cv::Scalar kTextColor(35, 35, 35);
const cv::Scalar kCovarianceColor(180, 180, 0);
const std::array<cv::Scalar, 3> kAxisColors {
    cv::Scalar(0, 0, 220),   // X / roll axis: red in BGR.
    cv::Scalar(0, 200, 0),   // Y / pitch axis: green.
    cv::Scalar(220, 0, 0),   // Z / yaw axis: blue.
};
constexpr double kRangePadding = 1.2;
constexpr double kAxisLengthFactor = 0.12;
constexpr double kCovarianceScale = 2.0;
constexpr double kCvPlotMarginLeft = 40.0;
constexpr double kCvPlotMarginRight = 10.0;
constexpr double kCvPlotMarginTop = 26.0;
constexpr double kCvPlotMarginBottom = 30.0;

enum class PosePlotPlane {
    XY,
    XZ,
    YZ,
};

struct PoseLayout {
    cv::Rect table;
    cv::Rect xy;
    cv::Rect xz;
    cv::Rect yz;
};

cv::Mat toBgr(const cv::Mat& image, const cv::Size& fallback_size = {})
{
    cv::Mat source = image;
    if (source.empty()) {
        const int width = std::max(1, fallback_size.width);
        const int height = std::max(1, fallback_size.height);
        source = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
    }

    cv::Mat bgr;
    switch (source.channels()) {
        case 1:
            cv::cvtColor(source, bgr, cv::COLOR_GRAY2BGR);
            break;
        case 3:
            bgr = source.clone();
            break;
        case 4:
            cv::cvtColor(source, bgr, cv::COLOR_BGRA2BGR);
            break;
        default:
            return {};
    }
    return bgr;
}

PoseLayout poseLayout(const cv::Size& size)
{
    const int margin = std::max(8, static_cast<int>(std::lround(std::min(size.width, size.height) * 0.012)));
    const int gap = std::max(6, margin / 2);
    const int content_width = std::max(3, size.width - 2 * margin);
    const int content_height = std::max(2, size.height - 2 * margin);

    // Allocate fractions of the available width rather than fixed panel
    // widths.  The compact table remains on the left while the three plots
    // stay tightly grouped on the right; smaller callers retain
    // non-overlapping panels as well.
    const int table_width = std::clamp(
        static_cast<int>(std::lround(content_width * 0.25)),
        260,
        std::max(260, static_cast<int>(std::lround(content_width * 0.29))));
    const int side_width = std::clamp(
        static_cast<int>(std::lround(content_width * 0.27)),
        220,
        std::max(220, static_cast<int>(std::lround(content_width * 0.32))));
    const int xy_width = std::max(120, content_width - table_width - side_width - 2 * gap);
    const int side_height = std::max(120, (content_height - gap) / 2);

    const int x_table = margin;
    const int x_xy = x_table + table_width + gap;
    const int x_side = x_xy + xy_width + gap;
    return {
        cv::Rect(x_table, margin, table_width, content_height),
        cv::Rect(x_xy, margin, xy_width, content_height),
        cv::Rect(x_side, margin, side_width, side_height),
        cv::Rect(x_side, margin + side_height + gap, side_width, content_height - side_height - gap),
    };
}

std::pair<double, double> planePosition(const PoseVisualizationPose& pose, PosePlotPlane plane)
{
    switch (plane) {
        case PosePlotPlane::XY:
            return {pose.position.y(), pose.position.x()};
        case PosePlotPlane::XZ:
            return {pose.position.x(), pose.position.z()};
        case PosePlotPlane::YZ:
            return {pose.position.y(), pose.position.z()};
    }
    return {0.0, 0.0};
}

Eigen::Vector2d projectAxis(const Eigen::Vector3d& axis, PosePlotPlane plane)
{
    switch (plane) {
        case PosePlotPlane::XY:
            return {axis.y(), axis.x()};
        case PosePlotPlane::XZ:
            return {axis.x(), axis.z()};
        case PosePlotPlane::YZ:
            return {axis.y(), axis.z()};
    }
    return Eigen::Vector2d::Zero();
}

Eigen::Matrix2d planeCovariance(const Eigen::Matrix3d& covariance, PosePlotPlane plane)
{
    switch (plane) {
        case PosePlotPlane::XY:
            return (Eigen::Matrix2d() << covariance(1, 1), covariance(1, 0), covariance(0, 1), covariance(0, 0)).finished();
        case PosePlotPlane::XZ:
            return (Eigen::Matrix2d() << covariance(0, 0), covariance(0, 2), covariance(2, 0), covariance(2, 2)).finished();
        case PosePlotPlane::YZ:
            return (Eigen::Matrix2d() << covariance(1, 1), covariance(1, 2), covariance(2, 1), covariance(2, 2)).finished();
    }
    return Eigen::Matrix2d::Identity();
}

std::string coordinateLabel(double value)
{
    if (std::abs(value) < 1.0e-9) {
        value = 0.0;
    }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(std::abs(value) >= 100.0 ? 1 : 2) << value;
    return stream.str();
}

cv::Point cvPlotPixel(double x, double y, double range_x, double range_y, int width, int height)
{
    const double plot_width = std::max(1.0, static_cast<double>(width) - kCvPlotMarginLeft - kCvPlotMarginRight);
    const double plot_height = std::max(1.0, static_cast<double>(height) - kCvPlotMarginTop - kCvPlotMarginBottom);
    const double scale_x = (plot_width / 2.0) / (std::max(0.001, range_x) * 1.05);
    const double scale_y = (plot_height / 2.0) / (std::max(0.001, range_y) * 1.05);
    return {
        static_cast<int>(std::lround(kCvPlotMarginLeft + plot_width * 0.5 + x * scale_x)),
        static_cast<int>(std::lround(kCvPlotMarginTop + plot_height * 0.5 - y * scale_y)),
    };
}

cv::Mat ensurePlotImageSize(cv::Mat plot, int width, int height)
{
    if (plot.empty()) {
        return {};
    }
    if (plot.channels() == 1) {
        cv::cvtColor(plot, plot, cv::COLOR_GRAY2BGR);
    } else if (plot.channels() == 4) {
        cv::cvtColor(plot, plot, cv::COLOR_BGRA2BGR);
    } else if (plot.type() != CV_8UC3) {
        return {};
    }
    if (plot.cols != width || plot.rows != height) {
        cv::resize(plot, plot, {width, height}, 0.0, 0.0, cv::INTER_NEAREST);
    }
    return plot;
}

void drawCvPlotCovarianceEllipse(
    cv::Mat& plot,
    const PoseVisualizationPose& pose,
    PosePlotPlane plane,
    double range_x,
    double range_y)
{
    const Eigen::Matrix2d plane_covariance = planeCovariance(pose.position_covariance, plane);
    const Eigen::Matrix2d covariance = 0.5 * (plane_covariance + plane_covariance.transpose());
    if (!covariance.allFinite()) {
        return;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    if (solver.info() != Eigen::Success) {
        return;
    }
    const Eigen::Vector2d standard_deviations = solver.eigenvalues().cwiseMax(0.0).cwiseSqrt();
    if (!standard_deviations.allFinite() || standard_deviations[1] <= 0.0) {
        return;
    }

    const auto [x, y] = planePosition(pose, plane);
    const Eigen::Vector2d major = solver.eigenvectors().col(1) * standard_deviations[1] * kCovarianceScale;
    const Eigen::Vector2d minor = solver.eigenvectors().col(0) * standard_deviations[0] * kCovarianceScale;
    const cv::Point center = cvPlotPixel(x, y, range_x, range_y, plot.cols, plot.rows);
    const cv::Point major_end = cvPlotPixel(x + major.x(), y + major.y(), range_x, range_y, plot.cols, plot.rows);
    const cv::Point minor_end = cvPlotPixel(x + minor.x(), y + minor.y(), range_x, range_y, plot.cols, plot.rows);
    const cv::Point2d major_delta = major_end - center;
    const cv::Point2d minor_delta = minor_end - center;
    const int major_radius = static_cast<int>(std::lround(cv::norm(major_delta)));
    const int minor_radius = static_cast<int>(std::lround(cv::norm(minor_delta)));
    if (major_radius <= 0 || minor_radius <= 0) {
        return;
    }
    const double angle = std::atan2(major_delta.y, major_delta.x) * 180.0 / std::numbers::pi;
    cv::ellipse(plot, center, {major_radius, minor_radius}, angle, 0.0, 360.0, kCovarianceColor, 1, cv::LINE_AA);
}

void drawCvPlotAxes(
    cv::Mat& plot,
    const PoseVisualizationPose& pose,
    PosePlotPlane plane,
    double range_x,
    double range_y)
{
    struct ProjectedAxis {
        Eigen::Vector2d direction;
        std::size_t color_index = 0U;
        double visibility = 0.0;
    };

    std::array<ProjectedAxis, 3> axes;
    for (std::size_t index = 0; index < axes.size(); ++index) {
        axes[index].direction = projectAxis(pose.rotation.col(static_cast<Eigen::Index>(index)), plane);
        axes[index].color_index = index;
        axes[index].visibility = axes[index].direction.squaredNorm();
    }
    std::stable_sort(axes.begin(), axes.end(), [](const ProjectedAxis& lhs, const ProjectedAxis& rhs) {
        return lhs.visibility < rhs.visibility;
    });

    const auto [x, y] = planePosition(pose, plane);
    const cv::Point origin = cvPlotPixel(x, y, range_x, range_y, plot.cols, plot.rows);
    const double axis_length = std::max(0.2, kAxisLengthFactor * std::min(range_x, range_y));
    for (const ProjectedAxis& axis : axes) {
        const cv::Point endpoint = cvPlotPixel(
            x + axis.direction.x() * axis_length,
            y + axis.direction.y() * axis_length,
            range_x,
            range_y,
            plot.cols,
            plot.rows);
        cv::line(plot, origin, endpoint, kAxisColors[axis.color_index], 2, cv::LINE_AA);
    }
}

cv::Mat renderCvPlot(
    const std::vector<PoseVisualizationPose>& poses,
    PosePlotPlane plane,
    const std::string& title,
    const std::string& horizontal_label,
    const std::string& vertical_label,
    double range_x,
    double range_y,
    int width,
    int height)
{
    auto axes = CvPlot::makePlotAxes();
    axes.setMargins(
        static_cast<int>(kCvPlotMarginLeft),
        static_cast<int>(kCvPlotMarginRight),
        static_cast<int>(kCvPlotMarginTop),
        static_cast<int>(kCvPlotMarginBottom));
    axes.create<CvPlot::Series>(std::vector<double>{-range_x, range_x}, std::vector<double>{0.0, 0.0}, "-k");
    axes.create<CvPlot::Series>(std::vector<double>{0.0, 0.0}, std::vector<double>{-range_y, range_y}, "-k");
    cv::Mat plot = ensurePlotImageSize(axes.render(std::max(1, width), std::max(1, height)), width, height);
    if (plot.empty()) {
        return {};
    }

    cv::putText(plot, title, {static_cast<int>(kCvPlotMarginLeft) + 22, static_cast<int>(kCvPlotMarginTop) - 6}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    cv::putText(plot, horizontal_label, {plot.cols - static_cast<int>(kCvPlotMarginRight) - 45, plot.rows - static_cast<int>(kCvPlotMarginBottom) / 2 + 5}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    cv::putText(plot, vertical_label, {6, static_cast<int>(kCvPlotMarginTop) - 6}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);

    for (const PoseVisualizationPose& pose : poses) {
        const auto [x, y] = planePosition(pose, plane);
        const cv::Point position = cvPlotPixel(x, y, range_x, range_y, plot.cols, plot.rows);
        if (position.x < 0 || position.y < 0 || position.x >= plot.cols || position.y >= plot.rows) {
            continue;
        }
        drawCvPlotCovarianceEllipse(plot, pose, plane, range_x, range_y);
        drawCvPlotAxes(plot, pose, plane, range_x, range_y);
        cv::putText(plot, "ID:" + std::to_string(pose.id), position + cv::Point(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }
    return plot;
}

std::array<double, 3> rollPitchYawDegrees(const Eigen::Matrix3d& rotation)
{
    const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    constexpr double radians_to_degrees = 180.0 / std::numbers::pi;
    return {roll * radians_to_degrees, pitch * radians_to_degrees, yaw * radians_to_degrees};
}

void drawPoseTable(cv::Mat& canvas, const cv::Rect& available_table, std::vector<PoseVisualizationPose> poses)
{
    std::sort(poses.begin(), poses.end(), [](const PoseVisualizationPose& lhs, const PoseVisualizationPose& rhs) {
        return lhs.id < rhs.id;
    });

    constexpr std::array<const char*, 7> headers {"ID", "x [m]", "y [m]", "z [m]", "r [deg]", "p [deg]", "y [deg]"};
    constexpr std::array<double, 7> column_weights {0.55, 1.0, 1.0, 1.0, 1.1, 1.1, 1.1};
    const double total_weight = std::accumulate(column_weights.begin(), column_weights.end(), 0.0);
    const int padding = std::max(6, available_table.width / 75);
    const int header_height = std::clamp(available_table.width / 15, 28, 34);
    const int available_rows = std::max(1, std::min(24, static_cast<int>(poses.size())));
    const int row_height = std::clamp(available_table.width / 14, 24, 32);
    const std::size_t display_count = std::min<std::size_t>(poses.size(), 24U);
    const int summary_height = poses.size() > display_count ? row_height : 0;
    const int desired_height = header_height + static_cast<int>(display_count) * row_height + summary_height + 2;
    const cv::Rect table {
        available_table.x,
        available_table.y,
        available_table.width,
        std::min(available_table.height, desired_height),
    };
    const double font_scale = std::clamp(row_height / 72.0, 0.28, 0.42);
    const int text_thickness = row_height >= 32 ? 1 : 1;

    cv::rectangle(canvas, table, kPanelColor, cv::FILLED);
    cv::rectangle(canvas, table, kBorderColor, 1, cv::LINE_AA);

    std::array<int, 8> columns {};
    columns[0] = table.x + padding;
    double cursor = static_cast<double>(columns[0]);
    for (std::size_t i = 0; i < column_weights.size(); ++i) {
        cursor += (table.width - 2.0 * padding) * column_weights[i] / total_weight;
        columns[i + 1] = static_cast<int>(std::lround(cursor));
    }
    columns.back() = table.br().x - padding;

    cv::rectangle(canvas, {table.x + 1, table.y + 1, table.width - 2, header_height}, cv::Scalar(232, 238, 242), cv::FILLED);
    for (std::size_t i = 1; i + 1 < columns.size(); ++i) {
        cv::line(canvas, {columns[i], table.y}, {columns[i], table.br().y}, kBorderColor, 1, cv::LINE_AA);
    }
    cv::line(canvas, {table.x, table.y + header_height}, {table.br().x, table.y + header_height}, kBorderColor, 1, cv::LINE_AA);

    for (std::size_t column = 0; column < headers.size(); ++column) {
        const cv::Size text_size = cv::getTextSize(headers[column], cv::FONT_HERSHEY_SIMPLEX, font_scale, text_thickness, nullptr);
        const int text_x = columns[column] + std::max(2, (columns[column + 1] - columns[column] - text_size.width) / 2);
        const int text_y = table.y + (header_height + text_size.height) / 2;
        cv::putText(canvas, headers[column], {text_x, text_y}, cv::FONT_HERSHEY_SIMPLEX, font_scale, kTextColor, text_thickness, cv::LINE_AA);
    }

    for (std::size_t row = 0; row < display_count; ++row) {
        const int top = table.y + header_height + static_cast<int>(row) * row_height;
        const int bottom = std::min(table.br().y, top + row_height);
        if (row % 2U == 1U) {
            cv::rectangle(canvas, {table.x + 1, top, table.width - 2, std::max(1, bottom - top)}, cv::Scalar(250, 250, 250), cv::FILLED);
        }
        cv::line(canvas, {table.x, bottom}, {table.br().x, bottom}, kGridColor, 1, cv::LINE_AA);

        const auto rpy = rollPitchYawDegrees(poses[row].rotation);
        const std::array<std::string, 7> values {
            std::to_string(poses[row].id),
            coordinateLabel(poses[row].position.x()), coordinateLabel(poses[row].position.y()), coordinateLabel(poses[row].position.z()),
            coordinateLabel(rpy[0]), coordinateLabel(rpy[1]), coordinateLabel(rpy[2]),
        };
        for (std::size_t column = 0; column < values.size(); ++column) {
            const cv::Size text_size = cv::getTextSize(values[column], cv::FONT_HERSHEY_SIMPLEX, font_scale, text_thickness, nullptr);
            const int text_x = columns[column] + std::max(2, (columns[column + 1] - columns[column] - text_size.width) / 2);
            const int text_y = top + (row_height + text_size.height) / 2;
            cv::putText(canvas, values[column], {text_x, text_y}, cv::FONT_HERSHEY_SIMPLEX, font_scale, kTextColor, text_thickness, cv::LINE_AA);
        }
    }

    if (poses.size() > display_count) {
        const std::string summary = "+" + std::to_string(poses.size() - display_count) + " more targets";
        cv::putText(canvas, summary, {table.x + padding, table.br().y - padding}, cv::FONT_HERSHEY_SIMPLEX, font_scale, kTextColor, 1, cv::LINE_AA);
    }
}

std::mutex& guiMutex()
{
    static std::mutex mutex;
    return mutex;
}

} // namespace

VisualizationWorker::VisualizationWorker(std::chrono::milliseconds minimum_interval)
    : minimum_interval_(std::max(std::chrono::milliseconds::zero(), minimum_interval))
    , thread_(&VisualizationWorker::run, this)
{
}

VisualizationWorker::~VisualizationWorker()
{
    stop();
}

void VisualizationWorker::submit(Task task)
{
    if (!task) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) {
            return;
        }
        pending_task_ = std::move(task);
    }
    cv_.notify_one();
}

void VisualizationWorker::stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) {
            return;
        }
        running_ = false;
        pending_task_ = {};
    }
    cv_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
}

void VisualizationWorker::run()
{
    std::optional<std::chrono::steady_clock::time_point> last_dispatch;
    std::unique_lock<std::mutex> lock(mutex_);
    while (running_) {
        cv_.wait(lock, [this] { return !running_ || static_cast<bool>(pending_task_); });
        if (!running_) {
            break;
        }

        if (last_dispatch && minimum_interval_ > std::chrono::milliseconds::zero()) {
            const auto deadline = *last_dispatch + minimum_interval_;
            cv_.wait_until(lock, deadline, [this] { return !running_; });
            if (!running_) {
                break;
            }
        }

        Task task = std::move(pending_task_);
        pending_task_ = {};
        lock.unlock();
        try {
            task();
        } catch (...) {
            // Rendering is optional and must never terminate a processing node.
        }
        last_dispatch = std::chrono::steady_clock::now();
        lock.lock();
    }
}

cv::Mat renderDetectionOverlay(const cv::Mat& image, const DetectionOverlay& overlay)
{
    cv::Mat visualization = toBgr(image);
    if (visualization.empty()) {
        return visualization;
    }
    for (const cv::Point2d& point : overlay.detected_points) {
        cv::circle(visualization, point, 5, cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
    }
    for (const cv::Point2d& point : overlay.sun_points) {
        cv::circle(visualization, point, 3, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }
    return visualization;
}

cv::Scalar trackingSignalColor(int signal_id)
{
    if (signal_id < 0) {
        return cv::Scalar(160, 160, 160);
    }
    const int hue = (signal_id * 37 + 17) % 255;
    return cv::Scalar(30 + hue % 226, 40 + hue / 2 % 215, 220 - hue / 3);
}

cv::Mat renderTrackingOverlay(const cv::Mat& image, const cv::Size& fallback_size, const std::vector<TrackingOverlayMarker>& markers)
{
    cv::Mat frame = toBgr(image, fallback_size);
    if (frame.empty()) {
        return frame;
    }
    constexpr int minimum_rows = 100;
    constexpr int minimum_columns = 100;
    if (frame.rows < minimum_rows || frame.cols < minimum_columns) {
        cv::resize(frame, frame, {std::max(frame.cols, minimum_columns), std::max(frame.rows, minimum_rows)});
    }

    for (const TrackingOverlayMarker& marker : markers) {
        const cv::Point center(std::lround(marker.position.x), std::lround(marker.position.y));
        const cv::Scalar color = trackingSignalColor(marker.signal_id);
        cv::circle(frame, center, 4, color, 2, cv::LINE_AA);
        cv::putText(frame, std::to_string(marker.signal_id), center + cv::Point(-6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv::LINE_AA);

        if (marker.has_prediction) {
            const cv::Point predicted(std::lround(marker.predicted_position.x), std::lround(marker.predicted_position.y));
            cv::circle(frame, predicted, 3, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
            if (marker.confidence_half_extent.x >= 0.0 && marker.confidence_half_extent.y >= 0.0) {
                const cv::Point half_extent(
                    static_cast<int>(std::ceil(marker.confidence_half_extent.x)),
                    static_cast<int>(std::ceil(marker.confidence_half_extent.y)));
                cv::rectangle(frame, predicted - half_extent, predicted + half_extent, cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
            }
        }
        if (marker.virtual_point) {
            cv::circle(frame, center, 2, cv::Scalar(80, 80, 80), 2, cv::LINE_AA);
        }
    }
    return frame;
}

cv::Mat PoseOverviewRenderer::render(const std::vector<PoseVisualizationPose>& poses, cv::Size canvas_size)
{
    canvas_size.width = std::max(800, canvas_size.width);
    canvas_size.height = std::max(500, canvas_size.height);
    cv::Mat canvas(canvas_size, CV_8UC3, kCanvasColor);
    const PoseLayout layout = poseLayout(canvas_size);

    const auto estimate_inner = [](const cv::Rect& outer) {
        return cv::Size(
            std::max(1, static_cast<int>(std::lround(outer.width - kCvPlotMarginLeft - kCvPlotMarginRight))),
            std::max(1, static_cast<int>(std::lround(outer.height - kCvPlotMarginTop - kCvPlotMarginBottom))));
    };
    const std::array<std::pair<cv::Rect, PosePlotPlane>, 3> views {{
        {layout.xy, PosePlotPlane::XY},
        {layout.xz, PosePlotPlane::XZ},
        {layout.yz, PosePlotPlane::YZ},
    }};

    double required_metres_per_pixel = 0.01;
    for (const auto& [outer, plane] : views) {
        const cv::Size inner = estimate_inner(outer);
        for (const PoseVisualizationPose& pose : poses) {
            const auto [x, y] = planePosition(pose, plane);
            required_metres_per_pixel = std::max(required_metres_per_pixel, 2.0 * std::abs(x) / static_cast<double>(inner.width));
            required_metres_per_pixel = std::max(required_metres_per_pixel, 2.0 * std::abs(y) / static_cast<double>(inner.height));
        }
    }
    required_metres_per_pixel *= kRangePadding;
    if (metres_per_pixel_ <= 0.0 || required_metres_per_pixel > metres_per_pixel_) {
        metres_per_pixel_ = required_metres_per_pixel;
    } else if (required_metres_per_pixel < metres_per_pixel_ * 0.70) {
        metres_per_pixel_ = required_metres_per_pixel;
    }

    const auto plot_ranges = [this, &estimate_inner](const cv::Rect& outer) {
        const cv::Size inner = estimate_inner(outer);
        return std::pair<double, double> {
            std::max(0.5, inner.width * metres_per_pixel_ * 0.5),
            std::max(0.5, inner.height * metres_per_pixel_ * 0.5),
        };
    };
    const auto [xy_range_x, xy_range_y] = plot_ranges(layout.xy);
    const auto [xz_range_x, xz_range_y] = plot_ranges(layout.xz);
    const auto [yz_range_x, yz_range_y] = plot_ranges(layout.yz);
    const cv::Mat xy = renderCvPlot(poses, PosePlotPlane::XY, "XY top-view", "y [m]", "x [m]", xy_range_x, xy_range_y, layout.xy.width, layout.xy.height);
    const cv::Mat xz = renderCvPlot(poses, PosePlotPlane::XZ, "XZ side-view", "x [m]", "z [m]", xz_range_x, xz_range_y, layout.xz.width, layout.xz.height);
    const cv::Mat yz = renderCvPlot(poses, PosePlotPlane::YZ, "YZ side-view", "y [m]", "z [m]", yz_range_x, yz_range_y, layout.yz.width, layout.yz.height);

    if (!xy.empty()) {
        xy.copyTo(canvas(layout.xy));
    }
    if (!xz.empty()) {
        xz.copyTo(canvas(layout.xz));
    }
    if (!yz.empty()) {
        yz.copyTo(canvas(layout.yz));
    }
    drawPoseTable(canvas, layout.table, poses);
    return canvas;
}

void showFrame(const std::string& window_name, const cv::Mat& frame)
{
    if (frame.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(guiMutex());
    cv::imshow(window_name, frame);
    cv::waitKey(1);
}

} // namespace uvdar_core::app::visualization
