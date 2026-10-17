#pragma once

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace uvdar_core::helpers::plotting {

/**
 * @brief Linear, origin-centred plot with ticks, grid, and positive-axis arrows.
 *
 * The supplied ranges are half extents in data coordinates. A 10% viewport
 * padding leaves room around the origin cross. Use pixel() for every overlay
 * so that it shares the grid's scaling and optional horizontal mirroring.
 * Rendering uses only OpenCV and never opens a GUI window.
 */
class Axes {
public:
    static constexpr int margin_left = 40;
    static constexpr int margin_right = 10;
    static constexpr int margin_top = 26;
    static constexpr int margin_bottom = 30;
    static constexpr double viewport_scale = 1.1;

    Axes(cv::Size size, double range_x, double range_y, bool mirror_horizontal = false)
        : size_(std::max(1, size.width), std::max(1, size.height))
        , range_x_(validRange(range_x))
        , range_y_(validRange(range_y))
        , mirror_horizontal_(mirror_horizontal)
    {
    }

    static cv::Size innerSize(cv::Size size)
    {
        return {std::max(1, size.width - margin_left - margin_right),
            std::max(1, size.height - margin_top - margin_bottom)};
    }

    cv::Point pixel(double x, double y) const
    {
        const cv::Size inner = innerSize(size_);
        if (mirror_horizontal_) {
            x = -x;
        }
        return {
            static_cast<int>(std::lround(margin_left + inner.width * (0.5 + 0.5 * (x / range_x_) / viewport_scale))),
            static_cast<int>(std::lround(margin_top + inner.height * (0.5 - 0.5 * (y / range_y_) / viewport_scale))),
        };
    }

    cv::Mat render() const
    {
        cv::Mat image(size_, CV_8UC3, cv::Scalar(255, 255, 255));
        if (size_.width <= margin_left + margin_right || size_.height <= margin_top + margin_bottom) {
            return image;
        }

        const cv::Size inner = innerSize(size_);
        const int right = margin_left + inner.width;
        const int bottom = margin_top + inner.height;
        const cv::Scalar black(0, 0, 0);
        const cv::Scalar grid(220, 220, 220);
        const double x_extent = range_x_ * viewport_scale;
        const double y_extent = range_y_ * viewport_scale;

        // Leave at least 30 pixels between horizontal labels, including their
        // signs and decimals. Vertical labels need only their text height.
        const double label_step = tickStep(x_extent, std::max(1, inner.width / 60));
        const int label_width = std::max(textSize(tickLabel(-x_extent, label_step)).width,
            textSize(tickLabel(x_extent, label_step)).width);
        const double x_step = tickStep(x_extent, std::max(1, inner.width / (label_width + 30)));
        const double y_step = tickStep(y_extent, std::max(1, inner.height / 30));

        forEachTick(x_extent, x_step, [&](double value) {
            const int x = pixel(value, 0.0).x;
            cv::line(image, {x, margin_top}, {x, bottom}, grid);
            cv::line(image, {x, bottom}, {x, bottom + 4}, black);
            const std::string label = tickLabel(value, x_step);
            cv::putText(image, label, {x - textSize(label).width / 2, bottom + 20},
                cv::FONT_HERSHEY_SIMPLEX, 0.4, black, 1, cv::LINE_AA);
        });
        forEachTick(y_extent, y_step, [&](double value) {
            const int y = pixel(0.0, value).y;
            cv::line(image, {margin_left, y}, {right, y}, grid);
            cv::line(image, {margin_left - 4, y}, {margin_left, y}, black);
            const std::string label = tickLabel(value, y_step);
            const cv::Size text_size = textSize(label);
            // Keep long labels (e.g. scientific notation) inside the margin.
            const double font_scale = 0.4 * std::min(1.0, static_cast<double>(margin_left - 8) / text_size.width);
            cv::putText(image, label, {std::max(1, margin_left - 8 - text_size.width), y + text_size.height / 2},
                cv::FONT_HERSHEY_SIMPLEX, font_scale, black, 1, cv::LINE_AA);
        });

        cv::rectangle(image, {margin_left, margin_top}, {right, bottom}, black);
        const cv::Point origin = pixel(0.0, 0.0);
        const cv::Point positive_x = pixel(range_x_, 0.0);
        const cv::Point positive_y = pixel(0.0, range_y_);
        cv::line(image, pixel(-range_x_, 0.0), positive_x, black, 1, cv::LINE_AA);
        cv::line(image, pixel(0.0, -range_y_), positive_y, black, 1, cv::LINE_AA);
        drawArrowhead(image, positive_x, origin);
        drawArrowhead(image, positive_y, origin);
        return image;
    }

private:
    static double validRange(double range)
    {
        return std::isfinite(range) ? std::clamp(range, 0.001, 1.0e100) : 0.001;
    }

    static cv::Size textSize(const std::string& text)
    {
        int baseline = 0;
        return cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
    }

    static double tickStep(double extent, int intervals)
    {
        const double raw_step = 2.0 * extent / intervals;
        const double magnitude = std::pow(10.0, std::floor(std::log10(raw_step)));
        const double fraction = raw_step / magnitude;
        const double rounded = fraction <= 1.0 ? 1.0 : fraction <= 2.0 ? 2.0 : fraction <= 5.0 ? 5.0 : 10.0;
        return rounded * magnitude;
    }

    static std::string tickLabel(double value, double step)
    {
        if (std::abs(value) < step * 1.0e-9) {
            return "0";
        }
        std::ostringstream stream;
        if (std::abs(value) >= 1.0e5 || step < 1.0e-3) {
            stream << std::setprecision(3) << value;
        } else {
            stream << std::fixed << std::setprecision(std::max(0, -static_cast<int>(std::floor(std::log10(step))))) << value;
        }
        return stream.str();
    }

    template <typename Draw>
    static void forEachTick(double extent, double step, Draw draw)
    {
        const int count = static_cast<int>(std::floor(extent / step));
        for (int index = -count; index <= count; ++index) {
            draw(index * step);
        }
    }

    static void drawArrowhead(cv::Mat& image, const cv::Point& tip, const cv::Point& origin)
    {
        const cv::Point2d to_origin = origin - tip;
        const double length = cv::norm(to_origin);
        if (length < 1.0) {
            return;
        }
        const cv::Point2d backwards = to_origin * (6.0 / length);
        const cv::Point2d sideways(-backwards.y * 0.55, backwards.x * 0.55);
        const auto rounded = [](const cv::Point2d& point) {
            return cv::Point(static_cast<int>(std::lround(point.x)), static_cast<int>(std::lround(point.y)));
        };
        const cv::Point2d endpoint(tip);
        cv::line(image, tip, rounded(endpoint + backwards + sideways), cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
        cv::line(image, tip, rounded(endpoint + backwards - sideways), cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }

    cv::Size size_;
    double range_x_;
    double range_y_;
    bool mirror_horizontal_;
};

} // namespace uvdar_core::helpers::plotting
