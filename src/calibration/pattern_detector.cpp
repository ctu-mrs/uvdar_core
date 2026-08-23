#include "uvdar_core/calibration/pattern_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "uvdar_core/calibration/grid_extractor.hpp"
#include "uvdar_core/detection/fimd/cpu_detector.hpp"

namespace uvdar_core::calibration {

namespace {

std::vector<cv::Point3f> makeObjectPoints(
    const PatternDetectorOptions& options,
    const bool column_major_led_order)
{
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<std::size_t>(options.rows * options.columns));
    if (column_major_led_order) {
        // LED image points are column-major and use row/column as world X/Y.
        for (int column = 0; column < options.columns; ++column) {
            for (int row = 0; row < options.rows; ++row) {
                points.emplace_back(
                    static_cast<float>(row * options.spacing),
                    static_cast<float>(column * options.spacing),
                    0.0F);
            }
        }
        return points;
    }
    for (int row = 0; row < options.rows; ++row) {
        for (int column = 0; column < options.columns; ++column) {
            points.emplace_back(
                static_cast<float>(column * options.spacing),
                static_cast<float>(row * options.spacing),
                0.0F);
        }
    }
    return points;
}

cv::Mat ensureGrayscale(const cv::Mat& image)
{
    if (image.empty()) {
        return {};
    }
    if (image.type() == CV_8UC1) {
        return image;
    }
    cv::Mat grayscale;
    if (image.channels() == 3) {
        cv::cvtColor(image, grayscale, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, grayscale, cv::COLOR_BGRA2GRAY);
    } else {
        image.convertTo(grayscale, CV_8UC1);
    }
    return grayscale;
}

struct CheckerboardSearchImage {
    cv::Mat pixels;
    double original_x_per_pixel = 1.0;
    double original_y_per_pixel = 1.0;
};

CheckerboardSearchImage makeCheckerboardSearchImage(
    const cv::Mat& grayscale,
    const int maximum_height)
{
    CheckerboardSearchImage search;
    if (grayscale.rows <= maximum_height) {
        search.pixels = grayscale;
        return search;
    }

    const double scale = static_cast<double>(maximum_height)
        / static_cast<double>(grayscale.rows);
    const cv::Size size(
        std::max(1, static_cast<int>(std::lround(grayscale.cols * scale))),
        maximum_height);
    cv::resize(grayscale, search.pixels, size, 0.0, 0.0, cv::INTER_LINEAR);
    search.original_x_per_pixel = static_cast<double>(grayscale.cols)
        / static_cast<double>(search.pixels.cols);
    search.original_y_per_pixel = static_cast<double>(grayscale.rows)
        / static_cast<double>(search.pixels.rows);
    return search;
}

bool findCompleteCheckerboardSb(
    const cv::Mat& image,
    const cv::Size& pattern_size,
    const int flags,
    std::vector<cv::Point2f>& corners)
{
    corners.clear();
    const bool found = cv::findChessboardCornersSB(
        image, pattern_size, corners, flags);
    return found && corners.size()
        == static_cast<std::size_t>(pattern_size.area());
}

bool findCompleteCheckerboardClassic(
    const cv::Mat& image,
    const cv::Size& pattern_size,
    std::vector<cv::Point2f>& corners)
{
    corners.clear();
    const bool found = cv::findChessboardCorners(
        image,
        pattern_size,
        corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    return found && corners.size()
        == static_cast<std::size_t>(pattern_size.area());
}

void restoreAndRefineCheckerboard(
    const cv::Mat& grayscale,
    const CheckerboardSearchImage& search,
    std::vector<cv::Point2f>& corners)
{
    for (cv::Point2f& corner : corners) {
        corner.x = static_cast<float>(
            corner.x * search.original_x_per_pixel);
        corner.y = static_cast<float>(
            corner.y * search.original_y_per_pixel);
    }

    // The ordered detector already provides subpixel coordinates. Refinement
    // against the sensor-resolution image recovers precision lost by resizing.
    cv::cornerSubPix(
        grayscale,
        corners,
        cv::Size(5, 5),
        cv::Size(-1, -1),
        cv::TermCriteria(
            cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
            40,
            1.0e-3));
}

bool detectCheckerboard(
    const cv::Mat& grayscale,
    const cv::Size& pattern_size,
    const int maximum_height,
    std::vector<cv::Point2f>& corners,
    std::string& detail)
{
    const CheckerboardSearchImage search = makeCheckerboardSearchImage(
        grayscale, maximum_height);
    const int thorough_flags = cv::CALIB_CB_EXHAUSTIVE
        | cv::CALIB_CB_ACCURACY | cv::CALIB_CB_NORMALIZE_IMAGE;

    // Uneven illumination and dark fisheye borders benefit from local
    // thresholding. This bounded-resolution sweep is the normal wide-angle
    // path and exits immediately when it obtains a complete ordered grid.
    constexpr std::array<int, 4> block_sizes {19, 25, 31, 37};
    constexpr std::array<int, 5> threshold_biases {-10, -5, 0, 5, 10};
    cv::Mat thresholded;
    for (const int block_size : block_sizes) {
        if (block_size >= std::min(search.pixels.rows, search.pixels.cols)) {
            continue;
        }
        for (const int bias : threshold_biases) {
            cv::adaptiveThreshold(
                search.pixels,
                thresholded,
                255,
                cv::ADAPTIVE_THRESH_MEAN_C,
                cv::THRESH_BINARY,
                block_size,
                bias);
            if (findCompleteCheckerboardSb(
                    thresholded,
                    pattern_size,
                    cv::CALIB_CB_EXHAUSTIVE,
                    corners)
                || findCompleteCheckerboardSb(
                    thresholded, pattern_size, 0, corners)) {
                restoreAndRefineCheckerboard(grayscale, search, corners);
                std::ostringstream message;
                message << "Checkerboard ordered with local threshold "
                        << block_size << '/' << bias;
                detail = message.str();
                return true;
            }
        }
    }

    if (findCompleteCheckerboardSb(
            search.pixels, pattern_size, thorough_flags, corners)) {
        restoreAndRefineCheckerboard(grayscale, search, corners);
        detail = "Checkerboard ordered";
        return true;
    }

    // Do not use CALIB_CB_FAST_CHECK here. Its inexpensive geometric test
    // assumes a near-perspective grid and rejects valid wide-angle views.
    if (findCompleteCheckerboardClassic(
            search.pixels, pattern_size, corners)) {
        restoreAndRefineCheckerboard(grayscale, search, corners);
        detail = "Checkerboard ordered with adaptive search";
        return true;
    }

    // Retain full sensor resolution as a final path when resizing erased a
    // distant target's smallest squares.
    if (search.pixels.size() != grayscale.size()
        && findCompleteCheckerboardSb(
            grayscale, pattern_size, thorough_flags, corners)) {
        CheckerboardSearchImage full_resolution;
        full_resolution.pixels = grayscale;
        restoreAndRefineCheckerboard(
            grayscale, full_resolution, corners);
        detail = "Checkerboard ordered at sensor resolution";
        return true;
    }

    std::ostringstream message;
    message << "No " << pattern_size.width << 'x' << pattern_size.height
            << " inner-corner checkerboard found";
    detail = message.str();
    corners.clear();
    return false;
}

} // namespace

struct CalibrationPatternDetector::Impl {
    explicit Impl(PatternDetectorOptions detector_options)
        : options(std::move(detector_options))
    {
        if (options.rows < 2 || options.columns < 2
            || options.spacing <= 0.0
            || options.checkerboard_max_detection_height < 64) {
            throw std::invalid_argument(
                "Calibration pattern rows/columns must be >=2, spacing must "
                "be positive, and checkerboard detection height must be >=64.");
        }
        object_points = makeObjectPoints(
            options, options.pattern == CalibrationPatternType::LedGrid);
        if (options.pattern == CalibrationPatternType::LedGrid) {
            fimd = std::make_unique<uvdar_core::detection::fimd::CpuDetector>(
                uvdar_core::detection::fimd::CpuDetectorConfig {
                    false,
                    false,
                    options.fimd_threshold,
                    options.fimd_threshold_diff,
                    255,
                    0,
                    options.fimd_max_markers,
                    0,
                    options.fimd_radii,
                    {},
                });
        }
    }

    PatternDetectorOptions options;
    std::vector<cv::Point3f> object_points;
    std::unique_ptr<uvdar_core::detection::fimd::CpuDetector> fimd;
};

CalibrationPatternType calibrationPatternFromString(const std::string& name)
{
    if (name == "checkerboard" || name == "chessboard") {
        return CalibrationPatternType::Checkerboard;
    }
    if (name == "led" || name == "led_grid" || name == "bright_points") {
        return CalibrationPatternType::LedGrid;
    }
    throw std::invalid_argument(
        "Unsupported calibration pattern '" + name + "'.");
}

std::string toString(const CalibrationPatternType pattern)
{
    return pattern == CalibrationPatternType::Checkerboard
        ? "checkerboard"
        : "led_grid";
}

CalibrationPatternDetector::CalibrationPatternDetector(
    PatternDetectorOptions options)
    : impl_(std::make_unique<Impl>(std::move(options)))
{
}

CalibrationPatternDetector::~CalibrationPatternDetector() = default;
CalibrationPatternDetector::CalibrationPatternDetector(
    CalibrationPatternDetector&&) noexcept = default;
CalibrationPatternDetector& CalibrationPatternDetector::operator=(
    CalibrationPatternDetector&&) noexcept = default;

PatternDetection CalibrationPatternDetector::detect(
    const cv::Mat& grayscale_image)
{
    PatternDetection result;
    const cv::Mat grayscale = ensureGrayscale(grayscale_image);
    if (grayscale.empty()) {
        result.detail = "Empty image";
        return result;
    }

    const cv::Size pattern_size(impl_->options.columns, impl_->options.rows);
    if (impl_->options.pattern == CalibrationPatternType::Checkerboard) {
        const bool found = detectCheckerboard(
            grayscale,
            pattern_size,
            impl_->options.checkerboard_max_detection_height,
            result.image_points,
            result.detail);
        result.candidates = result.image_points;
        result.success = found
            && result.image_points.size() == impl_->object_points.size();
    } else {
        uvdar_core::detection::DetectorOutput fimd_output;
        if (!impl_->fimd->processImage(grayscale, fimd_output)) {
            result.detail = "FIMD rejected the image format";
            return result;
        }
        result.candidates.reserve(fimd_output.detected_points.size());
        for (const auto& point : fimd_output.detected_points) {
            result.candidates.push_back(point.point);
        }
        if (result.candidates.size() > static_cast<std::size_t>(
                std::max(1, impl_->options.maximum_candidates))) {
            result.detail = "Too many FIMD candidates";
            return result;
        }
        if (result.candidates.size() < impl_->object_points.size()) {
            result.detail = "Not enough FIMD candidates";
            return result;
        }

        const GridExtractionResult extraction = extractGridFromFimdPoints(
            result.candidates,
            impl_->options.columns,
            impl_->options.rows,
            GridExtractorOptions {
                impl_->options.hull_maximum_concave_angle,
                impl_->options.hull_similar_angle,
            });
        result.image_points = extraction.image_points;
        result.hull_points = extraction.hull_points;
        result.success = extraction.success
            && result.image_points.size() == impl_->object_points.size();
        result.detail = extraction.detail;
    }

    if (result.success) {
        result.object_points = impl_->object_points;
    } else {
        result.image_points.clear();
    }
    return result;
}

const PatternDetectorOptions& CalibrationPatternDetector::options() const
{
    return impl_->options;
}

} // namespace uvdar_core::calibration
