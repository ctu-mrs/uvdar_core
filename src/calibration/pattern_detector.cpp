#include "uvdar_core/calibration/pattern_detector.hpp"

#include <algorithm>
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

} // namespace

struct CalibrationPatternDetector::Impl {
    explicit Impl(PatternDetectorOptions detector_options)
        : options(std::move(detector_options))
    {
        if (options.rows < 2 || options.columns < 2
            || options.spacing <= 0.0) {
            throw std::invalid_argument(
                "Calibration pattern rows/columns must be >=2 and spacing must be positive.");
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
        bool found = cv::findChessboardCornersSB(
            grayscale,
            pattern_size,
            result.image_points,
            cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY
                | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (!found) {
            found = cv::findChessboardCorners(
                grayscale,
                pattern_size,
                result.image_points,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE
                    | cv::CALIB_CB_FAST_CHECK);
            if (found) {
                cv::cornerSubPix(
                    grayscale,
                    result.image_points,
                    cv::Size(5, 5),
                    cv::Size(-1, -1),
                    cv::TermCriteria(
                        cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                        40,
                        1.0e-3));
            }
        }
        result.candidates = result.image_points;
        result.success = found
            && result.image_points.size() == impl_->object_points.size();
        result.detail = result.success
            ? "Checkerboard ordered"
            : "Checkerboard not found";
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
