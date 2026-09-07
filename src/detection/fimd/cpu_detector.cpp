#include "uvdar_core/detection/fimd/cpu_detector.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "uvdar_core/detection/fimd/generated_module.hpp"
#include "uvdar_core/detection/fimd/postprocess.hpp"

namespace uvdar_core::detection::fimd {

namespace {

    /**
     * @brief Apply binary mask when requested and return raw byte vector.
     */
    std::vector<unsigned char> applyMask(const cv::Mat& image, const std::vector<cv::Mat>& masks, int mask_id)
    {
        if (mask_id < 0 || mask_id >= static_cast<int>(masks.size())) {
            return std::vector<unsigned char>(image.data, image.data + image.total());
        }

        cv::Mat masked;
        cv::bitwise_and(image, masks[mask_id], masked);
        return std::vector<unsigned char>(masked.data, masked.data + masked.total());
    }

} // namespace

struct CpuDetector::Impl {
    /**
     * @brief Store detector runtime state and kernels.
     */
    explicit Impl(CpuDetectorConfig cfg)
        : config(std::move(cfg))
    {
    }

    /**
     * @brief Get an existing radius module from cache or construct a new one.
     */
    std::shared_ptr<const RuntimeFimdRadiusModule> getOrCreateModule(unsigned radius, unsigned width, unsigned height)
    {
        const auto module_key = RuntimeFimdRadiusModule::key(radius, width, height);
        const auto found      = modules.find(module_key);
        if (found != modules.end()) {
            return found->second;
        }

        auto module = std::make_shared<RuntimeFimdRadiusModule>(radius, width, height);
        modules.emplace(module_key, module);
        return module;
    }

    /**
     * @brief Rebuild generated kernels for current image size.
     */
    void rebuildKernels(unsigned width, unsigned height)
    {
        kernels.clear();
        const auto sun_points_count = config.detect_sun_points ? config.max_sun_points_count : 0;
        for (unsigned radius : config.radii) {
            kernels.push_back(std::make_unique<GeneratedFimdCpuKernel>(
                getOrCreateModule(radius, width, height),
                static_cast<unsigned char>(config.threshold),
                static_cast<unsigned char>(config.threshold_diff),
                static_cast<unsigned char>(config.threshold_sun),
                std::array<unsigned char, 2> { 0xFF, 0x00 },
                config.max_markers_count,
                sun_points_count,
                config.detect_sun_points));
        }
        initialized  = true;
        image_width  = width;
        image_height = height;
    }

    CpuDetectorConfig config;
    bool initialized      = false;
    unsigned image_width  = 0;
    unsigned image_height = 0;
    std::unordered_map<std::uint64_t, std::shared_ptr<const RuntimeFimdRadiusModule>> modules;
    std::vector<std::unique_ptr<GeneratedFimdCpuKernel>> kernels;
};

/**
 * @brief Construct CPU detector and move user configuration into implementation.
 */
CpuDetector::CpuDetector(CpuDetectorConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

/**
 * @brief Defaulted destructor.
 */
CpuDetector::~CpuDetector() = default;

/**
 * @brief Initialize detector kernels lazily from first frame size.
 */
bool CpuDetector::initDelayed(const cv::Mat& image)
{
    if (image.type() != CV_8UC1) {
        return false;
    }

    if (!impl_->initialized || impl_->image_width != static_cast<unsigned>(image.cols) || impl_->image_height != static_cast<unsigned>(image.rows)) {
        impl_->rebuildKernels(static_cast<unsigned>(image.cols), static_cast<unsigned>(image.rows));
    }
    return true;
}

/**
 * @brief Process one image with optional mask and deduplicate points.
 */
bool CpuDetector::processImage(const cv::Mat& image, DetectorOutput& output, int mask_id)
{
    if (!impl_->initialized && !initDelayed(image)) {
        return false;
    }

    const std::vector<unsigned char> prepared = applyMask(image, impl_->config.masks, mask_id);
    const auto image_width = static_cast<std::uint32_t>(image.cols);
    std::vector<WeightedPoint> raw_marker_points;
    std::vector<WeightedPoint> raw_sun_points;
    output.detected_points.clear();
    output.sun_points.clear();
    output.detected_points.reserve(100);
    output.sun_points.reserve(100);

    for (auto& kernel : impl_->kernels) {
        std::vector<std::uint32_t> raw_markers(kernel->get_max_markers_count());
        unsigned raw_markers_count = 0;

        std::vector<std::uint32_t> raw_sun_points_raw;
        unsigned raw_sun_points_count = 0;
        std::uint32_t* sun_points = nullptr;
        unsigned* sun_points_count = nullptr;

        if (impl_->config.detect_sun_points) {
            raw_sun_points_raw.resize(kernel->get_max_sun_points_count());
            sun_points      = raw_sun_points_raw.data();
            sun_points_count = &raw_sun_points_count;
        }

        kernel->detectRaw(
            prepared.data(),
            raw_markers.data(),
            &raw_markers_count,
            sun_points,
            sun_points_count,
            true);

        for (unsigned index = 0; index < raw_markers_count; ++index) {
            const std::uint32_t raw_marker = raw_markers[index];
            const std::uint32_t linear_pos = raw_marker >> 8u;
            raw_marker_points.push_back(WeightedPoint {
                cv::Point2f(
                    static_cast<float>(linear_pos % image_width),
                    static_cast<float>(linear_pos / image_width)),
                static_cast<float>(raw_marker & 0xFFu),
            });
        }
        if (impl_->config.detect_sun_points) {
            for (unsigned index = 0; index < raw_sun_points_count; ++index) {
                const std::uint32_t raw_sun_point = raw_sun_points_raw[index];
                const std::uint32_t linear_pos   = raw_sun_point >> 8u;
                raw_sun_points.push_back(WeightedPoint {
                    cv::Point2f(static_cast<float>(linear_pos % image_width), static_cast<float>(linear_pos / image_width)),
                    static_cast<float>(raw_sun_point & 0xFFu),
                });
            }
        }
    }

    if (impl_->config.detect_sun_points) {
        filterRawMarkersNearSunPoints(
            raw_marker_points,
            raw_sun_points,
            impl_->config.min_sun_marker_distance,
            impl_->image_width,
            impl_->image_height);
    }

    output.detected_points = collapseRawPoints(raw_marker_points, 5);
    if (impl_->config.detect_sun_points) {
        output.sun_points.reserve(raw_sun_points.size());
        for (const auto& raw_sun_point : raw_sun_points) {
            output.sun_points.push_back(uvdar_core::detection::DetectorPoint {
                raw_sun_point.point,
                1.0F / 12.0F,
                0.0F,
                0.0F,
                1.0F / 12.0F,
                1,
            });
        }
    }

    return true;
}

/**
 * @brief Whether debug mode is enabled.
 */
bool CpuDetector::get_debug() const { return impl_->config.debug; }
/**
 * @brief Enable or disable debug mode.
 */
void CpuDetector::set_debug(bool debug) { impl_->config.debug = debug; }
/**
 * @brief Whether sun-point detection is enabled.
 */
bool CpuDetector::get_detect_sun_points() const { return impl_->config.detect_sun_points; }
/**
 * @brief Enable or disable sun-point detection.
 */
void CpuDetector::set_detect_sun_points(bool detect_sun_points)
{
    impl_->config.detect_sun_points = detect_sun_points;
    impl_->initialized              = false;
}
/**
 * @brief Current marker threshold.
 */
int CpuDetector::get_threshold() const { return impl_->config.threshold; }
/**
 * @brief Update marker threshold and reinitialize kernels.
 */
void CpuDetector::set_threshold(int threshold)
{
    impl_->config.threshold = threshold;
    impl_->initialized      = false;
}
/**
 * @brief Current marker-to-boundary threshold difference.
 */
int CpuDetector::get_threshold_diff() const { return impl_->config.threshold_diff; }
/**
 * @brief Update threshold difference and reinitialize kernels.
 */
void CpuDetector::set_threshold_diff(int threshold_diff)
{
    impl_->config.threshold_diff = threshold_diff;
    impl_->initialized           = false;
}
/**
 * @brief Current sun-only threshold.
 */
int CpuDetector::get_threshold_sun() const { return impl_->config.threshold_sun; }
/**
 * @brief Update sun threshold and reinitialize kernels.
 */
void CpuDetector::set_threshold_sun(int threshold_sun)
{
    impl_->config.threshold_sun = threshold_sun;
    impl_->initialized         = false;
}
/**
 * @brief Current maximum markers.
 */
unsigned CpuDetector::get_max_markers_count() const { return impl_->config.max_markers_count; }
/**
 * @brief Update maximum markers and reinitialize kernels.
 */
void CpuDetector::set_max_markers_count(unsigned max_markers_count)
{
    impl_->config.max_markers_count = max_markers_count;
    impl_->initialized              = false;
}
/**
 * @brief Current maximum sun points.
 */
unsigned CpuDetector::get_max_sun_points_count() const { return impl_->config.max_sun_points_count; }
/**
 * @brief Update maximum sun points and reinitialize kernels.
 */
void CpuDetector::set_max_sun_points_count(unsigned max_sun_points_count)
{
    impl_->config.max_sun_points_count = max_sun_points_count;
    impl_->initialized                 = false;
}
/**
 * @brief Configured radii list.
 */
const std::vector<unsigned>& CpuDetector::get_radii() const { return impl_->config.radii; }
/**
 * @brief Replace radii and rebuild kernels.
 */
void CpuDetector::set_radii(std::vector<unsigned> radii)
{
    impl_->config.radii = std::move(radii);
    impl_->initialized  = false;
}
/**
 * @brief Read configured masks.
 */
const std::vector<cv::Mat>& CpuDetector::get_masks() const { return impl_->config.masks; }
/**
 * @brief Replace active mask list.
 */
void CpuDetector::set_masks(std::vector<cv::Mat> masks) { impl_->config.masks = std::move(masks); }

} // namespace uvdar_core::detection::fimd
