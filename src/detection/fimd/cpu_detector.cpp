#include "uvdar_core/detection/fimd/cpu_detector.hpp"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <opencv2/imgproc.hpp>

#include "uvdar_core/detection/fimd/generated_module.hpp"
#include "uvdar_core/detection/fimd/postprocess.hpp"

namespace uvdar_core::detection::fimd {

namespace {

    std::vector<unsigned char> applyMask(const cv::Mat& image, const std::vector<cv::Mat>& masks, int mask_id)
    {
        if (mask_id < 0 || mask_id >= static_cast<int>(masks.size())) {
            return std::vector<unsigned char>(image.data, image.data + image.total());
        }

        cv::Mat masked;
        cv::bitwise_and(image, masks[mask_id], masked);
        return std::vector<unsigned char>(masked.data, masked.data + masked.total());
    }

    std::uint64_t pointKey(const cv::Point2i& point)
    {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(point.x)) << 32U) | static_cast<std::uint32_t>(point.y);
    }

} // namespace

struct CpuDetector::Impl {
    explicit Impl(CpuDetectorConfig cfg)
        : config(std::move(cfg))
    {
    }

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

CpuDetector::CpuDetector(CpuDetectorConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

CpuDetector::~CpuDetector() = default;

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

bool CpuDetector::processImage(const cv::Mat& image, DetectorOutput& output, int mask_id)
{
    if (!impl_->initialized && !initDelayed(image)) {
        return false;
    }

    const std::vector<unsigned char> prepared = applyMask(image, impl_->config.masks, mask_id);
    std::unordered_set<std::uint64_t> marker_keys;
    std::unordered_set<std::uint64_t> sun_keys;
    output.detected_points.clear();
    output.sun_points.clear();

    for (auto& kernel : impl_->kernels) {
        std::vector<std::array<unsigned, 2>> raw_markers(kernel->get_max_markers_count());
        unsigned raw_markers_count = 0;

        std::vector<std::array<unsigned, 2>> raw_sun_points;
        unsigned raw_sun_points_count = 0;
        unsigned (*sun_points)[2] = nullptr;
        unsigned* sun_points_count = nullptr;

        if (impl_->config.detect_sun_points) {
            raw_sun_points.resize(kernel->get_max_sun_points_count());
            sun_points      = reinterpret_cast<unsigned (*)[2]>(raw_sun_points.data());
            sun_points_count = &raw_sun_points_count;
        }

        kernel->detectRaw(
            prepared.data(),
            reinterpret_cast<unsigned (*)[2]>(raw_markers.data()),
            &raw_markers_count,
            sun_points,
            sun_points_count,
            true);

        for (unsigned index = 0; index < raw_markers_count; ++index) {
            const cv::Point2i point(static_cast<int>(raw_markers[index][0]), static_cast<int>(raw_markers[index][1]));
            if (marker_keys.insert(pointKey(point)).second) {
                output.detected_points.push_back(point);
            }
        }
        if (impl_->config.detect_sun_points) {
            for (unsigned index = 0; index < raw_sun_points_count; ++index) {
                const cv::Point2i point(static_cast<int>(raw_sun_points[index][0]), static_cast<int>(raw_sun_points[index][1]));
                if (sun_keys.insert(pointKey(point)).second) {
                    output.sun_points.push_back(point);
                }
            }
        }
    }

    if (impl_->config.detect_sun_points) {
        filterMarkersNearSunPoints(output, impl_->config.min_sun_marker_distance);
    }

    return true;
}

bool CpuDetector::get_debug() const { return impl_->config.debug; }
void CpuDetector::set_debug(bool debug) { impl_->config.debug = debug; }
bool CpuDetector::get_detect_sun_points() const { return impl_->config.detect_sun_points; }
void CpuDetector::set_detect_sun_points(bool detect_sun_points)
{
    impl_->config.detect_sun_points = detect_sun_points;
    impl_->initialized              = false;
}
int CpuDetector::get_threshold() const { return impl_->config.threshold; }
void CpuDetector::set_threshold(int threshold)
{
    impl_->config.threshold = threshold;
    impl_->initialized      = false;
}
int CpuDetector::get_threshold_diff() const { return impl_->config.threshold_diff; }
void CpuDetector::set_threshold_diff(int threshold_diff)
{
    impl_->config.threshold_diff = threshold_diff;
    impl_->initialized           = false;
}
int CpuDetector::get_threshold_sun() const { return impl_->config.threshold_sun; }
void CpuDetector::set_threshold_sun(int threshold_sun)
{
    impl_->config.threshold_sun = threshold_sun;
    impl_->initialized         = false;
}
unsigned CpuDetector::get_max_markers_count() const { return impl_->config.max_markers_count; }
void CpuDetector::set_max_markers_count(unsigned max_markers_count)
{
    impl_->config.max_markers_count = max_markers_count;
    impl_->initialized              = false;
}
unsigned CpuDetector::get_max_sun_points_count() const { return impl_->config.max_sun_points_count; }
void CpuDetector::set_max_sun_points_count(unsigned max_sun_points_count)
{
    impl_->config.max_sun_points_count = max_sun_points_count;
    impl_->initialized                 = false;
}
const std::vector<unsigned>& CpuDetector::get_radii() const { return impl_->config.radii; }
void CpuDetector::set_radii(std::vector<unsigned> radii)
{
    impl_->config.radii = std::move(radii);
    impl_->initialized  = false;
}
const std::vector<cv::Mat>& CpuDetector::get_masks() const { return impl_->config.masks; }
void CpuDetector::set_masks(std::vector<cv::Mat> masks) { impl_->config.masks = std::move(masks); }

} // namespace uvdar_core::detection::fimd
