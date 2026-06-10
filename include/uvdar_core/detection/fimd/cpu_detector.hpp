#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "uvdar_core/detection/fimd/radius_module.hpp"
#include "uvdar_core/detection/i_detector.hpp"

namespace uvdar_core::detection::fimd {

/**
 * @brief Configuration for CPU-based FIMD detector.
 */
struct CpuDetectorConfig {
    bool debug                       = false;
    bool detect_sun_points           = true;
    int threshold                    = 120;
    int threshold_diff               = 60;
    int threshold_sun                = 240;
    unsigned min_sun_marker_distance = 20;
    unsigned max_markers_count       = 300;
    unsigned max_sun_points_count    = 10000;
    std::vector<unsigned> radii      = { 3, 5 };
    std::vector<cv::Mat> masks;
};

/**
 * @brief Runtime pure-CPU FIMD kernel implementation.
 */
class RuntimeFimdCpuKernel {
public:
    using Point2D = std::array<int, 2>;

    /**
     * @brief Construct kernel from radius and image geometry.
     */
    RuntimeFimdCpuKernel(
        unsigned radius,
        unsigned image_width,
        unsigned image_height,
        unsigned char threshold_center           = 120,
        unsigned char threshold_diff             = 60,
        unsigned char threshold_sun              = 240,
        std::array<unsigned char, 2> termination = { 0xFF, 0x00 },
        unsigned max_markers_count               = 0,
        unsigned max_sun_points_count            = 0,
        bool allocate_frame                      = true)
        : RuntimeFimdCpuKernel(
              std::make_shared<RuntimeFimdRadiusModule>(radius, image_width, image_height),
              threshold_center,
              threshold_diff,
              threshold_sun,
              termination,
              max_markers_count,
              max_sun_points_count,
              allocate_frame)
    {
    }

    /**
     * @brief Construct kernel from cached radius module.
     */
    RuntimeFimdCpuKernel(
        std::shared_ptr<const RuntimeFimdRadiusModule> module,
        unsigned char threshold_center           = 120,
        unsigned char threshold_diff             = 60,
        unsigned char threshold_sun              = 240,
        std::array<unsigned char, 2> termination = { 0xFF, 0x00 },
        unsigned max_markers_count               = 0,
        unsigned max_sun_points_count            = 0,
        bool allocate_frame                      = true)
        : module_(std::move(module))
        , threshold_center_(threshold_center)
        , threshold_diff_(threshold_diff)
        , threshold_sun_(threshold_sun)
        , termination_(termination)
        , configured_max_markers_count_(max_markers_count)
        , configured_max_sun_points_count_(max_sun_points_count)
    {
        if (!module_) {
            throw std::runtime_error("RuntimeFimdCpuKernel requires a valid radius module.");
        }
        if (allocate_frame) {
            frame_ = static_cast<unsigned char*>(std::malloc(frameBytes()));
        }
    }

    RuntimeFimdCpuKernel(const RuntimeFimdCpuKernel&)            = delete;
    RuntimeFimdCpuKernel& operator=(const RuntimeFimdCpuKernel&) = delete;

    /**
     * @brief Move constructor.
     */
    RuntimeFimdCpuKernel(RuntimeFimdCpuKernel&& other) noexcept
        : module_(std::move(other.module_))
        , threshold_center_(other.threshold_center_)
        , threshold_diff_(other.threshold_diff_)
        , threshold_sun_(other.threshold_sun_)
        , termination_(other.termination_)
        , configured_max_markers_count_(other.configured_max_markers_count_)
        , configured_max_sun_points_count_(other.configured_max_sun_points_count_)
        , marker_limit_(other.marker_limit_)
        , sun_limit_(other.sun_limit_)
        , frame_(other.frame_)
    {
        other.frame_ = nullptr;
    }

    /**
     * @brief Move assignment.
     */
    RuntimeFimdCpuKernel& operator=(RuntimeFimdCpuKernel&& other) noexcept
    {
        if (this != &other) {
            if (frame_ != nullptr) {
                std::free(frame_);
            }
            module_                          = std::move(other.module_);
            threshold_center_                = other.threshold_center_;
            threshold_diff_                  = other.threshold_diff_;
            threshold_sun_                   = other.threshold_sun_;
            termination_                     = other.termination_;
            configured_max_markers_count_    = other.configured_max_markers_count_;
            configured_max_sun_points_count_ = other.configured_max_sun_points_count_;
            marker_limit_                    = other.marker_limit_;
            sun_limit_                       = other.sun_limit_;
            frame_                           = other.frame_;
            other.frame_                     = nullptr;
        }
        return *this;
    }

    /**
     * @brief Release heap-allocated frame buffer.
     */
    ~RuntimeFimdCpuKernel()
    {
        if (frame_ != nullptr) {
            std::free(frame_);
        }
    }

    /**
     * @brief Detect markers and sun points from byte image.
     */
    unsigned detect(const unsigned char* image, std::vector<Point2D>& markers, std::vector<Point2D>& sun_points, bool make_copy = true)
    {
        std::vector<std::array<unsigned, 2>> raw_markers(marker_limit_ == std::numeric_limits<unsigned>::max() ? 0 : marker_limit_);
        std::vector<std::array<unsigned, 2>> raw_sun_points(sun_limit_ == std::numeric_limits<unsigned>::max() ? 0 : sun_limit_);
        unsigned markers_count    = 0;
        unsigned sun_points_count = 0;
        detectRaw(
            image,
            raw_markers.empty() ? nullptr : reinterpret_cast<unsigned (*)[2]>(raw_markers.data()),
            &markers_count,
            raw_sun_points.empty() ? nullptr : reinterpret_cast<unsigned (*)[2]>(raw_sun_points.data()),
            &sun_points_count,
            make_copy);
        markers.clear();
        sun_points.clear();
        markers.reserve(markers_count);
        sun_points.reserve(sun_points_count);
        for (unsigned index = 0; index < markers_count; ++index) {
            markers.push_back(Point2D { static_cast<int>(raw_markers[index][0]), static_cast<int>(raw_markers[index][1]) });
        }
        for (unsigned index = 0; index < sun_points_count; ++index) {
            sun_points.push_back(Point2D { static_cast<int>(raw_sun_points[index][0]), static_cast<int>(raw_sun_points[index][1]) });
        }
        return static_cast<unsigned>(markers_count + sun_points_count);
    }

    /**
     * @brief Detect markers/sun points with preallocated arrays.
     */
    unsigned detectRaw(
        const unsigned char* image,
        unsigned (*markers)[2],
        unsigned* markers_count,
        unsigned (*sun_points)[2],
        unsigned* sun_points_count,
        bool make_copy = true)
    {
        if (module_->image_width() < (2 * module_->radius() + 1) || module_->image_height() < (2 * module_->radius() + 1)) {
            if (markers_count != nullptr)
                *markers_count = 0;
            if (sun_points_count != nullptr)
                *sun_points_count = 0;
            return 0;
        }

        unsigned char* target_image    = nullptr;
        const std::size_t total_pixels = static_cast<std::size_t>(module_->image_width()) * static_cast<std::size_t>(module_->image_height());

        if (make_copy) {
            if (frame_ == nullptr) {
                frame_ = static_cast<unsigned char*>(std::malloc(frameBytes()));
            }
            std::memcpy(frame_, image, total_pixels * sizeof(unsigned char));
            target_image = frame_;
        } else {
            target_image = const_cast<unsigned char*>(image);
        }

        unsigned marker_count = 0;
        unsigned sun_count    = 0;
        if (markers_count != nullptr)
            *markers_count = 0;
        if (sun_points_count != nullptr)
            *sun_points_count = 0;

        writeTermination(target_image + total_pixels - termination_.size());
        const auto& boundary_offsets = module_->boundary_offsets();
        const auto& interior_offsets = module_->interior_offsets();
        const unsigned offset        = module_->offset();
        unsigned char* cursor        = target_image + offset;

        while (true) {
            if (hasTermination(cursor + offset - termination_.size())) {
                return static_cast<unsigned>((cursor - target_image) - static_cast<std::ptrdiff_t>(offset));
            }

            const unsigned char pixel_value = *(++cursor);
            if (pixel_value <= threshold_center_) {
                continue;
            }

            if ((pixel_value - *(cursor + boundary_offsets[0])) <= threshold_diff_) {
                if (pixel_value >= threshold_sun_) {
                    if (sunBoundaryRejected(cursor, pixel_value, boundary_offsets)) {
                        continue;
                    }

                    clearInterior(cursor, interior_offsets);
                    const auto point = coord1to2(static_cast<std::size_t>(cursor - target_image));
                    if (sun_points != nullptr) {
                        sun_points[sun_count][0] = static_cast<unsigned>(point[0]);
                        sun_points[sun_count][1] = static_cast<unsigned>(point[1]);
                    }
                    ++sun_count;
                    if (sun_points_count != nullptr)
                        *sun_points_count = sun_count;
                    if (sun_count == sun_limit_) {
                        writeTermination(cursor + offset - termination_.size());
                    }
                }
                continue;
            }

            if (markerBoundaryRejected(cursor, pixel_value, boundary_offsets)) {
                continue;
            }

            unsigned char peak        = 0;
            std::size_t peak_position = 0;
            scanInterior(cursor, target_image, peak, peak_position, interior_offsets);
            const auto point = coord1to2(peak_position);
            if (markers != nullptr) {
                markers[marker_count][0] = static_cast<unsigned>(point[0]);
                markers[marker_count][1] = static_cast<unsigned>(point[1]);
            }
            ++marker_count;
            if (markers_count != nullptr)
                *markers_count = marker_count;
            if (marker_count == marker_limit_) {
                writeTermination(cursor + offset - termination_.size());
            }
        }
    }

    /**
     * @brief Current kernel radius.
     */
    unsigned get_radius() const { return module_->radius(); }
    /**
     * @brief Current image width.
     */
    unsigned get_image_width() const { return module_->image_width(); }
    /**
     * @brief Current image height.
     */
    unsigned get_image_height() const { return module_->image_height(); }
    /**
     * @brief Threshold used for center pixel.
     */
    unsigned char get_threshold_center() const { return threshold_center_; }
    /**
     * @brief Threshold used for boundary difference.
     */
    unsigned char get_threshold_diff() const { return threshold_diff_; }
    /**
     * @brief Threshold used to qualify sun points.
     */
    unsigned char get_threshold_sun() const { return threshold_sun_; }
    /**
     * @brief Set center threshold.
     */
    void set_threshold_center(unsigned char value) { threshold_center_ = value; }
    /**
     * @brief Set boundary difference threshold.
     */
    void set_threshold_diff(unsigned char value) { threshold_diff_ = value; }
    /**
     * @brief Set sun threshold.
     */
    void set_threshold_sun(unsigned char value) { threshold_sun_ = value; }
    /**
     * @brief Current termination sequence.
     */
    std::array<unsigned char, 2> get_termination() const { return termination_; }
    /**
     * @brief Replace termination sequence.
     */
    void set_termination(std::array<unsigned char, 2> termination) { termination_ = termination; }
    /**
     * @brief Configured max markers count.
     */
    unsigned get_max_markers_count() const { return configured_max_markers_count_; }
    /**
     * @brief Configured max sun count.
     */
    unsigned get_max_sun_points_count() const { return configured_max_sun_points_count_; }
    /**
     * @brief Set max markers count.
     */
    void set_max_markers_count(unsigned value)
    {
        configured_max_markers_count_ = value;
        updateLimits();
    }
    /**
     * @brief Set max sun points count.
     */
    void set_max_sun_points_count(unsigned value)
    {
        configured_max_sun_points_count_ = value;
        updateLimits();
    }

private:
    /**
     * @brief Convert 1D pixel index to integer coordinates.
     */
    Point2D coord1to2(std::size_t coordinate) const
    {
        return Point2D { static_cast<int>(coordinate % module_->image_width()), static_cast<int>(coordinate / module_->image_width()) };
    }

    /**
     * @brief Compute frame size in bytes.
     */
    std::size_t frameBytes() const
    {
        return static_cast<std::size_t>(module_->image_width()) * static_cast<std::size_t>(module_->image_height()) * sizeof(unsigned char);
    }

    /**
     * @brief Recompute internal hard limits for allocations.
     */
    void updateLimits()
    {
        marker_limit_ = (configured_max_markers_count_ == 0) ? std::numeric_limits<unsigned>::max() : configured_max_markers_count_;
        sun_limit_    = (configured_max_sun_points_count_ == 0) ? std::numeric_limits<unsigned>::max() : configured_max_sun_points_count_;
    }

    /**
     * @brief Check whether scan position contains termination marker.
     */
    bool hasTermination(const unsigned char* position) const
    {
        return *reinterpret_cast<const std::uint16_t*>(position) == *reinterpret_cast<const std::uint16_t*>(termination_.data());
    }

    /**
     * @brief Write termination marker to scan buffer.
     */
    void writeTermination(unsigned char* position) const
    {
        *reinterpret_cast<std::uint16_t*>(position) = *reinterpret_cast<const std::uint16_t*>(termination_.data());
    }

    /**
     * @brief Check whether sun point should be rejected by boundary test.
     */
    bool sunBoundaryRejected(const unsigned char* cursor, unsigned char pixel_value, const std::vector<int>& boundary_offsets) const
    {
        for (std::size_t index = 1; index < boundary_offsets.size(); ++index) {
            if ((pixel_value - *(cursor + boundary_offsets[index])) > threshold_diff_) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Check whether marker should be rejected by boundary test.
     */
    bool markerBoundaryRejected(const unsigned char* cursor, unsigned char pixel_value, const std::vector<int>& boundary_offsets) const
    {
        for (std::size_t index = 1; index < boundary_offsets.size(); ++index) {
            if ((pixel_value - *(cursor + boundary_offsets[index])) <= threshold_diff_) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Clear all interior neighbors used by current candidate.
     */
    void clearInterior(unsigned char* cursor, const std::vector<int>& interior_offsets) const
    {
        for (int offset : interior_offsets) {
            *(cursor + offset) = static_cast<unsigned char>(0x00);
        }
    }

    /**
     * @brief Scan interior neighbors for local peak and clear candidates.
     */
    void scanInterior(unsigned char* cursor, unsigned char* target_image, unsigned char& peak, std::size_t& peak_position, const std::vector<int>& interior_offsets) const
    {
        for (int offset : interior_offsets) {
            unsigned char* interior_ptr = cursor + offset;
            if (*interior_ptr > peak) {
                peak          = *interior_ptr;
                peak_position = static_cast<std::size_t>(interior_ptr - target_image);
            }
            *interior_ptr = static_cast<unsigned char>(0x00);
        }
    }

    std::shared_ptr<const RuntimeFimdRadiusModule> module_;
    unsigned char threshold_center_;
    unsigned char threshold_diff_;
    unsigned char threshold_sun_;
    std::array<unsigned char, 2> termination_;
    unsigned configured_max_markers_count_    = 0;
    unsigned configured_max_sun_points_count_ = 0;
    unsigned marker_limit_                    = std::numeric_limits<unsigned>::max();
    unsigned sun_limit_                       = std::numeric_limits<unsigned>::max();
    unsigned char* frame_                     = nullptr;
};

class CpuDetector final : public uvdar_core::detection::IDetector {
public:
    /**
     * @brief Construct CPU detector.
     */
    explicit CpuDetector(CpuDetectorConfig config = { });
    /**
     * @brief Release implementation pointer.
     */
    ~CpuDetector() override;

    /**
     * @brief Initialize kernels on first frame.
     */
    bool initDelayed(const cv::Mat& image) override;
    /**
     * @brief Process one image and populate output points.
     */
    bool processImage(const cv::Mat& image, DetectorOutput& output, int mask_id = -1) override;

    /**
     * @brief Whether debug is enabled.
     */
    bool get_debug() const;
    /**
     * @brief Set debug mode.
     */
    void set_debug(bool debug);
    /**
     * @brief Whether sun detection is enabled.
     */
    bool get_detect_sun_points() const;
    /**
     * @brief Enable/disable sun point detection.
     */
    void set_detect_sun_points(bool detect_sun_points);
    /**
     * @brief Return marker threshold.
     */
    int get_threshold() const;
    /**
     * @brief Set marker threshold and reinitialize.
     */
    void set_threshold(int threshold);
    /**
     * @brief Return boundary delta threshold.
     */
    int get_threshold_diff() const;
    /**
     * @brief Set boundary delta threshold and reinitialize.
     */
    void set_threshold_diff(int threshold_diff);
    /**
     * @brief Return sun threshold.
     */
    int get_threshold_sun() const;
    /**
     * @brief Set sun threshold.
     */
    void set_threshold_sun(int threshold_sun);
    /**
     * @brief Current maximum markers cap.
     */
    unsigned get_max_markers_count() const;
    /**
     * @brief Set markers cap and reinitialize kernels.
     */
    void set_max_markers_count(unsigned max_markers_count);
    /**
     * @brief Current maximum sun cap.
     */
    unsigned get_max_sun_points_count() const;
    /**
     * @brief Set sun cap and reinitialize kernels.
     */
    void set_max_sun_points_count(unsigned max_sun_points_count);
    /**
     * @brief Configured radii list.
     */
    const std::vector<unsigned>& get_radii() const;
    /**
     * @brief Replace radii and reinitialize kernels.
     */
    void set_radii(std::vector<unsigned> radii);
    /**
     * @brief Configured masks.
     */
    const std::vector<cv::Mat>& get_masks() const;
    /**
     * @brief Replace mask set.
     */
    void set_masks(std::vector<cv::Mat> masks);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace uvdar_core::detection::fimd
