#include "uvdar_core/detection/fimd/gpu_detector.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdint>
#include <vector>
#include <mutex>
#include <numeric>
#include <string>
#include <utility>

#include "uvdar_core/detection/fimd/postprocess.hpp"
#include "uvdar_core/helpers/compute_shader.hpp"

extern "C" {
extern const unsigned char _binary_shaders_fimd_masked_no_sun_comp_start[];
extern const unsigned char _binary_shaders_fimd_masked_no_sun_comp_end[];
extern const unsigned char _binary_shaders_fimd_masked_with_sun_comp_start[];
extern const unsigned char _binary_shaders_fimd_masked_with_sun_comp_end[];
}

namespace uvdar_core::detection::fimd {

    namespace {
    /**
     * @brief Decode packed 32-bit GPU coordinates.
     */
    std::vector<WeightedPoint> decodePackedPoints(const std::vector<std::uint32_t>& raw_points, std::size_t count, unsigned width)
    {
        std::vector<WeightedPoint> points;
        points.reserve(count);
        for (std::size_t index = 0; index < count; ++index) {
            const auto raw_point   = raw_points[index];
            const auto linear_pos  = raw_point >> 8u;
            points.push_back(WeightedPoint {
                cv::Point2f(
                    static_cast<float>(linear_pos % static_cast<std::uint32_t>(width)),
                    static_cast<float>(linear_pos / static_cast<std::uint32_t>(width))),
                static_cast<float>(raw_point & 0xFFu),
            });
        }
        return points;
    }

    std::vector<uvdar_core::detection::DetectorPoint> makeDetectorPoints(
        const std::vector<WeightedPoint>& raw_points)
    {
        std::vector<uvdar_core::detection::DetectorPoint> points;
        points.reserve(raw_points.size());
        for (const auto& raw_point : raw_points) {
            points.push_back(uvdar_core::detection::DetectorPoint {
                raw_point.point,
                1.0F / 12.0F,
                0.0F,
                0.0F,
                1.0F / 12.0F,
                1,
            });
        }
        return points;
    }

    /**
     * @brief Build shader source string from embedded binary span.
     */
    std::string shaderSourceFromEmbedded(const unsigned char* start, const unsigned char* end)
    {
        return std::string(reinterpret_cast<const char*>(start), end - start);
    }

    /**
     * @brief Initialize SSBO and clear previous state.
     */
    bool initBuffer(
        uvdar_core::helpers::compute_shader::SSBO& buffer,
        GLuint binding,
        std::size_t element_count,
        const char* name)
    {
        buffer.destroy();
        buffer.resource.value = static_cast<GLint>(binding);
        const auto status     = buffer.init(nullptr, static_cast<GLint>(element_count));
        if (status != GL_NO_ERROR) {
            std::fprintf(stderr, "[gpu_detector] Failed to init SSBO '%s' (elements=%zu): GL error count %u.\n", name, element_count, status);
            return false;
        }
        return true;
    }

    /**
     * @brief Initialize ACBO and clear previous state.
     */
    bool initBuffer(
        uvdar_core::helpers::compute_shader::ACBO& buffer,
        GLuint binding,
        std::size_t element_count,
        const char* name)
    {
        buffer.destroy();
        buffer.resource.value = static_cast<GLint>(binding);
        const auto status     = buffer.init(nullptr, static_cast<GLint>(element_count));
        if (status != GL_NO_ERROR) {
            std::fprintf(stderr, "[gpu_detector] Failed to init ACBO '%s' (elements=%zu): GL error count %u.\n", name, element_count, status);
            return false;
        }
        return true;
    }

    /**
     * @brief Print queued GL messages for a stage.
     */
    void reportGlErrors(const std::string& stage, uvdar_core::helpers::compute_shader::Context& context)
    {
        const GLuint count = context.instance().flush_errors(stderr);
        if (count > 0) {
            std::fprintf(stderr, "[gpu_detector] %s (%u GL debug messages).\n", stage.c_str(), count);
        }
    }

    /**
     * @brief Write SSBO contents and report errors.
     */
    bool writeBuffer(uvdar_core::helpers::compute_shader::SSBO& buffer, const void* data, std::size_t element_count, const char* name)
    {
        if (buffer.write(data, static_cast<GLint>(element_count)) != GL_NO_ERROR) {
            std::fprintf(stderr, "[gpu_detector] Failed to write SSBO '%s'.\n", name);
            return false;
        }
        return true;
    }

    /**
     * @brief Convert image bytes to packed 32-bit format for shader access.
     */
    void packImageToUint32(const std::uint8_t* source, std::size_t pixel_count, std::vector<std::uint32_t>& destination)
    {
        const std::size_t packed_count = (pixel_count + 3U) >> 2;
        destination.assign(packed_count, 0U);
        for (std::size_t index = 0; index < pixel_count; ++index) {
            const std::size_t dword_index = index >> 2;
            const unsigned bit_offset     = static_cast<unsigned>((index & 3U) << 3);
            destination[dword_index] |= static_cast<std::uint32_t>(source[index]) << bit_offset;
        }
    }

    /**
     * @brief Reset atomic counter buffer to value.
     */
    bool writeCounter(uvdar_core::helpers::compute_shader::ACBO& buffer, GLuint value, const char* name)
    {
        if (buffer.write_uint_val(value) != GL_NO_ERROR) {
            std::fprintf(stderr, "[gpu_detector] Failed to reset ACBO '%s'.\n", name);
            return false;
        }
        return true;
    }

    struct ContextScope {
        uvdar_core::helpers::compute_shader::Context& context;
        bool active;
        std::string message;

        /**
         * @brief Bind context for current thread and keep active flag.
         */
        ContextScope(
            uvdar_core::helpers::compute_shader::Context& context_,
            const std::string& message_)
            : context(context_)
            , active(context.makeCurrent())
            , message(message_)
        {
            if (!active) {
                std::fprintf(stderr, "%s\n", message_.c_str());
            }
        }

        ~ContextScope()
        {
            if (active) {
                context.releaseCurrent();
            }
        }

        /**
         * @brief Convert scope to boolean state.
         */
        explicit operator bool() const
        {
            return active;
        }
    };

} // namespace

struct GpuDetector::Impl {
    /**
     * @brief Implementation holds all GPU runtime state.
     */
    explicit Impl(GpuDetectorConfig cfg)
        : config(std::move(cfg))
    {
    }

    /**
     * @brief Initialize GPU resources and shaders for first frame.
     */
    bool initDelayed(const cv::Mat& image)
    {
        if (image.type() != CV_8UC1) {
            return false;
        }

        if (!context.isInitialized()) {
            if (context_init_failed_ && context_init_retry_count_ > 0 && context_init_retry_count_ < 120) {
                ++context_init_retry_count_;
                return false;
            }

        if (!context.initFirstAvailable()) {
                if (!context_init_failed_) {
                    std::fprintf(stderr, "[gpu_detector] Failed to initialize compute context (retrying on new frames).\n");
                    context_init_failed_ = true;
                }
            context_init_retry_count_ = (context_init_retry_count_ == 0 ? 1 : context_init_retry_count_ + 1);
            return false;
        }
        }

        const ContextScope context_scope(context, "[gpu_detector] Failed to bind compute context to current thread.");
        if (!context_scope) {
            return false;
        }
        context_init_failed_      = false;
        context_init_retry_count_ = 0;

        if (initialized && image.cols == width && image.rows == height) {
            return true;
        }

        width  = static_cast<unsigned>(image.cols);
        height = static_cast<unsigned>(image.rows);
        const std::size_t image_pixels = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
        const std::size_t packed_pixels = (image_pixels + 3U) >> 2;
        full_mask_.assign(image_pixels, 0xFF);
        packed_mask_buffer_.clear();
        packImageToUint32(full_mask_.data(), image_pixels, packed_mask_buffer_);

        if (!initBuffer(image_buffer, 0, packed_pixels, "image_in")) {
            return false;
        }
        if (!initBuffer(mask_buffer, 1, packed_pixels, "mask")) {
            return false;
        }

        const auto [local_x, local_y, local_z] = context.getLocalSizes(width, height);

        if (program.destroy(false) != GL_NO_ERROR) {
            return false;
        }

        if (config.detect_sun_points) {
            if (!initBuffer(sun_counter, 3, 1, "sun_pts_count")) {
                return false;
            }
            if (!initBuffer(marker_counter, 4, 1, "markers_count")) {
                return false;
            }
            if (!initBuffer(config_buffer, 5, 8 + config.radii.size(), "configuration_buffer")) {
                return false;
            }
            if (!initBuffer(marker_buffer, 6, config.max_markers_count, "markers_buffer")) {
                return false;
            }
            if (!initBuffer(sun_buffer, 7, config.max_sun_points_count, "sun_pts_buffer")) {
                return false;
            }
        } else {
            if (!initBuffer(marker_counter, 3, 1, "markers_count")) {
                return false;
            }
            if (!initBuffer(config_buffer, 4, 8 + config.radii.size(), "configuration_buffer")) {
                return false;
            }
            if (!initBuffer(marker_buffer, 5, config.max_markers_count, "markers_buffer")) {
                return false;
            }
            sun_counter.destroy();
            sun_buffer.destroy();
            if (!program.init(context, shaderSourceFromEmbedded(_binary_shaders_fimd_masked_no_sun_comp_start, _binary_shaders_fimd_masked_no_sun_comp_end), local_x, local_y, local_z)) {
                reportGlErrors("Failed to init no-sun program", context);
                return false;
            }
            initialized = true;
            return true;
        }

        if (!program.init(context, shaderSourceFromEmbedded(_binary_shaders_fimd_masked_with_sun_comp_start, _binary_shaders_fimd_masked_with_sun_comp_end), local_x, local_y, local_z)) {
            reportGlErrors("Failed to init with-sun program", context);
            return false;
        }

        initialized = true;
        return true;
    }

    /**
     * @brief Build packed scalar config vector for compute shader.
     */
    std::vector<std::uint32_t> makeConfig() const
    {
        std::vector<std::uint32_t> values;
        values.reserve(8 + config.radii.size());
        values.push_back(width);
        values.push_back(height);
        values.push_back(static_cast<std::uint32_t>(config.threshold));
        values.push_back(static_cast<std::uint32_t>(config.threshold_diff));
        values.push_back(static_cast<std::uint32_t>(config.threshold_sun));
        values.push_back(config.max_markers_count);
        values.push_back(config.max_sun_points_count);
        values.push_back(static_cast<std::uint32_t>(config.radii.size()));
        for (unsigned radius : config.radii) {
            values.push_back(static_cast<std::uint32_t>(radius));
        }
        return values;
    }

    /**
     * @brief Process one frame through compute shader and decode points.
     */
    bool processImage(const cv::Mat& image, DetectorOutput& output, int mask_id)
    {
        std::scoped_lock<std::mutex> guard(context_mutex_);

        if (!initialized && !initDelayed(image)) {
            return false;
        }
        const ContextScope context_scope(context, "[gpu_detector] Failed to bind compute context to current thread for image processing.");
        if (!context_scope) {
            return false;
        }

        const std::size_t pixel_count = static_cast<std::size_t>(image.total());
        packImageToUint32(image.data, pixel_count, packed_image_buffer_);
        if (!writeBuffer(image_buffer, packed_image_buffer_.data(), packed_image_buffer_.size(), "image_in")) {
            reportGlErrors("Failed to write image 'image_in'", context);
            return false;
        }
        const auto packed_config = makeConfig();
        if (mask_id >= 0) {
            if (static_cast<std::size_t>(mask_id) >= config.masks.size()) {
                std::fprintf(stderr, "[gpu_detector] Mask index %d is out of range.\n", mask_id);
                return false;
            }
            if (config.masks[mask_id].size() != image.size() || config.masks[mask_id].type() != CV_8UC1) {
                std::fprintf(stderr, "[gpu_detector] The selected mask size/type does not match input image.\n");
                return false;
            }
            packImageToUint32(config.masks[mask_id].data, pixel_count, packed_mask_buffer_);
            if (!writeBuffer(mask_buffer, packed_mask_buffer_.data(), packed_mask_buffer_.size(), "mask")) {
                reportGlErrors("Failed to write image 'mask'", context);
                return false;
            }
        } else {
            if (packed_mask_buffer_.empty() || packed_mask_buffer_.size() * 4U < pixel_count) {
                packed_mask_buffer_.clear();
                packImageToUint32(full_mask_.data(), pixel_count, packed_mask_buffer_);
            }
            if (!writeBuffer(mask_buffer, packed_mask_buffer_.data(), packed_mask_buffer_.size(), "mask")) {
                reportGlErrors("Failed to write image 'mask'", context);
                return false;
            }
        }
        if (!writeBuffer(config_buffer, packed_config.data(), packed_config.size(), "configuration_buffer")) {
            reportGlErrors("Failed to write buffer 'configuration_buffer'", context);
            return false;
        }

        if (!writeCounter(marker_counter, 0U, "markers_count")) {
            reportGlErrors("Failed to write counter 'markers_count'", context);
            return false;
        }
        if (config.detect_sun_points) {
            if (!writeCounter(sun_counter, 0U, "sun_pts_count")) {
                reportGlErrors("Failed to write counter 'sun_pts_count'", context);
                return false;
            }
        }

        if (!program.dispatch(width, height, 1)) {
            return false;
        }

        GLuint marker_count = 0;
        if (marker_counter.read_uint_val(&marker_count) != GL_NO_ERROR) {
            std::fprintf(stderr, "[gpu_detector] Failed to read markers count.\n");
            return false;
        }
        marker_count = std::min(marker_count, static_cast<GLuint>(config.max_markers_count));
        std::vector<std::uint32_t> raw_markers(config.max_markers_count, 0U);
        if (marker_count > 0) {
            if (marker_buffer.read(raw_markers.data(), static_cast<GLint>(marker_count)) != GL_NO_ERROR) {
                std::fprintf(stderr, "[gpu_detector] Failed to read markers buffer.\n");
                return false;
            }
        }
        auto raw_marker_points = decodePackedPoints(raw_markers, marker_count, width);

        output.sun_points.clear();
        std::vector<WeightedPoint> raw_sun_points;
        if (config.detect_sun_points) {
            GLuint sun_count = 0;
            if (sun_counter.read_uint_val(&sun_count) != GL_NO_ERROR) {
                std::fprintf(stderr, "[gpu_detector] Failed to read sun points count.\n");
                return false;
            }
            sun_count = std::min(sun_count, static_cast<GLuint>(config.max_sun_points_count));
            std::vector<std::uint32_t> raw_sun(config.max_sun_points_count, 0U);
            if (sun_count > 0) {
                if (sun_buffer.read(raw_sun.data(), static_cast<GLint>(sun_count)) != GL_NO_ERROR) {
                    std::fprintf(stderr, "[gpu_detector] Failed to read sun points buffer.\n");
                    return false;
                }
            }
            raw_sun_points = decodePackedPoints(raw_sun, sun_count, width);
            filterRawMarkersNearSunPoints(
                raw_marker_points,
                raw_sun_points,
                config.min_sun_marker_distance,
                width,
                height);
            output.sun_points = makeDetectorPoints(raw_sun_points);
        }

        output.detected_points = collapseRawPoints(raw_marker_points, 5);

        return true;
    }

    GpuDetectorConfig config;
    bool initialized = false;
    bool context_init_failed_ = false;
    unsigned context_init_retry_count_ = 0;
    unsigned width   = 0;
    unsigned height  = 0;
    uvdar_core::helpers::compute_shader::Context context;
    uvdar_core::helpers::compute_shader::Program program;
    uvdar_core::helpers::compute_shader::SSBO image_buffer { "image_in", GL_UNSIGNED_INT, GL_DYNAMIC_DRAW };
    uvdar_core::helpers::compute_shader::SSBO mask_buffer { "mask", GL_UNSIGNED_INT, GL_DYNAMIC_DRAW };
    uvdar_core::helpers::compute_shader::ACBO marker_counter { "markers_count", GL_UNSIGNED_INT, GL_DYNAMIC_DRAW };
    uvdar_core::helpers::compute_shader::ACBO sun_counter { "sun_pts_count", GL_UNSIGNED_INT, GL_DYNAMIC_DRAW };
    uvdar_core::helpers::compute_shader::SSBO config_buffer { "configuration_buffer", GL_UNSIGNED_INT, GL_DYNAMIC_DRAW };
    uvdar_core::helpers::compute_shader::SSBO marker_buffer { "markers_buffer", GL_UNSIGNED_INT, GL_DYNAMIC_DRAW };
    uvdar_core::helpers::compute_shader::SSBO sun_buffer { "sun_pts_buffer", GL_UNSIGNED_INT, GL_DYNAMIC_DRAW };
    std::vector<unsigned char> full_mask_;
    std::vector<std::uint32_t> packed_image_buffer_;
    std::vector<std::uint32_t> packed_mask_buffer_;
    std::mutex context_mutex_;
};


GpuDetector::GpuDetector(GpuDetectorConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

/**
 * @brief Default destructor.
 */
GpuDetector::~GpuDetector() = default;

/**
 * @brief Initialize lazily.
 */
bool GpuDetector::initDelayed(const cv::Mat& image) { return impl_->initDelayed(image); }
/**
 * @brief Process one frame via impl.
 */
bool GpuDetector::processImage(const cv::Mat& image, DetectorOutput& output, int mask_id) { return impl_->processImage(image, output, mask_id); }
/**
 * @brief Return debug flag.
 */
bool GpuDetector::get_debug() const { return impl_->config.debug; }
/**
 * @brief Set debug flag.
 */
void GpuDetector::set_debug(bool debug) { impl_->config.debug = debug; }
/**
 * @brief Whether sun detection is enabled.
 */
bool GpuDetector::get_detect_sun_points() const { return impl_->config.detect_sun_points; }
/**
 * @brief Enable/disable sun detection and force reinitialize.
 */
void GpuDetector::set_detect_sun_points(bool detect_sun_points)
{
    impl_->config.detect_sun_points = detect_sun_points;
    impl_->initialized              = false;
}
/**
 * @brief Return marker threshold.
 */
int GpuDetector::get_threshold() const { return impl_->config.threshold; }
/**
 * @brief Set marker threshold.
 */
void GpuDetector::set_threshold(int threshold) { impl_->config.threshold = threshold; }
/**
 * @brief Return threshold difference.
 */
int GpuDetector::get_threshold_diff() const { return impl_->config.threshold_diff; }
/**
 * @brief Set threshold difference.
 */
void GpuDetector::set_threshold_diff(int threshold_diff) { impl_->config.threshold_diff = threshold_diff; }
/**
 * @brief Return sun threshold.
 */
int GpuDetector::get_threshold_sun() const { return impl_->config.threshold_sun; }
/**
 * @brief Set sun threshold.
 */
void GpuDetector::set_threshold_sun(int threshold_sun) { impl_->config.threshold_sun = threshold_sun; }
/**
 * @brief Return marker cap.
 */
unsigned GpuDetector::get_max_markers_count() const { return impl_->config.max_markers_count; }
/**
 * @brief Set marker cap and rebuild lazily.
 */
void GpuDetector::set_max_markers_count(unsigned max_markers_count)
{
    impl_->config.max_markers_count = max_markers_count;
    impl_->initialized              = false;
}
/**
 * @brief Return sun cap.
 */
unsigned GpuDetector::get_max_sun_points_count() const { return impl_->config.max_sun_points_count; }
/**
 * @brief Set sun cap and rebuild lazily.
 */
void GpuDetector::set_max_sun_points_count(unsigned max_sun_points_count)
{
    impl_->config.max_sun_points_count = max_sun_points_count;
    impl_->initialized                 = false;
}
/**
 * @brief Return radii list.
 */
const std::vector<unsigned>& GpuDetector::get_radii() const { return impl_->config.radii; }
/**
 * @brief Replace radii and rebuild lazily.
 */
void GpuDetector::set_radii(std::vector<unsigned> radii)
{
    impl_->config.radii = std::move(radii);
    impl_->initialized  = false;
}
/**
 * @brief Access configured masks.
 */
const std::vector<cv::Mat>& GpuDetector::get_masks() const { return impl_->config.masks; }
/**
 * @brief Replace mask list.
 */
void GpuDetector::set_masks(std::vector<cv::Mat> masks) { impl_->config.masks = std::move(masks); }

} // namespace uvdar_core::detection::fimd
