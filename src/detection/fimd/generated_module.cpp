#include "uvdar_core/detection/fimd/generated_module.hpp"

#include <dlfcn.h>

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace uvdar_core::detection::fimd {

namespace {

#ifndef UVDAR_FIMD_GENERATOR_COMPILER
#define UVDAR_FIMD_GENERATOR_COMPILER "cc"
#endif

#ifndef UVDAR_FIMD_GENERATOR_CFLAGS
#define UVDAR_FIMD_GENERATOR_CFLAGS "-shared -fPIC -O3 -DNDEBUG -march=native -mtune=native -fomit-frame-pointer -fno-semantic-interposition"
#endif

    /**
     * @brief Function signature produced by generated shared object.
     */
    using GeneratedDetectFn = unsigned char* (*)(unsigned char*, std::uint32_t*, std::uint32_t*, std::uint32_t*, std::uint32_t*);

    /**
     * @brief Return OS cache directory for generated kernels.
     */
    std::filesystem::path cacheDirectory()
    {
        return std::filesystem::temp_directory_path() / "uvdar_core_fimd_cache";
    }

    /**
     * @brief Build stable cache key from kernel configuration.
     */
    std::string makeKey(
        const RuntimeFimdRadiusModule& module,
        unsigned char threshold_center,
        unsigned char threshold_diff,
        unsigned char threshold_sun,
        unsigned max_markers_count,
        unsigned max_sun_points_count,
        bool detect_sun_points)
    {
        std::ostringstream stream;
        stream << "r" << module.radius()
               << "_w" << module.image_width()
               << "_h" << module.image_height()
               << "_tc" << static_cast<unsigned>(threshold_center)
               << "_td" << static_cast<unsigned>(threshold_diff)
               << "_ts" << static_cast<unsigned>(threshold_sun)
               << "_mm" << max_markers_count
               << "_ms" << max_sun_points_count
               << (detect_sun_points ? "_with_sun" : "_no_sun");
        return stream.str();
    }

    /**
     * @brief Check whether a file already contains the expected source text.
     */
    bool sourceEqualsCached(const std::filesystem::path& path, const std::string& source_code)
    {
        std::ifstream input(path);
        if (!input.good()) {
            return false;
        }

        std::ostringstream stored;
        stored << input.rdbuf();
        return stored.str() == source_code;
    }

    /**
     * @brief Generate C source implementing the FIMD kernel.
     */
    std::string generateSource(
        const RuntimeFimdRadiusModule& module,
        unsigned char threshold_center,
        unsigned char threshold_diff,
        unsigned char threshold_sun,
        unsigned max_markers_count,
        unsigned max_sun_points_count,
        bool detect_sun_points)
    {
        std::ostringstream source;
        source << "#include <stdint.h>\n"
               << "#include <stddef.h>\n"
               << "uint8_t* uvdar_generated_detect(uint8_t* img_ptr, uint32_t* markers, uint32_t* markers_num, uint32_t* sun_pts, uint32_t* sun_pts_num) {\n"
               << "  const uint32_t IM_WIDTH = " << module.image_width() << ";\n"
               << "  const uint32_t IM_HEIGHT = " << module.image_height() << ";\n"
               << "  const uint8_t FIMD_THRESHOLD_CENTER = " << static_cast<unsigned>(threshold_center) << ";\n"
               << "  const uint8_t FIMD_THRESHOLD_DIFF = " << static_cast<unsigned>(threshold_diff) << ";\n"
               << "  const uint8_t FIMD_THRESHOLD_SUN = " << static_cast<unsigned>(threshold_sun) << ";\n"
               << "  const uint32_t FIMD_MAX_MARKERS_COUNT = " << max_markers_count << ";\n"
               << "  const uint32_t FIMD_MAX_SUN_PTS_COUNT = " << max_sun_points_count << ";\n"
               << "  const uint16_t FIMD_TERM_SEQ = 0x00FF;\n"
               << "  const uint32_t FIMD_OFFSET = " << module.offset() << ";\n"
               << "  *((uint16_t*) (img_ptr + (IM_WIDTH * IM_HEIGHT) - 2)) = FIMD_TERM_SEQ;\n"
               << "  img_ptr = img_ptr + (FIMD_OFFSET - 1);\n"
               << "  uint8_t* image_start = img_ptr - (FIMD_OFFSET - 1);\n"
               << "LOOP:\n"
               << "  if (*((uint16_t*) (img_ptr + FIMD_OFFSET)) == FIMD_TERM_SEQ) return img_ptr;\n"
               << "  uint32_t linear_pos = 0U;\n"
                << "  uint8_t pix_val = *(++img_ptr);\n"
                << "  if (pix_val <= FIMD_THRESHOLD_CENTER) goto LOOP;\n";

        const auto& boundary_offsets = module.boundary_offsets();
        const auto& interior_offsets = module.interior_offsets();

        if (detect_sun_points) {
            source << "  if (pix_val - *((uint8_t*) (img_ptr + " << boundary_offsets[0] << ")) <= FIMD_THRESHOLD_DIFF) {\n"
                   << "    if (pix_val >= FIMD_THRESHOLD_SUN) goto SUN_TEST;\n"
                   << "  } else {\n"
                   << "    goto MARKER_TEST;\n"
                   << "  }\n"
                   << "  goto LOOP;\n"
                   << "SUN_TEST:\n"
                   << "  if (*sun_pts_num == FIMD_MAX_SUN_PTS_COUNT) { *((uint16_t*) (img_ptr + FIMD_OFFSET)) = FIMD_TERM_SEQ; goto LOOP; }\n";

            for (std::size_t index = 1; index < boundary_offsets.size(); ++index) {
                source << "  if (pix_val - *((uint8_t*) (img_ptr + " << boundary_offsets[index] << ")) > FIMD_THRESHOLD_DIFF) goto LOOP;\n";
            }
            for (int offset : interior_offsets) {
                source << "  *((uint8_t*) (img_ptr + " << offset << ")) = 0x00;\n";
            }
            source << "  linear_pos = (uint32_t)(img_ptr - image_start);\n"
                   << "  if (linear_pos > 0x00FFFFFFu) { *((uint16_t*) (img_ptr + FIMD_OFFSET)) = FIMD_TERM_SEQ; goto LOOP; }\n"
                   << "  if (*sun_pts_num >= FIMD_MAX_SUN_PTS_COUNT) { *((uint16_t*) (img_ptr + FIMD_OFFSET)) = FIMD_TERM_SEQ; goto LOOP; }\n"
                   << "  sun_pts[*sun_pts_num] = (linear_pos << 8u) | (uint32_t)pix_val;\n"
                   << "  (*sun_pts_num)++;\n"
                   << "  if (*sun_pts_num == FIMD_MAX_SUN_PTS_COUNT) { *((uint16_t*) (img_ptr + FIMD_OFFSET)) = FIMD_TERM_SEQ; }\n"
                   << "  goto LOOP;\n"
                   << "MARKER_TEST:\n";
        } else {
            source << "  if (pix_val - *((uint8_t*) (img_ptr + " << boundary_offsets[0] << ")) <= FIMD_THRESHOLD_DIFF) goto LOOP;\n";
        }

        for (std::size_t index = 1; index < boundary_offsets.size(); ++index) {
            source << "  if (pix_val - *((uint8_t*) (img_ptr + " << boundary_offsets[index] << ")) <= FIMD_THRESHOLD_DIFF) goto LOOP;\n";
        }

        source << "  linear_pos = (uint32_t)(img_ptr - image_start);\n"
               << "  if (linear_pos > 0x00FFFFFFu) { *((uint16_t*) (img_ptr + FIMD_OFFSET)) = FIMD_TERM_SEQ; goto LOOP; }\n"
               << "  if (*markers_num >= FIMD_MAX_MARKERS_COUNT) { *((uint16_t*) (img_ptr + FIMD_OFFSET)) = FIMD_TERM_SEQ; goto LOOP; }\n"
               << "  markers[*markers_num] = (linear_pos << 8u) | (uint32_t)pix_val;\n"
               << "  (*markers_num)++;\n"
               << "  if (*markers_num == FIMD_MAX_MARKERS_COUNT) *((uint16_t*) (img_ptr + FIMD_OFFSET)) = FIMD_TERM_SEQ;\n"
               << "  goto LOOP;\n"
               << "}\n";

        return source.str();
    }

    /**
     * @brief Compile generated C source into a shared object.
     */
    std::string quoteForShell(const std::string& value)
    {
        return "\"" + value + "\"";
    }

    bool compileModule(const std::filesystem::path& source_path, const std::filesystem::path& output_path)
    {
        const std::string command = std::string(UVDAR_FIMD_GENERATOR_COMPILER) + " "
                                  + UVDAR_FIMD_GENERATOR_CFLAGS + " "
                                  + "-o "
                                  + quoteForShell(output_path.string()) + " "
                                  + quoteForShell(source_path.string());
        return std::system(command.c_str()) == 0;
    }

} // namespace

    struct GeneratedFimdCpuKernel::Impl {
    /**
     * @brief Runtime state for generated module loading and fallback execution.
     */
    /**
     * @brief Construct implementation and generate or load shared module.
     */
    Impl(
        std::shared_ptr<const RuntimeFimdRadiusModule> radius_module,
        unsigned char center_threshold,
        unsigned char diff_threshold,
        unsigned char sun_threshold,
        std::array<unsigned char, 2> termination_sequence,
        unsigned max_markers,
        unsigned max_sun_points,
        bool detect_sun_points)
        : module(std::move(radius_module))
        , threshold_center(center_threshold)
        , threshold_diff(diff_threshold)
        , threshold_sun(sun_threshold)
        , termination(termination_sequence)
        , max_markers_count(max_markers == 0 ? std::numeric_limits<unsigned>::max() : max_markers)
        , max_sun_points_count(max_sun_points == 0 ? std::numeric_limits<unsigned>::max() : max_sun_points)
        , detect_sun_points(detect_sun_points)
        , fallback(module, center_threshold, diff_threshold, sun_threshold, termination_sequence, max_markers, max_sun_points)
    {
        const auto key = makeKey(*module, threshold_center, threshold_diff, threshold_sun, max_markers_count, max_sun_points_count, detect_sun_points);
        const auto cache_dir = cacheDirectory();
        std::filesystem::create_directories(cache_dir);
        source_path  = cache_dir / (key + ".c");
        library_path = cache_dir / (key + ".so");

        const std::string generated_source = generateSource(*module, threshold_center, threshold_diff, threshold_sun, max_markers_count, max_sun_points_count, detect_sun_points);
        const bool source_is_same          = sourceEqualsCached(source_path, generated_source);

        if (!std::filesystem::exists(source_path) || !source_is_same) {
            std::ofstream output(source_path);
            output << generated_source;
            output.close();
            generated_ready = compileModule(source_path, library_path);
        } else {
            if (!std::filesystem::exists(library_path)) {
                generated_ready = compileModule(source_path, library_path);
            } else {
                generated_ready = true;
            }
        }

        if (generated_ready) {
            handle = dlopen(library_path.c_str(), RTLD_NOW | RTLD_LOCAL);
            if (handle != nullptr) {
                generated_detect = reinterpret_cast<GeneratedDetectFn>(dlsym(handle, "uvdar_generated_detect"));
            }
            generated_ready = (handle != nullptr && generated_detect != nullptr);
        }
    }

    /**
     * @brief Release generated library handle and allocated frame.
     */
    ~Impl()
    {
        if (handle != nullptr) {
            dlclose(handle);
        }
        if (frame != nullptr) {
            std::free(frame);
        }
    }

    /**
     * @brief Run generated detector or fallback path.
     */
    unsigned detectRaw(
        const unsigned char* image,
        std::uint32_t* markers,
        unsigned* markers_count,
        std::uint32_t* sun_points,
        unsigned* sun_points_count,
        bool make_copy)
    {
        if (!generated_ready) {
            return fallback.detectRaw(image, markers, markers_count, sun_points, sun_points_count, make_copy);
        }

        const std::size_t total_pixels = static_cast<std::size_t>(module->image_width()) * static_cast<std::size_t>(module->image_height());
        unsigned char* working_image   = const_cast<unsigned char*>(image);
        if (make_copy) {
            if (frame == nullptr) {
                frame = static_cast<unsigned char*>(std::malloc(total_pixels));
            }
            std::memcpy(frame, image, total_pixels);
            working_image = frame;
        }

        if (marker_ptrs.size() != max_markers_count) {
            marker_ptrs.resize(max_markers_count);
        }
        if (detect_sun_points && sun_ptrs.size() != max_sun_points_count) {
            sun_ptrs.resize(max_sun_points_count);
        }

        std::uint32_t raw_markers_count    = 0;
        std::uint32_t raw_sun_points_count = 0;
        generated_detect(working_image, marker_ptrs.data(), &raw_markers_count, detect_sun_points ? sun_ptrs.data() : nullptr, &raw_sun_points_count);

        if (markers_count != nullptr) {
            *markers_count = std::min(raw_markers_count, static_cast<std::uint32_t>(max_markers_count));
        }
        if (sun_points_count != nullptr) {
            *sun_points_count = detect_sun_points ? std::min(raw_sun_points_count, static_cast<std::uint32_t>(max_sun_points_count)) : 0;
        }

        if (markers != nullptr) {
            const std::uint32_t max_markers_to_copy = std::min(raw_markers_count, static_cast<std::uint32_t>(max_markers_count));
            for (std::uint32_t index = 0; index < max_markers_to_copy; ++index) {
                markers[index] = marker_ptrs[index];
            }
        }
        if (detect_sun_points && sun_points != nullptr) {
            const std::uint32_t max_sun_to_copy = std::min(raw_sun_points_count, static_cast<std::uint32_t>(max_sun_points_count));
            for (std::uint32_t index = 0; index < max_sun_to_copy; ++index) {
                sun_points[index] = sun_ptrs[index];
            }
        }

        const std::uint32_t clamped_markers = std::min(raw_markers_count, static_cast<std::uint32_t>(max_markers_count));
        const std::uint32_t clamped_sun_points = detect_sun_points ? std::min(raw_sun_points_count, static_cast<std::uint32_t>(max_sun_points_count)) : 0;
        return clamped_markers + clamped_sun_points;
    }

    std::shared_ptr<const RuntimeFimdRadiusModule> module;
    unsigned char threshold_center;
    unsigned char threshold_diff;
    unsigned char threshold_sun;
    std::array<unsigned char, 2> termination;
    unsigned max_markers_count;
    unsigned max_sun_points_count;
    bool detect_sun_points;
    RuntimeFimdCpuKernel fallback;
    std::filesystem::path source_path;
    std::filesystem::path library_path;
    void* handle                       = nullptr;
    GeneratedDetectFn generated_detect = nullptr;
    bool generated_ready               = false;
    unsigned char* frame               = nullptr;
    std::vector<std::uint32_t> marker_ptrs;
    std::vector<std::uint32_t> sun_ptrs;
};

GeneratedFimdCpuKernel::GeneratedFimdCpuKernel(
    std::shared_ptr<const RuntimeFimdRadiusModule> module,
    unsigned char threshold_center,
    unsigned char threshold_diff,
    unsigned char threshold_sun,
    std::array<unsigned char, 2> termination,
    unsigned max_markers_count,
    unsigned max_sun_points_count,
    bool detect_sun_points)
    : impl_(std::make_unique<Impl>(std::move(module), threshold_center, threshold_diff, threshold_sun, termination, max_markers_count, max_sun_points_count, detect_sun_points))
{
}

/**
 * @brief Default destructor.
 */
GeneratedFimdCpuKernel::~GeneratedFimdCpuKernel() = default;

/**
 * @brief Run raw detection and decode point arrays.
 */
unsigned GeneratedFimdCpuKernel::detectRaw(
    const unsigned char* image,
    std::uint32_t* markers,
    unsigned* markers_count,
    std::uint32_t* sun_points,
    unsigned* sun_points_count,
    bool make_copy)
{
    return impl_->detectRaw(image, markers, markers_count, sun_points, sun_points_count, make_copy);
}

/**
 * @brief Whether generated shared library path was successfully loaded.
 */
bool GeneratedFimdCpuKernel::isUsingGeneratedPath() const
{
    return impl_->generated_ready;
}

/**
 * @brief Return max marker count.
 */
unsigned GeneratedFimdCpuKernel::get_max_markers_count() const
{
    return impl_->max_markers_count;
}

/**
 * @brief Return max sun point count.
 */
unsigned GeneratedFimdCpuKernel::get_max_sun_points_count() const
{
    return impl_->max_sun_points_count;
}

} // namespace uvdar_core::detection::fimd
