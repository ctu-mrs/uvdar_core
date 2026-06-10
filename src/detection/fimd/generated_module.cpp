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

    using GeneratedDetectFn = unsigned char* (*)(unsigned char*, uintptr_t*, std::uint32_t*, uintptr_t*, std::uint32_t*);

    std::filesystem::path cacheDirectory()
    {
        return std::filesystem::temp_directory_path() / "uvdar_core_fimd_cache";
    }

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
               << (detect_sun_points ? "_with_sun" : "_no_sun")
               << "_schema_v3";
        return stream.str();
    }

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
               << "uint8_t* uvdar_generated_detect(uint8_t* img_ptr, uintptr_t* markers, uint32_t* markers_num, uintptr_t* sun_pts, uint32_t* sun_pts_num) {\n"
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
               << "LOOP:\n"
               << "  if (*((uint16_t*) (img_ptr + FIMD_OFFSET)) == FIMD_TERM_SEQ) return img_ptr;\n"
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
            source << "  sun_pts[*sun_pts_num] = (uintptr_t) img_ptr;\n"
                   << "  (*sun_pts_num)++;\n"
                   << "  goto LOOP;\n"
                   << "MARKER_TEST:\n";
        } else {
            source << "  if (pix_val - *((uint8_t*) (img_ptr + " << boundary_offsets[0] << ")) <= FIMD_THRESHOLD_DIFF) goto LOOP;\n";
        }

        for (std::size_t index = 1; index < boundary_offsets.size(); ++index) {
            source << "  if (pix_val - *((uint8_t*) (img_ptr + " << boundary_offsets[index] << ")) <= FIMD_THRESHOLD_DIFF) goto LOOP;\n";
        }

        source << "  uint8_t peak = 0;\n"
               << "  uintptr_t peak_ptr = 0;\n"
               << "  uint8_t* curr_int_ptr = 0;\n";
        for (int offset : interior_offsets) {
            source << "  curr_int_ptr = (uint8_t*) (img_ptr + " << offset << ");\n"
                   << "  if (*curr_int_ptr > peak) { peak = *curr_int_ptr; peak_ptr = (uintptr_t) curr_int_ptr; }\n"
                   << "  *curr_int_ptr = 0x00;\n";
        }

        source << "  markers[*markers_num] = peak_ptr;\n"
               << "  (*markers_num)++;\n"
               << "  if (*markers_num == FIMD_MAX_MARKERS_COUNT) *((uint16_t*) (img_ptr + FIMD_OFFSET)) = FIMD_TERM_SEQ;\n"
               << "  goto LOOP;\n"
               << "}\n";

        return source.str();
    }

    bool compileModule(const std::filesystem::path& source_path, const std::filesystem::path& output_path)
    {
        const std::string command = "cc -shared -fPIC -O3 -DNDEBUG -march=native -mtune=native -fomit-frame-pointer -fno-semantic-interposition -o " + output_path.string() + " " + source_path.string();
        return std::system(command.c_str()) == 0;
    }

} // namespace

struct GeneratedFimdCpuKernel::Impl {
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

        if (!std::filesystem::exists(library_path)) {
            std::ofstream output(source_path);
            output << generateSource(*module, threshold_center, threshold_diff, threshold_sun, max_markers_count, max_sun_points_count, detect_sun_points);
            output.close();
            generated_ready = compileModule(source_path, library_path);
        } else {
            generated_ready = true;
        }

        if (generated_ready) {
            handle = dlopen(library_path.c_str(), RTLD_NOW | RTLD_LOCAL);
            if (handle != nullptr) {
                generated_detect = reinterpret_cast<GeneratedDetectFn>(dlsym(handle, "uvdar_generated_detect"));
            }
            generated_ready = (handle != nullptr && generated_detect != nullptr);
        }
    }

    ~Impl()
    {
        if (handle != nullptr) {
            dlclose(handle);
        }
        if (frame != nullptr) {
            std::free(frame);
        }
    }

    unsigned detectRaw(
        const unsigned char* image,
        unsigned (*markers)[2],
        unsigned* markers_count,
        unsigned (*sun_points)[2],
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
            *markers_count = raw_markers_count;
        }
        if (sun_points_count != nullptr) {
            *sun_points_count = detect_sun_points ? raw_sun_points_count : 0;
        }

        for (std::uint32_t index = 0; index < raw_markers_count; ++index) {
            const std::size_t pos1d = marker_ptrs[index] - reinterpret_cast<uintptr_t>(working_image);
            if (markers != nullptr) {
                markers[index][0] = static_cast<unsigned>(pos1d % module->image_width());
                markers[index][1] = static_cast<unsigned>(pos1d / module->image_width());
            }
        }
        for (std::uint32_t index = 0; index < raw_sun_points_count; ++index) {
            const std::size_t pos1d = sun_ptrs[index] - reinterpret_cast<uintptr_t>(working_image);
            if (sun_points != nullptr) {
                sun_points[index][0] = static_cast<unsigned>(pos1d % module->image_width());
                sun_points[index][1] = static_cast<unsigned>(pos1d / module->image_width());
            }
        }

        return raw_markers_count + raw_sun_points_count;
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
    std::vector<uintptr_t> marker_ptrs;
    std::vector<uintptr_t> sun_ptrs;
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

GeneratedFimdCpuKernel::~GeneratedFimdCpuKernel() = default;

unsigned GeneratedFimdCpuKernel::detectRaw(
    const unsigned char* image,
    unsigned (*markers)[2],
    unsigned* markers_count,
    unsigned (*sun_points)[2],
    unsigned* sun_points_count,
    bool make_copy)
{
    return impl_->detectRaw(image, markers, markers_count, sun_points, sun_points_count, make_copy);
}

bool GeneratedFimdCpuKernel::isUsingGeneratedPath() const
{
    return impl_->generated_ready;
}

unsigned GeneratedFimdCpuKernel::get_max_markers_count() const
{
    return impl_->max_markers_count;
}

unsigned GeneratedFimdCpuKernel::get_max_sun_points_count() const
{
    return impl_->max_sun_points_count;
}

} // namespace uvdar_core::detection::fimd
