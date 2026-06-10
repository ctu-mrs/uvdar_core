#pragma once

#include <array>
#include <cstdint>
#include <memory>

#include "uvdar_core/detection/fimd/cpu_detector.hpp"
#include "uvdar_core/detection/fimd/radius_module.hpp"

namespace uvdar_core::detection::fimd {

class GeneratedFimdCpuKernel {
public:
    GeneratedFimdCpuKernel(
        std::shared_ptr<const RuntimeFimdRadiusModule> module,
        unsigned char threshold_center           = 120,
        unsigned char threshold_diff             = 60,
        unsigned char threshold_sun              = 240,
        std::array<unsigned char, 2> termination = { 0xFF, 0x00 },
        unsigned max_markers_count               = 0,
        unsigned max_sun_points_count            = 0,
        bool detect_sun_points                   = true);
    ~GeneratedFimdCpuKernel();

    GeneratedFimdCpuKernel(const GeneratedFimdCpuKernel&)            = delete;
    GeneratedFimdCpuKernel& operator=(const GeneratedFimdCpuKernel&) = delete;

    unsigned detectRaw(
        const unsigned char* image,
        unsigned (*markers)[2],
        unsigned* markers_count,
        unsigned (*sun_points)[2],
        unsigned* sun_points_count,
        bool make_copy = true);

    bool isUsingGeneratedPath() const;
    unsigned get_max_markers_count() const;
    unsigned get_max_sun_points_count() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace uvdar_core::detection::fimd