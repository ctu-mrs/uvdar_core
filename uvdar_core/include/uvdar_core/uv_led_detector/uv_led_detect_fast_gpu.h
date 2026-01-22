#pragma once

#include <fstream>
#include <filesystem>
#include <stdexcept>

#include <kompute/Kompute.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <uvdar_core/uv_led_detector/uv_led_detect_fast_base.h>
#include <uvdar_core/uv_led_detector/gpu_context.h>

namespace uvdar {

class UvdarLedDetectFastGpu : public UvLedDetectFastBase {
 public:
  explicit UvdarLedDetectFastGpu(UvLedDetectConfig cfg, ILogger& logger);
  bool processImage(const cv::Mat image, std::vector<cv::Point2i>& detected_points,
                    std::vector<cv::Point2i>& sun_points, int mask_id = -1) override;
  bool initDelayed(const cv::Mat image) override;

 private:
  void logGpuProperties_();

 private:
  const std::string eval_fast_ring_shader_;
  GpuContext& gpu_mgr_;
};

} // namespace uvdar
