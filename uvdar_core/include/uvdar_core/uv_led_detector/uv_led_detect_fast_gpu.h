#pragma once

#include <fstream>
#include <filesystem>
#include <stdexcept>

#include <kompute/Kompute.hpp>
#include <vulkan/vulkan.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <uvdar_core/uv_led_detector/uv_led_detect_fast_base.h>

namespace uvdar {

class UvdarLedDetectFastGpu : public UvLedDetectFastBase {
 public:
  explicit UvdarLedDetectFastGpu(UvLedDetectConfig cfg, ILogger& logger, kp::Manager& gpu_manager);
  bool processImage(const cv::Mat image, std::vector<cv::Point2i>& detected_points,
                    std::vector<cv::Point2i>& sun_points, int mask_id = -1) override;
  bool initDelayed(const cv::Mat image) override;

 private:
  const std::string eval_fast_ring_shader_;
  kp::Manager& gpu_mgr_;
};

} // namespace uvdar
