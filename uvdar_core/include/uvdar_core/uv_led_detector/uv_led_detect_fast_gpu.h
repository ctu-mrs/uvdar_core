#pragma once

#include <fstream>
#include <kompute/Kompute.hpp>

#include <uvdar_core/uv_led_detector/uv_led_detect_fast_base.h>

namespace uvdar {

class UvdarLedDetectFastGpu : public UvLedDetectFastBase {
 public:
  explicit UvdarLedDetectFastGpu(UvLedDetectConfig cfg, ILogger& logger);
  bool processImage(const cv::Mat image, std::vector<cv::Point2i>& detected_points,
                    std::vector<cv::Point2i>& sun_points, int mask_id = -1) override;
  bool initDelayed(const cv::Mat image) override;

 private:
  kp::Manager mgr_;
};

} // namespace uvdar
