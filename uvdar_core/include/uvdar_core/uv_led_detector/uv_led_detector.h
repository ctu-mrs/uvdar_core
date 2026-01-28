#pragma once

#include <uvdar_core/uv_led_detector/uv_led_detect_fast_cpu.h>
#include <uvdar_core/uv_led_detector/uv_led_detect_fast_gpu.h>

namespace uvdar {

class UvLedDetector {
 public:
  UvLedDetector(ILogger& logger, const UvLedDetectConfig& cfg);
  ~UvLedDetector();

  [[nodiscard]] bool detect(const cv::Mat& image, std::vector<cv::Point2i>& detected_points,
                            std::vector<cv::Point2i>& sun_points);

  void initGpuProgram(const cv::Mat& image);

 private:
  std::unique_ptr<UvLedDetectFastBase> makeUvLedDetector_();

 private:
  ILogger& logger_;
  const UvLedDetectConfig cfg_;
  bool ready_to_process_{false};

  std::unique_ptr<UvLedDetectFastBase> detector_;

  std::vector<cv::Mat> images_current_;
  std::vector<std::vector<cv::Point>> detected_points_;
  std::vector<std::vector<cv::Point>> sun_points_;
};

} // namespace uvdar