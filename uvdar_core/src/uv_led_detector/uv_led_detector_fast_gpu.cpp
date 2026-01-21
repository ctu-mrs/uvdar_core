#include <uvdar_core/uv_led_detector/uv_led_detect_fast_gpu.h>

namespace uvdar {

/* ParseShader //{ */
static std::string ParseShader(const std::string& filepath) {
  std::ifstream stream(filepath);

  std::string line;
  std::stringstream ss;
  while (getline(stream, line)) {
    ss << line << "\n"; // TODO: maybe "\n" is not needed
  }

  ss.str();
}
//}

/* UvdarLedDetectFastCpu constructor //{ */
UvdarLedDetectFastGpu::UvdarLedDetectFastGpu(UvLedDetectConfig cfg, ILogger& logger)
    : UvLedDetectFastBase(std::move(cfg), logger) {
  //   initFast_();
}
//}

/* processImage //{ */
bool UvdarLedDetectFastGpu::processImage(const cv::Mat image, std::vector<cv::Point2i>& detected_points,
                                         std::vector<cv::Point2i>& sun_points, int mask_id) {
}
//}

/* initDelayed //{ */
bool UvdarLedDetectFastGpu::initDelayed(const cv::Mat image) {
}
//}

} // namespace uvdar
