#include <uvdar_core/uv_led_detector/uv_led_detector.h>

namespace uvdar {

/* UvLedDetector constructor //{ */
UvLedDetector::UvLedDetector(ILogger& logger, const UvLedDetectConfig& cfg, kp::Manager& gpu_manager)
    : logger_(logger), cfg_(cfg), gpu_manager_(gpu_manager) {
  detector_ = makeUvLedDetector_();
}
//}

/* UvLedDetector destructor //{ */
UvLedDetector::~UvLedDetector() {
}
//}

/* makeUvLedDetector_ //{ */
std::unique_ptr<UvLedDetectFastBase> UvLedDetector::makeUvLedDetector_() {
  if (cfg_.gpu) {
    logger_.info("[UVDARDetector]: Initializing FAST-based marker detection running on GPU...");
    return std::make_unique<UvdarLedDetectFastGpu>(cfg_, logger_);
  } else {
    logger_.info("[UVDARDetector]: Initializing FAST-based marker detection running on CPU...");
    return std::make_unique<UvdarLedDetectFastCpu>(cfg_, logger_, gpu_manager_);
  }
}
//}

/* detect //{ */
bool UvLedDetector::detect(const cv::Mat i_image, std::vector<cv::Point2i>& detected_points,
                           std::vector<cv::Point2i>& sun_points) {
  return detector_->processImage(i_image, detected_points, sun_points);
}
//}

} // namespace uvdar