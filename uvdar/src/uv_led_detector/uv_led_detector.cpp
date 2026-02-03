#include <uvdar/uv_led_detector/uv_led_detector.h>

namespace uvdar {

/* UvLedDetector constructor //{ */
UvLedDetector::UvLedDetector(ILogger& logger, const UvLedDetectConfig& cfg) : logger_(logger), cfg_(cfg) {
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
    ready_to_process_ = false;
    logger_.info("[UVDARDetector]: Initializing FAST-based marker detection running on GPU...");
    return std::make_unique<UvdarLedDetectFastGpu>(cfg_, logger_);
  } else {
    ready_to_process_ = true;
    logger_.info("[UVDARDetector]: Initializing FAST-based marker detection running on CPU...");
    return std::make_unique<UvdarLedDetectFastCpu>(cfg_, logger_);
  }
}
//}

/* detect //{ */
bool UvLedDetector::detect(const cv::Mat& image, std::vector<cv::Point2i>& detected_points,
                           std::vector<cv::Point2i>& sun_points) {
  if (ready_to_process_) {
    return detector_->processImage(image, detected_points, sun_points);
  }
  logger_.warn("[UvLedDetector]: Not ready to detect. The GPU program has not been initialized.");
  return false;
}
//}

/* initGpuProgram //{ */
void UvLedDetector::initGpuProgram(const cv::Mat& image) {
  detector_->initGpuProgram(image);
  ready_to_process_ = true;
}
//}

} // namespace uvdar