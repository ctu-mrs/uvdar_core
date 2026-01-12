#include <uvdar_core/uv_led_detector/uv_led_detector.h>

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
    logger_.info("[UVDARDetector]: Initializing FAST-based marker detection running on GPU...");
    logger_.error("[UVDARDetector]: GPU version is not implemented!");
    return std::make_unique<UvdarLedDetectFastCPU>(cfg_, logger_);
  } else {
    logger_.info("[UVDARDetector]: Initializing FAST-based marker detection running on CPU...");
    return std::make_unique<UvdarLedDetectFastCPU>(cfg_, logger_);
  }
}
//}

/* detect //{ */
void UvLedDetector::detect(const cv::Mat i_image) {
  // detector_->processImage(i_image, )
}
//}

} // namespace uvdar