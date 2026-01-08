#include <uvdar_core/uv_led_detector/uv_led_detect_fast_cpu.h>

namespace uvdar {

/* UvdarLedDetectFastCPU constructor //{ */
UvdarLedDetectFastCPU::UvdarLedDetectFastCPU(UvLedDetectConfig cfg, ILogger& logger)
    : UvdarLedDetectFastBase(std::move(cfg), logger) {
  initFast_();
}
//}

/* clearMarks_() //{ */
void UvdarLedDetectFastCPU::clearMarks_() {
}
//}

/* initFast_ //{ */
void UvdarLedDetectFastCPU::initFast_() {
  initFastPointsSet_();
  initFastInteriorSet_();
}
//}

/* initFastPointsSet_ //{ */
void UvdarLedDetectFastCPU::initFastPointsSet_() {
  fast_points_set_.clear();

  // clang-format off
  fast_points_set_.push_back({
    {0, -3}, {0,  3}, { 3, 0}, {-3, 0},
    {2, -2}, {-2, 2}, {-2,-2}, { 2, 2},
    {-1,-3}, {1,  3}, { 3,-1}, {-3, 1},
    {1, -3}, {-1, 3}, { 3, 1}, {-3,-1},
  });

  fast_points_set_.push_back({
    {0, -4}, {0,  4}, { 4, 0}, {-4, 0},
    {3, -3}, {-3, 3}, {-3,-3}, { 3, 3},
    {-1,-4}, {1,  4}, { 4,-1}, {-4, 1},
    {1, -4}, {-1, 4}, { 4, 1}, {-4,-1},
    {-2,-4}, {2,  4}, { 4,-2}, {-4, 2},
    {2, -4}, {-2, 4}, { 4, 2}, {-4,-2},
  });
  // clang-format on
}
//}

/* initFastInteriorSet_ //{ */
void UvdarLedDetectFastCPU::initFastInteriorSet_() {
  fast_interior_set_.clear();

  // clang-format off
  fast_interior_set_.push_back({
    {0,0},  {1,0},  {2,0},
    {-2,1}, {-1,1},
    {0,1},  {1,1},  {2,1},
    {-1,2},
    {0,2},  {1,2},
  });

  fast_interior_set_.push_back({
    {0,0},  {1,0},  {2,0},  {3,0},
    {-3,1}, {-2,1}, {-1,1},
    {0,1},  {1,1},  {2,1},  {3,1},
    {-3,2}, {-2,2}, {-1,2},
    {0,2},  {1,2},  {2,2},  {3,2},
    {-2,3}, {-1,3},
    {0,3},  {1,3},  {2,3},
  });
  // clang-format on
}
//}

/* processImage //{ */
bool UvdarLedDetectFastCPU::processImage(const cv::Mat image, std::vector<cv::Point2i>& detected_points,
                                         std::vector<cv::Point2i>& sun_points, int mask_id) {
  detected_points = std::vector<cv::Point2i>();
  image_curr_     = image;

  if (mask_id >= 0) {
    if (mask_id >= static_cast<int>(cfg_.masks.size())) {
      logger_.error("[UVDARDetectorFASTCPU]: Mask index " + std::to_string(mask_id) +
                    " is greater than the current number of loaded masks!");
      return false;
    }
    if (image_curr_.size() != cfg_.masks[mask_id].size()) {
      logger_.error("[UVDARDetectorFASTCPU]: The size of the selected mask does not match the current image!");
      return false;
    }
  }

  return true;
}
//}

} // namespace uvdar
