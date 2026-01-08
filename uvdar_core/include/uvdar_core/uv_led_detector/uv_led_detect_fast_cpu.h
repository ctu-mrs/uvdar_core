#pragma once

#include <uvdar_core/uv_led_detector/uv_led_detect_fast_base.h>

namespace uvdar {

class UvdarLedDetectFastCPU : public UvdarLedDetectFastBase {
 public:
  explicit UvdarLedDetectFastCPU(UvLedDetectConfig cfg, ILogger& logger);
  bool processImage(const cv::Mat image, std::vector<cv::Point2i>& detected_points,
                    std::vector<cv::Point2i>& sun_points, int mask_id = -1) override;
  bool initDelayed(const cv::Mat image) override;

 private:
  /**
   * @brief Resets a helper matrix used for suppression of clustered bright pixels
   */
  void clearMarks_();

  /**
   * @brief Initializes points used in FAST-like bright point detection
   */
  void initFast_();

  void initFastPointsSet_();
  void initFastInteriorSet_();

 private:
  std::vector<std::vector<cv::Point>> fast_points_set_;
  std::vector<std::vector<cv::Point>> fast_interior_set_;
  bool initialized_{false};
  bool first_{true};

  bool lines_;
  int accumLength_;

  cv::Mat image_curr_;
  cv::Mat image_check_;
  cv::Mat image_view_;
  cv::Rect roi_;

  int step_in_period_{0};
};

} // namespace uvdar
