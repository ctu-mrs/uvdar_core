#pragma once

#include <uvdar_core/uv_led_detector/uv_led_detect_config.h>
#include <uvdar_core/utils/i_logger.h>

namespace uvdar {

inline int index2d(int x, int y, int cols) noexcept {
  return cols * y + x;
}

class UvLedDetectFastBase {
 public:
  virtual ~UvLedDetectFastBase() = default;

  /**
   * @brief Adds an image matrix used for masking out portions of the input stream
   *
   * @param mask Image of the size of the input stream image - pixels of the input images at positions where the mask
   * has the value 0 will be discarded. This is useful for eliminating markers on the body of the observer or for
   * masking out reflective parts of its body
   */
  void addMask(const cv::Mat& mask) {
    cfg_.masks.push_back(mask.clone());
  }

  /**
   * @brief Retrieves bright, concentrated points from the input images
   *        Must be overriden by inheriting class
   *
   * @param image The input image
   * @param detected_points The retrieved bright points
   * @param sun_points Points presumed to correspond with directly observed sun in the image
   * @param mask_id The index of the mask (previously added) to use for discarding sections of the input image
   *
   * @return
   */
  virtual bool processImage(const cv::Mat image, std::vector<cv::Point2i>& detected_points,
                            std::vector<cv::Point2i>& sun_points, int mask_id = -1) = 0;

  /**
   * @brief Arbitrary initialization procedures that happen outside of the constructor at a later time, after the first
   * image is retrieved to use for parameters Must be overriden by inheriting class. Useful only for GPU detector
   *
   */
  virtual void initGpuProgram(const cv::Mat image) = 0;

 protected:
  explicit UvLedDetectFastBase(UvLedDetectConfig cfg, ILogger& logger) : cfg_(std::move(cfg)), logger_(logger) {
  }

 protected:
  UvLedDetectConfig cfg_;
  ILogger& logger_;
};

} // namespace uvdar