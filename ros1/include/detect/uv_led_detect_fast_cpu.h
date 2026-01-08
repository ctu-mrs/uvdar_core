#ifndef UV_LED_FAST_CPU_H
#define UV_LED_FAST_CPU_H

#include "uv_led_detect_fast.h"

namespace uvdar {

  class UVDARLedDetectFASTCPU : public UVDARLedDetectFAST {
    public:
      UVDARLedDetectFASTCPU(bool i_gui, bool i_debug, int i_threshold, int i_threshold_diff, int i_threshold_sun, std::vector<cv::Mat> i_masks);
      bool processImage(const cv::Mat i_image, std::vector<cv::Point2i>& detected_points, std::vector<cv::Point2i>& sun_points, int mask_id=-1);
      bool initDelayed(const cv::Mat i_image);

    private:

      /**
       * @brief Resets a helper matrix used for suppression of clustered bright pixels
       */
      void clearMarks();

      /**
       * @brief Initializes points used in FAST-like bright point detection
       */
      void initFAST();

      std::vector<std::vector< cv::Point >> fast_points_set_;
      std::vector<std::vector< cv::Point >> fast_interior_set_;
      bool initialized_ = false;
      bool first_ = true;

      bool                   m_lines_;
      int                    m_accumLength_;

      cv::Mat image_curr_;
      cv::Mat image_check_;
      cv::Mat  image_view_;
      cv::Rect roi_;

      int step_in_period_ = 0;

  };
}


#endif  // UV_LED_FAST_CPU_H
