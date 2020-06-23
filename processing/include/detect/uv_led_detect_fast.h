#ifndef UV_LED_FAST_H
#define UV_LED_FAST_H

#include <opencv2/core/core.hpp>
#include <memory>
/* #include <opencv2/features2d/features2d.hpp> */
/* #include <opencv2/video/tracking.hpp> */

namespace uvdar {

  /**
   * @brief The class for retrieving bright concentrated points from image, expected to represent markers
   */
  class UVDARLedDetectFAST {
    public:

      /**
       * @brief The constructor of the class
       *
       * @param i_gui If true, on-line visualization of the detected markers will be provided - DO NOT use this if no monitor is attached, doing so should result in program crash
       * @param i_debug If true, debugging outputs will be sent to the console
       * @param i_threshold The threshold difference between a bright point and its surroundings used in selecting pixels representing the markers
       * @param i_masks Vector of images of the size of the input stream image - pixels of the input images at positions where the mask has the value 0 will be discarded. This is useful for eliminating markers on the body of the observer or for masking out reflective parts of its body 
       */
      UVDARLedDetectFAST(bool i_gui, bool i_debug, int i_threshold, std::vector<cv::Mat> i_masks);

      /**
       * @brief Adds an image matrix used for masking out portions of the input stream
       *
       * @param i_mask Image of the size of the input stream image - pixels of the input images at positions where the mask has the value 0 will be discarded. This is useful for eliminating markers on the body of the observer or for masking out reflective parts of its body 
       */
      void addMask(cv::Mat i_mask);

      /**
       * @brief Retrieves bright, concentrated points from the input images
       *
       * @param i_image The input image
       * @param detected_points The retrieved bright points
       * @param sun_points Points presumed to correspond with directly observed sun in the image
       * @param mask_id The index of the mask (previously added) to use for discarding sections of the input image
       *
       * @return 
       */
      bool processImage(const cv::Mat i_image, std::vector<cv::Point2i>& detected_points, std::vector<cv::Point2i>& sun_points, int mask_id=-1);

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

      bool _debug_;
      bool _gui_;
      unsigned char _threshold_;
      int step_in_period_ = 0;

      std::vector<cv::Mat> masks_;

  };
}


#endif  // UV_LED_FAST_H
