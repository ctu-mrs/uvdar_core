#ifndef UV_LED_FAST_H
#define UV_LED_FAST_H

#include <opencv2/core/core.hpp>
#include <memory>
/* #include <opencv2/features2d/features2d.hpp> */
/* #include <opencv2/video/tracking.hpp> */

namespace uvdar {

  /**
   * @brief The interface class for retrieving bright concentrated points from image, expected to represent markers
   */
  class UVDARLedDetectFAST {
    public:

      /**
       * @brief The constructor of the class
       *
       * @param i_gui If true, on-line visualization of the detected markers will be provided - DO NOT use this if no monitor is attached, doing so should result in program crash
       * @param i_debug If true, debugging outputs will be sent to the console
       * @param i_threshold The threshold for even considering a pixel for a FAST test
       * @param i_threshold_diff The threshold difference between a bright point and its surroundings used in selecting pixels representing the markers
       * @param i_threshold_sun The threshold for even considering a pixel to be a part of the sun
       * @param i_masks Vector of images of the size of the input stream image - pixels of the input images at positions where the mask has the value 0 will be discarded. This is useful for eliminating markers on the body of the observer or for masking out reflective parts of its body 
       */
      UVDARLedDetectFAST(bool i_gui, bool i_debug, int i_threshold, int i_threshold_diff, int i_threshold_sun, std::vector<cv::Mat> i_masks) : _debug_(i_debug), _gui_(i_gui), _threshold_(i_threshold), _threshold_diff_(i_threshold_diff), _threshold_sun_(i_threshold_sun)
      {
          if (_debug_) {
            std::cout << "[UVDARDetectorFAST]: Threshold: " << _threshold_ << std::endl;
            std::cout << "[UVDARDetectorFAST]: Threshold for difference: " << _threshold_diff_ << std::endl;
            std::cout << "[UVDARDetectorFAST]: Threshold for usn: " << _threshold_sun_ << std::endl;
          }
          
          for (auto mask : i_masks) {
            addMask(mask);
          }
      };

      /**
       * @brief Adds an image matrix used for masking out portions of the input stream
       *
       * @param i_mask Image of the size of the input stream image - pixels of the input images at positions where the mask has the value 0 will be discarded. This is useful for eliminating markers on the body of the observer or for masking out reflective parts of its body 
       */
      void addMask(cv::Mat i_mask)
      {
         masks_.push_back(i_mask);
      }

      /**
       * @brief Retrieves bright, concentrated points from the input images
       *        Must be overriden by inheriting class
       *
       * @param i_image The input image
       * @param detected_points The retrieved bright points
       * @param sun_points Points presumed to correspond with directly observed sun in the image
       * @param mask_id The index of the mask (previously added) to use for discarding sections of the input image
       *
       * @return 
       */
      virtual bool processImage(const cv::Mat i_image, std::vector<cv::Point2i>& detected_points, std::vector<cv::Point2i>& sun_points, int mask_id=-1) = 0;
    
      /**
       * @brief Arbitrary initialization procedures that happen outside of the constructor at a later time, after the first image is retrieved to use for parameters
       *        Must be overriden by inheriting class
       *
       */
      virtual bool initDelayed(const cv::Mat i_image) = 0;

    protected:
      bool _debug_;
      bool _gui_;
      unsigned char _threshold_;
      unsigned char _threshold_diff_;
      unsigned char _threshold_sun_;

      std::vector<cv::Mat> masks_;
  };
}


#endif  // UV_LED_FAST_H
