#ifndef UVDD_FAST_H
#define UVDD_FAST_H

#include <opencv2/core/core.hpp>
#include <memory>
/* #include <opencv2/features2d/features2d.hpp> */
/* #include <opencv2/video/tracking.hpp> */

class UVLedDetectFAST {
public:
  UVLedDetectFAST(bool i_gui, bool i_debug, int i_threshold, std::vector<cv::Mat> i_masks);
  void addMask(cv::Mat i_mask);
  bool processImage(const cv::Mat i_image, std::vector<cv::Point2i>& detected_points, std::vector<cv::Point2i>& sun_points, int mask_id=-1);

private:
void clearMarks();
bool miniFAST(cv::Point input, cv::Point &maximum, unsigned char threshold);
  void initFAST();

  /* std::vector< cv::Point > fast_points_; */
  std::vector<std::vector< cv::Point >> fast_points_set_;
  /* std::vector< cv::Point > fast_interior_; */
  std::vector<std::vector< cv::Point >> fast_interior_set_;
  bool initialized_ = false;
  bool                     first_ = true;
  bool                     gotBlinkBefore_;

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


  cv::Mat  m_imShowWindows_;
  int      m_baseWindowSize_;
  cv::Size m_monitorSize_;
  std::vector<cv::Mat> masks_;

};


#endif  // UVDD_FAST_H
