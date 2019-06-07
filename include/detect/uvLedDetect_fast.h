#ifndef UVDD_FAST_H
#define UVDD_FAST_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

class uvLedDetect_fast {
public:
  uvLedDetect_fast();

  std::vector< cv::Point2i > processImage(cv::Mat imCurr_t, cv::Mat imView_t, bool i_gui = true, bool i_debug = true,  int threshVal = 200);


  bool initialized;

private:
  /* void markOutInterior(cv::Point input); */
void clearMarks();
bool miniFAST(cv::Point input, cv::Point &maximum, unsigned char threshold);
  void initFAST();

  std::vector< cv::Point > fastPoints;
  std::vector< cv::Point > fastInterior;
  bool                     m_first;
  bool                     m_second;
  bool                     m_gotBlinkBefore;
  char *                   m_kernelSource;

  bool                   m_lines;
  int                    m_accumLength;
  int x,y;

  cv::Mat m_imCurr;
  cv::Mat m_imCheck;
  cv::Mat  m_imView;
  cv::Rect m_roi;

  bool DEBUG;
  bool gui;
  int stepInPeriod;


  cv::Mat  m_imShowWindows;
  int      m_baseWindowSize;
  cv::Size m_monitorSize;

};


#endif  // UVDD_FAST_H
