#include "uv_led_detect_fast.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <dirent.h>
#include <algorithm>
#include <iostream>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#define maxCornersPerBlock 96
#define invalidFlow -5555
#define enableBlankBG false
#define maxPassedPoints 2000
#define maxConsideredWindows 4
#define windowAvgMin 10
#define windowExtendedShell 20
#define simpleDisplay true
#define maxWindowNumber 3
#define vanishTime 2.0
#define vanishArea 40000.0
#define maxContours 30
#define maxConnectionDist 50
#define minConnectionSimilarity 10

#define index2d(X, Y) (image_curr_.cols * (Y) + (X))

uvLedDetect_fast::uvLedDetect_fast(bool i_gui, bool i_debug, int i_threshold) {
  _debug_                             = i_debug;
  _gui_                               = i_gui;
  _threshold_                         = i_threshold;

  if (_debug_)
    std::cout << "[UVDARDetectorFAST]: Threshold: " << _threshold_ << std::endl;
  initFAST();
  return;
}

void uvLedDetect_fast::addMask(cv::Mat i_mask){
  masks_.push_back(i_mask);
}

bool uvLedDetect_fast::processImage(const cv::Mat i_image, std::vector<cv::Point2i>& detected_points, std::vector<cv::Point2i>& sun_points, int mask_id) {
  detected_points = std::vector< cv::Point2i >();
  image_curr_=i_image;

  if (mask_id >= 0){
    if (image_curr_.size() != masks_[mask_id].size()){
      std::cerr << "[UVDARDetectorFAST]: The size of the selected mask does not match the current image!" << std::endl;
      return false;
    }
    if (mask_id >= (int)(masks_.size())){
      std::cerr << "[UVDARDetectorFAST]: Mask index " << mask_id << " is greater than the current number of loaded masks!" << std::endl;
      return false;
    }
  }

  if (_gui_){
    (image_curr_).copyTo(image_view_);
  }
   
  if (first_) {
    first_ = false;
    roi_     = cv::Rect(cv::Point(0, 0), image_curr_.size());
    image_check_ = cv::Mat(image_curr_.size(), CV_8UC1);
    image_check_ = cv::Scalar(0);
  }
  /* std::cout << "hey" << std::endl; */
  /* end         = std::clock(); */
  /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
  /* std::cout << "2: " << elapsedTime << " s" << std::endl; */

  /* begin = std::clock(); */
  clearMarks();

  /* end         = std::clock(); */
  /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
  /* std::cout << "3: " << elapsedTime << " s" << std::endl; */

  cv::Point peakPoint;

  int x,y;
  /* begin = std::clock(); */
  bool          test;
  unsigned char maximumVal = 0;
  /* bool          gotOne     = false; */
  bool sunPointPotential = false;
  /* std::vector<cv::Point> sun_points; */
  for (int j = 0; j < image_curr_.rows; j++) {
    for (int i = 0; i < image_curr_.cols; i++) {
      if (mask_id >= 0){
        if (masks_[mask_id].data[index2d(i, j)] == 0){
          continue;
        }
      }
      if (image_check_.data[index2d(i, j)] == 0) {
        if (image_curr_.data[index2d(i, j)] > _threshold_) {
          int sunTestPoints = 0;
          if (image_curr_.data[index2d(i, j)] > (_threshold_*2)) {
            sunPointPotential = true;
          }
          /* gotOne = true; */
          test   = true;
          for (int m = 0; m < (int)(fast_points_.size()); m++) {
            x = i + fast_points_[m].x;
            if (x < 0) {
              test = false;
              break;
            }
            if (x >= roi_.width) {
              test = false;
              break;
            }

            y = j + fast_points_[m].y;
            if (y < 0) {
              test = false;
              break;
            }
            if (y >= roi_.height) {
              test = false;
              break;
            }

            if (image_check_.data[index2d(x,y)] == 255){
              test =false;
              break;
            }


            if ((image_curr_.data[index2d(i, j)] - image_curr_.data[index2d(x, y)]) < (_threshold_/2)) {
              /* std::cout << "BREACH" << std::endl; */

              test = false;
              if (!sunPointPotential)
                break;
              else sunTestPoints++;
            }
            else 
              sunPointPotential = false;
            /* std::cout << (int)(image_curr_.at< unsigned char >(j, i) - image_curr_.at< unsigned char >(y, x)) << std::endl; */
          }
          /* std::cout << "here: " << x << ":" << y << std::endl; */
          if (test) {
            maximumVal = 0;
            for (int m = 0; m < (int)(fast_interior_.size()); m++) {
            /* for (int m = 0; m < 1; m++) { */
              x = i + fast_interior_[m].x;
              if (x < 0) {
                continue;
              }
              if (x >= roi_.width) {
                continue;
              }

              y = j + fast_interior_[m].y;
              if (y < 0) {
                continue;
              }
              if (y >= roi_.height) {
                continue;
              }
              /* std::cout << "here: " << x << ":" << y << std::endl; */

              if (image_check_.data[index2d(x, y)] == 0) {
                if (image_curr_.data[index2d(x, y)] > maximumVal) {
                  maximumVal  = image_curr_.data[index2d(x, y)];
                  peakPoint.x = x;
                  peakPoint.y = y;
                }
                image_check_.data[index2d(x, y)] = 255;
              }
            }
            detected_points.push_back(peakPoint);
          }
          else{
            if (sunPointPotential)
              if (sunTestPoints == (int)(fast_points_.size()))
                sun_points.push_back(cv::Point(i,j));
          }

        }
      }
    }
  }
  /* end         = std::clock(); */
  /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
  /* std::cout << "5: " << elapsedTime << " s" << std::endl; */

  /* begin = std::clock(); */


  if (_gui_ && (step_in_period_ == 75)) {
    for (int i = 0; i < (int)(detected_points.size()); i++) {
      cv::circle(image_view_, detected_points[i], 5, cv::Scalar(200));
    }
    cv::imshow("ocv_uvdar_detector_fast", image_view_);
    step_in_period_ = 0;
    cv::waitKey(0);
  }
  step_in_period_++;

  /* end         = std::clock(); */
  /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
  /* std::cout << "1: " << elapsedTime << " s" << "f: " << 1.0/elapsedTime << std::endl; */

  /* begin = std::clock(); */
  /* if ((detected_points.size() == 0) && (gotOne)) */
  /*   std::cout << "@2" <<std::endl; */
  /* else if (detected_points.size() == 0) */
  /*   std::cout << "@0" <<std::endl; */
  /* else */
  /*   std::cout << "@1" <<std::endl; */

  for (int i=0; i<(int)(detected_points.size()); i++){
    for (int j=0; j<(int)(sun_points.size()); j++){
      if (cv::norm(detected_points[i]-sun_points[j])<50){
        detected_points.erase(detected_points.begin()+i);
        i--;
        break;
      }
    }
  }

  return true;
}

void uvLedDetect_fast::clearMarks() {
  for (int j = 0; j < image_curr_.rows; j++) {
    for (int i = 0; i < image_curr_.cols; i++) {
      if (image_check_.at< unsigned char >(j, i) == 255) {
        image_check_.at< unsigned char >(j, i) = 0;
      }
    }
  }
}


void uvLedDetect_fast::initFAST() {
  /* fast_points_.clear(); */

  /* fast_points_.push_back(cv::Point(0, -3)); */
  /* fast_points_.push_back(cv::Point(0, 3)); */
  /* fast_points_.push_back(cv::Point(3, 0)); */
  /* fast_points_.push_back(cv::Point(-3, 0)); */

  /* fast_points_.push_back(cv::Point(2, -2)); */
  /* fast_points_.push_back(cv::Point(-2, 2)); */
  /* fast_points_.push_back(cv::Point(-2, -2)); */
  /* fast_points_.push_back(cv::Point(2, 2)); */

  /* fast_points_.push_back(cv::Point(-1, -3)); */
  /* fast_points_.push_back(cv::Point(1, 3)); */
  /* fast_points_.push_back(cv::Point(3, -1)); */
  /* fast_points_.push_back(cv::Point(-3, 1)); */

  /* fast_points_.push_back(cv::Point(1, -3)); */
  /* fast_points_.push_back(cv::Point(-1, 3)); */
  /* fast_points_.push_back(cv::Point(3, 1)); */
  /* fast_points_.push_back(cv::Point(-3, -1)); */

  /* fast_interior_.clear(); */

  /* /1* fast_interior_.push_back(cv::Point(-1, -2)); *1/ */
  /* /1* fast_interior_.push_back(cv::Point(0, -2)); *1/ */
  /* /1* fast_interior_.push_back(cv::Point(1, -2)); *1/ */

  /* /1* fast_interior_.push_back(cv::Point(-2, -1)); *1/ */
  /* /1* fast_interior_.push_back(cv::Point(-1, -1)); *1/ */
  /* /1* fast_interior_.push_back(cv::Point(0, -1)); *1/ */
  /* /1* fast_interior_.push_back(cv::Point(1, -1)); *1/ */
  /* /1* fast_interior_.push_back(cv::Point(2, -1)); *1/ */

  /* /1* fast_interior_.push_back(cv::Point(-2, 0)); *1/ */
  /* /1* fast_interior_.push_back(cv::Point(-1, 0)); *1/ */
  /* fast_interior_.push_back(cv::Point(0, 0)); */
  /* fast_interior_.push_back(cv::Point(1, 0)); */
  /* fast_interior_.push_back(cv::Point(2, 0)); */

  /* /1* fast_interior_.push_back(cv::Point(-2, 1)); *1/ */
  /* /1* fast_interior_.push_back(cv::Point(-1, 1)); *1/ */
  /* fast_interior_.push_back(cv::Point(0, 1)); */
  /* fast_interior_.push_back(cv::Point(1, 1)); */
  /* fast_interior_.push_back(cv::Point(2, 1)); */

  /* /1* fast_interior_.push_back(cv::Point(-1, 2)); *1/ */
  /* fast_interior_.push_back(cv::Point(0, 2)); */
  /* fast_interior_.push_back(cv::Point(1, 2)); */
  fast_points_.clear();

  fast_points_.push_back(cv::Point(0, -4));
  fast_points_.push_back(cv::Point(0, 4));
  fast_points_.push_back(cv::Point(4, 0));
  fast_points_.push_back(cv::Point(-4, 0));

  fast_points_.push_back(cv::Point(3, -3));
  fast_points_.push_back(cv::Point(-3, 3));
  fast_points_.push_back(cv::Point(-3, -3));
  fast_points_.push_back(cv::Point(3, 3));

  fast_points_.push_back(cv::Point(-1, -4));
  fast_points_.push_back(cv::Point(1, 4));
  fast_points_.push_back(cv::Point(4, -1));
  fast_points_.push_back(cv::Point(-4, 1));

  fast_points_.push_back(cv::Point(1, -4));
  fast_points_.push_back(cv::Point(-1, 4));
  fast_points_.push_back(cv::Point(4, 1));
  fast_points_.push_back(cv::Point(-4, -1));

  fast_points_.push_back(cv::Point(-2, -4));
  fast_points_.push_back(cv::Point(2, 4));
  fast_points_.push_back(cv::Point(4, -2));
  fast_points_.push_back(cv::Point(-4, 2));

  fast_points_.push_back(cv::Point(2, -4));
  fast_points_.push_back(cv::Point(-2, 4));
  fast_points_.push_back(cv::Point(4, 2));
  fast_points_.push_back(cv::Point(-4, -2));

  fast_interior_.clear();

  /* fast_interior_.push_back(cv::Point(-1, -2)); */
  /* fast_interior_.push_back(cv::Point(0, -2)); */
  /* fast_interior_.push_back(cv::Point(1, -2)); */

  /* fast_interior_.push_back(cv::Point(-2, -1)); */
  /* fast_interior_.push_back(cv::Point(-1, -1)); */
  /* fast_interior_.push_back(cv::Point(0, -1)); */
  /* fast_interior_.push_back(cv::Point(1, -1)); */
  /* fast_interior_.push_back(cv::Point(2, -1)); */

  /* fast_interior_.push_back(cv::Point(-2, 0)); */
  /* fast_interior_.push_back(cv::Point(-1, 0)); */
  fast_interior_.push_back(cv::Point(0, 0));
  fast_interior_.push_back(cv::Point(1, 0));
  fast_interior_.push_back(cv::Point(2, 0));
  fast_interior_.push_back(cv::Point(3, 0));

  /* fast_interior_.push_back(cv::Point(-2, 1)); */
  /* fast_interior_.push_back(cv::Point(-1, 1)); */
  fast_interior_.push_back(cv::Point(0, 1));
  fast_interior_.push_back(cv::Point(1, 1));
  fast_interior_.push_back(cv::Point(2, 1));
  fast_interior_.push_back(cv::Point(3, 1));

  fast_interior_.push_back(cv::Point(0, 2));
  fast_interior_.push_back(cv::Point(1, 2));
  fast_interior_.push_back(cv::Point(2, 2));
  fast_interior_.push_back(cv::Point(3, 2));

  /* fast_interior_.push_back(cv::Point(-1, 2)); */
  fast_interior_.push_back(cv::Point(0, 3));
  fast_interior_.push_back(cv::Point(1, 3));
  fast_interior_.push_back(cv::Point(2, 3));
  make double pass with smaller and then larger circle
}
