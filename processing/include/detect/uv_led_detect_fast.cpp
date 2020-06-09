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

UVLedDetectFAST::UVLedDetectFAST(bool i_gui, bool i_debug, int i_threshold) {
  _debug_                             = i_debug;
  _gui_                               = i_gui;
  _threshold_                         = i_threshold;

  if (_debug_)
    std::cout << "[UVDARDetectorFAST]: Threshold: " << _threshold_ << std::endl;
  initFAST();
  return;
}

void UVLedDetectFAST::addMask(cv::Mat i_mask){
  masks_.push_back(i_mask);
}

bool UVLedDetectFAST::processImage(const cv::Mat i_image, std::vector<cv::Point2i>& detected_points, std::vector<cv::Point2i>& sun_points, int mask_id) {
  detected_points = std::vector< cv::Point2i >();
  image_curr_=i_image;

  if (mask_id >= 0){
    if (mask_id >= (int)(masks_.size())){
      std::cerr << "[UVDARDetectorFAST]: Mask index " << mask_id << " is greater than the current number of loaded masks!" << std::endl;
      return false;
    }
    if (image_curr_.size() != masks_[mask_id].size()){
      std::cerr << "[UVDARDetectorFAST]: The size of the selected mask does not match the current image!" << std::endl;
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
  clearMarks();

  cv::Point peakPoint;

  int x,y;
  /* begin = std::clock(); */
  bool          marker_potential;
  unsigned char maximumVal = 0;
  /* bool          gotOne     = false; */
  bool sun_point_potential = false;
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
            sun_point_potential = true;
          }
          /* gotOne = true; */
          marker_potential   = true;

          int n = -1;
          for (auto fast_points : fast_points_set_){
            n++;
            for (int m = 0; m < (int)(fast_points.size()); m++) {
              x = i + fast_points[m].x;
              if (x < 0) {
                marker_potential = false;
                break;
              }
              if (x >= roi_.width) {
                marker_potential = false;
                break;
              }

              y = j + fast_points[m].y;
              if (y < 0) {
                marker_potential = false;
                break;
              }
              if (y >= roi_.height) {
                marker_potential = false;
                break;
              }

              if (image_check_.data[index2d(x,y)] == 255){
                marker_potential =false;
                break;
              }


              if ((image_curr_.data[index2d(i, j)] - image_curr_.data[index2d(x, y)]) < (_threshold_/2)) {
                /* std::cout << "BREACH" << std::endl; */

                marker_potential = false;
                if (!sun_point_potential)
                  break;
                else sunTestPoints++;
              }
              else 
                sun_point_potential = false;
            }
            if (marker_potential){
              break;
            }
          }
          if (marker_potential) {
            maximumVal = 0;
            for (int m = 0; m < (int)(fast_interior_set_[n].size()); m++) {
            /* for (int m = 0; m < 1; m++) { */
              x = i + fast_interior_set_[n][m].x;
              if (x < 0) {
                continue;
              }
              if (x >= roi_.width) {
                continue;
              }

              y = j + fast_interior_set_[n][m].y;
              if (y < 0) {
                continue;
              }
              if (y >= roi_.height) {
                continue;
              }

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
            if (sun_point_potential)
              if (sunTestPoints == (int)(fast_points_set_[n].size()))
                sun_points.push_back(cv::Point(i,j));
          }

        }
      }
    }
  }

  if (_gui_ && (step_in_period_ == 75)) {
    for (int i = 0; i < (int)(detected_points.size()); i++) {
      cv::circle(image_view_, detected_points[i], 5, cv::Scalar(200));
    }
    cv::imshow("ocv_uvdar_detector_fast", image_view_);
    step_in_period_ = 0;
    cv::waitKey(0);
  }
  step_in_period_++;

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

void UVLedDetectFAST::clearMarks() {
  for (int j = 0; j < image_curr_.rows; j++) {
    for (int i = 0; i < image_curr_.cols; i++) {
      if (image_check_.at< unsigned char >(j, i) == 255) {
        image_check_.at< unsigned char >(j, i) = 0;
      }
    }
  }
}


void UVLedDetectFAST::initFAST() {
  std::vector< cv::Point > fast_points;

  fast_points.push_back(cv::Point(0, -3));
  fast_points.push_back(cv::Point(0, 3));
  fast_points.push_back(cv::Point(3, 0));
  fast_points.push_back(cv::Point(-3, 0));

  fast_points.push_back(cv::Point(2, -2));
  fast_points.push_back(cv::Point(-2, 2));
  fast_points.push_back(cv::Point(-2, -2));
  fast_points.push_back(cv::Point(2, 2));

  fast_points.push_back(cv::Point(-1, -3));
  fast_points.push_back(cv::Point(1, 3));
  fast_points.push_back(cv::Point(3, -1));
  fast_points.push_back(cv::Point(-3, 1));

  fast_points.push_back(cv::Point(1, -3));
  fast_points.push_back(cv::Point(-1, 3));
  fast_points.push_back(cv::Point(3, 1));
  fast_points.push_back(cv::Point(-3, -1));

  fast_points_set_.push_back(fast_points);

  fast_points.clear();

  fast_points.push_back(cv::Point(0, -4));
  fast_points.push_back(cv::Point(0, 4));
  fast_points.push_back(cv::Point(4, 0));
  fast_points.push_back(cv::Point(-4, 0));

  fast_points.push_back(cv::Point(3, -3));
  fast_points.push_back(cv::Point(-3, 3));
  fast_points.push_back(cv::Point(-3, -3));
  fast_points.push_back(cv::Point(3, 3));

  fast_points.push_back(cv::Point(-1, -4));
  fast_points.push_back(cv::Point(1, 4));
  fast_points.push_back(cv::Point(4, -1));
  fast_points.push_back(cv::Point(-4, 1));

  fast_points.push_back(cv::Point(1, -4));
  fast_points.push_back(cv::Point(-1, 4));
  fast_points.push_back(cv::Point(4, 1));
  fast_points.push_back(cv::Point(-4, -1));

  fast_points.push_back(cv::Point(-2, -4));
  fast_points.push_back(cv::Point(2, 4));
  fast_points.push_back(cv::Point(4, -2));
  fast_points.push_back(cv::Point(-4, 2));

  fast_points.push_back(cv::Point(2, -4));
  fast_points.push_back(cv::Point(-2, 4));
  fast_points.push_back(cv::Point(4, 2));
  fast_points.push_back(cv::Point(-4, -2));

  fast_points_set_.push_back(fast_points);


  std::vector< cv::Point > fast_interior;

  fast_interior.push_back(cv::Point(0, 0));
  fast_interior.push_back(cv::Point(1, 0));
  fast_interior.push_back(cv::Point(2, 0));

  fast_interior.push_back(cv::Point(0, 1));
  fast_interior.push_back(cv::Point(1, 1));
  fast_interior.push_back(cv::Point(2, 1));

  fast_interior.push_back(cv::Point(0, 2));
  fast_interior.push_back(cv::Point(1, 2));

  fast_interior_set_.push_back(fast_interior);

  fast_interior.clear();

  fast_interior.push_back(cv::Point(0, 0));
  fast_interior.push_back(cv::Point(1, 0));
  fast_interior.push_back(cv::Point(2, 0));
  fast_interior.push_back(cv::Point(3, 0));


  fast_interior.push_back(cv::Point(0, 1));
  fast_interior.push_back(cv::Point(1, 1));
  fast_interior.push_back(cv::Point(2, 1));
  fast_interior.push_back(cv::Point(3, 1));

  fast_interior.push_back(cv::Point(0, 2));
  fast_interior.push_back(cv::Point(1, 2));
  fast_interior.push_back(cv::Point(2, 2));
  fast_interior.push_back(cv::Point(3, 2));

  fast_interior.push_back(cv::Point(0, 3));
  fast_interior.push_back(cv::Point(1, 3));
  fast_interior.push_back(cv::Point(2, 3));

  fast_interior_set_.push_back(fast_interior);

}
