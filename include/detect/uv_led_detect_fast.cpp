#include "uv_led_detect_fast.h"
#include <dirent.h>
#include <algorithm>
#include <iostream>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

//addressing indices this way is noticeably faster than the "propper" way with .at method - numerous unnecessary checks are skipped. This of course means that we have to do necessary checks ourselves
#define index2d(X, Y) (image_curr_.cols * (Y) + (X))

uvdar::UVDARLedDetectFAST::UVDARLedDetectFAST(bool i_gui, bool i_debug, int i_threshold, std::vector<cv::Mat> i_masks) {
  _debug_     = i_debug;
  _gui_       = i_gui;
  _threshold_ = i_threshold;

  if (_debug_)
    std::cout << "[UVDARDetectorFAST]: Threshold: " << _threshold_ << std::endl;
  initFAST();
  for (auto mask : i_masks) {
    addMask(mask);
  }
  return;
}

void uvdar::UVDARLedDetectFAST::addMask(cv::Mat i_mask) {
  masks_.push_back(i_mask);
}

bool uvdar::UVDARLedDetectFAST::processImage(const cv::Mat i_image, std::vector<cv::Point2i>& detected_points, std::vector<cv::Point2i>& sun_points, int mask_id) {
  detected_points = std::vector<cv::Point2i>();
  image_curr_     = i_image;

  if (mask_id >= 0) {
    if (mask_id >= (int)(masks_.size())) {
      std::cerr << "[UVDARDetectorFAST]: Mask index " << mask_id << " is greater than the current number of loaded masks!" << std::endl;
      return false;
    }
    if (image_curr_.size() != masks_[mask_id].size()) {
      std::cerr << "[UVDARDetectorFAST]: The size of the selected mask does not match the current image!" << std::endl;
      return false;
    }
  }

  if (_gui_) {
    (image_curr_).copyTo(image_view_);
  }

  if (first_) {
    first_       = false;
    roi_         = cv::Rect(cv::Point(0, 0), image_curr_.size());
    image_check_ = cv::Mat(image_curr_.size(), CV_8UC1);
    image_check_ = cv::Scalar(0);
  }
  clearMarks();

  cv::Point peak_point;

  int x, y;
  /* begin = std::clock(); */
  bool marker_potential;
  /* bool          gotOne     = false; */
  bool sun_point_potential = false;
  /* std::vector<cv::Point> sun_points; */
  for (int j = 0; j < image_curr_.rows; j++) { for (int i = 0; i < image_curr_.cols; i++) { //iterate over the image points 
    if (mask_id >= 0) {
      if (masks_[mask_id].data[index2d(i, j)] == 0) { //skip over masked out points
        continue;
      }
    }
    if (image_check_.data[index2d(i, j)] == 0) { // skip over marked points (suppresses clustered bright pixels)
      if (image_curr_.data[index2d(i, j)] > _threshold_) { //if the point is bright
        int sun_test_points = 0;
        if (image_curr_.data[index2d(i, j)] > (_threshold_ * 2)) { //if the point is "very bright" it might be a part of the image of directly observed sun
          sun_point_potential = true;
        }

        int n = -1;
        for (auto fast_points : fast_points_set_) { //iterate over the pre-selected sets of points for different sizes of the FAST radius
          marker_potential = true;
          n++;
          /* if (n>0){ */
          /*   std::cout << "HERE A" << std::endl; */
          /* } */
          for (int m = 0; m < (int)(fast_points.size()); m++) { //iterate over the points in the current FAST radius
            x = i + fast_points[m].x;
            y = j + fast_points[m].y;

            //check for image border breach
            if (x < 0) {
              marker_potential = false;
              break;
            }
            if (x >= roi_.width) {
              marker_potential = false;
              break;
            }
            if (y < 0) {
              marker_potential = false;
              break;
            }
            if (y >= roi_.height) {
              marker_potential = false;
              break;
            }

            if ((image_curr_.data[index2d(i, j)] - image_curr_.data[index2d(x, y)]) < (_threshold_ / 2)) { //if the difference between the current point and a surrounding point is smaller than desired
              /* if (n>0){ */
              /*     std::cout << "HERE B: broken at fast point " << m << " by: " << (int)(image_curr_.data[index2d(i, j)] - image_curr_.data[index2d(x, y)]) << std::endl; */
              /* } */

              marker_potential = false; //this is not a marker (not concentrated enough)
              if (!sun_point_potential) //if we expected this to be a part of the sun, can still confirm this hypthesis
                break;
              else
                sun_test_points++;
            }
            else { //if the difference is small, this is likely not a part of the sun, as the point is too concentrated (sun usually saturates bigger area in the image than our FAST neighborhood)
              sun_point_potential = false;
            }
          }
          /* if (n>0){ */
          /*   std::cout << "HERE E: here? " << std::endl; */
          /* } */
          if (marker_potential) { //if the smaller radius check determines that this point is a marker, the larger radius is unnecessary
              /* std::cout << "HERE C: passed" << std::endl; */
            /* if (n>0){ */
              /* std::cout << "HERE D: passed on second ring" << std::endl; */
            /* } */
            break;
          }
        }
        /* if (n>0){ */
        /*   /1* if (marker_potential){ *1/ */
        /*   /1*   std::cout << "HERE B" << std::endl; *1/ */
        /*   /1* } *1/ */
        /* } */
        unsigned char maximum_val;
        if (marker_potential) {
          maximum_val = 0;
          n = (int)(fast_interior_set_.size())-1;
          for (int m = 0; m < (int)(fast_interior_set_[n].size()); m++) { //iterate over a subset of points inside of the FAST neighborhood (lower right corner only, due to iterating over the image in this direction)
            x = i + fast_interior_set_[n][m].x;
            y = j + fast_interior_set_[n][m].y;

            //check for image border breach
            if (x < 0) {
              continue;
            }
            if (x >= roi_.width) {
              continue;
            }
            if (y < 0) {
              continue;
            }
            if (y >= roi_.height) {
              continue;
            }

            if (image_check_.data[index2d(x, y)] == 0) {
              if (image_curr_.data[index2d(x, y)] > maximum_val) { //non-maxima suppression - select the brightest point inside the FAST neighborhood
                maximum_val = image_curr_.data[index2d(x, y)];
                peak_point.x = x;
                peak_point.y = y;
              }
              image_check_.data[index2d(x, y)] = 255; //mark interior point to prevent additional detections in the same area
            /* std::cout << "Setting point " << x << ":" << y << " as checked" << std::endl; */
            }
          }
          detected_points.push_back(peak_point); //store detected marker point
            /* std::cout << "Outputting point " << peak_point.x << ":" << peak_point.y << " as checked" << std::endl; */
        } else {
          if (sun_point_potential) 
            if (sun_test_points == (int)(fast_points_set_[n].size())){ //declare this pixel a part of the image of the sun if even its FAST neighborhood was bright
              sun_points.push_back(cv::Point(i, j));
            }
        }
      }
    }
  } }

  for (int i = 0; i < (int)(detected_points.size()); i++) { //iterate over the detected marker points
    for (int j = 0; j < (int)(sun_points.size()); j++) { //iterate over the detected sun points
      if (cv::norm(detected_points[i] - sun_points[j]) < 50) { //if the current detected marker point is close to the sun, it might be merely glare, so we discard it rather than to have numerous false detections here
        detected_points.erase(detected_points.begin() + i);
        i--;
        break;
      }
    }
  }

  return true;
}

void uvdar::UVDARLedDetectFAST::clearMarks() {
  for (int j = 0; j < image_curr_.rows; j++) {
    for (int i = 0; i < image_curr_.cols; i++) {
      if (image_check_.at<unsigned char>(j, i) == 255) {
        image_check_.at<unsigned char>(j, i) = 0;
      }
    }
  }
}


void uvdar::UVDARLedDetectFAST::initFAST() {
  std::vector<cv::Point> fast_points;

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


  std::vector<cv::Point> fast_interior;

  fast_interior.push_back(cv::Point(0, 0));
  fast_interior.push_back(cv::Point(1, 0));
  fast_interior.push_back(cv::Point(2, 0));

  fast_interior.push_back(cv::Point(-2, 1));
  fast_interior.push_back(cv::Point(-1, 1));

  fast_interior.push_back(cv::Point(0, 1));
  fast_interior.push_back(cv::Point(1, 1));
  fast_interior.push_back(cv::Point(2, 1));

  fast_interior.push_back(cv::Point(-1, 1));

  fast_interior.push_back(cv::Point(0, 2));
  fast_interior.push_back(cv::Point(1, 2));

  fast_interior_set_.push_back(fast_interior);

  fast_interior.clear();

  fast_interior.push_back(cv::Point(0, 0));
  fast_interior.push_back(cv::Point(1, 0));
  fast_interior.push_back(cv::Point(2, 0));
  fast_interior.push_back(cv::Point(3, 0));

  fast_interior.push_back(cv::Point(-3, 1));
  fast_interior.push_back(cv::Point(-2, 1));
  fast_interior.push_back(cv::Point(-1, 1));

  fast_interior.push_back(cv::Point(0, 1));
  fast_interior.push_back(cv::Point(1, 1));
  fast_interior.push_back(cv::Point(2, 1));
  fast_interior.push_back(cv::Point(3, 1));

  fast_interior.push_back(cv::Point(-3, 2));
  fast_interior.push_back(cv::Point(-2, 2));
  fast_interior.push_back(cv::Point(-1, 2));

  fast_interior.push_back(cv::Point(0, 2));
  fast_interior.push_back(cv::Point(1, 2));
  fast_interior.push_back(cv::Point(2, 2));
  fast_interior.push_back(cv::Point(3, 2));

  fast_interior.push_back(cv::Point(-2, 3));
  fast_interior.push_back(cv::Point(-1, 3));

  fast_interior.push_back(cv::Point(0, 3));
  fast_interior.push_back(cv::Point(1, 3));
  fast_interior.push_back(cv::Point(2, 3));

  fast_interior_set_.push_back(fast_interior);
}
