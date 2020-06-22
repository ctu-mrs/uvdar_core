#ifndef HT4D_H
#define HT4D_H
#include "ht4d.h"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

#define WEIGHT_FACTOR 0.0 // we prioritize blinking signal retrieval to origin point position accuracy - the paper is outdated in this respect

#define USE_VISIBLE_ORIGINS true // if there is a visible marker in the latest time, prefer its position to what is estimated by the HT

#define SMALLER_FAST false // find Hough peaks by neighborhood of radius of 4 pixels

#define CONSTANT_NEWER false // defines whether (if inputs are weighted in favor of the most recent) a number of newest inputs should retain equal weight

#define index2d(X, Y) (im_res_.width * (Y) + (X))
#define index3d(X, Y, Z) (im_area_ * (Z) + im_res_.width * (Y) + (X))
#define indexYP(X, Y) (pitch_steps_ * (Y) + (X))
#define getPitchIndex(X) ((X) % pitch_steps_)
#define getYawIndex(X) ((X) / pitch_steps_)

double HT4DBlinkerTracker::acot(double input) {
  if (input > 0) {
    return (M_PI / 2.0) - atan(input);
  } else if (input < 0) {
    return -(M_PI / 2.0) + atan(input);
  } else {
    throw std::invalid_argument("Invalid input value; Cannot do arccot of zero");
  }
}

double HT4DBlinkerTracker::mod2(double a, double n) {
  return a - (floor(a / n) * n);
}

double HT4DBlinkerTracker::angDiff(double a, double b) {
  double diff = a - b;
  diff        = mod2(diff + M_PI, 2 * M_PI) - M_PI;
  return diff;
}

double HT4DBlinkerTracker::angMeanXY(std::vector< cv::Point > input) {
  cv::Point sum       = std::accumulate(input.begin(), input.end(), cv::Point(0.0, 0.0));
  cv::Point mean      = (cv::Point2d)sum / (double)(input.size());
  double    mean_angle = atan2(mean.y, mean.x);
  return mean_angle;
}

HT4DBlinkerTracker::HT4DBlinkerTracker(
    int i_mem_steps,
    int i_pitch_steps,
    int i_yaw_steps,
    int i_max_pixel_shift,
    cv::Size i_im_res,
    int i_nullify_radius,
    int i_reasonable_radius,
    double i_framerate) {
  std::cout << "Initiating HT4DBlinkerTracker..." << std::endl;
  mem_steps_      = i_mem_steps;
  pitch_steps_    = i_pitch_steps;
  yaw_steps_      = i_yaw_steps;
  total_steps_    = pitch_steps_*yaw_steps_;
  framerate_     = i_framerate;
  max_pixel_shift_ = i_max_pixel_shift;
  frame_scale_    = round(3 * framerate_ / 10);
  mask_width_     = 1 + 2 * max_pixel_shift_ * (frame_scale_ - 1);

  double weight_coeff;
  if (CONSTANT_NEWER){
    weight_coeff = 0.625;
  }
  else {
    weight_coeff = 0.5;
  }
  if (( mem_steps_ ) * (1+(weight_coeff*WEIGHT_FACTOR))  > (UINT_MAX))
    scaling_factor_ = 1.0/(( mem_steps_ ) * (1+(weight_coeff*WEIGHT_FACTOR)) / (UINT_MAX)); //optimization of scaling. The scaling is necessary to account for integer overflows
  else
    scaling_factor_       = 1.0;
  hough_thresh_      =
    (unsigned int)((mem_steps_ ) * (1+(weight_coeff*WEIGHT_FACTOR)) * 0.5 * 0.8 * scaling_factor_); // threshold over which we expect maxima to appear
  nullify_radius_    = i_nullify_radius;
  reasonable_radius_ = i_reasonable_radius;
  im_res_            = i_im_res;
  im_area_           = im_res_.width * im_res_.height;
  im_rect_           = cv::Rect(cv::Point(0, 0), im_res_);

  debug_    = false;
  vis_debug_ = false;

  for (int i = 0; i < mem_steps_; i++) {
    pts_per_layer_.push_back(0);
  }

  sin_set_.clear();
  cos_set_.clear();
  cot_set_max_.clear();
  cot_set_min_.clear();

  min_pitch_ = acot(max_pixel_shift_);
  step_div_ = (double)(max_pixel_shift_)/(double)(pitch_steps_+0.5);
  for (int i = 0; i < pitch_steps_; i++) {
    pitch_vals_.push_back(acot(step_div_*(pitch_steps_-(i + 0.5))));

    if (i == (pitch_steps_-1))
      cot_set_min_.push_back(0.0);
    else
      cot_set_min_.push_back(step_div_*(pitch_steps_-(i+1.5)));


    if (i == 0)
      cot_set_max_.push_back(step_div_*(pitch_steps_+0.5));
    else
      cot_set_max_.push_back(step_div_*(pitch_steps_-(i-0.5)));
  }


  yaw_div_ = (2.0 * M_PI) / yaw_steps_;
  for (int i = 0; i < yaw_steps_; i++) {
    yaw_vals_.push_back((i)*yaw_div_);
    sin_set_.push_back(sin(yaw_vals_[i]));
    cos_set_.push_back(cos(yaw_vals_[i]));
  }

  hough_space_ = new unsigned int[im_area_ * pitch_steps_ * yaw_steps_];
  resetToZero(hough_space_, im_area_ * pitch_steps_ * yaw_steps_);
  hough_space_maxima_ = new unsigned int[im_area_];
  index_matrix_           = cv::Mat(im_res_, CV_8UC1, cv::Scalar(0));
  touched_matrix_ = new unsigned char[im_area_];
  resetToZero(touched_matrix_, im_area_);

  accumulator_.push_back(std::vector< cv::Point2i >());
  accumulator_local_copy_.push_back(std::vector< cv::Point2i >());

  generateMasks();

  initFast();

  curr_batch_processed_ = false;

  std::cout << "...finished." << std::endl;
  return;
}

template < typename T >
void HT4DBlinkerTracker::resetToZero(T *__restrict__ input, int steps) {
  for (int i = 0; i < steps; i++) {
    input[i] = (T)(0);
  }
}


void HT4DBlinkerTracker::setDebug(bool i_DEBUG, bool i_VisDEBUG) {
  debug_    = i_DEBUG;
  vis_debug_ = i_VisDEBUG;
}

void HT4DBlinkerTracker::updateFramerate(double input) {
  if (input > 1.0)
    framerate_ = input;
}

void HT4DBlinkerTracker::updateResolution(cv::Size i_size){
  mutex_accumulator_.lock();
  im_res_ = i_size;
  im_area_           = im_res_.width * im_res_.height;
  im_rect_           = cv::Rect(cv::Point(0, 0), im_res_);

  delete hough_space_;
  hough_space_ = new unsigned int[im_area_ * pitch_steps_ * yaw_steps_];
  resetToZero(hough_space_, im_area_ * pitch_steps_ * yaw_steps_);
  delete hough_space_maxima_;
  hough_space_maxima_ = new unsigned int[im_area_];
  index_matrix_           = cv::Mat(im_res_, CV_8UC1, cv::Scalar(0));
  delete touched_matrix_;
  touched_matrix_ = new unsigned char[im_area_];
  resetToZero(touched_matrix_, im_area_);
  accumulator_.clear();
  curr_batch_processed_ = false;
  mutex_accumulator_.unlock();
}

HT4DBlinkerTracker::~HT4DBlinkerTracker() {
  return;
}


void HT4DBlinkerTracker::insertFrame(std::vector< cv::Point > newPoints) {
  mutex_accumulator_.lock();
  {
    accumulator_.insert(accumulator_.begin(), newPoints);
    pts_per_layer_.insert(pts_per_layer_.begin(), (int)(newPoints.size()));
    if ((int)(accumulator_.size()) > mem_steps_) {
      accumulator_.pop_back();
      pts_per_layer_.pop_back();
    }
    /* if (debug_) */
    /* std::cout << "Expected matches: " << expected_matches_ << std::endl; */
    curr_batch_processed_ = false;
  }
  mutex_accumulator_.unlock();
  return;
}

bool HT4DBlinkerTracker::isCurrentBatchProcessed() {
  return curr_batch_processed_;
}

int HT4DBlinkerTracker::getTrackerCount() {
  /* if (!curr_batch_processed_) */
  /*   return 0; */
  return frequencies_.size();
}
double HT4DBlinkerTracker::getFrequency(int index) {
  /* if (!curr_batch_processed_) */
  /*   return -666.0; */
  /* throw std::logic_error("Cannot retrieve frequencies_. Current batch is not processed yet"); */
  return frequencies_[index];
}
double HT4DBlinkerTracker::getYaw(int index) {
  /* if (!curr_batch_processed_) */
  /*   return -666.0; */
  /* throw std::logic_error("Cannot retrieve yaws. Current batch is not processed yet"); */
  return yaw_averages_[index];
}
double HT4DBlinkerTracker::getPitch(int index) {
  /* if (!curr_batch_processed_) */
  /*   return -666.0; */
  /* throw std::logic_error("Cannot retrieve yaws. Current batch is not processed yet"); */
  return pitch_averages_[index];
}
std::vector<double> HT4DBlinkerTracker::getYaw() {
  /* if (!curr_batch_processed_) */
  /*   return -666.0; */
  /* throw std::logic_error("Cannot retrieve yaws. Current batch is not processed yet"); */
  return yaw_averages_;
}
std::vector<double> HT4DBlinkerTracker::getPitch() {
  /* if (!curr_batch_processed_) */
  /*   return -666.0; */
  /* throw std::logic_error("Cannot retrieve yaws. Current batch is not processed yet"); */
  return pitch_averages_;
}

std::vector< cv::Point3d > HT4DBlinkerTracker::getResults() {
  accumulator_local_copy_.clear();
  pts_per_layer_local_copy_.clear();
  mutex_accumulator_.lock();
  for (int i = 0; i < (int)(accumulator_.size()); i++) {
    accumulator_local_copy_.push_back(std::vector< cv::Point2i >());
    for (int j = 0; j < (int)(accumulator_[i].size()); j++) {
      accumulator_local_copy_[i].push_back(accumulator_[i][j]);
    }
    pts_per_layer_local_copy_.push_back(pts_per_layer_[i]);
  }
  mutex_accumulator_.unlock();
  expected_matches_ = *std::max_element(pts_per_layer_local_copy_.begin(), pts_per_layer_local_copy_.end()) - pts_per_layer_local_copy_[0];
  if (debug_){
    std::cout << "Exp. Matches: " << expected_matches_ << std::endl;
    std::cout << "Visible Matches: " << pts_per_layer_local_copy_[0] << std::endl;
  }

  /* end         = std::clock(); */
  /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
  /* std::cout << "Fetching points: " << elapsedTime << " s" << std::endl; */
  /* begin = std::clock(); */

  projectAccumulatorToHT();
  /* end         = std::clock(); */
  /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
  /* if (debug_) */
  /*   std::cout << "Projecting to HT: " << elapsedTime << " s" << std::endl; */
  /* begin = std::clock(); */

  /* end         = std::clock(); */
  /* elapsedTime = double(end - begin) / CLOCKS_PER_SEC; */
  /* std::cout << "Nullification of known: " << elapsedTime << " s" << std::endl; */


  std::vector< cv::Point > originPts = nullifyKnown();
  std::vector< cv::Point > originPtsOut = accumulator_local_copy_[0];
  if (debug_)
    std::cout << "Orig. pt. count: " << originPts.size() << std::endl;

  /* for(int i =0; i<originPts.size();i++) { */
  /*   for(int j = 0; j<originPts.size();j++) { */
  /*     if (i == j) */
  /*       continue; */

  /*     if (cv::norm(originPts[i] - originPts[j]) < reasonable_radius_){ */
  /*       originPts[i] = (originPts[i]+originPts[j])/2.0; */
  /*       originPts.erase(originPts.begin() + j); */
  /*       j--; */
  /*     } */

  /*   } */

  /* } */


  std::vector< cv::Point > houghOrigins = findHoughPeaks(expected_matches_);
  if (debug_){
    std::cout << "Hough peaks count: " << houghOrigins.size() <<  std::endl;
  }
  originPts.insert(originPts.end(), houghOrigins.begin(), houghOrigins.end());
  originPtsOut.insert(originPtsOut.end(), houghOrigins.begin(), houghOrigins.end());
  std::vector< cv::Point3d > result;

  pitch_averages_.clear();
  yaw_averages_.clear();
  frequencies_.clear();

  if (debug_)
    std::cout << "Orig. pt. count: " << originPts.size() << std::endl;
  for (int i = 0; i < (int)(originPts.size()); i++) {
    if (debug_)
      std::cout << "Curr. orig. pt: " << originPts[i] << std::endl;
    double frequency, yawAvg, pitchAvg;
    /* frequency = retrieveFreqency(originPts[i], yawAvg, pitchAvg); */
    frequency = retrieveFreqency(originPts[i], yawAvg, pitchAvg);
    /* frequency = retrieveFreqency(originPtsOut[i], yawAvg, pitchAvg); */
    result.push_back(cv::Point3d(originPtsOut[i].x, originPtsOut[i].y, frequency));
    frequencies_.push_back(frequency);
    yaw_averages_.push_back(yawAvg);
    pitch_averages_.push_back(pitchAvg);
  }
  if (debug_){
    std::cout << "Differences from the detected: [" << std::endl;
    for (int op = 0; op < (int)(originPts.size()); op++){
      std::cout << originPts[op] - originPtsOut[op] << std::endl;
    }
    std::cout << "]" << std::endl;
  }
  curr_batch_processed_ = true;
  return result;
}

//changing the approach - origin point estimate stays on the active markers, but for frequency estimate we'll use local maximum
std::vector<cv::Point> HT4DBlinkerTracker::nullifyKnown() {
  cv::Point currKnownPt;
  int       top, left, bottom, right;
  std::vector<cv::Point> maxima;
  for (int i = 0; i < (int)(accumulator_local_copy_[0].size()); i++) {
    currKnownPt = (USE_VISIBLE_ORIGINS?
        accumulator_local_copy_[0][i]:
        findHoughPeakLocal(accumulator_local_copy_[0][i]));
    top         = std::max(0, currKnownPt.y - (int)(nullify_radius_));
    left        = std::max(0, currKnownPt.x - (int)(nullify_radius_));
    bottom      = std::min(im_res_.height - 1, currKnownPt.y + (int)(nullify_radius_));
    right       = std::min(im_res_.width - 1, currKnownPt.x + (int)(nullify_radius_));
    for (int x = left; x <= (right); x++) {
      for (int y = top; y <= (bottom); y++) {
        hough_space_maxima_[index2d(x, y)] = 0;
      }
    }
    maxima.push_back(currKnownPt);
  }
  return maxima;
  /* expected_matches_ -= accumulator_local_copy_[0].size(); */
}

void HT4DBlinkerTracker::generateMasks() {
  int     center = mask_width_ / 2;
  cv::Mat radiusBox(mask_width_, mask_width_, CV_32F);
  cv::Mat yawBox(mask_width_, mask_width_, CV_32F);
  for (int x = 0; x < mask_width_; x++) {
    for (int y = 0; y < mask_width_; y++) {
      radiusBox.at< float >(y, x) = sqrt((x - center) * (x - center) + (y - center) * (y - center));
      /* yawBox.at< float >(y, x)    = atan2((y - center), (x - center))+M_PI; */
      yawBox.at< float >(y, x)    = atan2((y - center), (x - center));
      /* if ( (x>39) && (x<45) && (y>39) && (y<45)) */
      /*   std::cout << "yawBox[" << x << "," << y << "]: " << yawBox.at<float>(y,x) << " atan2: " << atan2((y - center), (x - center)) << std::endl; */
    }
  }
  radiusBox.at<float>(center,center)=1;

  std::vector< int > yawCol, pitchCol;
  for (int i = 0; i < mem_steps_; i++) {
    /* pitchMasks.push_back(std::vector< cv::Point3i >()); */
    /* yawMasks.push_back(std::vector< cv::Point3i >()); */
    hybrid_masks_.push_back(std::vector< cv::Point3i >());
    //CHECK: we may only need one column
    for (int x = 0; x < mask_width_; x++) {
      for (int y = 0; y < mask_width_; y++) {
        pitchCol.clear();
        yawCol.clear();
        for (int j = 0; j < pitch_steps_; j++) {
          /* double Rmin = (1.0 / tan(pitch_vals_[j] + (pitchDiv / 2))) * i; */
          /* double Rmax = (1.0 / tan(pitch_vals_[j] - (pitchDiv / 2))) * i; */
          /* double Rmin = (cot_set_min_[j]*i); */
          /* double Rmax = (cot_set_max_[j]*i); */
          double Rcen = (1.0 / tan(pitch_vals_[j])) * i;
          /* if ((radiusBox.at< float >(y, x) >= (Rmin-1)) && (radiusBox.at< float >(y, x) <= (Rmax+1))) { */
          if ((ceil(radiusBox.at< float >(y, x)) >= (Rcen-1)) && (floor(radiusBox.at< float >(y, x)) <= (Rcen+1))) {
            pitchCol.push_back(j);
          }
        }
        int sR = 1;
        for (int j = 0; j < yaw_steps_; j++) {
          /*   for (int k =-sR; k<=sR; k++) for (int l =-sR; l<=sR; l++) */
          /* yawMasks[i].push_back(cv::Point3i(k, l, j)); */
          if (radiusBox.at< float >(y, x) < (i * max_pixel_shift_)) {               // decay
            bool spreadTest=false;
            for (int k =-sR; k<=sR; k++) for (int l =-sR; l<=sR; l++){
              if ((((x+l) == center) && ((y+k) == center))){  // yaw steps
                spreadTest = true;
                break;
              }
              if (spreadTest) break;
            }
            if (spreadTest  || (fabs(angDiff(yawBox.at< float >(y, x), yaw_vals_[j])) < yaw_div_*0.75))
              yawCol.push_back(j);
          }
        }

        //permutate the 3D masks to generate 4D masks
        for (auto& yp : yawCol)
          for (auto& pp : pitchCol)
            hybrid_masks_[i].push_back(cv::Point3i(x-center,y-center,indexYP(pp,yp)));

      }
    }
  }
  /* std::cout << yawMasks[3] << std::endl; */
  return;
}

void HT4DBlinkerTracker::applyMasks( double i_weight_factor,bool i_constant_newer,int i_break_point) {
  /* for (int i=0;i<hough_space_.size[0];i++){ */
  /*   for (int j=0;j<hough_space_.size[1];j++){ */
  /*     for (int k=0;k<hough_space_.size[2];k++){ */
  /*       hough_space_.at<uint16_t>(i,j,k) = 0; */
  /*     }}} */
  /* hough_space_ = cv::Scalar(0); */
  int x, y, z, w;
  for (int t = 0; t < std::min((int)(accumulator_local_copy_.size()), mem_steps_); t++) {
    for (int j = 0; j < (int)(accumulator_local_copy_[t].size()); j++) {
      for (int m = 0; m < (int)(hybrid_masks_[t].size()); m++) {
        /* std::cout << "here a" << std::endl; */
        /* std::cout << "here b: " << cv::Point(hybrid_masks_[t][m].x + accumulator_local_copy_[t][j].x, hybrid_masks_[t][m].y + accumulator_local_copy_[t][j].y) << std::endl; */
        /* std::cout <<  "here" << std::endl; */
        x = hybrid_masks_[t][m].x + accumulator_local_copy_[t][j].x;
        y = hybrid_masks_[t][m].y + accumulator_local_copy_[t][j].y;
        z = hybrid_masks_[t][m].z;
        /* if (!(im_rect_.contains(cv::Point(x, y)))) */
        if (x < 0)
          continue;
        if (y < 0)
          continue;
        if (x >= im_res_.width)
          continue;
        if (y >= im_res_.height)
          continue;

        if (i_weight_factor < 0.001)
          /* hough_space_[index3d(x, y, z)] +=(mem_steps_-t)+mem_steps_/2; */
          /* hough_space_[index3d(x, y, z)] +=(mem_steps_-t); */
          hough_space_[index3d(x, y, z)]++;
        else
          hough_space_[index3d(x, y, z)] += ((i_weight_factor * (i_constant_newer?std::min((mem_steps_ - t),mem_steps_-i_break_point):std::max((mem_steps_ - t),mem_steps_-i_break_point)) + mem_steps_) * scaling_factor_);
        touched_matrix_[index2d(x, y)] = 255;
      }
    }
  }
}

void HT4DBlinkerTracker::flattenTo2D() {
  int thickness = yaw_steps_*pitch_steps_;
  unsigned int tempPos;
  unsigned int tempMax;
  unsigned int index;
  for (int y = 0; y < im_res_.height; y++) {
    for (int x = 0; x < im_res_.width; x++) {
      /* cv::Range ranges[3] = {cv::Range(x, x + 1), cv::Range(y, y + 1), cv::Range::all()}; */
      if (touched_matrix_[index2d(x, y)] == 0)
        continue;

      tempMax = 0;
      tempPos = 0;
      index = index3d(x, y, 0);
      for (int j = 0; j < thickness; j++) {
          if (hough_space_[index] > tempMax) {
          tempMax = hough_space_[index];
          tempPos = j;
        }
        index+=im_area_;
      }

      hough_space_maxima_[index2d(x, y)]             = tempMax;
      index_matrix_.at< unsigned char >(y, x) = tempPos;
      /* if (cv::norm(tempPos)>0) */
      /*   std::cout << "Loc" << tempPos.x << tempPos.y << std::endl; */
    }
  }
}

void HT4DBlinkerTracker::cleanTouched() {
  unsigned int index;
  for (int i = 0; i < im_res_.height; i++) {
    for (int j = 0; j < im_res_.width; j++) {
      if (touched_matrix_[index2d(j, i)] == 255) {
        index = index3d(j, i, 0);
        for (int k = 0; k < total_steps_; k++) {
          if (hough_space_[index] != 0)
            hough_space_[index] = 0;
          index+=im_area_;
        }

        /* houghSpacePitchMaxima.at< uint16_t >(i, j) = 0; */
        /* houghSpaceYawMaxima.at< uint16_t >(i, j)   = 0; */
        /* pitchMatrix.at<uint16_t>(i,j) = 0; */
        /* yawMatrix.at<uint16_t>(i,j) = 0; */
        hough_space_maxima_[index2d(j, i)] = 0;
        touched_matrix_[index2d(j, i)] = 0;
      }
    }
  }
}

void HT4DBlinkerTracker::projectAccumulatorToHT() {
  cleanTouched();
  applyMasks( WEIGHT_FACTOR, CONSTANT_NEWER, 0);
  flattenTo2D();
  
  if (vis_debug_) {
    cv::Mat viewerA = getCvMat(hough_space_maxima_,hough_thresh_*4);
    cv::Mat viewerB = index_matrix_*(255.0/(double)(pitch_steps_*yaw_steps_));
    
    cv::hconcat(viewerA, viewerB, visualization_);
  }
  return;
}

/* cv::Mat HT4DBlinkerTracker::downSample(const cv::Mat &input, cv::Mat &output,int bits){ */
/*   /1* if (input.size() != output.size()) *1/ */
/*   /1*   output = cv::Mat(input.size(),CV_16UC1); *1/ */


/* } */

std::vector< cv::Point > HT4DBlinkerTracker::findHoughPeaks(int peak_count) {
  /* cv::Mat                  inputCp = input.clone(); */
  std::vector< cv::Point > peaks;
  int             currMax;
  cv::Point                currMaxPos;
  bool storeCurrent;
  for (int i = 0; i < peak_count; i++) {
    storeCurrent = false;
    currMax = 0;
    for (int y = 0; y < im_res_.height; y++) {
      for (int x = 0; x < im_res_.width; x++) {
        if (touched_matrix_[index2d(x, y)] == 0)
          continue;

        if (hough_space_maxima_[index2d(x,y)] > (unsigned int)currMax) {
          currMax    = hough_space_maxima_[index2d(x,y)];
          currMaxPos = cv::Point(x, y);
          storeCurrent = true;
        }
          /*     currMax    = currMinDiff; */
          /*     currMaxPos = cv::Point(x, y); */
          /*     std::cout << "currMinDiff: " << currMinDiff << std::endl; */
          /*   } */
          /*   storeCurrent = true; */
          /* } */

      }
    }
    if (hough_space_maxima_[index2d(currMaxPos.x,currMaxPos.y)] < hough_thresh_) {
      storeCurrent = false;
        if (debug_)
      std::cout << "Point " << currMaxPos << " with value of " <<hough_space_maxima_[index2d(currMaxPos.x,currMaxPos.y)] <<" Failed threshold test. Threshold is " << hough_thresh_ << ". Breaking." << std::endl;
      break;
    }
    if (!miniFast(currMaxPos.x,currMaxPos.y, hough_thresh_/4)) {
      /* i--; */
        storeCurrent = false;
        if (debug_)
          std::cout << "Point " << currMaxPos << " Failed FAST test." << std::endl;
        /* break; */

      }
/* if (miniFast(x,y, hough_thresh_*0.10, currMinDiff)) { */
/*   if (currMinDiff > currMax) { */

      /* if (!storeCurrent){ */
      /*   if (debug_) */
      /*     std::cout << "No more high points found, stopping search." << std::endl; */
      /*   break; */
      /* } */

      /* if (currMax < hough_thresh_/8) { */
      /*   storeCurrent = false; */
      /*   break; */
      /* } */
      /* if (input[index2d(currMaxPos.x,currMaxPos.y)] > hough_thresh_) { */
      /*   if (debug_) */
      /*     std::cout << "Point " << currMaxPos << " passed value test, its value is " << input[index2d(currMaxPos.x,currMaxPos.y)] << " against thresh. of " << hough_thresh_ << std::endl; */
      /* } */
      /* else { */
      /*   storeCurrent = false; */
      /*   if (debug_) */
      /*   std::cout << "Point " << currMaxPos << " failed value test, its value is " << input[index2d(currMaxPos.x,currMaxPos.y)]  << " against thresh. of " << hough_thresh_ << std::endl; */
      /*   break; */
      /* } */
    /* if (debug_) */
    /*   std::cout << "Bit Shift: " << bit_shift_ << " Thresh: " << hough_thresh_ << " Curr. Peak: " << currMax << std::endl; */
    /* if (currMax < hough_thresh_){ */
    /*   if (debug_){ std::cout << "Maximum is low" << std::endl; */
    /*   } */
    /*   break; */
    /* } */

    /* if (!miniFast(currMaxPos, hough_thresh_/8 << bit_shift_)) { */
    /*   if (debug_){ std::cout << "FAST failed" << std::endl; */
    /*   } */
    /*   storeCurrent = false; */
    /*   i--; //to try again */
    /* } */

    int top, left, bottom, right;
    top    = std::max(0, currMaxPos.y - (int)(nullify_radius_));
    left   = std::max(0, currMaxPos.x - (int)(nullify_radius_));
    bottom = std::min(im_res_.height - 1, currMaxPos.y + (int)(nullify_radius_));
    right  = std::min(im_res_.width - 1, currMaxPos.x + (int)nullify_radius_);
    for (int x = left; x <= (right); x++) {
      for (int y = top; y <= (bottom); y++) {
        hough_space_maxima_[index2d(x, y)] = 0;
      }
    }
    if (storeCurrent){
      /* if (debug_) */
      /*   std::cout << "Point " << currMaxPos<< " passed FAST test, smallest diff. is " << currMax << std::endl; */
      peaks.push_back(currMaxPos);
    }
    /* else */ 
    /*   break; */

  }
  return peaks;
}

cv::Point HT4DBlinkerTracker::findHoughPeakLocal(cv::Point expected_pos) {
  /* cv::Mat                  inputCp = input.clone(); */
  std::vector< cv::Point > peaks;
  cv::Point                currMaxPos = expected_pos;
  int top, left, bottom, right;
  bool found = false;
  //expanding square outline
  for (int r = 0; r <= ((int)nullify_radius_); r++) {
    top    = std::max(0, expected_pos.y - (int)(r));
    left   = std::max(0, expected_pos.x - (int)(r));
    bottom = std::min(im_res_.height - 1, expected_pos.y + (int)r);
    right  = std::min(im_res_.width - 1, expected_pos.x + (int)r);

    /* std::cout << "Outline with r=" << r << std::endl; */
    //top and bottom lines of square outline
    for (int y = top; y <= (bottom); y+=((r==0)?1:(bottom-top))){
    /* std::cout << "first loop, y=" << y << std::endl; */
      for (int x = left; x <= (right); x++) {
        /* std::cout << "first loop, x=" << x << std::endl; */
        if (miniFast(x,y, 0)){
          found = true;
          currMaxPos = cv::Point(x,y);
          break;
        }
      }
      if(found)
        break;
    }
    if(found)
      break;
    //left and right lines of square outline
    for (int x = left; x <= (right); x+=((r==0)?1:(right-left))) {
    /* std::cout << "second loop, x=" << x << std::endl; */
      for (int y = top+1; y <= bottom-1; y++){
    /* std::cout << "second loop, y=" << y << std::endl; */
        if (miniFast(x,y, 0)){
          found = true;
          currMaxPos = cv::Point(x,y);
          break;
        }
      }
      if(found)
        break;
    }
    if(found)
      break;
  }
  if (debug_)
    std::cout << "Finding for visible: Scaling factor: " << scaling_factor_ << " Thresh: " << hough_thresh_ << " Curr. Peak: " << hough_space_maxima_[index2d(currMaxPos.x,currMaxPos.y)] << std::endl;
  /* if ((currMax < hough_thresh_) || !miniFast(currMaxPos))  */
  /* if (!miniFast(currMaxPos)) { */
  /*   break; */
  /* } */
  /* if (debug_) */
  /*   std::cout << "Passed FAST test" << std::endl; */
  return currMaxPos;
}


double HT4DBlinkerTracker::retrieveFreqency(cv::Point origin_points, double &avg_yaw, double &avg_pitch) {
  unsigned char initIndex = index_matrix_.at< unsigned char >(origin_points);
  unsigned char pitchIndex = getPitchIndex(initIndex);
  unsigned char yawIndex = getYawIndex(initIndex);
  if (debug_){
    std::cout << "Initial pitch, yaw: estimate: [" << pitch_vals_[pitchIndex]*(180/M_PI) <<  ", " <<  yaw_vals_[yawIndex]*(180/M_PI) << "] deg" << std::endl;
  }
  int                      stepCount = std::min((int)(accumulator_local_copy_.size()), mem_steps_);
  double                   radExpectedMax, radExpectedMin, yawExpected, currPointRadius, currPointYaw;
  double avgPitchCot;
  int currPointMaxDim, currPointRadiusRound;
  std::vector< cv::Point > positivePointAccum;
  std::vector< cv::Point > positivePointAccumPitch;
  std::vector<double>      pitchCotAccum;
  cv::Point                currPoint, currPointCentered;
  std::vector< int >       positiveCountAccum = std::vector< int >(accumulator_local_copy_.size(), 0);
  std::vector< double> yawAccum;
  /* double reasonableRadiusScaled; */
  for (int t = 0; t < stepCount; t++) {
    /* radExpectedMin        = (cot_set_min_[pitchIndex] * t) - (reasonable_radius_);  //+t*0.2 */
    /* radExpectedMax        = (cot_set_max_[pitchIndex] * t) + (reasonable_radius_);  //+t*0.2 */
    radExpectedMin        = floor(cot_set_min_[pitchIndex] * t)-1;  //+t*0.2
    radExpectedMax        = ceil(cot_set_max_[pitchIndex] * t)+1;  //+t*0.2
    yawExpected           = yaw_vals_[yawIndex]-M_PI;
    positiveCountAccum[t] = 0;
    for (int k = 0; k < (int)(accumulator_local_copy_[t].size()); k++) {
      currPoint         = accumulator_local_copy_[t][k];
      currPointCentered = currPoint - origin_points;
      currPointRadius   = cv::norm(currPointCentered);
      currPointRadiusRound   = round(currPointRadius);
      currPointMaxDim = std::max(currPointCentered.x,currPointCentered.y);

      /* if (currPointRadius > ((cot_set_max_[0] * t) + (reasonable_radius_))){ */
      /* /1* if (debug_) *1/ */
      /* /1*     std::cout << "currPointRadius: " << currPointRadius << "is greater than max. admissible of " << (((cot_set_max_[0] * t) + (reasonable_radius_))) << std::endl; *1/ */
      /*   continue; */
      /* } */
       
      currPointYaw = atan2(currPointCentered.y, currPointCentered.x);

      if (currPointRadiusRound >= radExpectedMin){
        if (currPointRadiusRound <= radExpectedMax) {
          if ((fabs(angDiff(currPointYaw, yawExpected)) <= (yaw_div_)) || (currPointMaxDim <= 4)) {
            positivePointAccum.push_back(currPointCentered);
            yawAccum.push_back(currPointYaw);
            positivePointAccumPitch.push_back(cv::Point(currPointRadius, t));
            if (t>0)
              pitchCotAccum.push_back(currPointRadius/((double)t));
            else
              pitchCotAccum.push_back(currPointRadius/(1.0));
            positiveCountAccum[t]++;
          }
        }
      }
      if (debug_){
        if ((currPointRadiusRound < radExpectedMin) || (currPointRadiusRound > radExpectedMax))
          std::cout << "currPointRadius: " << currPointRadiusRound << ", t " << t << ":" << k <<" failed vs. [" << radExpectedMin<< "," <<radExpectedMax << "]"  << std::endl;
        else
          std::cout << "currPointRadius: " << currPointRadiusRound << ", t " << t << ":" << k <<" passed vs. [" << radExpectedMin<< "," <<radExpectedMax << "]"  << std::endl;

        if ( ((fabs(angDiff(currPointYaw, yawExpected)) > (yaw_div_)) && (currPointMaxDim > 4)) )
          std::cout << "currPointYaw: " << currPointYaw << ", t " << t << ":" << k <<" failed vs. [" << yawExpected<< "] by "  << angDiff(currPointYaw, yawExpected) << " vs " << yaw_div_ << std::endl;
        else
          std::cout << "currPointYaw: " << currPointYaw << ", t " << t << ":" << k <<" passed vs. [" << yawExpected<< "] by "  << angDiff(currPointYaw, yawExpected) << " vs " << yaw_div_ << std::endl;
      }
    }
  }
  if (positivePointAccum.size() == 0) {
    return -666.0;
  }

  /* use the fact that no close points were found as indication! */

  if (debug_){
    std::cout << "Accumulated:" << std::endl;
    //CHECK: is the culling still necessary
    for (int i = 0; i < (int)(positiveCountAccum.size()); i++) {
      std::cout << positiveCountAccum[i];
    }
  std::cout << std::endl;
  }

  avg_yaw                      = angMeanXY(positivePointAccum);
  avg_pitch                      = angMeanXY(positivePointAccumPitch);
  avgPitchCot                      = accumulate(pitchCotAccum.begin(), pitchCotAccum.end(), 0.0)/pitchCotAccum.size(); 
  //CHECK:total averaging in 3D
  std::vector< bool > correct = std::vector< bool >(positivePointAccum.size(), true);
  for (int u = 0; u < (int)(positivePointAccum.size()); u++) {
    /* std::cout << "correct: "; */
    cv::Point expPt(cos(avg_yaw)*avgPitchCot*positivePointAccumPitch[u].y, sin(avg_yaw)*avgPitchCot*positivePointAccumPitch[u].y);
    if (floor(cv::norm(expPt-positivePointAccum[u])) > (reasonable_radius_*2)) {
    /* if ((cv::norm(expPt*positivePointAccumPitch[u].y-positivePointAccum[u]) > (CV_PI / 20.0)) && (cv::norm(positivePointAccum[u]) > (reasonable_radius_))) { */
    /* if ((fabs(angDiff(yawAccum[u], avg_yaw)) > (CV_PI / 20.0)) && (cv::norm(positivePointAccum[u]) > (reasonable_radius_))) { */
    /*   std::cout << "Culling" << std::endl; */
    /*   std::cout << "Yaw: " << yawAccum[u] << " vs " << avg_yaw << std::endl; */
    /*   correct[u] = false; */
    /* } */
    /* else if (fabs((pitchCotAccum[u] - avgPitchCot)) > (reasonable_radius_)) { */
    /*   /1* if ((fabs(angDiff(yawAccum[u], avg_yaw)) > (CV_PI / 4.0)) )  *1/ */
      if (debug_){
        std::cout << "Culling" << std::endl;
        std::cout << "avgPitchCot: " << avgPitchCot << " avg_yaw " <<  avg_yaw << std::endl;
        std::cout << "pitchCotSum: " << accumulate(pitchCotAccum.begin(), pitchCotAccum.end(), 0.0)<< " pitchCotSize " <<  pitchCotAccum.size() << std::endl;
        /* std::cout << "pitchCot: "; for (auto &pcp : pitchCotAccum) { std::cout << pcp << " "; } std::cout << std::endl; */
        std::cout << "diff: " << expPt << " vs " <<  positivePointAccum[u] << std::endl;
      }
      correct[u] = false;
    }
    /* std::cout << correct[u]; */
  }
  int o = 0;
  for (int u = 0; u < (int)(correct.size()); u++) {
    if (!correct[u]) {
      /* std::cout <<  "Culling n. " << u << std::endl; */
      positiveCountAccum[positivePointAccumPitch[o].y]--;
      positivePointAccum.erase(positivePointAccum.begin() + o);
      positivePointAccumPitch.erase(positivePointAccumPitch.begin() + o);
      o--;
    }
    o++;
  }
  avg_yaw   = angMeanXY(positivePointAccum);
  avg_pitch = angMeanXY(positivePointAccumPitch);

  if (debug_){
  std::cout << "After culling" << std::endl;
  for (int i = 0; i < (int)(positiveCountAccum.size()); i++) {
    std::cout << positiveCountAccum[i];
  }
  std::cout << std::endl;
  }

  bool   state             = false;
  bool   prevState         = state;
  bool   downStep          = false;
  int    lastDownStepIndex = 0;
  bool   upStep            = false;
  int    lastUpStepIndex   = 0;
  double period;
   double upDur, downDur;
  double maxPeriod = 0;
  double minPeriod = mem_steps_;
  int    cntPeriod = 0;
  int    sumPeriod = 0;
  for (int t = 0; t < (int)(accumulator_local_copy_.size()); t++) {
    if (positiveCountAccum[t] > 0)
      state = true;
    else
      state = false;

    //11100000011111110011110
    if (!state && prevState) {
        upDur = (t-lastUpStepIndex);
      if (downStep) {
        /* std::cout <<  t << std::endl; */
        period            = (t - lastDownStepIndex);
        if (period < minPeriod)
          minPeriod = period;
        if (period > maxPeriod)
          maxPeriod = period;
        lastDownStepIndex = t;
        sumPeriod         = sumPeriod + period;
        /* std::cout << "up " << period << std::endl; */
        cntPeriod = cntPeriod + 1;
      }
      if (!downStep) {
        /* std::cout <<  "here" << std::endl; */
        lastDownStepIndex = t;
        downStep          = true;
      } 
    }
    if (state && !prevState && (t > 0)) {
       downDur = (t-lastDownStepIndex);
      if (upStep) {
        /* std::cout <<  t << std::endl; */
        period          = (t - lastUpStepIndex);
        if (period < minPeriod)
          minPeriod = period;
        if (period > maxPeriod)
          maxPeriod = period;
        lastUpStepIndex = t;
        sumPeriod       = sumPeriod + period;
        /* std::cout << "dn " << period << std::endl; */
        cntPeriod = cntPeriod + 1;
      }
      if (!upStep) {
        /* std::cout <<  "here" << std::endl; */
        lastUpStepIndex = t;
        upStep          = true;
      }
    }
    prevState = state;
    /* if (downStep) std::cout <<"DownStep active" << std::endl; */
    /* if (upStep) std::cout <<"UpStep active" << std::endl; */
  }

  double periodAvg;

  if (cntPeriod == 0){
  /* if ((cntPeriod == 0) && (accumulator_local_copy_.size() == mem_steps_)) { */
    /* std::cout << "No steps recorded???" << std::endl; */
    /* if (upStep && downStep) { */
    /*   if (lastDownStepIndex > (lastUpStepIndex + 3)) { */
    /*     periodAvg = 2 * (lastDownStepIndex - lastUpStepIndex); */
    /*   } */
    /* } */

    if (debug_){
      std::cout << "Not one whole period retrieved, returning;" <<std::endl;
    }
    return -1;
  } else {
    periodAvg = (double)(sumPeriod) / (double)cntPeriod;
  }

  if ((maxPeriod - minPeriod) > ceil(periodAvg/2)){
    if (debug_)
      std::cout << "Spread too wide: "<<maxPeriod-minPeriod<<" compared to average of " << periodAvg <<", returning" <<std::endl;
    return -3;
  }

  if ((periodAvg * ((double)(cntPeriod+1)/2.0) ) < ( mem_steps_ - (1.5*periodAvg))){
    if (debug_){
      std::cout << "Not enough periods retrieved: "<< cntPeriod <<" for " << periodAvg <<" on average; returning" <<std::endl;
    }
    return -4;
  }
  if ((cntPeriod == 1) && (fabs(upDur-downDur)>(periodAvg/2))){
    if (debug_){
      std::cout << "Long period with uneven phases: "<< upDur-downDur <<" for " << periodAvg <<" on average; returning" <<std::endl;
    }
    return -5;
  }


  if (debug_){
  /* std::cout << "After culling" << std::endl; */
  /* std::cout << "CNT: " << cntPeriod << " SUM: " <<sumPeriod << std::endl; */
  /* std::cout << "count is " << cntPeriod << std::endl; */
  /* std::cout << framerate_ << std::endl; */
  std::cout << "Frequency: " << (double)framerate_ / periodAvg << std::endl;
  }
  return (double)framerate_ / periodAvg;
}

cv::Mat HT4DBlinkerTracker::getCvMat(unsigned int *__restrict__ input, unsigned int threshold){
  cv::Mat output(im_res_,CV_8UC1);
  for (int i=0; i<im_res_.height; i++){
    for (int j=0; j<im_res_.width; j++){
      output.data[index2d(j,i)] = input[index2d(j,i)]*(255.0/threshold);
    } }
  /* std::cout << "testval: " << (int)(output[index2d(im_res_.width/2,im_res_.height/2)]) << std::endl; */
  return output;
}

cv::Mat HT4DBlinkerTracker::getVisualization(){
  return visualization_;
}

bool HT4DBlinkerTracker::miniFast(int x, int y, unsigned int thresh) {
  int border;
  if (SMALLER_FAST) border = 3;
  else border = 4;

  if (x<border)
    return false;
  if (y<border)
    return false;
  if (x>(im_res_.width-(border+1)))
    return false;
  if (y>(im_res_.height-(border+1)))
    return false;
  bool foundOneFit = false;
  int diff;
  for (int i = 0; i < (int)(fast_points_.size()); i++) {
  diff = 
        (hough_space_maxima_[index2d(x, y)] - hough_space_maxima_[index2d(x + fast_points_[i].x, y + fast_points_[i].y)]);
  if (diff > (int)(thresh)){
    foundOneFit = true;
    /* diffsum+=diff; */
    /* else std::cout << "DIFF: " << diff << std::endl; */
  }
  else
    return false;
  }

  return foundOneFit;
}

void HT4DBlinkerTracker::initFast() {
  fast_points_.clear();

  if (SMALLER_FAST){

  fast_points_.push_back(cv::Point(0, -3));
  fast_points_.push_back(cv::Point(0, 3));
  fast_points_.push_back(cv::Point(3, 0));
  fast_points_.push_back(cv::Point(-3, 0));

  fast_points_.push_back(cv::Point(2, -2));
  fast_points_.push_back(cv::Point(-2, 2));
  fast_points_.push_back(cv::Point(-2, -2));
  fast_points_.push_back(cv::Point(2, 2));

  fast_points_.push_back(cv::Point(-1, -3));
  fast_points_.push_back(cv::Point(1, 3));
  fast_points_.push_back(cv::Point(3, -1));
  fast_points_.push_back(cv::Point(-3, 1));

  fast_points_.push_back(cv::Point(1, -3));
  fast_points_.push_back(cv::Point(-1, 3));
  fast_points_.push_back(cv::Point(3, 1));
  fast_points_.push_back(cv::Point(-3, -1));

  } else {

  fast_points_.push_back(cv::Point(0, -4));
  fast_points_.push_back(cv::Point(0, 4));
  fast_points_.push_back(cv::Point(4, 0));
  fast_points_.push_back(cv::Point(-4, 0));

  fast_points_.push_back(cv::Point(2, -3));
  fast_points_.push_back(cv::Point(-2, 3));
  fast_points_.push_back(cv::Point(-3, -2));
  fast_points_.push_back(cv::Point(3, 2));

  fast_points_.push_back(cv::Point(3, -2));
  fast_points_.push_back(cv::Point(-3, 2));
  fast_points_.push_back(cv::Point(-2, -3));
  fast_points_.push_back(cv::Point(2, 3));

  fast_points_.push_back(cv::Point(-1, -4));
  fast_points_.push_back(cv::Point(1, 4));
  fast_points_.push_back(cv::Point(4, -1));
  fast_points_.push_back(cv::Point(-4, 1));

  fast_points_.push_back(cv::Point(1, -4));
  fast_points_.push_back(cv::Point(-1, 4));
  fast_points_.push_back(cv::Point(4, 1));
  fast_points_.push_back(cv::Point(-4, -1));

  }
}

#endif  // HT4D_H
