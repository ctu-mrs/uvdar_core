#include "ht4d_cpu.h"

using namespace uvdar;

HT4DBlinkerTrackerCPU::HT4DBlinkerTrackerCPU (
    int i_mem_steps,
    int i_pitch_steps,
    int i_yaw_steps,
    int i_max_pixel_shift,
    cv::Size i_im_res,
    int i_allowed_BER_per_seq,
    int i_nullify_radius,
    int i_reasonable_radius,
    double i_framerate) : HT4DBlinkerTracker(i_mem_steps, i_pitch_steps, i_yaw_steps, i_max_pixel_shift, i_im_res, i_allowed_BER_per_seq, i_nullify_radius, i_reasonable_radius, i_framerate) {
  std::cout << "Initiating HT4DBlinkerTrackerCPU..." << std::endl;

  hough_space_ = new unsigned int[im_area_ * pitch_steps_ * yaw_steps_];
  resetToZero(hough_space_, im_area_ * pitch_steps_ * yaw_steps_);
  
  generateMasks();

  std::cout << "...finished." << std::endl;
  return;
}

HT4DBlinkerTrackerCPU::~HT4DBlinkerTrackerCPU() {
  delete[] hough_space_;
  return;
}

void HT4DBlinkerTrackerCPU::updateResolution(cv::Size i_size){
  updateInterfaceResolution(i_size);

  delete[] hough_space_;
  hough_space_ = new unsigned int[im_area_ * pitch_steps_ * yaw_steps_];
  resetToZero(hough_space_, im_area_ * pitch_steps_ * yaw_steps_);
}

std::vector< std::pair<cv::Point2d,int> > HT4DBlinkerTrackerCPU::getResults() {
  if (!getResultsStart()) {
    return std::vector<std::pair<cv::Point2d,int>>();
  }
  
  projectAccumulatorToHT();
  return getResultsEnd();
}


void HT4DBlinkerTrackerCPU::generateMasks() {
  int     center = mask_width_ / 2;
  cv::Mat radius_box(mask_width_, mask_width_, CV_32F); //matrix with values corresponding to distance of each element from the center
  cv::Mat yaw_box(mask_width_, mask_width_, CV_32F);    //matrix with valuex corresponding to the polar angle of each element
  for (int x = 0; x < mask_width_; x++) {
    for (int y = 0; y < mask_width_; y++) {
      radius_box.at< float >(y, x) = sqrt((x - center) * (x - center) + (y - center) * (y - center));
      yaw_box.at< float >(y, x)    = atan2((y - center), (x - center));
    }
  }
  radius_box.at<float>(center,center)=1;

  std::vector< int > yaw_col, pitch_col; // arrays corresponding to a single column of masks in the Hough spaces of X-Y-Yaw and X-Y-Pitch. These will be permutated to form masks for 4D Hough space of X-Y-Yaw-Pitch
  for (int i = 0; i < mem_steps_; i++) { // iterate over the length of the accumulator
    hybrid_masks_.push_back(std::vector< cv::Point3i >()); // each "age" of a point in terms of image frames to the past has its own mask (set of positions which are incremented in the Hough voting). When applied, these are merely shifted to the corresponding X-Y position of each input point
    for (int x = 0; x < mask_width_; x++) { for (int y = 0; y < mask_width_; y++) { //iterate over X-Y positions of the maximum allowed size of the masks - each column of the 4D mask will be generted separately
      pitch_col.clear();
      yaw_col.clear();
      for (int j = 0; j < pitch_steps_; j++) { //check for each pitch step in the Hough space resolution to decide if this element of the pitch mask should be added
        double r_center = (1.0 / tan(pitch_vals_[j])) * i;
        if ((ceil(radius_box.at< float >(y, x)) >= (r_center-1)) && (floor(radius_box.at< float >(y, x)) <= (r_center+1))) { //add element to the pitch mask if they correspond to the range of "pitch angles" for the current layer. Adds overlaps in "layers" (input point ages) are enforced, to ensure that close votes have some consensus
          pitch_col.push_back(j);
        }
      }
      int sR = 1;
      for (int j = 0; j < yaw_steps_; j++) { //check for each yaw step in the Hough space resolution to decide if this element of the yaw mask should be added
        if (radius_box.at< float >(y, x) <= (i * max_pixel_shift_)) { // decay - the "yaw" of significantly shifted points should not affect the yaw consensus of the current point
          bool spread_test = abs(x - center) <= sR && abs(y - center) <= sR; // expand the yaw layer of the mask by dilation of the center with a square of the radius sR. This is to generate larger overlaps between layers and thus to enforce consensus of close votes
          if (spread_test || (fabs(angDiff(yaw_box.at< float >(y, x), yaw_vals_[j])) < yaw_div_*0.75)) // add elements to the yaw mask if they correspond to the range of "yaw angles" for the current layer
            yaw_col.push_back(j);
        }
      }

      //permutate the 3D masks to generate 4D masks
      for (auto& yp : yaw_col){ for (auto& pp : pitch_col){
        hybrid_masks_[i].push_back(cv::Point3i(x-center,y-center,indexYP(pp,yp))); //add new element to the mask for the 4D X-Y-Yaw-Pitch mask, corresponding to every pair of element from the "pitch and yaw masks"
      } }

    } }
  }
  return;
}

void HT4DBlinkerTrackerCPU::applyMasks( double i_weight_factor,bool i_constant_newer,int i_break_point) {
  int x, y, z;
  for (int t = 0; t < std::min((int)(accumulator_local_copy_.size()), mem_steps_); t++) { //iterate over the accumulator frames
    for (int j = 0; j < (int)(accumulator_local_copy_[t].size()); j++) { //iterate over the points in the current accumulator frame
      for (int m = 0; m < (int)(hybrid_masks_[t].size()); m++) { //iterate over the elements of the Hough space mask for the current frame "age"
        x = hybrid_masks_[t][m].x + accumulator_local_copy_[t][j].x;  // the absolute X coorinate of the mask element
        y = hybrid_masks_[t][m].y + accumulator_local_copy_[t][j].y;  // the absolute Y coorinate of the mask element
        z = hybrid_masks_[t][m].z;                                    // the permutated index representing a combination of Pitch and Yaw steps in the 4D Hough space

        //check for border breach
        if (x < 0)
          continue;
        if (y < 0)
          continue;
        if (x >= im_res_.width)
          continue;
        if (y >= im_res_.height)
          continue;

        if (i_weight_factor < 0.001)
          hough_space_[index3d(x, y, z)]++; //merely increment the element
        else
          hough_space_[index3d(x, y, z)] += ((i_weight_factor * (i_constant_newer?std::min((mem_steps_ - t),mem_steps_-i_break_point):std::max((mem_steps_ - t),mem_steps_-i_break_point)) + mem_steps_) * scaling_factor_); //increase element value with weighting
        touched_matrix_[index2d(x, y)] = 255; //mark X-Y elements in the helper matrix for faster nullification before next processing iteration
      }
    }
  }
}

void HT4DBlinkerTrackerCPU::flattenTo2D() {
  int thickness = yaw_steps_*pitch_steps_;
  unsigned int temp_pos;
  unsigned int temp_max;
  unsigned int index;
  for (int y = 0; y < im_res_.height; y++) { for (int x = 0; x < im_res_.width; x++) { //iterate over the X-Y image coordinates
    if (touched_matrix_[index2d(x, y)] == 0) //save time on coordinates where no mask element has been applied
      continue;

    temp_max = 0;
    temp_pos = 0;
    index = index3d(x, y, 0);
    for (int j = 0; j < thickness; j++) { //iterate over the joined Yaw-Pitch dimension of the Hough space
      if (hough_space_[index] > temp_max) { //find maximum value and index in the given X-Y position
        temp_max = hough_space_[index];
        temp_pos = j;
      }
      index+=im_area_;
    }

    hough_space_maxima_[index2d(x, y)]      = temp_max; //assign the maximum value to this 2D matrix 
    index_matrix_.at< unsigned char >(y, x) = temp_pos; //assign the index of the maximum to this 2D matrix 
  } }
}

void HT4DBlinkerTrackerCPU::cleanTouched() {
  int index;
  for (int i = 0; i < im_res_.height; i++) {
    for (int j = 0; j < im_res_.width; j++) {
      if (touched_matrix_[index2d(j, i)] == 255) {
        index = index3d(j, i, 0);
        for (int k = 0; k < total_steps_; k++) {
          if (hough_space_[index] != 0) {
            hough_space_[index] = 0;
          }
          index+=im_area_;
        }

        hough_space_maxima_[index2d(j, i)] = 0;
        touched_matrix_[index2d(j, i)] = 0;
      }
    }
  }
}

void HT4DBlinkerTrackerCPU::projectAccumulatorToHT() {
  cleanTouched();
  applyMasks( WEIGHT_FACTOR, CONSTANT_NEWER, 0);
  flattenTo2D();
  return;
}

