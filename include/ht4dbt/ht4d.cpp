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


using namespace uvdar;

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
    scaling_factor_ = 1.0/(( mem_steps_ ) * (1+(weight_coeff*WEIGHT_FACTOR)) / (UINT_MAX)); // the scaling is necessary to account for integer overflows
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

void HT4DBlinkerTracker::setSequences(std::vector<std::vector<bool>> i_sequences){
  sequences_ = i_sequences;
  matcher_ = std::make_unique<SignalMatcher>(sequences_);
}

void HT4DBlinkerTracker::setDebug(bool i_debug, bool i_vis_debug) {
  debug_    = i_debug;
  vis_debug_ = i_vis_debug;
}

void HT4DBlinkerTracker::updateFramerate(double input) {
  if (input > 1.0)
    framerate_ = input;
}

void HT4DBlinkerTracker::updateResolution(cv::Size i_size){
  if (debug_){
    std::cout << "Setting resolution to " << i_size << std::endl;
  }
  std::scoped_lock lock(mutex_accumulator_);
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
}

HT4DBlinkerTracker::~HT4DBlinkerTracker() {
  return;
}


void HT4DBlinkerTracker::insertFrame(std::vector< cv::Point > new_points) {
  std::scoped_lock lock(mutex_accumulator_);
  {
    accumulator_.insert(accumulator_.begin(), new_points);
    pts_per_layer_.insert(pts_per_layer_.begin(), (int)(new_points.size()));
    if ((int)(accumulator_.size()) > mem_steps_) {
      accumulator_.pop_back();
      pts_per_layer_.pop_back();
    }
    curr_batch_processed_ = false;
  }
  return;
}

bool HT4DBlinkerTracker::isCurrentBatchProcessed() {
  return curr_batch_processed_;
}

int HT4DBlinkerTracker::getTrackerCount() {
  return signals_.size();
}
std::vector<bool> HT4DBlinkerTracker::getSignal(int index) {
  return signals_[index];
}
double HT4DBlinkerTracker::getYaw(int index) {
  return yaw_averages_[index];
}
double HT4DBlinkerTracker::getPitch(int index) {
  return pitch_averages_[index];
}
std::vector<double> HT4DBlinkerTracker::getYaw() {
  return yaw_averages_;
}
std::vector<double> HT4DBlinkerTracker::getPitch() {
  return pitch_averages_;
}

std::vector< std::pair<cv::Point2d,int> > HT4DBlinkerTracker::getResults() {
  if ((im_res_.width <= 0) || (im_res_.height <= 0)){
    if (debug_){
      std::cout << "Resolution was not yet set..." << std::endl;
    }
    return std::vector<std::pair<cv::Point2d,int>>();
  }

  accumulator_local_copy_.clear();
  pts_per_layer_local_copy_.clear();
  {
    std::scoped_lock lock(mutex_accumulator_);
    for (int i = 0; i < (int)(accumulator_.size()); i++) {
      accumulator_local_copy_.push_back(std::vector< cv::Point2i >());
      for (int j = 0; j < (int)(accumulator_[i].size()); j++) {
        accumulator_local_copy_[i].push_back(accumulator_[i][j]);
      }
      pts_per_layer_local_copy_.push_back(pts_per_layer_[i]);
    }
  }
  if (pts_per_layer_local_copy_.empty()){
    return std::vector<std::pair<cv::Point2d,int>>();
  }

  expected_matches_ = *std::max_element(pts_per_layer_local_copy_.begin(), pts_per_layer_local_copy_.end()) - pts_per_layer_local_copy_[0];
  if (debug_){
    std::cout << "Exp. Matches: " << expected_matches_ << std::endl;
    std::cout << "Visible Matches: " << pts_per_layer_local_copy_[0] << std::endl;
  }

  projectAccumulatorToHT();

  std::vector< cv::Point > origin_pts = nullifyKnown();
  std::vector< cv::Point > origin_pts_out = accumulator_local_copy_[0];
  if (debug_)
    std::cout << "Orig. pt. count: " << origin_pts.size() << std::endl;

  std::vector< cv::Point > houghOrigins = findHoughPeaks(expected_matches_);
  if (debug_){
    std::cout << "Hough peaks count: " << houghOrigins.size() <<  std::endl;
  }
  origin_pts.insert(origin_pts.end(), houghOrigins.begin(), houghOrigins.end());
  origin_pts_out.insert(origin_pts_out.end(), houghOrigins.begin(), houghOrigins.end());
  std::vector< std::pair<cv::Point2d,int> > result;

  pitch_averages_.clear();
  yaw_averages_.clear();
  signals_.clear();

  if (debug_)
    std::cout << "Orig. pt. count: " << origin_pts.size() << std::endl;
  for (int i = 0; i < (int)(origin_pts.size()); i++) {
    if (debug_)
      std::cout << "Curr. orig. pt: " << origin_pts[i] << std::endl;
    std::vector<bool> blink_signal;
    int signal_id;
    double yawAvg, pitchAvg;
    signal_id = retrieveSignalID(origin_pts[i], yawAvg, pitchAvg, blink_signal);
    result.push_back(std::pair<cv::Point2d,int>(cv::Point2d(origin_pts_out[i].x, origin_pts_out[i].y), signal_id));
    signals_.push_back(blink_signal);

    std::cout << "Retrieved signal: " << std::endl;
    for (int i = 0; i < (int)(blink_signal.size()); i++) {
      std::cout << (int)(blink_signal[i]) << ",";
    }
    std::cout << std::endl;

    yaw_averages_.push_back(yawAvg);
    pitch_averages_.push_back(pitchAvg);
  }
  if (debug_){
    std::cout << "Differences from the detected: [" << std::endl;
    for (int op = 0; op < (int)(origin_pts.size()); op++){
      std::cout << origin_pts[op] - origin_pts_out[op] << std::endl;
    }
    std::cout << "]" << std::endl;
  }
  curr_batch_processed_ = true;
  return result;
}

//changing the approach - origin point estimate stays on the active markers, but for frequency estimate we will use local maximum
std::vector<cv::Point> HT4DBlinkerTracker::nullifyKnown() {
  cv::Point current_known_pt;
  int       b_top, b_left, b_bottom, b_right;
  std::vector<cv::Point> maxima;
  for (int i = 0; i < (int)(accumulator_local_copy_[0].size()); i++) {
    current_known_pt = (USE_VISIBLE_ORIGINS?
        accumulator_local_copy_[0][i]:
        findHoughPeakLocal(accumulator_local_copy_[0][i]));
    b_top         = std::max(0, current_known_pt.y - (int)(nullify_radius_));
    b_left        = std::max(0, current_known_pt.x - (int)(nullify_radius_));
    b_bottom      = std::min(im_res_.height - 1, current_known_pt.y + (int)(nullify_radius_));
    b_right       = std::min(im_res_.width - 1, current_known_pt.x + (int)(nullify_radius_));
    for (int x = b_left; x <= (b_right); x++) {
      for (int y = b_top; y <= (b_bottom); y++) {
        hough_space_maxima_[index2d(x, y)] = 0;
      }
    }
    maxima.push_back(current_known_pt);
  }
  return maxima;
}

void HT4DBlinkerTracker::generateMasks() {
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
        if (radius_box.at< float >(y, x) < (i * max_pixel_shift_)) { // decay - the "yaw" of significantly shifted points should not affect the yaw consensus of the current point
          bool spread_test=false;
          for (int k =-sR; k<=sR; k++){ for (int l =-sR; l<=sR; l++){ // expand the yaw layer of the mask by dilation of the center with a square of the radius sR. This is to generate larger overlaps between layers and thus to enforce consensus of close votes
            if ((((x+l) == center) && ((y+k) == center))){
              spread_test = true;
              break;
            }
            if (spread_test) break;
          } }
          if (spread_test  || (fabs(angDiff(yaw_box.at< float >(y, x), yaw_vals_[j])) < yaw_div_*0.75)) // add elements to the yaw mask if they correspond to the range of "yaw angles" for the current layer
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

void HT4DBlinkerTracker::applyMasks( double i_weight_factor,bool i_constant_newer,int i_break_point) {
  int x, y, z, w;
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

void HT4DBlinkerTracker::flattenTo2D() {
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
    cv::Mat viewer_A = getCvMat(hough_space_maxima_,hough_thresh_*4);
    cv::Mat viewer_B = index_matrix_*(255.0/(double)(pitch_steps_*yaw_steps_));

    cv::hconcat(viewer_A, viewer_B, visualization_);
  }
  return;
}

std::vector< cv::Point > HT4DBlinkerTracker::findHoughPeaks(int peak_count) {
  std::vector< cv::Point > peaks;
  int             curr_max;
  cv::Point       curr_max_pos;
  bool store_current;
  for (int i = 0; i < peak_count; i++) { //repeat for the number of peaks that is expected to appear
    store_current = false;
    curr_max = 0;
    for (int y = 0; y < im_res_.height; y++) { for (int x = 0; x < im_res_.width; x++) { //iterate over X-Y image positions
      if (touched_matrix_[index2d(x, y)] == 0) //save time on positions that have not been affected by mask application
        continue;

      if (hough_space_maxima_[index2d(x,y)] > (unsigned int)curr_max) { //find position with the highest value of the Hough space maximum
        curr_max    = hough_space_maxima_[index2d(x,y)];
        curr_max_pos = cv::Point(x, y);
        store_current = true;
      }
    } }
    if (hough_space_maxima_[index2d(curr_max_pos.x,curr_max_pos.y)] < hough_thresh_) { //stop if the highest remaining value is below threshold
      store_current = false;
      if (debug_)
        std::cout << "Point " << curr_max_pos << " with value of " <<hough_space_maxima_[index2d(curr_max_pos.x,curr_max_pos.y)] <<" Failed threshold test. Threshold is " << hough_thresh_ << ". Breaking." << std::endl;
      break;
    }
    if (!miniFast(curr_max_pos.x,curr_max_pos.y, hough_thresh_/4)) { //check if the position with the retrieved maximum is a concentrated peak
      store_current = false;
      if (debug_)
        std::cout << "Point " << curr_max_pos << " Failed FAST test." << std::endl;
    }

    //nullify elements around the retrieved Hough peak - the next pass should find the next highest peak, corresponing to another origin point
    int b_top, b_left, b_bottom, b_right;
    b_top    = std::max(0, curr_max_pos.y - (int)(nullify_radius_));
    b_left   = std::max(0, curr_max_pos.x - (int)(nullify_radius_));
    b_bottom = std::min(im_res_.height - 1, curr_max_pos.y + (int)(nullify_radius_));
    b_right  = std::min(im_res_.width - 1, curr_max_pos.x + (int)nullify_radius_);
    for (int x = b_left; x <= (b_right); x++) {
      for (int y = b_top; y <= (b_bottom); y++) {
        hough_space_maxima_[index2d(x, y)] = 0;
      }
    }
    if (store_current){
      peaks.push_back(curr_max_pos); //store the current peak
    }
  }
  return peaks;
}

cv::Point HT4DBlinkerTracker::findHoughPeakLocal(cv::Point expected_pos) {
  std::vector< cv::Point > peaks;
  cv::Point                curr_max_pos = expected_pos;
  int b_top, b_left, b_bottom, b_right;
  bool found = false;

  for (int r = 0; r <= ((int)nullify_radius_); r++) { //expanding square outline - look increasingly far away from the expected image position - further positions are less likely, so this saves processing time (note the breaks)
    b_top    = std::max(0, expected_pos.y - (int)(r));
    b_left   = std::max(0, expected_pos.x - (int)(r));
    b_bottom = std::min(im_res_.height - 1, expected_pos.y + (int)r);
    b_right  = std::min(im_res_.width - 1, expected_pos.x + (int)r);

    //check in the square outline with size dependent on r
    for (int y = b_top; y <= (b_bottom); y+=((r==0)?1:(b_bottom-b_top))){ //top and bottom lines of square outline
      for (int x = b_left; x <= (b_right); x++) {
        if (miniFast(x,y, 0)){ //check for peak
          found = true;
          curr_max_pos = cv::Point(x,y);
          break;
        }
      }
      if(found)
        break;
    }
    if(found)
      break;
    for (int x = b_left; x <= (b_right); x+=((r==0)?1:(b_right-b_left))) { //left and right lines of square outline
      for (int y = b_top+1; y <= b_bottom-1; y++){
        if (miniFast(x,y, 0)){ //check for peak
          found = true;
          curr_max_pos = cv::Point(x,y);
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
    std::cout << "Finding for visible: Scaling factor: " << scaling_factor_ << " Thresh: " << hough_thresh_ << " Curr. Peak: " << hough_space_maxima_[index2d(curr_max_pos.x,curr_max_pos.y)] << std::endl;
  return curr_max_pos;
}


double HT4DBlinkerTracker::retrieveFreqency(cv::Point origin_point, double &avg_yaw, double &avg_pitch) {
  //get rough "image pitch" and "image yaw" based on the maxima index in the origin point X-Y cooridnate of the Hough space
  unsigned char init_index = index_matrix_.at< unsigned char >(origin_point);
  unsigned char pitch_index = getPitchIndex(init_index);
  unsigned char yaw_index = getYawIndex(init_index);
  if (debug_){
    std::cout << "Initial pitch, yaw: estimate: [" << pitch_vals_[pitch_index]*(180/M_PI) <<  ", " <<  yaw_vals_[yaw_index]*(180/M_PI) << "] deg" << std::endl;
  }
  int                      step_count = std::min((int)(accumulator_local_copy_.size()), mem_steps_); //do not iterate over the accumulator futher than to the first inserted frame (for initial states when the number of inserted frames is less than mem_steps_)
  double                   rad_expectec_max, rad_expected_min, yaw_expected, curr_point_radius, curr_point_yaw;
  double avg_pitch_cot;
  int curr_point_max_dim, curr_point_radius_round;
  std::vector< cv::Point > positive_point_accum;
  std::vector< cv::Point > positive_point_accum_pitch;
  std::vector<double>      pitch_cot_accum;
  cv::Point                curr_point, curr_point_centerd;
  std::vector< int >       positive_count_accum = std::vector< int >(accumulator_local_copy_.size(), 0); //this represents the binary blinking signal of the presumed marker (0 - off, otherwise - on)
  for (int t = 0; t < step_count; t++) { //iterate over the accumulator frames
    rad_expected_min        = floor(cot_set_min_[pitch_index] * t)-1;
    rad_expectec_max        = ceil(cot_set_max_[pitch_index] * t)+1;
    yaw_expected           = yaw_vals_[yaw_index]-M_PI;
    positive_count_accum[t] = 0;
    for (int k = 0; k < (int)(accumulator_local_copy_[t].size()); k++) { //iterate over the points in the current accumulator frame
      curr_point         = accumulator_local_copy_[t][k];
      curr_point_centerd = curr_point - origin_point;
      curr_point_radius   = cv::norm(curr_point_centerd);
      curr_point_radius_round   = round(curr_point_radius);
      curr_point_max_dim = std::max(curr_point_centerd.x,curr_point_centerd.y);

      curr_point_yaw = atan2(curr_point_centerd.y, curr_point_centerd.x);

      if (curr_point_radius_round >= rad_expected_min){ if (curr_point_radius_round <= rad_expectec_max) { //select points in the expected pitch range (by comparing with the expected range of distances from the origin point)
          if ((fabs(angDiff(curr_point_yaw, yaw_expected)) <= (yaw_div_)) || (curr_point_max_dim <= 4)) { //select points in the expected yaw range or points close enough to the origin point for yaw to be considered meaningless 
            //store pitch and yaw of selected points expressed in two coordinates for easy averaging
            positive_point_accum.push_back(curr_point_centerd);
            positive_point_accum_pitch.push_back(cv::Point(curr_point_radius, t));
            if (t>0)
              pitch_cot_accum.push_back(curr_point_radius/((double)t));
            else
              pitch_cot_accum.push_back(curr_point_radius/(1.0));
            positive_count_accum[t]++;
          }
        } }
      if (debug_){
        if ((curr_point_radius_round < rad_expected_min) || (curr_point_radius_round > rad_expectec_max))
          std::cout << "curr_point_radius: " << curr_point_radius_round << ", t " << t << ":" << k <<" failed vs. [" << rad_expected_min<< "," <<rad_expectec_max << "]"  << std::endl;
        else
          std::cout << "curr_point_radius: " << curr_point_radius_round << ", t " << t << ":" << k <<" passed vs. [" << rad_expected_min<< "," <<rad_expectec_max << "]"  << std::endl;

        if ( ((fabs(angDiff(curr_point_yaw, yaw_expected)) > (yaw_div_)) && (curr_point_max_dim > 4)) )
          std::cout << "curr_point_yaw: " << curr_point_yaw << ", t " << t << ":" << k <<" failed vs. [" << yaw_expected<< "] by "  << angDiff(curr_point_yaw, yaw_expected) << " vs " << yaw_div_ << std::endl;
        else
          std::cout << "curr_point_yaw: " << curr_point_yaw << ", t " << t << ":" << k <<" passed vs. [" << yaw_expected<< "] by "  << angDiff(curr_point_yaw, yaw_expected) << " vs " << yaw_div_ << std::endl;
      }
    }
  }
  if (positive_point_accum.size() == 0) { //if no points were collected around the estimated trajectory, return severe error code - this should never happen
    return -666.0;
  }

  if (debug_){
    std::cout << "Accumulated:" << std::endl;
    for (int i = 0; i < (int)(positive_count_accum.size()); i++) {
      std::cout << positive_count_accum[i];
    }
    std::cout << std::endl;
  }

  //get averages for more precise trajectory estimates - the original estimates were quantized by the Hough space
  avg_yaw         = angMeanXY(positive_point_accum);
  avg_pitch       = angMeanXY(positive_point_accum_pitch);
  avg_pitch_cot   = accumulate(pitch_cot_accum.begin(), pitch_cot_accum.end(), 0.0)/pitch_cot_accum.size(); 
  std::vector< bool > correct = std::vector< bool >(positive_point_accum.size(), true);
  for (int u = 0; u < (int)(positive_point_accum.size()); u++) { //iterate over the previously selected points
    cv::Point exp_pt(cos(avg_yaw)*avg_pitch_cot*positive_point_accum_pitch[u].y, sin(avg_yaw)*avg_pitch_cot*positive_point_accum_pitch[u].y);
    if (floor(cv::norm(exp_pt-positive_point_accum[u])) > (reasonable_radius_)) { //mark points that do not comply with the new trajectory as outliers
      if (debug_){
        std::cout << "Culling" << std::endl;
        std::cout << "avg_pitch_cot: " << avg_pitch_cot << " avg_yaw " <<  avg_yaw << std::endl;
        std::cout << "pitchCotSum: " << accumulate(pitch_cot_accum.begin(), pitch_cot_accum.end(), 0.0)<< " pitchCotSize " <<  pitch_cot_accum.size() << std::endl;
        std::cout << "diff: " << exp_pt << " vs " <<  positive_point_accum[u] << std::endl;
      }
      correct[u] = false;
    }
  }
  int o = 0;
  for (int u = 0; u < (int)(correct.size()); u++) { //remove the marked outlier points
    if (!correct[u]) {
      positive_count_accum[positive_point_accum_pitch[o].y]--;
      positive_point_accum.erase(positive_point_accum.begin() + o);
      positive_point_accum_pitch.erase(positive_point_accum_pitch.begin() + o);
      o--;
    }
    o++;
  }

  //recalculate averages with the cleaned up point set to suppres the influence of outliers on the estimated point trajectory line
  avg_yaw   = angMeanXY(positive_point_accum);
  avg_pitch = angMeanXY(positive_point_accum_pitch);

  if (debug_){
    std::cout << "After culling" << std::endl;
    for (int i = 0; i < (int)(positive_count_accum.size()); i++) {
      std::cout << positive_count_accum[i];
    }
    std::cout << std::endl;
  }

  //retrieve frequency (or detect incorrect signal) using a state machine pass over the retrieved signal
  bool   state                  = false;
  bool   prev_state             = state;
  bool   down_step              = false;
  int    last_down_step_index   = 0;
  bool   up_step                = false;
  int    last_up_step_index     = 0;
  double period;
  double up_dur, down_dur;
  double max_period = 0;
  double min_period = mem_steps_;
  int    cnt_period = 0;
  int    sum_period = 0;
  for (int t = 0; t < (int)(accumulator_local_copy_.size()); t++) { //iterate over the accumulator frames
    if (positive_count_accum[t] > 0) //set current value of the signal based on presence of selected points in the current accumulator frame
      state = true;
    else
      state = false;

    if (!state && prev_state) { //falling step
      up_dur = (t-last_up_step_index);
      if (down_step) {
        period            = (t - last_down_step_index);
        if (period < min_period)
          min_period = period;
        if (period > max_period)
          max_period = period;
        last_down_step_index = t;
        sum_period         = sum_period + period;
        cnt_period = cnt_period + 1;
      }
      if (!down_step) { //never encountered falling step
        last_down_step_index = t;
        down_step          = true;
      } 
    }
    if (state && !prev_state && (t > 0)) { //rising step (accounting for the initial state)
      down_dur = (t-last_down_step_index);
      if (up_step) {
        period          = (t - last_up_step_index);
        if (period < min_period)
          min_period = period;
        if (period > max_period)
          max_period = period;
        last_up_step_index = t;
        sum_period       = sum_period + period;
        cnt_period = cnt_period + 1;
      }
      if (!up_step) { //never encountered rising step
        last_up_step_index = t;
        up_step          = true;
      }
    }
    prev_state = state;
  }

  double avg_period;

  if (cnt_period == 0){
    if (debug_){
      std::cout << "Not one whole period retrieved, returning;" <<std::endl;
    }
    return -1;
  } else {
    avg_period = (double)(sum_period) / (double)cnt_period;
  }

  if ((max_period - min_period) > ceil(avg_period/2)){
    if (debug_)
      std::cout << "Spread too wide: "<<max_period-min_period<<" compared to average of " << avg_period <<", returning" <<std::endl;
    return -3;
  }

  if ((avg_period * ((double)(cnt_period+1)/2.0) ) < ( mem_steps_ - (1.5*avg_period))){
    if (debug_){
      std::cout << "Not enough periods retrieved: " << cnt_period << " for " << avg_period <<" on average; returning" <<std::endl;
    }
    return -4;
  }
  if ((cnt_period == 1) && (fabs(up_dur-down_dur)>(avg_period/2))){
    if (debug_){
      std::cout << "Long period with uneven phases: "<< up_dur-down_dur <<" for " << avg_period <<" on average; returning" <<std::endl;
    }
    return -5;
  }


  if (debug_){
    std::cout << "Frequency: " << (double)framerate_ / avg_period << std::endl;
  }
  return (double)framerate_ / avg_period;
}

int HT4DBlinkerTracker::retrieveSignalID(cv::Point origin_point, double &avg_yaw, double &avg_pitch, std::vector<bool> &blink_signal) {
  blink_signal = retrieveSignalSequence(origin_point, avg_yaw, avg_pitch);

  int ret_id = matcher_->matchSignal(blink_signal);
  std::cout << "Signal ID is: " << ret_id << std::endl;
  return  ret_id;

  /* //retrieve frequency (or detect incorrect signal) using a state machine pass over the retrieved signal */
  /* bool   state                  = false; */
  /* bool   prev_state             = state; */
  /* bool   down_step              = false; */
  /* int    last_down_step_index   = 0; */
  /* bool   up_step                = false; */
  /* int    last_up_step_index     = 0; */
  /* double period; */
  /* double up_dur, down_dur; */
  /* double max_period = 0; */
  /* double min_period = mem_steps_; */
  /* int    cnt_period = 0; */
  /* int    sum_period = 0; */
  /* for (int t = 0; t < (int)(accumulator_local_copy_.size()); t++) { //iterate over the accumulator frames */
  /*   if (positive_count_accum[t] > 0) //set current value of the signal based on presence of selected points in the current accumulator frame */
  /*     state = true; */
  /*   else */
  /*     state = false; */

  /*   if (!state && prev_state) { //falling step */
  /*     up_dur = (t-last_up_step_index); */
  /*     if (down_step) { */
  /*       period            = (t - last_down_step_index); */
  /*       if (period < min_period) */
  /*         min_period = period; */
  /*       if (period > max_period) */
  /*         max_period = period; */
  /*       last_down_step_index = t; */
  /*       sum_period         = sum_period + period; */
  /*       cnt_period = cnt_period + 1; */
  /*     } */
  /*     if (!down_step) { //never encountered falling step */
  /*       last_down_step_index = t; */
  /*       down_step          = true; */
  /*     } */ 
  /*   } */
  /*   if (state && !prev_state && (t > 0)) { //rising step (accounting for the initial state) */
  /*     down_dur = (t-last_down_step_index); */
  /*     if (up_step) { */
  /*       period          = (t - last_up_step_index); */
  /*       if (period < min_period) */
  /*         min_period = period; */
  /*       if (period > max_period) */
  /*         max_period = period; */
  /*       last_up_step_index = t; */
  /*       sum_period       = sum_period + period; */
  /*       cnt_period = cnt_period + 1; */
  /*     } */
  /*     if (!up_step) { //never encountered rising step */
  /*       last_up_step_index = t; */
  /*       up_step          = true; */
  /*     } */
  /*   } */
  /*   prev_state = state; */
  /* } */

  /* double avg_period; */

  /* if (cnt_period == 0){ */
  /*   if (debug_){ */
  /*     std::cout << "Not one whole period retrieved, returning;" <<std::endl; */
  /*   } */
  /*   return -1; */
  /* } else { */
  /*   avg_period = (double)(sum_period) / (double)cnt_period; */
  /* } */

  /* if ((max_period - min_period) > ceil(avg_period/2)){ */
  /*   if (debug_) */
  /*     std::cout << "Spread too wide: "<<max_period-min_period<<" compared to average of " << avg_period <<", returning" <<std::endl; */
  /*   return -3; */
  /* } */

  /* if ((avg_period * ((double)(cnt_period+1)/2.0) ) < ( mem_steps_ - (1.5*avg_period))){ */
  /*   if (debug_){ */
  /*     std::cout << "Not enough periods retrieved: "<< cnt_period <<" for " << avg_period <<" on average; returning" <<std::endl; */
  /*   } */
  /*   return -4; */
  /* } */
  /* if ((cnt_period == 1) && (fabs(up_dur-down_dur)>(avg_period/2))){ */
  /*   if (debug_){ */
  /*     std::cout << "Long period with uneven phases: "<< up_dur-down_dur <<" for " << avg_period <<" on average; returning" <<std::endl; */
  /*   } */
  /*   return -5; */
  /* } */


  /* if (debug_){ */
  /*   std::cout << "Frequency: " << (double)framerate_ / avg_period << std::endl; */
  /* } */
  /* return (double)framerate_ / avg_period; */
}

std::vector<bool> HT4DBlinkerTracker::retrieveSignalSequence(cv::Point origin_point, double &avg_yaw, double &avg_pitch) {
  //get signal, as well as rough "image pitch" and "image yaw" based on the maxima index in the origin point X-Y cooridnate of the Hough space
  unsigned char init_index = index_matrix_.at< unsigned char >(origin_point);
  unsigned char pitch_index = getPitchIndex(init_index);
  unsigned char yaw_index = getYawIndex(init_index);
  if (debug_){
    std::cout << "Initial pitch, yaw: estimate: [" << pitch_vals_[pitch_index]*(180/M_PI) <<  ", " <<  yaw_vals_[yaw_index]*(180/M_PI) << "] deg" << std::endl;
  }
  int                      step_count = std::min((int)(accumulator_local_copy_.size()), mem_steps_); //do not iterate over the accumulator futher than to the first inserted frame (for initial states when the number of inserted frames is less than mem_steps_)
  double                   rad_expectec_max, rad_expected_min, yaw_expected, curr_point_radius, curr_point_yaw;
  double avg_pitch_cot;
  int curr_point_max_dim, curr_point_radius_round;
  std::vector< cv::Point > positive_point_accum;
  std::vector< cv::Point > positive_point_accum_pitch;
  std::vector<double>      pitch_cot_accum;
  cv::Point                curr_point, curr_point_centerd;
  std::vector< int >       positive_count_accum = std::vector< int >(accumulator_local_copy_.size(), 0); //this represents the binary blinking signal of the presumed marker (0 - off, otherwise - on)
  for (int t = 0; t < step_count; t++) { //iterate over the accumulator frames
    rad_expected_min        = floor(cot_set_min_[pitch_index] * t)-1;
    rad_expectec_max        = ceil(cot_set_max_[pitch_index] * t)+1;
    yaw_expected           = yaw_vals_[yaw_index]-M_PI;
    positive_count_accum[t] = 0;
    for (int k = 0; k < (int)(accumulator_local_copy_[t].size()); k++) { //iterate over the points in the current accumulator frame
      curr_point         = accumulator_local_copy_[t][k];
      curr_point_centerd = curr_point - origin_point;
      curr_point_radius   = cv::norm(curr_point_centerd);
      curr_point_radius_round   = round(curr_point_radius);
      curr_point_max_dim = std::max(curr_point_centerd.x,curr_point_centerd.y);

      curr_point_yaw = atan2(curr_point_centerd.y, curr_point_centerd.x);

      if (curr_point_radius_round >= rad_expected_min){ if (curr_point_radius_round <= rad_expectec_max) { //select points in the expected pitch range (by comparing with the expected range of distances from the origin point)
          if ((fabs(angDiff(curr_point_yaw, yaw_expected)) <= (yaw_div_)) || (curr_point_max_dim <= 4)) { //select points in the expected yaw range or points close enough to the origin point for yaw to be considered meaningless 
            //store pitch and yaw of selected points expressed in two coordinates for easy averaging
            positive_point_accum.push_back(curr_point_centerd);
            positive_point_accum_pitch.push_back(cv::Point(curr_point_radius, t));
            if (t>0)
              pitch_cot_accum.push_back(curr_point_radius/((double)t));
            else
              pitch_cot_accum.push_back(curr_point_radius/(1.0));
            positive_count_accum[t]++;
          }
        } }
      if (debug_){
        if ((curr_point_radius_round < rad_expected_min) || (curr_point_radius_round > rad_expectec_max))
          std::cout << "curr_point_radius: " << curr_point_radius_round << ", t " << t << ":" << k <<" failed vs. [" << rad_expected_min<< "," <<rad_expectec_max << "]"  << std::endl;
        else
          std::cout << "curr_point_radius: " << curr_point_radius_round << ", t " << t << ":" << k <<" passed vs. [" << rad_expected_min<< "," <<rad_expectec_max << "]"  << std::endl;

        if ( ((fabs(angDiff(curr_point_yaw, yaw_expected)) > (yaw_div_)) && (curr_point_max_dim > 4)) )
          std::cout << "curr_point_yaw: " << curr_point_yaw << ", t " << t << ":" << k <<" failed vs. [" << yaw_expected<< "] by "  << angDiff(curr_point_yaw, yaw_expected) << " vs " << yaw_div_ << std::endl;
        else
          std::cout << "curr_point_yaw: " << curr_point_yaw << ", t " << t << ":" << k <<" passed vs. [" << yaw_expected<< "] by "  << angDiff(curr_point_yaw, yaw_expected) << " vs " << yaw_div_ << std::endl;
      }
    }
  }
  if (positive_point_accum.size() == 0) { //if no points were collected around the estimated trajectory, return severe error code - this should never happen
    /* return -666; */
    std::cerr << "No markers were found around the estimated trajectory!" << std::endl;
    return std::vector<bool>();
  }

  if (debug_){
    std::cout << "Accumulated:" << std::endl;
    for (int i = 0; i < (int)(positive_count_accum.size()); i++) {
      std::cout << positive_count_accum[i];
    }
    std::cout << std::endl;
  }

  //get averages for more precise trajectory estimates - the original estimates were quantized by the Hough space
  avg_yaw         = angMeanXY(positive_point_accum);
  avg_pitch       = angMeanXY(positive_point_accum_pitch);
  avg_pitch_cot   = accumulate(pitch_cot_accum.begin(), pitch_cot_accum.end(), 0.0)/pitch_cot_accum.size(); 
  std::vector< bool > correct = std::vector< bool >(positive_point_accum.size(), true);
  for (int u = 0; u < (int)(positive_point_accum.size()); u++) { //iterate over the previously selected points
    cv::Point exp_pt(cos(avg_yaw)*avg_pitch_cot*positive_point_accum_pitch[u].y, sin(avg_yaw)*avg_pitch_cot*positive_point_accum_pitch[u].y);
    if (floor(cv::norm(exp_pt-positive_point_accum[u])) > (reasonable_radius_)) { //mark points that do not comply with the new trajectory as outliers
      if (debug_){
        std::cout << "Culling" << std::endl;
        std::cout << "avg_pitch_cot: " << avg_pitch_cot << " avg_yaw " <<  avg_yaw << std::endl;
        std::cout << "pitchCotSum: " << accumulate(pitch_cot_accum.begin(), pitch_cot_accum.end(), 0.0)<< " pitchCotSize " <<  pitch_cot_accum.size() << std::endl;
        std::cout << "diff: " << exp_pt << " vs " <<  positive_point_accum[u] << std::endl;
      }
      correct[u] = false;
    }
  }
  int o = 0;
  for (int u = 0; u < (int)(correct.size()); u++) { //remove the marked outlier points
    if (!correct[u]) {
      positive_count_accum[positive_point_accum_pitch[o].y]--;
      positive_point_accum.erase(positive_point_accum.begin() + o);
      positive_point_accum_pitch.erase(positive_point_accum_pitch.begin() + o);
      o--;
    }
    o++;
  }

  //recalculate averages with the cleaned up point set to suppres the influence of outliers on the estimated point trajectory line
  avg_yaw   = angMeanXY(positive_point_accum);
  avg_pitch = angMeanXY(positive_point_accum_pitch);

  if (debug_){
    std::cout << "After culling" << std::endl;
    for (int i = 0; i < (int)(positive_count_accum.size()); i++) {
      std::cout << positive_count_accum[i];
    }
    std::cout << std::endl;
  }

  std::vector<bool> output;
  /* for (auto& pos_count : positive_count_accum){ */
  for (auto it=positive_count_accum.end(); it!=positive_count_accum.begin(); --it){
    output.push_back(*it > 0);
  }
  return output;

}

cv::Mat HT4DBlinkerTracker::getCvMat(unsigned int *__restrict__ input, unsigned int threshold){
  cv::Mat output(im_res_,CV_8UC1);
  for (int i=0; i<im_res_.height; i++){
    for (int j=0; j<im_res_.width; j++){
      output.data[index2d(j,i)] = input[index2d(j,i)]*(255.0/threshold);
    } }
  return output;
}

cv::Mat HT4DBlinkerTracker::getVisualization(){
  return visualization_;
}

bool HT4DBlinkerTracker::miniFast(int x, int y, unsigned int thresh) {
  int border = (SMALLER_FAST?3:4);

  //check for potential breach of the image borders
  if (x<border)
    return false;
  if (y<border)
    return false;
  if (x>(im_res_.width-(border+1)))
    return false;
  if (y>(im_res_.height-(border+1)))
    return false;

  bool found_one_fit = false;
  int diff;
  for (int i = 0; i < (int)(fast_points_.size()); i++) { //iterate over the previously generated FAST-like sampling points
    diff = 
      (hough_space_maxima_[index2d(x, y)] - hough_space_maxima_[index2d(x + fast_points_[i].x, y + fast_points_[i].y)]);
    if (diff > (int)(thresh)){ //check for surrounding points that are not sufficiently different from the center
      found_one_fit = true;
    }
    else
      return false;
  }

  return found_one_fit;
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
