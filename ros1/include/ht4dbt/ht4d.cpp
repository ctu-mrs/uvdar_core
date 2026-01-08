#include "ht4d.h"


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
    int i_allowed_BER_per_seq,
    int i_nullify_radius,
    int i_reasonable_radius,
    double i_framerate) {
  std::cout << "Initiating HT4DBlinkerTracker..." << std::endl;
  mem_steps_      = i_mem_steps;
  pitch_steps_    = i_pitch_steps;
  yaw_steps_      = i_yaw_steps;
  total_steps_    = pitch_steps_*yaw_steps_;
  framerate_     = i_framerate;
  allowed_BER_per_seq_ = i_allowed_BER_per_seq;
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
    (unsigned int)((mem_steps_ ) * (1+(weight_coeff*WEIGHT_FACTOR)) * 0.5 * 1.0 * scaling_factor_); // threshold over which we expect maxima to appear
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
  accumulator_.push_back(std::vector< cv::Point2i >());
  accumulator_local_copy_.push_back(std::vector< cv::Point2i >());

  hough_space_maxima_ = new unsigned int[im_area_];
  index_matrix_           = cv::Mat(im_res_, CV_8UC1, cv::Scalar(0));
  touched_matrix_ = new unsigned char[im_area_];
  resetToZero(touched_matrix_, im_area_);
  
  initFast();

  curr_batch_processed_ = false;

  std::cout << "...finished." << std::endl;
  return;
}

HT4DBlinkerTracker::~HT4DBlinkerTracker() {
  delete[] hough_space_maxima_;
  delete[] touched_matrix_;
}

void HT4DBlinkerTracker::resetToZero(unsigned int *input, int steps) {
  for (int i = 0; i < steps; i++) {
    input[i] = (unsigned int)(0);
  }
}

void HT4DBlinkerTracker::resetToZero(unsigned char *input, int steps) {
  for (int i = 0; i < steps; i++) {
    input[i] = (unsigned char)(0);
  }
}

void HT4DBlinkerTracker::setSequences(std::vector<std::vector<bool>> i_sequences){
  sequences_ = i_sequences;
  matcher_ = std::make_unique<SignalMatcher>(sequences_, allowed_BER_per_seq_);
}

void HT4DBlinkerTracker::setDebug(bool i_debug, bool i_vis_debug) {
  debug_    = i_debug;
  vis_debug_ = i_vis_debug;
}

void HT4DBlinkerTracker::updateFramerate(double input) {
  if (input > 1.0)
    framerate_ = input;
}

void HT4DBlinkerTracker::updateInterfaceResolution(cv::Size i_size){
  if (debug_){
    std::cout << "Setting resolution to " << i_size << std::endl;
  }
  std::scoped_lock lock(mutex_accumulator_);
  im_res_ = i_size;
  im_area_           = im_res_.width * im_res_.height;
  im_rect_           = cv::Rect(cv::Point(0, 0), im_res_);

  accumulator_.clear();
  curr_batch_processed_ = false;

  delete[] hough_space_maxima_;
  hough_space_maxima_ = new unsigned int[im_area_];
  index_matrix_           = cv::Mat(im_res_, CV_8UC1, cv::Scalar(0));
  delete[] touched_matrix_;
  touched_matrix_ = new unsigned char[im_area_];
  resetToZero(touched_matrix_, im_area_);
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
    /* if (!miniFast(curr_max_pos.x,curr_max_pos.y, hough_thresh_/4)) { //check if the position with the retrieved maximum is a concentrated peak */
    /*   store_current = false; */
    /*   if (debug_) */
    /*     std::cout << "Point " << curr_max_pos << " Failed FAST test." << std::endl; */
    /* } */

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

int HT4DBlinkerTracker::retrieveSignalID(cv::Point origin_point, double &avg_yaw, double &avg_pitch, std::vector<bool> &blink_signal) {
  blink_signal = retrieveSignalSequence(origin_point, avg_yaw, avg_pitch);

  int ret_id = matcher_->matchSignal(blink_signal);
  if (debug_)
    std::cout << "Signal ID is: " << ret_id << std::endl;
  return  ret_id;

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
      /* if (debug_){ */
      if (false){
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
  for (auto it=positive_count_accum.rbegin(); it!=positive_count_accum.rend(); ++it){
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

bool HT4DBlinkerTracker::getResultsStart() {
  if ((im_res_.width <= 0) || (im_res_.height <= 0)){
    if (debug_){
      std::cout << "Resolution was not yet set..." << std::endl;
    }
    return false;
  }

  accumulator_local_copy_.clear();
  pts_per_layer_local_copy_.clear();
  int points_total_count = 0;
  {
    std::scoped_lock lock(mutex_accumulator_);
    for (int i = 0; i < (int)(accumulator_.size()); i++) {
      accumulator_local_copy_.push_back(std::vector< cv::Point2i >());
      for (int j = 0; j < (int)(accumulator_[i].size()); j++) {
        accumulator_local_copy_[i].push_back(accumulator_[i][j]);
      }
      pts_per_layer_local_copy_.push_back(pts_per_layer_[i]);
      points_total_count +=pts_per_layer_[i];
    }
  }
  if (pts_per_layer_local_copy_.empty()){
    return false;
  }

  expected_matches_ = *std::max_element(pts_per_layer_local_copy_.begin(), pts_per_layer_local_copy_.end()) - pts_per_layer_local_copy_[0];
  if (debug_){
    std::cout << "Exp. Matches: " << expected_matches_ << std::endl;
    std::cout << "Visible Matches: " << pts_per_layer_local_copy_[0] << std::endl;
  }
  return true;
}

std::vector< std::pair<cv::Point2d,int> > HT4DBlinkerTracker::getResultsEnd() {
  if (vis_debug_) {
    cv::Mat viewer_A = getCvMat(hough_space_maxima_,hough_thresh_*4);
    cv::Mat viewer_B = index_matrix_*(255.0/(double)(pitch_steps_*yaw_steps_));

    cv::hconcat(viewer_A, viewer_B, visualization_);
  }

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

  if (debug_){
    std::cout << "Retrieved signal: " << std::endl;
    for (int i = 0; i < (int)(blink_signal.size()); i++) {
      std::cout << (int)(blink_signal[i]) << ",";
    }
    std::cout << std::endl;
  }

    yaw_averages_.push_back(yawAvg);
    pitch_averages_.push_back(pitchAvg);
  }
  /* if (debug_){ */
  if (false){
    std::cout << "Differences from the detected: [" << std::endl;
    for (int op = 0; op < (int)(origin_pts.size()); op++){
      std::cout << origin_pts[op] - origin_pts_out[op] << std::endl;
    }
    std::cout << "]" << std::endl;
  }
  curr_batch_processed_ = true;
  return result;
}

