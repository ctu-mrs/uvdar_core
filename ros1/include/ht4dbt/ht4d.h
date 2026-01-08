#ifndef HT4D_H
#define HT4D_H

#include <mutex>
#include <memory>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <iostream>

#include "opencv2/highgui/highgui.hpp"
#include "signal_matcher/signal_matcher.h"


#define WEIGHT_FACTOR 0.0 // we prioritize blinking signal retrieval to origin point position accuracy - the paper is outdated in this respect

#define USE_VISIBLE_ORIGINS true // if there is a visible marker in the latest time, prefer its position to what is estimated by the HT

#define SMALLER_FAST false // find Hough peaks by neighborhood of radius of 4 pixels

#define CONSTANT_NEWER false // defines whether (if inputs are weighted in favor of the most recent) a number of newest inputs should retain equal weight

namespace uvdar {

/**
 * @brief The class for retrieving frequencies and image positions of moving blinking markers
 */
class HT4DBlinkerTracker {
public:

  /**
   * @brief The constructor of the class
   *
   * @param i_mem_steps - The length of the accumulator in terms of the number of past camera frames (determines the minimal frequency retrievable by limiting the longest period that can fit in the accumulator - the higher the value the slower the calculation and the worse the reliability in case of second-order dynamics of the marker images)
   * @param i_pitch_steps - The resolution of the "Pitch" dimension of the 4D Hough space (corresponds to the speed in the image at which the point has been moving)
   * @param i_yaw_steps - The resolution of the "Yaw" dimension of the 4D Hough space (corresponds to the direction in the image in which the point has been moving)
   * @param i_max_pixel_shift - The maximum number of pixels in either dimension the marker image is expected to move per frame
   * @param i_im_res - The initial resolution of input image. Determines the resolution of the "X" and "Y" dimensions of the Hough space. This can be updated with the updateResolution method
   * @param i_nullify_radius - Defines the side (in pixels) of the rectangle centered on an X-Y position of the Hough space inside of which matrix elements will be nullified while searching for maxima
   * @param i_reasonable_radius - Radius (in pixels) around an estimated image trajectory line, inside of which the input points will be counted for the final blinking signal retrieval
   * @param i_framerate - The initial expected framerate (in hz) of the input stream. The requisite minimum framerate must follow the Nyquist criterion for signal retrieval (more than twice the highest expected blinking frequency). This value can be changed on the fly with the updateFramerate method.
   */
  HT4DBlinkerTracker(
      int i_mem_steps,
      int i_pitch_steps,
      int i_yaw_steps,
      int i_max_pixel_shift,
      cv::Size i_im_res,
      int i_allowed_BER_per_seq,
      int i_nullify_radius,
      int i_reasonable_radius,
      double i_framerate);

  ~HT4DBlinkerTracker();

  /**
   * @brief Inserts a set of image points corresponding to the markers in a single new image frame
   *
   * @param new_points - The input image points
   */
  void insertFrame(std::vector< cv::Point > new_points);

  /**
   * @brief Retrieval of the blinking image points
   *
   * @return - A set of image points - x and y correspond to the expected position of a marker in the newest frame and z corresponds to its blinking frequency (or an error code for points with wrong blinking signal)
   */

  virtual std::vector< std::pair<cv::Point2d,int> > getResults() = 0;

  /**
   * @brief Sets the image resolution of the input image (and consequently of all processing matrices such as Hough spaces) to a new value
   *
   * @param i_size - the new image resolution
   */
  virtual void updateResolution(cv::Size i_size) = 0;

  /**
   * @brief Returns the number of tracked image points obtained in the last retrieval cycle. Not all of these will be blinking with an expected signal - some may be e.g. static reflections
   *
   * @return - The number of the persistent image points
   */
  int getTrackerCount();

  /**
   * @brief Retrieves the blinking signal of a given marker obtained in the last retrieval cycle
   *
   * @param index - The index of the retrieved marker
   *
   * @return - The retrieved signal
   */
  std::vector<bool> getSignal(int index);

  /**
   * @brief Retrieves the ID associated with the blinkig signal of a selected marker
   *
   * @param origin_point - The image point presumed to correspond to a blinking marker
   * @param avg_yaw - The output "image yaw" of the blinking marker (corresponds to the direction in the image in which the point has been moving)
   * @param avg_pitch - The output "image pitch" of the blinking marker (corresponds to the speed in the image at which the point has been moving)
   * @param blink_signal - The output binary blinking signal of the blinking marker
   *
   * @return - The retrieved signal ID
   */
  int retrieveSignalID(cv::Point origin_point, double &avg_yaw, double &avg_pitch, std::vector<bool> &blink_signal);


  /**
   * @brief Retrieves the ID associated with the blinkig signal of a selected marker
   *
   * @param origin_point - The image point presumed to correspond to a blinking marker
   * @param avg_yaw - The output "image yaw" of the blinking marker (corresponds to the direction in the image in which the point has been moving)
   * @param avg_pitch - The output "image pitch" of the blinking marker (corresponds to the speed in the image at which the point has been moving)
   *
   * @return - The output binary blinking signal of the blinking marker
   */
  std::vector<bool> retrieveSignalSequence(cv::Point origin_point, double &avg_yaw, double &avg_pitch);

  /**
   * @brief Retrieves the "image yaw" of a given marker obtained in the last retrieval cycle (corresponds to the direction in the image in which the point has been moving)
   *
   * @param index - The index of the retrieved marker
   *
   * @return - The retrieved "image yaw"
   */
  double getYaw(int index);

  /**
   * @brief Retrieves the "image pitch" of a given marker obtained in the last retrieval cycle (corresponds to the speed in the image in which the point has been moving)
   *
   * @param index - The index of the retrieved marker
   *
   * @return - The retrieved "image pitch"
   */
  double getPitch(int index);

  /**
   * @brief Retrieves the set of "image yaw" values of a markers obtained in the last retrieval cycle (they correspond to the direction in the image in which the point has been moving)
   *
   * @return - The retrieved "image yaw" values
   */
  std::vector<double> getYaw();

  /**
   * @brief Retrieves the set of "image pitch" values of a markers obtained in the last retrieval cycle (they correspond to the speed in the image in which the point has been moving)
   *
   * @return - The retrieved "image pitch" values
   */
  std::vector<double> getPitch();

  /**
   * @brief Informs on whether the current set of points in the accumulator has already been processed (this flag is reset upon every new received set of points)
   *
   * @return - True if the batch has been processed already
   */
  bool isCurrentBatchProcessed();

  /**
   * @brief Sets the expected current input framerate to a new value
   *
   * @param input - The new framerate (in hz)
   */
  void updateFramerate(double input);

  /**
   * @brief Change the blinking signal templates
   *
   * @param i_sequences - Defines the new set of template signals
   */
  void setSequences(std::vector<std::vector<bool>> i_sequences);

  /**
   * @brief Change the debugging level of the class
   *
   * @param i_debug - Defines whether console debugging is active
   * @param i_vis_debug - Defines whether visual debugging is active (use with care, if active this has significant performance impact)
   */
  void setDebug(bool i_debug, bool i_vis_debug);


  /**
   * @brief Returns the OpenCV matrix with the latest visualization. This visualization is only generated if `i_vis_debug` is set to true with the setDebug method
   *
   * @return - The visualization OpenCV matrix. If visual debugging has not been activated yet, this will be an empty matrix.
   */
  cv::Mat getVisualization();

  /**
   * @brief Sets the image resolution of the input image to a new value
   *
   * @param i_size - the new image resolution
   */
  void updateInterfaceResolution(cv::Size i_size);

  /**
   * @brief Resets an array of numbers to zero
   *
   * @param steps - Number of elements from the initial address to reset
   */
  void resetToZero(unsigned int *input, int steps);

  /**
   * @brief Resets an array of numbers to zero
   *
   * @param steps - Number of elements from the initial address to reset
   */
  void resetToZero(unsigned char *input, int steps);

protected:

    /**
     * @brief Arccotangent function
     *
     * @param input - The cotangent of the desired angle (must not be 0)
     *
     * @return - Arccotangent of the input
     */
  double acot(double input);

  /**
   * @brief Generates OpenCV matrix out of internal 2D array for optional visualization
   *
   * @param input - The address of the given 2D array
   * @param threshold - Maximum value of the 2D array - will be represented by pure white in the OpenCV matrix
   *
   * @return - The visualization OpenCV matrix
   */
  cv::Mat getCvMat(unsigned int *__restrict__ input, unsigned int threshold);

  /**
   * @brief Retrieves peaks in the Hough space
   *
   * @param peak_count - The expected number of hough peaks to seek
   *
   * @return - A vector of the retrieved image points corresponding to the peaks
   */
  std::vector< cv::Point > findHoughPeaks(int peak_count);

  /**
   * @brief Find a single peak in the Hough space in the vicinity of the expected image position
   *
   * @param expected_pos - The image position nearby the expected peak
   *
   * @return - The position of the found peak
   */
  cv::Point findHoughPeakLocal(cv::Point expected_pos);

  /**
   * @brief Sets values in matrices at indices corresponding to the visible markers in the newest frame to zero 
   *
   * @return - The image points the surroundings of which have been nullified
   */
  std::vector<cv::Point> nullifyKnown();
  /**
   * @brief Checks if a position in the flattened Hough space is a peak using a method similar to FAST. Must be preceded by calling initFast.
   *
   * @param x - The X coordinate of the image point
   * @param y - The Y coordinate of the image point
   * @param thresh - The minimum value difference between the given point and its surroundings for it to be considered a peak
   *
   * @return 
   */
  bool miniFast(int x, int y, unsigned int thresh);


  bool getResultsStart();

  std::vector< std::pair<cv::Point2d,int> > getResultsEnd();

  /**
   * @brief - Initializes points used in the miniFast method
   */
  void initFast();

  /**
   * @brief Specialized modulo function for use in angle difference calculation
   *
   * @param a - Numerator
   * @param n - Denominator
   *
   * @return - The modulo of a and n
   */
  double mod2(double a, double n);

  /**
   * @brief The smallest difference betewen two angles expressed w.r.t. common zero angle
   *
   * @param a - The first angle
   * @param b - The second angle
   *
   * @return - The angle difference
   */
  double angDiff(double a, double b);

  /**
   * @brief Returns the mean polar angle of the input points w.r.t. the origin
   *
   * @param input - A vector of image points, representing a set of polar angles
   *
   * @return - The mean polar angle
   */
  double angMeanXY(std::vector< cv::Point > input);

  template < typename T >
  inline T index2d(T X, T Y) { return (im_res_.width * (Y) + (X)); }

  template < typename T >
  inline T index3d(T X, T Y, T Z) { return (im_area_ * (Z) + im_res_.width * (Y) + (X)); }

  template < typename T >
  inline T indexYP(T X, T Y) { return (pitch_steps_ * (Y) + (X)); }

  template < typename T >
  inline T getPitchIndex(T X) { return ((X) % pitch_steps_); }

  template < typename T >
  inline T getYawIndex(T X) { return ((X) / pitch_steps_); }

  int          mem_steps_, pitch_steps_, yaw_steps_, total_steps_;
  unsigned int frame_scale_, hough_thresh_, nullify_radius_;
  double       scaling_factor_;
  int          mask_width_;
  int          expected_matches_;
  double       min_pitch_, pitch_div_, yaw_div_, step_div_;
  int          reasonable_radius_;
  double       framerate_;
  int          max_pixel_shift_;
  int          allowed_BER_per_seq_;

  cv::Size     im_res_;
  unsigned int im_area_;
  cv::Rect     im_rect_;

  std::vector< std::vector< cv::Point2i > > accumulator_;
  std::vector< std::vector< cv::Point2i > > accumulator_local_copy_;
  std::vector< int >                        pts_per_layer_;
  std::vector< int >                        pts_per_layer_local_copy_;
  cv::Mat                                   index_matrix_;
  unsigned char * touched_matrix_;
  unsigned int * __restrict__ hough_space_maxima_;
  std::vector< cv::Point > fast_points_;
  std::vector< double >                     pitch_vals_,
                                            yaw_vals_,
                                            cot_set_min_,
                                            cot_set_max_,
                                            sin_set_,
                                            cos_set_;

  std::vector< double > yaw_averages_, pitch_averages_;
  std::vector<std::vector<bool>> signals_;

  bool curr_batch_processed_;

  std::mutex mutex_accumulator_;

  std::unique_ptr<SignalMatcher> matcher_;

  bool debug_, vis_debug_;

  std::vector<std::vector<bool>> sequences_;

  cv::Mat visualization_;
};

} //namespace uvdar

#endif // HT4D_H
