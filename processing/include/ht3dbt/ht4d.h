#include <mutex>
#include <numeric>
#include <opencv2/core/core.hpp>

class HT4DBlinkerTracker {
public:
  HT4DBlinkerTracker(
      int i_mem_steps,
      int i_pitch_steps,
      int i_yaw_steps,
      int i_max_pixel_shift,
      cv::Size i_im_res,
      int i_nullify_radius = 8,
      int i_reasonable_radius = 3,
      double i_framerate = 72);
  ~HT4DBlinkerTracker();

  void insertFrame(std::vector< cv::Point > newPoints);
  std::vector< cv::Point3d > getResults();
  int                        getTrackerCount();
  double getFrequency(int index);
  double getYaw(int index);
  double getPitch(int index);
  std::vector<double> getYaw();
  std::vector<double> getPitch();
  bool isCurrentBatchProcessed();
  void updateFramerate(double input);
  void updateResolution(cv::Size i_size);

  void setDebug(bool i_DEBUG, bool i_VisDEBUG);


  cv::Mat getVisualization();

private:
    /**
     * @brief Arccotangent function
     *
     * @param input - The cotangent of the desired angle (must not be 0)
     *
     * @return - Arccotangent of the input
     */
  double acot(double input);

  template < typename T >
  /**
   * @brief Resets an array of numbers to zero
   *
   * @param steps - Number of elements from the initial address to reset
   */
  void resetToZero(T *__restrict__ input, int steps);

  /**
   * @brief - generates the Hough masks that are applied to the Hough space for each input point
   */
  void generateMasks();

  /**
   * @brief Applies Hough masks to a Hough space for each input point in the accumulator
   *
   * @param i_weight_factor - Factor by which the weight of the newest points in the accumulator is raised above 1
   * @param i_constant_newer - Defines whether points from a number of newest frames have equal weight
   * @param i_break_point - The past frame before which input points start scaling their weight (if i_constant_newer = true)
   */
  void applyMasks(double i_weightFactor,bool i_constantNewer,int i_breakPoint);

  /**
   * @brief Resets matrices to zero at indices that have been previously altered. This is faster than blindly resetting all elements.
   */
  void cleanTouched();

  /**
   * @brief Projects all points in the accumulator to the Hough space
   */
  void projectAccumulatorToHT();


  /**
   * @brief Generates 2D matrices (of the input image size) with the maxima in the Hough space per pixel and with the indices of these maxima
   */
  void flattenTo2D();

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
   * @brief Retrieve the blinkig frequency of the given origin point
   *
   * @param origin_point - The image point presumed to correspond to a blinking marker
   * @param avg_yaw - The output "image yaw" of the blinking marker (corresponds to the direction in the image in which the point has been moving)
   * @param avg_pitch - The output "image pitch" of the blinking marker (corresponds to the speed in the image at which the point has been moving)
   *
   * @return - The frequency of blinking of the marker, or negative value in case of error
   */
  double retrieveFreqency(cv::Point origin_point, double &avg_yaw, double &avg_pitch);

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

  /**
   * @brief - Initializes points used in the miniFast method
   */
  void initFast();


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

  int          mem_steps_, pitch_steps_, yaw_steps_, total_steps_;
  unsigned int frame_scale_, hough_thresh_, nullify_radius_;
  double       scaling_factor_;
  int          mask_width_;
  int          expected_matches_;
  double       min_pitch_, pitch_div_, yaw_div_, step_div_;
  int          reasonable_radius_;
  double       framerate_;
  int          max_pixel_shift_;

  cv::Size     im_res_;
  unsigned int im_area_;
  cv::Rect     im_rect_;

  std::vector< std::vector< cv::Point2i > > accumulator_;
  std::vector< std::vector< cv::Point2i > > accumulator_local_copy_;
  std::vector< int >                        pts_per_layer_;
  std::vector< int >                        pts_per_layer_local_copy_;
  unsigned char *                           touched_matrix_;
  unsigned int * __restrict__ hough_space_;
  unsigned int * __restrict__ hough_space_maxima_;
  cv::Mat                                   index_matrix_;
  std::vector< std::vector< cv::Point3i > > hybrid_masks_;
  std::vector< double >                     pitch_vals_,
                                            yaw_vals_,
                                            cot_set_min_,
                                            cot_set_max_,
                                            sin_set_,
                                            cos_set_;

  std::vector< double > frequencies_, yaw_averages_, pitch_averages_;

  std::vector< cv::Point > fast_points_;

  bool curr_batch_processed_;

  std::mutex mutex_accumulator_;

  bool debug_, vis_debug_;

  cv::Mat visualization_;
};
