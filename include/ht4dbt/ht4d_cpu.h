#ifndef HT4D_CPU_H
#define HT4D_CPU_H

#include "ht4d.h"

namespace uvdar {

class HT4DBlinkerTrackerCPU : public HT4DBlinkerTracker {
public:

  HT4DBlinkerTrackerCPU(
      int i_mem_steps,
      int i_pitch_steps,
      int i_yaw_steps,
      int i_max_pixel_shift,
      cv::Size i_im_res,
      int i_allowed_BER_per_seq,
      int i_nullify_radius = 8,
      int i_reasonable_radius = 6,
      double i_framerate = 72);

  ~HT4DBlinkerTrackerCPU();

  std::vector< std::pair<cv::Point2d,int> > getResults();

  void updateResolution(cv::Size i_size);

private:

  /**
   * @brief Resets matrices to zero at indices that have been previously altered. This is faster than blindly resetting all elements.
   */
  void cleanTouched();

  /**
   * @brief - generates the Hough masks that are applied to the Hough space for each input point. These are sets of 3D coordinates (w.r.t. the X-Y position of an input point) to be incremented in Hough voting. The 3rd dimension represents an index of the permutated pitch and yaw indices and thus it represents a point in 4D space.
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
   * @brief Projects all points in the accumulator to the Hough space
   */
  void projectAccumulatorToHT();


  /**
   * @brief Generates 2D matrices (of the size of the input image) with maxima in the Hough space per pixel (X-Y coordinate) and with the indices of these maxima
   */
  void flattenTo2D();

  unsigned int * __restrict__ hough_space_;
  std::vector< std::vector< cv::Point3i > > hybrid_masks_;
};

} //namespace uvdar

#endif // HT4D_CPU_H
