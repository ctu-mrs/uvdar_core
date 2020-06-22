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
  template < typename T >
  void resetToZero(T *__restrict__ input, int steps);
  void generateMasks();
  void applyMasks(std::vector< std::vector< cv::Point3i > > & __restrict__ maskSet, unsigned int *__restrict__ houghSpace, double i_weightFactor,bool i_constantNewer,int i_breakPoint);
  void cleanTouched();
  void projectAccumulatorToHT();
  /* cv::Mat downSample(const cv::Mat &input, cv::Mat &output, int bits=8); */
  void flattenTo2D(unsigned int *__restrict__ input,
      int thickness, unsigned int *__restrict__ outputMaxima, cv::Mat &outputIndices);
  std::vector< cv::Point > findHoughPeaks(unsigned int *__restrict__ input, int peakCount);
  cv::Point findHoughPeakLocal(cv::Point expectedPos);
  double retrieveFreqency(cv::Point originPoint, double &avgYaw, double &avgPitch);
  std::vector<cv::Point> nullifyKnown();
  static int dummy;
  bool miniFast(int x, int y, unsigned int thresh, int &smallestDiff = dummy);
  void initFast();

  cv::Mat getCvMat(unsigned int *__restrict__ input, unsigned int threshold);

  double mod2(double a, double n);
  double angDiff(double a, double b);
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
