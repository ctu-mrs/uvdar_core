#include <std_msgs/UInt32MultiArray.h>
#include <mutex>
#include <numeric>
#include <opencv2/core/core.hpp>

class HT3DBlinkerTracker {
public:
  HT3DBlinkerTracker(int i_memSteps,
      int i_pitchSteps,
      int i_yawSteps,
      int i_maxPixelShift,
      cv::Size i_imRes,
      int i_nullifyRadius = 8,
      int i_reasonableRadius = 3,
      double i_framerate = 72);
  ~HT3DBlinkerTracker();

  void insertFrame(std::vector< cv::Point > newPoints);
  std::vector< cv::Point3d > getResults();
  int                        getTrackerCount();
  double getFrequency(int index);
  double getYaw(int index);
  double getPitch(int index);
  bool isCurrentBatchProcessed();
  void updateFramerate(double input);

  void setDebug(bool i_DEBUG, bool i_VisDEBUG);


  cv::Mat getVisualization();

private:
  template < typename T >
  void resetToZero(T *__restrict__ input, int steps);
  void generateMasks();
  void applyMasks(std::vector< std::vector< cv::Point3i > > & __restrict__ maskSet,
      unsigned int *__restrict__ houghSpace);
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

  int          memSteps, pitchSteps, yawSteps;
  unsigned int frameScale, houghThresh, nullifyRadius, bitShift;
  int maskWidth;
  int          expectedMatches;
  double       minPitch, pitchDiv, yawDiv;
  int          reasonableRadius;
  double       framerate;
  int          maxPixelShift;

  cv::Size     imRes;
  unsigned int imArea;
  cv::Rect     imRect;

  std::vector< std::vector< cv::Point2i > > accumulator;
  std::vector< std::vector< cv::Point2i > > accumulatorLocalCopy;
  std::vector< int >                        ptsPerLayer;
  std::vector< int >                        ptsPerLayerLocalCopy;
  unsigned char *                            touchedMatrix;
  /* cv::Mat                                   houghSpacePitch, houghSpaceYaw; */
  unsigned int * __restrict__ houghSpacePitch,
               *__restrict__ houghSpaceYaw;
  /* cv::Mat                                   houghSpacePitchMaxima, houghSpaceYawMaxima, combinedMaximaMatrix; */
  unsigned int * __restrict__ houghSpacePitchMaxima,
               *__restrict__ houghSpaceYawMaxima,
               *__restrict__ combinedMaximaMatrix;
  cv::Mat                                   pitchMatrix,
    yawMatrix;
  std::vector< std::vector< cv::Point3i > > pitchMasks, yawMasks;
  std::vector< double >                     pitchVals,
    yawVals, cotSetMin, cotSetMax, sinSet, cosSet;

  std::vector< double > frequencies, yawAvgs, pitchAvgs;

  std::vector< cv::Point > fastPoints;

  bool currBatchProcessed;

  std::mutex mutex_accum;

  bool DEBUG, VisDEBUG;

  cv::Mat visualization;
};
