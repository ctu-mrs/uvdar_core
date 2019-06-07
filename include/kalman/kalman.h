#ifndef KALMAN_H
#define KALMAN_H

#include <opencv2/video/tracking.hpp>


class KalmanFilterTarget {
  void initKalman();
  void correctKalman(double meas_distance);
  void updateKalman(double dT);
  double getKalmanDistance();
  cv::KalmanFilter kfd;  // distance
  cv::Mat          meas, state;
  bool first;
};
#endif  // KALMAN_H
