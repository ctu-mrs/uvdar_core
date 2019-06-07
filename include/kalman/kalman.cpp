#include "kalman/kalman.h"

void KalmanFilterTarget::initKalman() {
  first = true;

  // >>>> Kalman Filter
  int stateSize = 2;
  int measSize  = 1;
  int contrSize = 0;

  unsigned int type = CV_32F;
  kfd               = cv::KalmanFilter(stateSize, measSize, contrSize, type);  // distance

  state = cv::Mat(stateSize, 1, type);  // [s_l,s_dl]
  meas  = cv::Mat(measSize, 1, type);   // [m_l]
  // cv::Mat procNoise(stateSize, 1, type)
  // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

  // Transition State Matrix A
  // Note: set dT at each processing step!
  // [ 1 dT]
  // [ 0 1 ]
  cv::setIdentity(kfd.transitionMatrix);

  // Measure Matrix H
  // [ 1 0 ]
  kfd.measurementMatrix                = cv::Mat::zeros(measSize, stateSize, type);
  kfd.measurementMatrix.at< float >(0) = 1.0f;
  /* kf.measurementMatrix.at<float>(7) = 1.0f; */

  // Process Noise Covariance Matrix Q
  // [ El   0    ]
  // [ 0    Edl   ]
  // cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
  kfd.processNoiseCov.at< float >(0) = 1e-2;
  kfd.processNoiseCov.at< float >(3) = 1e-2;

  // Measures Noise Covariance Matrix R
  cv::setIdentity(kfd.measurementNoiseCov, cv::Scalar(1e-1));
  // <<<< Kalman Filter


  // >>>>> Main loop
  while (ch != 'q' && ch != 'Q') {
    double precTick = ticks;
    ticks           = (double)cv::getTickCount();

    double dT = (ticks - precTick) / cv::getTickFrequency();  // seconds



}


void KalmanFilterTarget::correctKalman(double meas_distance) {
  meas.at< float >(0) = meas_distance;
    if (!first) {
      // >>>> Matrix A
      // <<<< Matrix A

    }
    if (first)  // First detection!
    {
      // >>>> Initialization
      kfd.errorCovPre.at< float >(0)  = meas_distance*meas_distance*0.04;  // m
      kfd.errorCovPre.at< float >(3)  = 4;  // m

      state.at< float >(0) = meas_distance;
      state.at< float >(1) = 0;
      // <<<< Initialization

      kfd.statePost = state;

      first = false;
    }   // <<<<< Kalman Update
    else{
      kfd.
        M

      kfd.correct(meas);  // Kalman Correction
    }
}

void KalmanFilterTarget::updateKalman(double dT) {
  kfd.transitionMatrix.at< float >(2) = dT;
  state = kfd.predict();
}

double KalmanFilterTarget::getKalmanDistance() {
}
}
