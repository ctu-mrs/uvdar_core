#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#include <mutex>
#include <thread>
#include <utility>

namespace uvdar {

/* UvLedDetectConfig //{ */
struct UvLedDetectConfig {
  bool gpu;
  bool gui;
  bool debug;
  int threshold;
  int threshold_diff;
  int threshold_sun;
  int threshold_sun_dist;
  std::vector<cv::Mat> masks;
};
//}

/* FastTestResult //{ */
struct FastTestResult {
  bool marker_candidate{false};
  bool sun_candidate{false};
  int i{0};
  int j{0};
  int sun_test_points{0};
  int ring_idx{0};
};
//}

/* SunCluster //{ */
struct SunCluster {
  cv::Point sum;
  int count;

  cv::Point centroid() const {
    return sum / count;
  }

  SunCluster(cv::Point s, int c) : sum(s), count(c) {
  }
};
//}

} // namespace uvdar