#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#include <mutex>
#include <thread>
#include <utility>

namespace uvdar {

struct UvLedDetectConfig {
  bool gui;
  bool debug;
  int threshold;
  int threshold_diff;
  int threshold_sun;
  std::vector<cv::Mat> masks;
};

} // namespace uvdar