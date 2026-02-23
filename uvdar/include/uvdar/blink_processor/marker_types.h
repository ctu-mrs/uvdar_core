#pragma once

#include <vector>
#include <memory>
#include <chrono>

#include <Eigen/Dense>
#include <opencv2/core/types.hpp>

namespace uvdar::blink_processor {

using Clock         = std::chrono::steady_clock;
using TimePoint     = Clock::time_point;
using Sequence      = std::vector<bool>;
using SequenceBytes = std::vector<uint8_t>;

/* PredictionStatistics //{ */
struct PredictionStatistics {
  double time_pred{-1.0};
  bool poly_reg_computed{false};
  std::vector<double> coeff{};
  double predicted_coordinate{-1.0};
  double confidence_interval{-1.0};
};
//}

/* PointState //{ */
struct PointState {
  cv::Point2d point{};
  bool led_state{false};
  TimePoint insert_time{};

  PredictionStatistics x_stats{};
  PredictionStatistics y_stats{};
};
//}

using SeqPtr = std::shared_ptr<std::vector<PointState>>;

/* TrackedMarker //{ */
struct TrackedMarker {
  PointState last_point;
  int id;
};
//}

/* TrackCopyWindow //{ */
struct TrackCopyWindow {
  PointState last_point;
  std::vector<bool> led_window;
};
//}
} // namespace uvdar::blink_processor