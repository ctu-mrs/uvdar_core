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
  bool extended_search{false};
  std::vector<double> coeff{};
  Eigen::VectorXd predicted_vals_past;
  double mean_dependent{0.0};
  double mean_independent{0.0};
  double predicted_coordinate{-1.0};
  double confidence_interval{-1.0};
  Eigen::MatrixXd vandermonde_mat;
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

/* AmiTrackerConfig //{ */
struct AmiTrackerConfig {
  cv::Point2d max_px_shift{};
  int max_zeros_consecutive{0};
  // the multiplication factor how long the sequence should be for calculating the trajectory
  int stored_seq_len_factor{0};
  int max_buffer_length{0};
  int poly_order{0};
  double decay_factor{0.0};
  double conf_probab_percent{0.0};
  int allowed_BER_per_seq{0};
  int frame_length{0};
  size_t blinking_patterns_size{0};
};
//}

/* TrackedMarker //{ */
struct TrackedMarker {
  PointState last_point;
  int id;
};
//}

} // namespace uvdar::blink_processor