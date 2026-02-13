#pragma once

#include <mutex>

#include <uvdar/blink_processor/ami_tracker_types.h>
#include <uvdar/blink_processor/tseries_ops.h>
#include <uvdar/blink_processor/polynomial_regression_predictor.h>

namespace uvdar::blink_processor {

class ExtendedSearch {
 public:
  explicit ExtendedSearch(const std::shared_ptr<AmiTrackerConfig> cfg);
  void run(std::vector<PointState>& unassigned_points, std::vector<SeqPtr>& buffer);

 private:
  bool performLocalCheck_(PointState& last_point, std::vector<PointState>& unassigned_points,
                          std::vector<PointState>& tseries, const cv::Point2d& pred_point,
                          const cv::Point2d& conf_point);
  std::vector<PointState>::iterator findNearestPoint_(std::vector<PointState>& points, const PointState& reference);
  void insertPointToSequence_(std::vector<PointState>& sequence, const PointState signal);
  // void insertVirtualPointToSequence_(std::vector<PointState>& sequence, const TimePoint& time);
  // void clipBuffer_();

 private:
  std::shared_ptr<AmiTrackerConfig> cfg_;
  std::mutex mutex_gen_sequences_;

  const double PREDICTION_MARGIN_{0.0};
  std::unique_ptr<PolynomialRegressionPredictor> poly_predictor_;
};

} // namespace uvdar::blink_processor