#include <uvdar/blink_processor/extended_search.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>

namespace uvdar::blink_processor {

/* ExtendedSearch constructor //{ */
ExtendedSearch::ExtendedSearch(const ExtendedSearchConfig& cfg) : cfg_(cfg) {
  poly_predictor_ = std::make_unique<PolynomialRegressionPredictor>(cfg.poly_reg);
}
//}

/* run //{ */
void ExtendedSearch::run(std::vector<PointState>& unassigned_points, std::vector<SeqPtr>& buffer) {
  if (unassigned_points.empty()) {
    return;
  }

  double insert_time = std::chrono::duration<double>(unassigned_points[0].insert_time.time_since_epoch()).count();

  auto it = buffer.begin();
  while (it != buffer.end()) {
    auto& tseries = *it;

    if (tseries->empty()) {
      ++it;
      continue;
    }

    auto [x_predictions, y_predictions] = poly_predictor_->predict(insert_time, *tseries);
    if (!x_predictions.poly_reg_computed || !y_predictions.poly_reg_computed) {
      ++it;
      continue;
    }

    PointState predicted_point;
    predicted_point.point.x = x_predictions.predicted_coordinate;
    predicted_point.point.y = y_predictions.predicted_coordinate;
    predicted_point.x_stats = x_predictions;
    predicted_point.y_stats = y_predictions;

    clipPredictionInterval_(predicted_point);

    const cv::Point2d confidence_interval(predicted_point.x_stats.confidence_interval,
                                          predicted_point.y_stats.confidence_interval);

    PointState& last_point = tseries->back();
    last_point.x_stats     = predicted_point.x_stats;
    last_point.y_stats     = predicted_point.y_stats;

    if (performLocalCheck_(tseries->back(), predicted_point, unassigned_points, *tseries, confidence_interval)) {
      it = buffer.erase(it);
    } else {
      ++it;
    }
  }
}
//}

/* performLocalCheck_ //{ */
bool ExtendedSearch::performLocalCheck_(const PointState& last_observed, const PointState& predicted_point,
                                        std::vector<PointState>& unassigned_points, std::vector<PointState>& tseries,
                                        const cv::Point2d& conf_point) {
  cv::Point2d pred_point(predicted_point.point);
  cv::Point2d bb_left_top     = pred_point - conf_point;
  cv::Point2d bb_right_bottom = pred_point + conf_point;

  auto nearest_point_it = findNearestPoint_(unassigned_points, last_observed);
  if (nearest_point_it == unassigned_points.end()) {
    return false;
  }

  if (!isInsideBoundingBox(nearest_point_it->point, bb_left_top, bb_right_bottom)) {
    return false;
  }

  nearest_point_it->x_stats = predicted_point.x_stats;
  nearest_point_it->y_stats = predicted_point.y_stats;
  insertPointToSequence_(tseries, *nearest_point_it);

  unassigned_points.erase(nearest_point_it);
  return true;
}
//}

/* findNearestPoint_ //{ */
std::vector<PointState>::iterator ExtendedSearch::findNearestPoint_(std::vector<PointState>& points,
                                                                    const PointState& reference) {
  if (points.empty()) {
    return points.end();
  }

  return std::min_element(points.begin(), points.end(), [&](const PointState& a, const PointState& b) {
    return euclideanDistanceSq(a.point, reference.point) < euclideanDistanceSq(b.point, reference.point);
  });
}
//}

/* insertPointToSequence_ //{ */
void ExtendedSearch::insertPointToSequence_(std::vector<PointState>& sequence, const PointState signal) {
  sequence.push_back(signal);
  if (sequence.size() > cfg_.seq.getMaxSequenceLength()) {
    sequence.erase(sequence.begin());
  }
}
//}

/* clipPredictionInterval_ //{ */
void ExtendedSearch::clipPredictionInterval_(PointState& predicted_point) {
  const double max_pred_interval = static_cast<double>(cfg_.poly_reg.max_predict_interval_px);

  predicted_point.x_stats.confidence_interval =
      std::clamp(predicted_point.x_stats.confidence_interval, 0.0, max_pred_interval);
  predicted_point.y_stats.confidence_interval =
      std::clamp(predicted_point.y_stats.confidence_interval, 0.0, max_pred_interval);
}
//}

} // namespace uvdar::blink_processor