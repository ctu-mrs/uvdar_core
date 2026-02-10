#include <uvdar/ami_tracker/extended_search.h>

namespace uvdar::ami {

/* ExtendedSearch constructor //{ */
ExtendedSearch::ExtendedSearch(const AmiTrackerConfig& cfg) : cfg_(cfg) {
  poly_predictor_ = std::make_unique<PolynomialRegressionPredictor>(cfg);
}
//}

/* run //{ */
void ExtendedSearch::run(std::vector<PointState>& unassigned_points, std::vector<SeqPtr>& buffer) {
  if (unassigned_points.empty()) {
    return;
  }

  double insert_time =
      std::chrono::duration_cast<std::chrono::seconds>(unassigned_points[0].insert_time.time_since_epoch()).count() +
      PREDICTION_MARGIN_;

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

    PointState& last_point = tseries->back();
    last_point.x_stats     = x_predictions;
    last_point.y_stats     = y_predictions;

    double x_predicted = last_point.x_stats.predicted_coordinate;
    double y_predicted = last_point.y_stats.predicted_coordinate;

    last_point.x_stats.confidence_interval =
        std::clamp(last_point.x_stats.confidence_interval, cfg_.max_px_shift.x, cfg_.max_px_shift.x * 2);
    last_point.y_stats.confidence_interval =
        std::clamp(last_point.y_stats.confidence_interval, cfg_.max_px_shift.y, cfg_.max_px_shift.y * 2);

    double x_conf = last_point.x_stats.confidence_interval;
    double y_conf = last_point.y_stats.confidence_interval;

    if (performLocalCheck_(last_point, unassigned_points, *tseries, cv::Point2d(x_predicted, y_predicted),
                           cv::Point2d(x_conf, y_conf))) {
      it = buffer.erase(it);
    } else {
      ++it;
    }
  }
}
//}

/* performLocalCheck_ //{ */
bool ExtendedSearch::performLocalCheck_(PointState& last_point, std::vector<PointState>& unassigned_points,
                                        std::vector<PointState>& tseries, const cv::Point2d& pred_point,
                                        const cv::Point2d& conf_point) {
  cv::Point2d bb_left_top     = pred_point - conf_point;
  cv::Point2d bb_right_bottom = pred_point + conf_point;

  auto nearest_point_it = findNearestPoint_(unassigned_points, last_point);
  if (nearest_point_it == unassigned_points.end()) {
    return false;
  }

  if (!isInsideBoundingBox(nearest_point_it->point, bb_left_top, bb_right_bottom)) {
    return false;
  }

  nearest_point_it->x_stats = last_point.x_stats;
  nearest_point_it->y_stats = last_point.y_stats;
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
  if (sequence.size() > (cfg_.blinking_patterns_size * cfg_.stored_seq_len_factor)) {
    sequence.erase(sequence.begin());
  }
}
//}

} // namespace uvdar::ami