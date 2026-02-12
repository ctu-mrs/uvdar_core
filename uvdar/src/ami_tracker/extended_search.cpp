#include <uvdar/ami_tracker/extended_search.h>
#include <iostream>
#include <fstream>
#include <iomanip>

namespace uvdar::ami {

void logToCSV(int seq_id, const std::string& type, double x, double y, double cx, double cy, double conf_x,
              double conf_y) {

  static std::ofstream log_file;
  if (!log_file.is_open()) {
    log_file.open("/home/tomas/Desktop/python_tests/extended_search_debug.csv");
    // Headers: type defines if the row is an 'original_point', 'prediction', 'box', or 'unassigned'
    log_file << "seq_id,type,x,y,conf_x,conf_y\n";
  }

  log_file << seq_id << "," << type << "," << std::fixed << std::setprecision(3) << x << "," << y << "," << conf_x
           << "," << conf_y << "\n";
}

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
  int seq_counter = 0;

  for (auto& up : unassigned_points) {
    logToCSV(-1, "unassigned_pool", up.point.x, up.point.y, 0, 0, 0, 0);
  }
  double insert_time =
      std::chrono::duration<double>(unassigned_points[0].insert_time.time_since_epoch()).count() + PREDICTION_MARGIN_;

  auto it = buffer.begin();
  while (it != buffer.end()) {
    auto& tseries = *it;

    for (auto& pt : *tseries) {
      logToCSV(seq_counter, "history", pt.point.x, pt.point.y, 0, 0, 0, 0);
    }

    if (tseries->empty()) {
      ++it;
      seq_counter++;
      continue;
    }

    auto [x_predictions, y_predictions] = poly_predictor_->predict(insert_time, *tseries);
    if (!x_predictions.poly_reg_computed || !y_predictions.poly_reg_computed) {
      ++it;
      continue;
    }

    logToCSV(seq_counter, "prediction", x_predictions.predicted_coordinate, y_predictions.predicted_coordinate, 0,
             0, // cx, cy (unused)
             x_predictions.confidence_interval, y_predictions.confidence_interval);

    // PointState& last_point = tseries->back();
    // last_point.x_stats     = x_predictions;
    // last_point.y_stats     = y_predictions;
    PointState predicted_point;
    predicted_point.point.x = x_predictions.predicted_coordinate;
    predicted_point.point.y = y_predictions.predicted_coordinate;

    const double& x_conf = x_predictions.confidence_interval;
    const double& y_conf = y_predictions.confidence_interval;

    if (performLocalCheck_(predicted_point, unassigned_points, *tseries, predicted_point.point,
                           cv::Point2d(x_conf, y_conf))) {
      it = buffer.erase(it);
    } else {
      ++it;
    }
    ++seq_counter;
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
    // log_to_csv("reject", nearest_point_it->point, bb_left_top, bb_right_bottom);
    return false;
  }

  nearest_point_it->x_stats = last_point.x_stats;
  nearest_point_it->y_stats = last_point.y_stats;
  insertPointToSequence_(tseries, *nearest_point_it);

  // log_to_csv("match", nearest_point_it->point, bb_left_top, bb_right_bottom);

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