#include <uvdar/blink_processor/polynomial_regression_predictor.h>
#include <boost/math/distributions/students_t.hpp>

namespace uvdar::blink_processor {

/* PredictionStatistics //{ */
PolynomialRegressionPredictor::PolynomialRegressionPredictor(const PolyRegressionConfig& cfg) : cfg_(cfg) {
  if (cfg_.seq.getMaxSequenceLength() <= 0) {
    throw std::invalid_argument("Maximum sequence length for polynomial regression has to be positive!");
  }

  X_vandermonde_.resize(cfg_.seq.getMaxSequenceLength(), cfg_.poly_order + 1);
  y_workspace_.resize(cfg_.seq.getMaxSequenceLength());
}
//}

/* PredictionStatistics //{ */
OnLedHistory PolynomialRegressionPredictor::extractLedOnHistory_(const std::vector<PointState>& tseries) {
  OnLedHistory history;

  for (const auto& point : tseries) {
    if (point.led_state) {
      history.x.push_back(point.point.x);
      history.y.push_back(point.point.y);
      using dsec   = std::chrono::duration<double>;
      auto seconds = std::chrono::duration_cast<dsec>(point.insert_time.time_since_epoch()).count();
      history.time.push_back(seconds);
    }
  }

  return history;
}
//}

/* predict //{ */
std::tuple<PredictionStatistics, PredictionStatistics>
PolynomialRegressionPredictor::predict(const double insert_time, std::vector<PointState>& tseries) {

  auto on_led_history = extractLedOnHistory_(tseries);

  PredictionStatistics x_predictions = selectStatisticsValues(on_led_history.x, on_led_history.time, insert_time);
  PredictionStatistics y_predictions = selectStatisticsValues(on_led_history.y, on_led_history.time, insert_time);

  int dof = static_cast<int>(on_led_history.x.size()) - (cfg_.poly_order + 1);

  if (dof > 0) {
    boost::math::students_t dist(dof);
    double prob = cfg_.conf_prob_percentage / 100.0;
    double t    = quantile(dist, (1.0 + prob) / 2.0);

    x_predictions.confidence_interval = t * x_predictions.confidence_interval + cfg_.min_prediction_tol_px;
    y_predictions.confidence_interval = t * y_predictions.confidence_interval + cfg_.min_prediction_tol_px;
  } else {
    x_predictions.confidence_interval = cfg_.min_prediction_tol_px;
    y_predictions.confidence_interval = cfg_.min_prediction_tol_px;
  }

  return {x_predictions, y_predictions};
}
//}

/* selectStatisticsValues //{ */
PredictionStatistics PolynomialRegressionPredictor::selectStatisticsValues(const std::vector<double>& coordinates,
                                                                           const std::vector<double>& time,
                                                                           const double& insert_time) {
  auto [weights, sum_raw_weights] = computeNormalizedWeightVect(time);

  PredictionStatistics stats;
  stats.time_pred         = insert_time;
  stats.poly_reg_computed = false;
  size_t poly_order       = static_cast<size_t>(cfg_.poly_order);

  if (coordinates.size() <= 1) {
    return stats;
  }
  if (coordinates.size() < poly_order) {
    poly_order = coordinates.size() - 2;
  }

  std::tie(stats.predicted_coordinate, stats.confidence_interval) =
      calculatePredictionInterval(coordinates, time, weights, sum_raw_weights, insert_time);

  stats.poly_reg_computed = true;

  return stats;
}
//}

/* computeNormalizedWeightVect //{ */
std::pair<std::vector<double>, double>
PolynomialRegressionPredictor::computeNormalizedWeightVect(const std::vector<double>& time) {
  std::vector<double> weights;
  weights.reserve(time.size());

  double sum_weights{0.0};
  double reference_time = time.back();
  for (const auto& t : time) {
    double delta_t = reference_time - t;
    double w       = exp(-cfg_.decay_factor * delta_t);
    sum_weights += w;
    weights.push_back(w);
  }

  // Normalize weights to sum to 1 for numerical stability.
  // The raw sum is returned separately for use in the prediction interval.
  for (auto& w : weights) {
    w /= sum_weights;
  }

  return {weights, sum_weights};
}
//}

/* computeWeightedMean_ //{ */
double PolynomialRegressionPredictor::computeWeightedMean_(const double* values, const double* weights, int n) {
  double weighted_sum = 0.0;
  for (int i = 0; i < n; ++i)
    weighted_sum += values[i] * weights[i];

  return weighted_sum;
}
//}

/* calculatePredictionInterval //{ */
std::tuple<double, double> PolynomialRegressionPredictor::calculatePredictionInterval(
    const std::vector<double>& coordinate, const std::vector<double>& time, const std::vector<double>& weights,
    double sum_raw_weights, const double time_next) {
  const int n        = static_cast<int>(std::min(coordinate.size(), cfg_.seq.getMaxSequenceLength()));
  const int p        = std::min(cfg_.poly_order, n - 2);
  const int n_coeffs = p + 1;
  const int dof      = n - n_coeffs;

  // Get pointers to the start of the searching window
  const double* t_ptr = &time.back() - (n - 1);
  const double* c_ptr = &coordinate.back() - (n - 1);
  const double* w_ptr = &weights.back() - (n - 1);

  Eigen::Map<const Eigen::VectorXd> t_vec(t_ptr, n);
  Eigen::Map<const Eigen::VectorXd> w_vec(w_ptr, n);
  double time_mean = t_vec.dot(w_vec) / w_vec.sum();

  if (time_mean == -1.0 || dof <= 0) {
    return {std::numeric_limits<double>::quiet_NaN(), 1e10};
  }

  auto X_active = X_vandermonde_.block(0, 0, n, n_coeffs);
  auto y_active = y_workspace_.head(n);

  for (int i = 0; i < n; ++i) {
    const double t_rel = t_ptr[i] - time_mean;
    const double rw    = std::sqrt(w_ptr[i]);

    y_active(i) = c_ptr[i] * rw;

    double t_pow = rw;
    for (int j = 0; j < n_coeffs; ++j) {
      X_active(i, j) = t_pow;
      t_pow *= t_rel;
    }
  }

  auto qr              = X_active.householderQr();
  Eigen::VectorXd beta = qr.solve(y_active);

  // residuals
  double w_ssr  = (X_active * beta - y_active).squaredNorm();
  double sigma2 = w_ssr / std::max(1, dof);

  // Prediction using Horner’s Method
  const double t_next_rel = time_next - time_mean;
  double mu               = 0.0;
  for (int j = n_coeffs - 1; j >= 0; --j) {
    mu = mu * t_next_rel + beta(j);
  }

  // Optimized Leverage
  Eigen::VectorXd phi_next(n_coeffs);
  double tn_pow = 1.0;
  for (int j = 0; j < n_coeffs; ++j) {
    phi_next(j) = tn_pow;
    tn_pow *= t_next_rel;
  }

  Eigen::MatrixXd R = qr.matrixQR().topRows(n_coeffs).triangularView<Eigen::Upper>();
  Eigen::VectorXd v = R.transpose().triangularView<Eigen::Lower>().solve(phi_next);
  double leverage   = v.squaredNorm();

  // Final standard error for prediction.
  // With normalized weights, the new-observation term is 1/w_new.
  // Since w_new_raw = exp(0) = 1, w_new_normalized = 1/sum_raw_weights,
  // so 1/w_new_normalized = sum_raw_weights.
  double s_pred = std::sqrt(sigma2 * (sum_raw_weights + leverage));

  return {mu, s_pred};
}
//}

/* computeWeightedSumSquaredResiduals_ //{ */
double PolynomialRegressionPredictor::computeWeightedSumSquaredResiduals_(const Eigen::VectorXd& predictions,
                                                                          const std::vector<double>& values,
                                                                          const std::vector<double>& weights) {
  double sum_squared_residuals = 0;
  for (size_t i = 0; i < values.size(); ++i) {
    sum_squared_residuals += (weights[i] * pow((predictions(i) - values[i]), 2));
  }
  return sum_squared_residuals;
}
//}

} // namespace uvdar::blink_processor