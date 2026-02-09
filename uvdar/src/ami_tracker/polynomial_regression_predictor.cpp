#include <uvdar/ami_tracker/polynomial_regression_predictor.h>

namespace uvdar::ami {

/* PredictionStatistics //{ */
PolynomialRegressionPredictor::PolynomialRegressionPredictor(const AmiTrackerConfig& cfg) : cfg_(cfg) {
}
//}

/* PredictionStatistics //{ */
OnLedHistory PolynomialRegressionPredictor::extractLedOnHistory_(const std::vector<PointState>& tseries) {
  OnLedHistory history;

  for (const auto point : tseries) {
    if (point.led_state) {
      history.x.push_back(point.point.x);
      history.y.push_back(point.point.y);
      auto seconds = std::chrono::duration_cast<std::chrono::seconds>(point.insert_time.time_since_epoch()).count();
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

  PointState& last_point             = tseries.back();
  PredictionStatistics x_predictions = selectStatisticsValues_(on_led_history.x, on_led_history.time, insert_time);
  PredictionStatistics y_predictions = selectStatisticsValues_(on_led_history.y, on_led_history.time, insert_time);

  return {x_predictions, y_predictions};
}
//}

/* selectStatisticsValues_ //{ */
PredictionStatistics PolynomialRegressionPredictor::selectStatisticsValues_(const std::vector<double>& coordinates,
                                                                            const std::vector<double>& time,
                                                                            const double& insert_time) {
  auto weights = computeNormalizedWeightVect_(time);

  PredictionStatistics stats;
  stats.mean_independent  = computeWeightedMean_(coordinates, weights);
  stats.time_pred         = insert_time;
  stats.poly_reg_computed = false;
  stats.extended_search   = true;
  int poly_order          = cfg_.poly_order;

  if (coordinates.size() > 0 && coordinates.size() < poly_order) {
    poly_order = coordinates.size() - 2;
  }
  if (coordinates.size() <= 1) {
    return stats;
  }

  auto reg_result = polyReg_(coordinates, time, weights, poly_order);
  stats.coeff     = reg_result.coeffs;
  stats.predicted_vals_past - reg_result.predictions;

  double t_power{1};
  for (const auto& coeff : reg_result.coeffs) {
    stats.predicted_coordinate += coeff * t_power;
    t_power *= insert_time;
  }

  stats.confidence_interval = computeConfidenceInterval_(stats, time, coordinates, weights);

  stats.poly_reg_computed = true;

  return stats;
}
//}

/* computeNormalizedWeightVect_ //{ */
std::vector<double> PolynomialRegressionPredictor::computeNormalizedWeightVect_(const std::vector<double>& time) {
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

  for (auto& w : weights) {
    w /= sum_weights;
  }

  return weights;
}
//}

/* computeWeightedMean_ //{ */
double PolynomialRegressionPredictor::computeWeightedMean_(const std::vector<double>& values,
                                                           const std::vector<double>& weights) {
  if (values.size() != weights.size()) {
    return -1;
  }

  double weighted_sum{0.0};
  for (size_t i = 0; i < values.size(); ++i) {
    weighted_sum += (values[i] * weights[i]);
  }

  return weighted_sum;
}
//}

/* polyReg_ //{ */
RegressionResult PolynomialRegressionPredictor::polyReg_(const std::vector<double>& coordinate,
                                                         const std::vector<double>& time,
                                                         const std::vector<double>& weights, const int poly_order) {

  Eigen::MatrixXd design_mat(time.size(), poly_order + 1);

  for (size_t i = 0; i < time.size(); ++i) {
    double t_power{1};
    for (size_t j = 0; j < poly_order + 1; ++j) {
      design_mat(i, j) = t_power;
      t_power *= time[i];
    }
  }

  Eigen::Map<const Eigen::VectorXd> pixel_vect(coordinate.data(), coordinate.size());
  Eigen::Map<const Eigen::VectorXd> weight_vect(weights.data(), weights.size());

  // Solve for weighted linear least sequare fit
  Eigen::VectorXd sqrt_weights = weight_vect.cwiseSqrt();
  Eigen::MatrixXd weighted_A   = sqrt_weights.asDiagonal() * design_mat;
  Eigen::VectorXd weighted_y   = sqrt_weights.asDiagonal() * pixel_vect;

  RegressionResult ans;
  Eigen::VectorXd result = weighted_A.householderQr().solve(weighted_y);
  ans.coeffs             = std::vector<double>(result.data(), result.data() + result.size());
  ans.predictions        = design_mat * result;

  return ans;
}
//}

/* computeConfidenceInterval_ //{ */
double PolynomialRegressionPredictor::computeConfidenceInterval_(PredictionStatistics& stats,
                                                                 const std::vector<double>& coordinate,
                                                                 const std::vector<double>& time,
                                                                 const std::vector<double>& weights) {
  const int n   = static_cast<int>(coordinate.size());
  const int p   = static_cast<int>(stats.coeff.size());
  const int dof = n - p;
  if (stats.mean_independent == -1.0 || dof <= 0) {
    return -1.0;
  }

  // variance of error
  double w_ssr    = computeWeightedSumSquaredResiduals_(stats.predicted_vals_past, coordinate, weights);
  double sigma_sq = w_ssr / dof;

  // sum of squares for time
  Eigen::Map<const Eigen::VectorXd> t_vec(time.data(), n);
  double sum_sq_diff_time = (t_vec.array() - stats.mean_independent).square().sum();

  // Standard error of prediction
  double time_diff      = stats.time_pred - stats.mean_independent;
  double leverage       = (time_diff * time_diff) / sum_sq_diff_time;
  double standard_error = std::sqrt(sigma_sq * (1.0 + 1.0 / n + leverage));

  // ChaGPT magic for replacing Boost student's t-distribution
  double t_critical = getTCriticalValue_(dof, cfg_.conf_probab_percent);
  return t_critical * standard_error;
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

/* getTCriticalValue_ //{ */
double PolynomialRegressionPredictor::getTCriticalValue_(int dof, int percentage) {
  // clang-format off
  // For 95% confidence
  if (percentage == 95) {
      if (dof <= 0)  return 0.0;
      if (dof == 1)  return 12.706;
      if (dof == 2)  return 4.303;
      if (dof == 3)  return 3.182;
      if (dof == 4)  return 2.776;
      if (dof == 5)  return 2.571;
      if (dof < 30)  return 2.042; // Approximation for mid-range
      return 1.960;               // Large sample size (Normal Distribution)
  }
  return 2.0; // Default fallback
  // clang-format on
}
//}

} // namespace uvdar::ami