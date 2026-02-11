#include <uvdar/ami_tracker/polynomial_regression_predictor.h>
#include <boost/math/distributions/students_t.hpp>

namespace uvdar::ami {

/* PredictionStatistics //{ */
PolynomialRegressionPredictor::PolynomialRegressionPredictor(const AmiTrackerConfig& cfg) : cfg_(cfg) {
}
//}

/* PredictionStatistics //{ */
OnLedHistory PolynomialRegressionPredictor::extractLedOnHistory_(const std::vector<PointState>& tseries) {
  OnLedHistory history;

  for (const auto& point : tseries) {
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

  PredictionStatistics x_predictions = selectStatisticsValues(on_led_history.x, on_led_history.time, insert_time);
  PredictionStatistics y_predictions = selectStatisticsValues(on_led_history.y, on_led_history.time, insert_time);

  return {x_predictions, y_predictions};
}
//}

/* selectStatisticsValues //{ */
PredictionStatistics PolynomialRegressionPredictor::selectStatisticsValues(const std::vector<double>& coordinates,
                                                                           const std::vector<double>& time,
                                                                           const double& insert_time) {
  auto weights = computeNormalizedWeightVect(time);

  PredictionStatistics stats;
  stats.mean_independent  = computeWeightedMean_(time, weights);
  stats.time_pred         = insert_time;
  stats.poly_reg_computed = false;
  stats.extended_search   = true;
  size_t poly_order       = static_cast<size_t>(cfg_.poly_order);

  if (coordinates.size() < poly_order) {
    poly_order = coordinates.size() - 2;
  }
  if (coordinates.size() <= 1) {
    return stats;
  }

  auto reg_result           = polyReg_(coordinates, time, weights, poly_order);
  stats.coeff               = reg_result.coeffs;
  stats.predicted_vals_past = reg_result.predictions;

  double t_power{1};
  stats.predicted_coordinate = 0.0;
  for (const auto& coeff : reg_result.coeffs) {
    stats.predicted_coordinate += coeff * t_power;
    t_power *= insert_time;
  }

  stats.confidence_interval = computeConfidenceInterval_(stats, coordinates, time, weights);

  stats.poly_reg_computed = true;

  return stats;
}
//}

/* computeNormalizedWeightVect //{ */
std::vector<double> PolynomialRegressionPredictor::computeNormalizedWeightVect(const std::vector<double>& time) {
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
                                                         const std::vector<double>& weights, const size_t poly_order) {

  // Vandermonde Matrix
  Eigen::MatrixXd design_mat(time.size(), poly_order + 1);

  for (size_t i = 0; i < time.size(); ++i) {
    double t_power{1};
    for (size_t j = 0; j <= poly_order; ++j) {
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

  double w_ssr    = computeWeightedSumSquaredResiduals_(stats.predicted_vals_past, coordinate, weights);
  double sigma_sq = w_ssr / dof; // estimated measurement noise

  // sum of squares for time
  double var_time = 0.0; //
  for (auto t : time) {
    var_time += pow((t - stats.mean_independent), 2);
  }
  if (var_time <= 0.0) {
    return -1;
  }

  double standard_error =
      sqrt(sigma_sq * (1.0 + 1.0 / n + (std::pow(stats.time_pred - stats.mean_independent, 2) / var_time)));

  double percentage_scaled    = double(cfg_.conf_probab_percent) / 100.0;
  double percentage_two_sided = (1 - percentage_scaled) / 2 + percentage_scaled;
  boost::math::students_t dist(dof);
  double t = quantile(dist, percentage_two_sided);
  return t * standard_error;

  // Eigen::Map<const Eigen::VectorXd> t_vec(time.data(), n);
  // double sum_sq_diff_time = (t_vec.array() - stats.mean_independent).square().sum();
  // if (sum_sq_diff_time <= 0.0) {
  //   return -1.0;
  // }

  // double time_diff = stats.time_pred - stats.mean_independent; // predictions father from data center are more
  // uncertain double leverage  = (time_diff * time_diff) / sum_sq_diff_time; // how extreme prediction location is
  // // double standard_error = std::sqrt(sigma_sq * (1.0 + 1.0 / n + leverage));
  // double standard_error = std::sqrt(sigma_sq * (1.0 + leverage));

  // double percentage_scaled    = double(cfg_.conf_probab_percent) / 100.0;
  // double percentage_two_sided = (1 - percentage_scaled) / 2 + percentage_scaled;
  // boost::math::students_t dist(dof);
  // double t_critical = quantile(dist, percentage_two_sided);

  // ChaGPT magic for replacing Boost student's t-distribution
  // double t_critical = getTCriticalValue_(dof, cfg_.conf_probab_percent);
  // return t_critical * standard_error;
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
  if (percentage == 75) {
    if (dof <= 0) return 0.0;
    if (dof == 1) return 2.414;
    if (dof == 2) return 1.604;
    if (dof == 3) return 1.423;
    if (dof == 4) return 1.344;
    if (dof == 5) return 1.301;
    if (dof < 30) return 1.174; // ~dof 29
    return 1.150;              // normal approx
  }
  return 2.0; // Default fallback
  // clang-format on
}
//}

} // namespace uvdar::ami