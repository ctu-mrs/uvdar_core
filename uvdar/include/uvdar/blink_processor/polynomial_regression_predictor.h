#pragma once

#include <uvdar/blink_processor/blink_processor_types.h>
#include <uvdar/blink_processor/marker_types.h>
#include <uvdar/blink_processor/tseries_ops.h>

namespace uvdar::blink_processor {

struct OnLedHistory {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> time;
};

struct RegressionResult {
  Eigen::VectorXd prediction;
  double std_error;
};

class PolynomialRegressionPredictor {
 public:
  PolynomialRegressionPredictor(const PolyRegressionConfig& cfg);

  std::tuple<PredictionStatistics, PredictionStatistics> predict(const double insert_time,
                                                                 std::vector<PointState>& tseries);

  std::vector<double> computeNormalizedWeightVect(const std::vector<double>& time);

  PredictionStatistics selectStatisticsValues(const std::vector<double>& coordinates, const std::vector<double>& time,
                                              const double& insert_time);

  std::tuple<double, double> calculatePredictionInterval(const std::vector<double>& coordinate,
                                                         const std::vector<double>& time,
                                                         const std::vector<double>& weights, const double time_next);

 private:
  OnLedHistory extractLedOnHistory_(const std::vector<PointState>& tseries);

  double computeWeightedMean_(const double* values, const double* weights, int n);

  double computeWeightedSumSquaredResiduals_(const Eigen::VectorXd& predictions, const std::vector<double>& values,
                                             const std::vector<double>& weights);

 private:
  const PolyRegressionConfig cfg_;

  Eigen::Matrix<double, Eigen::Dynamic, 5> X_vandermonde_;
  Eigen::VectorXd y_workspace_;
  std::vector<double> t_quantile_lut_;
};

} // namespace uvdar::blink_processor