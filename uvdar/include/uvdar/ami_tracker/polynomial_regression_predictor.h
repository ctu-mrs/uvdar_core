#pragma once

#include <uvdar/ami_tracker/ami_tracker_types.h>
#include <uvdar/ami_tracker/tseries_ops.h>

namespace uvdar::ami {

struct OnLedHistory {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> time;
};

struct RegressionResult {
  std::vector<double> coeffs;
  Eigen::VectorXd predictions;
};

class PolynomialRegressionPredictor {
 public:
  PolynomialRegressionPredictor(const AmiTrackerConfig& cfg);

  std::tuple<PredictionStatistics, PredictionStatistics> predict(const double insert_time,
                                                                 std::vector<PointState>& tseries);

 private:
  OnLedHistory extractLedOnHistory_(const std::vector<PointState>& tseries);

  PredictionStatistics selectStatisticsValues_(const std::vector<double>& coordinates, const std::vector<double>& time,
                                               const double& insert_time);

  std::vector<double> computeNormalizedWeightVect_(const std::vector<double>& time);

  double computeWeightedMean_(const std::vector<double>& values, const std::vector<double>& weights);

  RegressionResult polyReg_(const std::vector<double>& coordinate, const std::vector<double>& time,
                            const std::vector<double>& weights, const int poly_order);

  double computeConfidenceInterval_(PredictionStatistics& stats, const std::vector<double>& coordinate,
                                    const std::vector<double>& time, const std::vector<double>& weights);

  double computeWeightedSumSquaredResiduals_(const Eigen::VectorXd& predictions, const std::vector<double>& values,
                                             const std::vector<double>& weights);

  ///@brief Chatgpt magic for replacing boost::math::students_t
  double getTCriticalValue_(int dof, int percentage);

 private:
  const AmiTrackerConfig& cfg_;
};

} // namespace uvdar::ami