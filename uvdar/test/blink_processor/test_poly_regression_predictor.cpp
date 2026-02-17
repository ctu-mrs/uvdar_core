#include <gtest/gtest.h>
#include <random>
#include "../dummy_logger.h"
#include "../timer.h"
#include <thread>
#include <random>

#include <uvdar/blink_processor/polynomial_regression_predictor.h>
#include <fstream>
#include <iostream>

using namespace uvdar::blink_processor;

double gaussianNoise(double mean, double stddev) {
  static std::mt19937 rng(0); //
  std::normal_distribution<double> dist(mean, stddev);
  return dist(rng);
}

/* TEST(PolyRegressionPredictor, NormalizedWeights_SumToOne_AndNewestLargest) //{ */
TEST(PolyRegressionPredictor, NormalizedWeights_SumToOne_AndNewestLargest) {
  PolyRegressionConfig cfg;
  cfg.poly_order            = 4;
  cfg.decay_factor          = 0.5;
  cfg.conf_prob_percentage  = 95;
  cfg.min_prediction_tol_px = 0;

  std::vector<double> time{0.000, 0.005, 0.010};
  PolynomialRegressionPredictor predictor(cfg);

  auto weights = predictor.computeNormalizedWeightVect(time);

  ASSERT_EQ(weights.size(), time.size()); // a weight for each timestep

  // all positive and sum==1
  double sum = 0.0;
  for (const auto& w : weights) {
    EXPECT_GT(w, 0.0);
    sum += w;
  }
  EXPECT_NEAR(sum, 1.0, 1e-9);

  // newer should have larger weight
  EXPECT_GT(weights[1], weights[0]);
  EXPECT_GT(weights[2], weights[1]);
}
//}

/* TEST(PolyRegressionPredictor, LinearPerfectFit_ZeroConfidenceInterval) //{ */
TEST(PolyRegressionPredictor, LinearPerfectFit_ZeroConfidenceInterval) {
  const double A_COEFF = 10;
  const double B_COEFF = 2;

  PolyRegressionConfig cfg;
  cfg.poly_order            = 1;
  cfg.decay_factor          = 0;
  cfg.conf_prob_percentage  = 95;
  cfg.min_prediction_tol_px = 0;

  PolynomialRegressionPredictor predictor(cfg);

  std::vector<double> time = {0, 1, 2, 3, 4};
  double insert_time       = 5;
  std::vector<double> coords, weights;
  for (size_t t = 0; t < time.size(); ++t) {
    coords.push_back(A_COEFF + B_COEFF * t);
    weights.push_back(1);
  }

  auto stats              = predictor.selectStatisticsValues(coords, time, insert_time);
  const double expected_y = A_COEFF + B_COEFF * insert_time;

  ASSERT_TRUE(stats.poly_reg_computed);
  EXPECT_NEAR(stats.predicted_coordinate, expected_y, 1e-9);
  EXPECT_NEAR(stats.confidence_interval, 0.0, 1e-9);
}
//}

/* TEST(PolyRegressionPredictor, ConfidenceIntervalGrowsWithNoise) //{ */
TEST(PolyRegressionPredictor, ConfidenceIntervalGrowsWithNoise) {
  const double A_COEFF = 10;
  const double B_COEFF = 2;

  PolyRegressionConfig cfg;
  cfg.poly_order            = 1;
  cfg.decay_factor          = 0.0;
  cfg.conf_prob_percentage  = 95;
  cfg.min_prediction_tol_px = 2;

  PolynomialRegressionPredictor predictor(cfg);

  std::vector<double> time = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  double insert_time       = 10.0;

  // Base line: y = 10 + 2t
  std::vector<double> clean, noisy_small, noisy_big;
  for (double t : time) {
    clean.push_back(A_COEFF + B_COEFF * t);
    noisy_small.push_back(A_COEFF + B_COEFF * t + gaussianNoise(0.0, 0.2));
    noisy_big.push_back(A_COEFF + B_COEFF * t + gaussianNoise(0.0, 0.5));
  }

  auto stats_clean = predictor.selectStatisticsValues(clean, time, insert_time);
  auto stats_small = predictor.selectStatisticsValues(noisy_small, time, insert_time);
  auto stats_big   = predictor.selectStatisticsValues(noisy_big, time, insert_time);

  ASSERT_TRUE(stats_clean.poly_reg_computed);
  ASSERT_TRUE(stats_small.poly_reg_computed);
  ASSERT_TRUE(stats_big.poly_reg_computed);

  EXPECT_NEAR(stats_clean.confidence_interval, 0.0, 1e-9);
  EXPECT_GT(stats_small.confidence_interval, 0.0);
  EXPECT_GT(stats_big.confidence_interval, stats_small.confidence_interval);
}
//}

/* TEST(PolyRegressionPredictor, NonUniformTimeStressTest) //{ */
TEST(PolyRegressionPredictor, NonUniformTimeStressTest) {
  PolyRegressionConfig cfg;
  cfg.poly_order            = 2; // Quadratic is safer for irregular gaps
  cfg.decay_factor          = 0.1;
  cfg.conf_prob_percentage  = 95;
  cfg.min_prediction_tol_px = 2;

  PolynomialRegressionPredictor predictor(cfg);

  // Irregular gaps: [0.1, 0.5, 0.2, 1.2, 0.3...]
  std::vector<double> time = {0.0, 0.1, 0.6, 0.8, 2.0, 2.3, 2.5, 3.5, 3.6, 4.0};
  double insert_time       = 5.0; // Predict 1.0s into the future

  std::vector<double> coords;
  for (double t : time) {
    // Trajectory: y = 5 + 1t + 0.5t^2 + noise
    double val = 5.0 + 1.0 * t + 0.5 * t * t + gaussianNoise(0.0, 0.1);
    coords.push_back(val);
  }

  auto stats = predictor.selectStatisticsValues(coords, time, insert_time);

  ASSERT_TRUE(stats.poly_reg_computed);
  ASSERT_FALSE(std::isnan(stats.predicted_coordinate));
}
//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}