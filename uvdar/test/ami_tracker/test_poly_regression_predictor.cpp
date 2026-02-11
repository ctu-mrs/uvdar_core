#include <gtest/gtest.h>
#include <random>
#include "../dummy_logger.h"
#include "../timer.h"
#include <thread>
#include <random>

#include <uvdar/ami_tracker/polynomial_regression_predictor.h>

double gaussianNoise(double mean, double stddev) {
  static std::mt19937 rng(0); //
  std::normal_distribution<double> dist(mean, stddev);
  return dist(rng);
}

/* TEST(PolyRegressionPredictor, NormalizedWeights_SumToOne_AndNewestLargest) //{ */
TEST(PolyRegressionPredictor, NormalizedWeights_SumToOne_AndNewestLargest) {
  using namespace uvdar::ami;

  AmiTrackerConfig cfg;
  cfg.poly_order   = 4;
  cfg.decay_factor = 5.0;

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
  using namespace uvdar::ami;

  AmiTrackerConfig cfg;
  cfg.poly_order          = 1;
  cfg.decay_factor        = 0;
  cfg.conf_probab_percent = 95;
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

  EXPECT_NEAR(stats.predicted_coordinate, stats.coeff[0] + stats.coeff[1] * insert_time, 1e-12);
  ASSERT_TRUE(stats.poly_reg_computed);
  EXPECT_NEAR(stats.predicted_coordinate, expected_y, 1e-9);
  EXPECT_NEAR(stats.confidence_interval, 0.0, 1e-9);
}
//}

TEST(PolyRegressionPredictor, ConfidenceIntervalGrowsWithNoise) {
  const double A_COEFF = 10;
  const double B_COEFF = 2;
  using namespace uvdar::ami;

  AmiTrackerConfig cfg;
  cfg.poly_order          = 1;
  cfg.decay_factor        = 0.0;
  cfg.conf_probab_percent = 95;
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

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}