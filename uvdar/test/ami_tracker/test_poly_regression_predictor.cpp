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

/* TEST(PolyRegressionPredictor, ConfidenceInterval) //{ */
TEST(PolyRegressionPredictor, ConfidenceInterval) {
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

/* TEST(PolyRegressionPredictor, ConfidenceInterval_StaticPoints) //{ */
// TEST(PolyRegressionPredictor, ConfidenceInterval_StaticPoints) {
//   const double TRUE_POSITION = 50.0;
//   const int NUM_SAMPLES      = 30;
//   const int DELTA_T_MS       = 10;
//   const double SIGMA         = 2.0;

//   const int TRIALS    = 2000;
//   const double TARGET = 0.75;
//   const double TOL    = 0.03;

//   using namespace uvdar::ami;

//   AmiTrackerConfig cfg;
//   cfg.poly_order          = 0;   // stationary points
//   cfg.decay_factor        = 0.0; // uniform weights
//   cfg.conf_probab_percent = 75.0;
//   PolynomialRegressionPredictor predictor(cfg);

//   int counter = 0;
//   for (int trial = 0; trial < TRIALS; ++trial) {

//     std::vector<PointState> single_seq;
//     auto time = Clock::now();

//     for (size_t n = 0; n < NUM_SAMPLES; ++n) {
//       auto x = TRUE_POSITION + gaussianNoise(0.0, SIGMA);

//       PointState marker;
//       marker.insert_time = time + std::chrono::milliseconds(n * DELTA_T_MS);
//       marker.point       = cv::Point2d(x, 0.0);
//       marker.led_state   = true;
//       single_seq.push_back(std::move(marker));
//     }

//     auto new_time             = time + std::chrono::milliseconds(NUM_SAMPLES * DELTA_T_MS);
//     double insert_time        = std::chrono::duration<double>(new_time.time_since_epoch()).count();
//     auto [x_stats, y_ignored] = predictor.predict(insert_time, single_seq);

//     auto x_next = TRUE_POSITION + gaussianNoise(0.0, SIGMA);
//     if (std::abs(x_next - x_stats.predicted_coordinate) <= x_stats.confidence_interval) {
//       counter++;
//     }
//   }

//   double coverage = static_cast<double>(counter) / TRIALS;
//   EXPECT_NEAR(coverage, TARGET, TOL);
// }
//}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}