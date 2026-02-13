#include <gtest/gtest.h>
#include <random>
#include "../dummy_logger.h"
#include "../timer.h"
#include <thread>

#include <uvdar/blink_processor/extended_search.h>

double gaussianNoise(double mean, double stddev) {
  static std::mt19937 rng(0); //
  std::normal_distribution<double> dist(mean, stddev);
  return dist(rng);
}

// TEST(ExtendedSearch, LogicVerification_WithLogging) {
//   using namespace uvdar::ami;

//   AmiTrackerConfig cfg;
//   cfg.max_px_shift           = {2, 2}; // Strict base shift
//   cfg.poly_order             = 1;      // Linear prediction
//   cfg.conf_probab_percent    = 99;     // High confidence for tight matching
//   cfg.stored_seq_len_factor  = 20;
//   cfg.blinking_patterns_size = 8; // 8 bit pattern

//   ExtendedSearch extended_search(cfg);

//   auto t0 = Clock::now();

//   // --- SEQUENCE A: Moving Diagonally ---
//   SeqPtr seq_a = std::make_shared<std::vector<PointState>>();
//   for (int i = 0; i < 5; ++i) {
//     PointState p;
//     p.point       = cv::Point2d(100 + i * 10, 100 + i * 10); // (100,100) -> (140,140)
//     p.led_state   = true;
//     p.insert_time = t0 + std::chrono::milliseconds(i * 10);
//     seq_a->push_back(p);
//   }

//   // --- UNPROCESSED POINTS ---
//   // Point 1: Valid (follows trend)
//   PointState valid_pt;
//   valid_pt.point       = cv::Point2d(150, 150);
//   valid_pt.led_state   = true;
//   valid_pt.insert_time = t0 + std::chrono::milliseconds(50);

//   // Point 2: Outlier (too far away)
//   PointState outlier_pt;
//   outlier_pt.point       = cv::Point2d(200, 200);
//   outlier_pt.led_state   = true;
//   outlier_pt.insert_time = t0 + std::chrono::milliseconds(50);

//   std::vector<PointState> unprocessed = {valid_pt, outlier_pt};
//   std::vector<SeqPtr> active_tseries  = {seq_a};

//   // Run the logic
//   extended_search.run(unprocessed, active_tseries);

//   // --- ASSERTIONS ---
//   // 1. The valid point should have been added to seq_a
//   EXPECT_EQ(seq_a->size(), 6);
//   EXPECT_NEAR(seq_a->back().point.x, 150.0, 0.1);

//   // 2. The valid point should be removed from the 'unprocessed' list
//   // 3. The outlier should still be in 'unprocessed' because it didn't fit seq_a
//   EXPECT_EQ(unprocessed.size(), 1);
//   EXPECT_NEAR(unprocessed[0].point.x, 200.0, 0.1);
// }

TEST(ExtendedSearch, DiverseTrajectories_Poly4) {
  const int NUM_POINTS  = 30;
  const int NUM_TSERIES = 2;

  using namespace uvdar::blink_processor;

  AmiTrackerConfig cfg;
  // This ensures that even with a perfect fit, the box is at least 10x10 px
  cfg.max_px_shift           = {10.0, 10.0};
  cfg.poly_order             = 4;
  cfg.decay_factor           = 0.5;
  cfg.conf_probab_percent    = 95;
  cfg.stored_seq_len_factor  = 20;
  cfg.blinking_patterns_size = 8;
  auto shared_cfg            = std::make_shared<AmiTrackerConfig>(cfg);

  ExtendedSearch extended_search(shared_cfg);

  auto t0 = Clock::now();
  std::vector<SeqPtr> active_tseries;

  for (int s = 0; s < NUM_TSERIES; ++s) {
    SeqPtr seq = std::make_shared<std::vector<PointState>>();
    for (int t = 0; t < NUM_POINTS; ++t) {
      PointState point;
      // Use t directly as a spatial and temporal coordinate
      double x_val = t * 10.0;

      if (s == 0) {
        // Trajectory 1: Parabola y = 0.01 * x^2
        point.point.x = x_val;
        point.point.y = 0.01 * (x_val * x_val);
      } else {
        // Trajectory 2: Sine wave
        point.point.x = x_val;
        point.point.y = 100.0 + 10.0 * std::sin(x_val / 50.0);
      }

      point.led_state = true;
      // 100ms intervals between points
      point.insert_time =
          t0 + std::chrono::milliseconds(t * 100) + std::chrono::milliseconds((int)gaussianNoise(0.0, 20));

      seq->push_back(std::move(point));
    }
    active_tseries.push_back(std::move(seq));
  }
  auto original_tseries_buffer = active_tseries;

  // --- Setup the "Future" point (t = 30) ---
  int next_t           = NUM_POINTS;               // 30
  double next_x        = next_t * 10.0;            // 300.0
  double next_y        = 0.01 * (next_x * next_x); // 0.01 * 90000 = 900.0
  auto prediction_time = t0 + std::chrono::milliseconds(next_t * 100);

  PointState valid_pt;
  valid_pt.point.x     = next_x;
  valid_pt.point.y     = next_y;
  valid_pt.led_state   = true;
  valid_pt.insert_time = prediction_time;

  PointState reject_pt;
  reject_pt.point.x     = next_x;
  reject_pt.point.y     = next_y + 50.0; // Outside the 10px search floor
  reject_pt.led_state   = true;
  reject_pt.insert_time = prediction_time;

  std::vector<PointState> unassigned_points = {valid_pt, reject_pt};

  // The critical run
  extended_search.run(unassigned_points, active_tseries);

  // --- Verification ---
  // If this fails, the CSV logger will show you if the "prediction" x,y
  // actually matched 300, 900.
  ASSERT_EQ(unassigned_points.size(), 1) << "Valid point was ignored. Check if prediction drifted from 900.0";
  EXPECT_NEAR(unassigned_points[0].point.y, next_y + 50.0, 0.1);

  // Check the sequence updated
  EXPECT_EQ(original_tseries_buffer[0]->size(), NUM_POINTS + 1);
  EXPECT_NEAR(original_tseries_buffer[0]->back().point.y, 900.0, 1.0);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}