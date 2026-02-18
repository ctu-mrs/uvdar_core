#include <gtest/gtest.h>
#include <random>
#include "../dummy_logger.h"
#include "../timer.h"
#include <thread>

#include <uvdar/blink_processor/extended_search.h>

using namespace uvdar::blink_processor;

double gaussianNoise(double mean, double stddev) {
  static std::mt19937 rng(0); //
  std::normal_distribution<double> dist(mean, stddev);
  return dist(rng);
}

/* TEST(ExtendedSearch, DiverseTrajectories_Poly4) //{ */
TEST(ExtendedSearch, DiverseTrajectories_Poly4) {
  const int NUM_POINTS  = 30;
  const int NUM_TSERIES = 2;

  ExtendedSearchConfig ext_cfg;
  ext_cfg.poly_reg.poly_order                   = 4;
  ext_cfg.poly_reg.decay_factor                 = 0.5;
  ext_cfg.poly_reg.min_prediction_tol_px        = 10;
  ext_cfg.poly_reg.conf_prob_percentage         = 95;
  ext_cfg.seq.blinking_patterns_length          = 8;
  ext_cfg.seq.stored_seq_len_factor             = 20;
  ext_cfg.poly_reg.seq.blinking_patterns_length = 8;
  ext_cfg.poly_reg.seq.stored_seq_len_factor    = 20;

  ExtendedSearch extended_search(ext_cfg);

  auto t0 = Clock::now();
  std::vector<SeqPtr> active_tseries;

  for (int s = 0; s < NUM_TSERIES; ++s) {
    SeqPtr seq = std::make_shared<std::vector<PointState>>();
    for (int t = 0; t < NUM_POINTS; ++t) {
      PointState point;
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

  extended_search.run(unassigned_points, active_tseries);

  // actually matched 300, 900.
  ASSERT_EQ(unassigned_points.size(), 1) << "Valid point was ignored. Check if prediction drifted from 900.0";
  EXPECT_NEAR(unassigned_points[0].point.y, next_y + 50.0, 0.1);

  // Check the sequence updated
  EXPECT_EQ(original_tseries_buffer[0]->size(), NUM_POINTS + 1);
  EXPECT_NEAR(original_tseries_buffer[0]->back().point.y, 900.0, 1.0);
}
//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}