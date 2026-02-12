#include <gtest/gtest.h>
#include <random>
#include "../dummy_logger.h"
#include "../timer.h"
#include <thread>

#include <uvdar/ami_tracker/extended_search.h>

/* TEST(ExtendedSearch, MatchesPointWithinBoundsAndErases) //{ */
TEST(ExtendedSearch, MatchesPointWithinBoundsAndErases) {
  const int NUM_POINTS = 10;
  const int DELTA_POS  = 3;

  using namespace uvdar::ami;

  AmiTrackerConfig cfg;
  cfg.max_px_shift           = {5, 5};
  cfg.stored_seq_len_factor  = 20;
  cfg.decay_factor           = 0.0;
  cfg.poly_order             = 0;
  cfg.conf_probab_percent    = 95;
  cfg.blinking_patterns_size = 8; // 8 bit pattern

  ExtendedSearch extended_search(cfg);

  // given some history of points
  // 1. compute the prediction probability window
  // 2. local check with respect the bounding box

  // One 1 tseries with moving points
  std::vector<SeqPtr> active_tseries_buffer;
  SeqPtr single_seq = std::make_shared<std::vector<PointState>>();
  active_tseries_buffer.push_back(single_seq);
  auto t0 = Clock::now();
  for (size_t t = 0; t < NUM_POINTS; ++t) {
    PointState marker;
    marker.point       = cv::Point2d(50 + t * DELTA_POS, 50 + t * DELTA_POS);
    marker.led_state   = true;
    marker.insert_time = t0 + std::chrono::milliseconds(t * 5);
    single_seq->push_back(std::move(marker));
  }

  // New point
  PointState new_marker;
  new_marker.point       = cv::Point2d(50 + NUM_POINTS * DELTA_POS, 50 + NUM_POINTS * DELTA_POS);
  new_marker.led_state   = true;
  new_marker.insert_time = t0 + std::chrono::milliseconds(NUM_POINTS * 5);
  std::vector<PointState> unprocessed_markers{new_marker};

  extended_search.run(unprocessed_markers, active_tseries_buffer);

  TEST_COUT << "x_pred=" << single_seq->back().x_stats.predicted_coordinate
            << " y_pred=" << single_seq->back().y_stats.predicted_coordinate
            << " x_ci=" << single_seq->back().x_stats.confidence_interval
            << " y_ci=" << single_seq->back().y_stats.confidence_interval << " new=(" << new_marker.point.x << ","
            << new_marker.point.y << ")\n";

  EXPECT_TRUE(single_seq->back().x_stats.poly_reg_computed);
  EXPECT_TRUE(single_seq->back().y_stats.poly_reg_computed);
  EXPECT_EQ(active_tseries_buffer.size(), 0);    // all series were processed
  EXPECT_TRUE(unprocessed_markers.empty());      // detected marker has been processed
  EXPECT_EQ(single_seq->size(), NUM_POINTS + 1); // detected marker has been moved to t-series
}
//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}