#include <gtest/gtest.h>
#include <random>
#include "../dummy_logger.h"
#include "../timer.h"
#include <thread>

#include <uvdar/blink_processor/local_search.h>

using namespace uvdar::blink_processor;

/* TEST(LocalSearch, MatchesPointWithinBoundsAndErases) //{ */
TEST(LocalSearch, MatchesPointWithinBoundsAndErases) {
  LocalSearchConfig local_cfg;
  local_cfg.max_px_shift                 = cv::Point2d(10, 10);
  local_cfg.seq.blinking_patterns_length = 8;
  local_cfg.seq.stored_seq_len_factor    = 20;

  LocalSearch local_search(local_cfg);
  std::vector<SeqPtr> active_tseries_buffer;
  SeqPtr single_seq = std::make_shared<std::vector<PointState>>();
  active_tseries_buffer.push_back(single_seq);

  // Add a marker from past
  PointState marker_t0;
  marker_t0.point = cv::Point2d(50, 50);
  single_seq->push_back(marker_t0);

  /* Receive a new frame where the original marker
     moved within cfg.max_px_shift */
  PointState marker_t1;
  marker_t1.point = cv::Point2d(55, 55); // still in the bounding box
  std::vector<PointState> unprocessed_markers{marker_t1};

  local_search.run(unprocessed_markers, active_tseries_buffer);

  EXPECT_EQ(active_tseries_buffer.size(), 0);                // all series were processed
  EXPECT_TRUE(unprocessed_markers.empty());                  // detected marker has been processed
  EXPECT_EQ(single_seq->size(), 2);                          // detected marker has been moved to t-series
  EXPECT_EQ(single_seq->back().point.x, marker_t1.point.x);  // new point should be last
  EXPECT_EQ(single_seq->front().point.x, marker_t0.point.x); // old point should be first
}
//}

/* TEST(LocalSearch, MatchesPointToSecondTseriesAndRemovesMatchedSeries) //{ */
TEST(LocalSearch, MatchesPointToSecondTseriesAndRemovesMatchedSeries) {
  LocalSearchConfig local_cfg;
  local_cfg.max_px_shift                 = cv::Point2d(10, 10);
  local_cfg.seq.blinking_patterns_length = 8;
  local_cfg.seq.stored_seq_len_factor    = 20;

  LocalSearch local_search(local_cfg);
  std::vector<SeqPtr> active_tseries_buffer;
  SeqPtr first_seq  = std::make_shared<std::vector<PointState>>();
  SeqPtr second_seq = std::make_shared<std::vector<PointState>>();
  active_tseries_buffer.push_back(first_seq);
  active_tseries_buffer.push_back(second_seq);

  // Add markers from past
  PointState marker_s1_t0, marker_s2_t0;
  marker_s1_t0.point = cv::Point2d(50, 50);
  marker_s2_t0.point = cv::Point2d(180, 50);
  first_seq->push_back(marker_s1_t0);
  second_seq->push_back(marker_s2_t0);

  /* Receive a new frame */
  PointState marker_s2_t1;
  marker_s2_t1.point = cv::Point2d(183, 55); // still in the bounding box
  std::vector<PointState> unprocessed_markers{marker_s2_t1};

  local_search.run(unprocessed_markers, active_tseries_buffer);

  EXPECT_EQ(active_tseries_buffer.size(), 1);                   // 1 t-series should remain
  EXPECT_TRUE(unprocessed_markers.empty());                     // detected marker has been processed
  EXPECT_EQ(first_seq->size(), 1);                              // the first t-series should be intact
  EXPECT_EQ(second_seq->size(), 2);                             // detected marker has been moved to the second t-series
  EXPECT_EQ(second_seq->back().point.x, marker_s2_t1.point.x);  // new point should be last
  EXPECT_EQ(second_seq->front().point.x, marker_s2_t0.point.x); // old point should be first
}
//}

/* TEST(LocalSearch, MultiplePointsAndMatchedSeries) //{ */
TEST(LocalSearch, MultiplePointsAndMatchedSeries) {
  const int NUM_TIMESTEPS = 3;

  LocalSearchConfig local_cfg;
  local_cfg.max_px_shift                 = cv::Point2d(10, 10);
  local_cfg.seq.blinking_patterns_length = 8;
  local_cfg.seq.stored_seq_len_factor    = 20;

  LocalSearch local_search(local_cfg);

  std::vector<SeqPtr> active_tseries_buffer;
  SeqPtr first_seq  = std::make_shared<std::vector<PointState>>();
  SeqPtr second_seq = std::make_shared<std::vector<PointState>>();
  SeqPtr third_seq  = std::make_shared<std::vector<PointState>>();

  active_tseries_buffer.push_back(first_seq);
  active_tseries_buffer.push_back(second_seq);
  active_tseries_buffer.push_back(third_seq);

  // Add markers from past to each t-series (distinct trajectories)
  for (size_t seq_id = 0; seq_id < active_tseries_buffer.size(); ++seq_id) {
    auto& seq = active_tseries_buffer[seq_id];
    for (int t = 0; t < NUM_TIMESTEPS; ++t) {
      PointState marker;
      marker.point = cv::Point2d(8 * t + 100 * seq_id, 8 * t + 100 * seq_id);
      seq->push_back(marker);
    }
  }

  /* Receive a new frame */
  // - one point near second t-series last point -> should match second
  // - one point near third t-series last point  -> should match third
  // - one far point -> should remain unassigned
  PointState p_match_second, p_match_third, p_far;
  p_match_second.point = cv::Point2d(118, 114); // within +/-10 of (116,116)
  p_match_third.point  = cv::Point2d(211, 220); // within +/-10 of (216,216)
  p_far.point          = cv::Point2d(1000, 1000);
  std::vector<PointState> unprocessed_markers{p_match_second, p_match_third, p_far};

  local_search.run(unprocessed_markers, active_tseries_buffer);

  EXPECT_EQ(active_tseries_buffer.size(), 1); // 1 umatched t-series
  EXPECT_EQ(unprocessed_markers.size(), 1);   // 1 point was far from all t-series

  EXPECT_EQ(first_seq->size(), NUM_TIMESTEPS);
  EXPECT_EQ(second_seq->size(), NUM_TIMESTEPS + 1);
  EXPECT_EQ(third_seq->size(), NUM_TIMESTEPS + 1);

  // Matched points should be appended as last element
  EXPECT_DOUBLE_EQ(second_seq->back().point.x, p_match_second.point.x);
  EXPECT_DOUBLE_EQ(second_seq->back().point.y, p_match_second.point.y);
  EXPECT_DOUBLE_EQ(third_seq->back().point.x, p_match_third.point.x);
  EXPECT_DOUBLE_EQ(third_seq->back().point.y, p_match_third.point.y);

  // Previous front should still be the original first element
  EXPECT_DOUBLE_EQ(second_seq->front().point.x, 100.0); // seq1 at t=0: (100,100)
  EXPECT_DOUBLE_EQ(second_seq->front().point.y, 100.0);
  EXPECT_DOUBLE_EQ(third_seq->front().point.x, 200.0); // seq2 at t=0: (200,200)
  EXPECT_DOUBLE_EQ(third_seq->front().point.y, 200.0);
}
//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}