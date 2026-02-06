#include <gtest/gtest.h>
#include <random>
#include "../dummy_logger.h"
#include "../timer.h"
#include <thread>

#include <uvdar/ami_tracker/local_search.h>

/* TEST(LocalSearch, MatchesPointWithinBoundsAndErases) //{ */
TEST(LocalSearch, MatchesPointWithinBoundsAndErases) {
  using namespace uvdar::ami;

  AmiTrackerConfig cfg;
  cfg.max_px_shift           = {10, 10};
  cfg.stored_seq_len_factor  = 20;
  cfg.blinking_patterns_size = 8; // 8 bit pattern

  LocalSearch local_search(cfg);
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
  using namespace uvdar::ami;

  AmiTrackerConfig cfg;
  cfg.max_px_shift           = {10, 10};
  cfg.stored_seq_len_factor  = 20;
  cfg.blinking_patterns_size = 8; // 8 bit pattern

  LocalSearch local_search(cfg);
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

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}