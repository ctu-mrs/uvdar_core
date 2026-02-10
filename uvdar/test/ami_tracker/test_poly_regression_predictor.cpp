#include <gtest/gtest.h>
#include <random>
#include "../dummy_logger.h"
#include "../timer.h"
#include <thread>

#include <uvdar/ami_tracker/polynomial_regression_predictor.h>

/* TEST(PolyRegressionPredictor, MatchesPointWithinBoundsAndErases) //{ */
TEST(ExtendedSearch, MatchesPointWithinBoundsAndErases) {
  const int NUM_POINTS = 3;
  const int DELTA_POS  = 3;

  using namespace uvdar::ami;

  AmiTrackerConfig cfg;
  cfg.max_px_shift           = {5, 5};
  cfg.stored_seq_len_factor  = 20;
  cfg.blinking_patterns_size = 8; // 8 bit pattern
}
//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}