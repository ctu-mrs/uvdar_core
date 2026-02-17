#include <gtest/gtest.h>
#include <random>
#include "../dummy_logger.h"
#include "../timer.h"
#include <thread>

#include <uvdar/blink_processor/blink_processor.h>

using namespace uvdar::blink_processor;

/* makeSinglePointFrame//{ */
static std::vector<PointState> makeSinglePointFrame(double x, double y, bool led_state, const TimePoint& t) {
  PointState ps;
  ps.point.x     = x;
  ps.point.y     = y;
  ps.led_state   = led_state;
  ps.insert_time = t;
  return {ps};
}
//}

/* makeTwoPointFrame_//{ */
static std::vector<PointState> makeTwoPointFrame_(const cv::Point2d& p0, bool led0, const cv::Point2d& p1, bool led1,
                                                  const TimePoint& t, bool swap_order) {
  PointState a;
  a.point       = p0;
  a.led_state   = led0;
  a.insert_time = t;

  PointState b;
  b.point       = p1;
  b.led_state   = led1;
  b.insert_time = t;

  if (swap_order) {
    return {b, a};
  }
  return {a, b};
}
//}

/* dist2 //{ */
static double dist2(const cv::Point2d& a, const cv::Point2d& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}
//}

/* idsByExpectedPosition_ //{ */
static std::pair<int, int> idsByExpectedPosition(const std::vector<TrackedMarker>& results, const cv::Point2d& expA,
                                                 const cv::Point2d& expB) {
  // Expect exactly 2 tracks for this test
  const auto& r0 = results[0];
  const auto& r1 = results[1];

  const double r0_to_A = dist2(r0.last_point.point, expA);
  const double r0_to_B = dist2(r0.last_point.point, expB);
  const double r1_to_A = dist2(r1.last_point.point, expA);
  const double r1_to_B = dist2(r1.last_point.point, expB);

  // assignment that minimizes total distance
  const double cost01 = r0_to_A + r1_to_B;
  const double cost10 = r0_to_B + r1_to_A;

  if (cost01 <= cost10) {
    return {r0.id, r1.id}; // (A_id, B_id)
  } else {
    return {r1.id, r0.id}; // (A_id, B_id)
  }
}
//}

/* TEST(BlinkProcessor, AssignsCorrectIdForNewMarker_StaticPoint) //{ */
TEST(BlinkProcessor, AssignsCorrectIdForNewMarker_StaticPoint) {
  const int GROUND_TRUTH_ID = 1;

  TestLogger test_logger;
  std::vector<Sequence> patterns{
      {0, 1, 0, 1, 1, 0, 1, 1},
      {0, 1, 0, 1, 0, 1, 0, 1},
  };

  Sequence emitted = patterns[GROUND_TRUTH_ID];
  std::rotate(emitted.begin(), emitted.begin() + 2, emitted.end());

  // --- Config ---
  auto cfg =
      BlinkProcessorConfig()
          .setSequence({.blinking_patterns_length = static_cast<int>(patterns[0].size()), .stored_seq_len_factor = 20})
          .setPoly({.poly_order = 0, .decay_factor = 0.0, .min_prediction_tol_px = 3, .conf_prob_percentage = 95})
          .setVerification({.max_buffer_length = 100, .max_consecutive_zeros = 4, .allowed_BER_per_seq = 0})
          .setMaxShift({.x = 10.0, .y = 10.0});

  BlinkProcessor bp(cfg, test_logger);
  ASSERT_TRUE(bp.setBlinkingPatterns(patterns));

  const double X = 100.0;
  const double Y = 200.0;
  auto t0        = Clock::now();
  const auto dt  = std::chrono::milliseconds(33);
  for (std::size_t i = 0; i < emitted.size(); ++i) {
    const bool led     = (emitted[i] != 0);
    const TimePoint ti = t0 + dt * static_cast<int>(i);

    auto frame = makeSinglePointFrame(X, Y, led, ti);
    bp.processBuffer(frame);
  }

  auto results = bp.getResults();

  // Expect exactly 1 active marker
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].id, GROUND_TRUTH_ID);
  EXPECT_NEAR(results[0].last_point.point.x, X, 1e-6);
  EXPECT_NEAR(results[0].last_point.point.y, Y, 1e-6);
}
//}

/* TEST(BlinkProcessor, TwoMarkers_ComplexTrajectories_AssignsCorrectIds) //{ */
TEST(BlinkProcessor, TwoMarkers_ComplexTrajectories_AssignsCorrectIds) {
  const int ID_A = 0;
  const int ID_B = 1;

  TestLogger test_logger;
  std::vector<Sequence> patterns{
      {0, 1, 0, 1, 1, 0, 1, 1},
      {0, 1, 0, 1, 0, 1, 0, 1},
  };

  // --- Config ---
  auto cfg =
      BlinkProcessorConfig()
          .setSequence({.blinking_patterns_length = static_cast<int>(patterns[0].size()), .stored_seq_len_factor = 20})
          .setPoly({.poly_order = 4, .decay_factor = 0.1, .min_prediction_tol_px = 3, .conf_prob_percentage = 95})
          .setVerification({.max_buffer_length = 100, .max_consecutive_zeros = 4, .allowed_BER_per_seq = 0})
          .setMaxShift({.x = 10.0, .y = 10.0});

  BlinkProcessor bp(cfg, test_logger);
  ASSERT_TRUE(bp.setBlinkingPatterns(patterns));

  Sequence emitA = patterns[ID_A];
  Sequence emitB = patterns[ID_B];
  std::rotate(emitA.begin(), emitA.begin() + 1, emitA.end());
  std::rotate(emitB.begin(), emitB.begin() + 3, emitB.end());

  const int T   = static_cast<int>(patterns[0].size());
  const auto t0 = Clock::now();
  const auto dt = std::chrono::milliseconds(40);

  // Marker A: diagonal-ish linear motion
  auto posA = [](int k) -> cv::Point2d { return cv::Point2d(50.0 + 8.0 * k, 60.0 + 6.0 * k); };

  // Marker B: a more curved / wavy path
  auto posB = [](int k) -> cv::Point2d {
    const double x = 300.0 + 5.0 * k;
    const double y = 200.0 + 0.25 * k * k + 8.0 * std::sin(0.6 * k);
    return cv::Point2d(x, y);
  };

  // Feed unassigned points to the processor
  for (int k = 0; k < T; ++k) {
    const TimePoint tk = t0 + dt * k;

    const bool ledA = (emitA[static_cast<std::size_t>(k)] != 0);
    const bool ledB = (emitB[static_cast<std::size_t>(k)] != 0);

    const cv::Point2d pA = posA(k);
    const cv::Point2d pB = posB(k);

    // Shuffle order to ensure association doesn't rely on input ordering
    const bool swap = (k % 2 == 1);

    auto frame = makeTwoPointFrame_(pA, ledA, pB, ledB, tk, swap);
    bp.processBuffer(frame);
  }

  auto results = bp.getResults();
  ASSERT_EQ(results.size(), 2u);

  const cv::Point2d expA  = posA(T - 1);
  const cv::Point2d expB  = posB(T - 1);
  const auto [gotA, gotB] = idsByExpectedPosition(results, expA, expB);
  EXPECT_EQ(gotA, ID_A);
  EXPECT_EQ(gotB, ID_B);
}
//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}