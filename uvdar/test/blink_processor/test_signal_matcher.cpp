#include <gtest/gtest.h>
#include <random>
#include "../dummy_logger.h"
#include "../timer.h"
#include <thread>

#include <uvdar/blink_processor/signal_matcher.h>

/* TEST(SignalMatcher, ExactLengthSignal) //{ */
TEST(SignalMatcher, ExactLengthSignal) {
  using namespace uvdar::blink_processor;

  // clang-format off
  const int GROUND_TRUTH_ID{1};
  std::vector<Sequence> seqs{
    {0, 1, 0, 1, 1, 0, 1, 1}, 
    {0, 1, 0, 1, 0, 1, 0, 1}
  };
  // clang-format on

  Sequence signal = seqs[GROUND_TRUTH_ID];
  std::rotate(signal.begin(), signal.begin() + 2, signal.end());

  const int allowed_BER_per_seq{0};
  SignalMatcher matcher(seqs, allowed_BER_per_seq);
  const int signal_id = matcher.matchSignal(signal);

  EXPECT_EQ(GROUND_TRUTH_ID, signal_id);
}
//}

/* TEST(SignalMatcher, LongerLengthSignal) //{ */
TEST(SignalMatcher, LongerLengthSignal) {
  using namespace uvdar::blink_processor;

  // clang-format off
  const int GROUND_TRUTH_ID{1};
  std::vector<Sequence> seqs{
    {0, 1, 0, 1, 1, 0, 1, 1}, 
    {0, 1, 0, 1, 0, 1, 0, 1}
  };
  // clang-format on

  Sequence signal = seqs[GROUND_TRUTH_ID];
  signal.insert(signal.end(), seqs[GROUND_TRUTH_ID].begin(), seqs[GROUND_TRUTH_ID].end());
  std::rotate(signal.begin(), signal.begin() + 5, signal.end());

  const int allowed_BER_per_seq{0};
  SignalMatcher matcher(seqs, allowed_BER_per_seq);
  const int signal_id = matcher.matchSignal(signal);

  EXPECT_EQ(GROUND_TRUTH_ID, signal_id);
}
//}

/* TEST(SignalMatcher, ShorterLengthSignal) //{ */
TEST(SignalMatcher, ShorterLengthSignal) {
  using namespace uvdar::blink_processor;

  // clang-format off
  const int GROUND_TRUTH_ID{1};
  std::vector<Sequence> seqs{
    {0, 1, 0, 1, 1, 0, 1, 1}, 
    {0, 1, 0, 1, 0, 1, 0, 1}
  };
  // clang-format on

  Sequence signal(seqs[GROUND_TRUTH_ID].begin(), seqs[GROUND_TRUTH_ID].begin() + 5);
  std::rotate(signal.begin(), signal.begin() + 2, signal.end());

  const int allowed_BER_per_seq{0};
  SignalMatcher matcher(seqs, allowed_BER_per_seq);
  const int signal_id = matcher.matchSignal(signal);

  EXPECT_EQ(MatchStatus::SIGNAL_TOO_SHORT, signal_id);
}
//}

/* TEST(SignalMatcher, LongerSequenceThan32bits) //{ */
TEST(SignalMatcher, LongerSequenceThan32bits) {
  using namespace uvdar::blink_processor;

  const int LONGER_SEQUENCE{33};
  std::vector<Sequence> seqs{Sequence(LONGER_SEQUENCE, true)};
  const int allowed_BER_per_seq{0};

  EXPECT_THROW({ SignalMatcher matcher(seqs, allowed_BER_per_seq); }, std::runtime_error);
}
//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}