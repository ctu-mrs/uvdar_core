#include <uvdar/blink_processor/signal_matcher.h>

namespace uvdar::blink_processor {

/* SignalMatcher constructor //{ */
SignalMatcher::SignalMatcher(const SignalMatcherConfig& config, const std::vector<Sequence>& sequences) : cfg_(config) {
  if (2 * cfg_.seq.blinking_patterns_length >= 64) {
    throw std::runtime_error("[UVDARBlinkProcessor]: Maximum sequence size is bigger than 32 bits.");
  }
  initReferenceSignalCodes_(sequences);
}
//}

/* matchSignal //{ */
int SignalMatcher::matchSignal(const Sequence& signal) const {
  const auto valid_size = checkSignalSequenceSize_(signal);
  if (valid_size != MatchStatus::SIGNAL_SIZE_CORRECT) {
    return valid_size;
  }

  const uint32_t signal_value = packSignalPrefix_(signal);
  const uint32_t mask         = (1ULL << cfg_.seq.blinking_patterns_length) - 1;

  for (size_t seq_id = 0; seq_id < reference_signal_codes_.size(); ++seq_id) {
    const uint64_t seq = reference_signal_codes_[seq_id];

    for (size_t phase_offset = 0; phase_offset < cfg_.seq.blinking_patterns_length; ++phase_offset) {

      uint32_t current_window = static_cast<uint32_t>((seq >> phase_offset) & mask);

      const int match_errors = computeHammingDistance_(current_window, signal_value);
      if (match_errors <= cfg_.allowed_BER_per_seq) {
        return static_cast<int>(seq_id);
      }
    }
  }

  return MatchStatus::SIGNAL_INVALID;
}
//}

/* computeHammingDistance_ //{ */
int SignalMatcher::computeHammingDistance_(const uint32_t x, const uint32_t y) const {
  // // Brian-Kernighan approach
  // int dist{0};
  // uint32_t Xor = x ^ y;
  // while (Xor) {
  //   dist++;
  //   Xor &= Xor - 1;
  // }

  // return dist;

  return std::popcount(x ^ y);
}
//}

/* packSignalPrefix_ //{ */
uint32_t SignalMatcher::packSignalPrefix_(const Sequence& signal) const {
  uint32_t v = 0;
  for (size_t i = 0; i < cfg_.seq.blinking_patterns_length; ++i) {
    v |= (uint32_t(signal[i]) << i); // LSB-first
  }
  return v;
}
//}

/* initReferenceSignalCodes_ //{ */
void SignalMatcher::initReferenceSignalCodes_(const std::vector<Sequence>& seqs) {
  reference_signal_codes_.clear();
  for (const auto& seq : seqs) {
    uint64_t packed = 0;
    // Pack the sequence twice into a 64-bit integer
    for (size_t i = 0; i < seq.size() * 2; ++i) {
      if (seq[i % seq.size()]) {
        packed |= (1ULL << i);
      }
    }
    reference_signal_codes_.push_back(packed);
  }
}
//}

/* checkSignalSequenceSize_ //{ */
MatchStatus SignalMatcher::checkSignalSequenceSize_(const Sequence& signal) const {
  const auto& seq_size = signal.size();
  if (seq_size == 0) {
    return MatchStatus::SIGNAL_INVALID;
  } else if (seq_size < cfg_.seq.blinking_patterns_length) {
    return MatchStatus::SIGNAL_TOO_SHORT;
  }
  return MatchStatus::SIGNAL_SIZE_CORRECT;
}
//}

} // namespace uvdar::blink_processor