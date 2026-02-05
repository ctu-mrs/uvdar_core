#include <uvdar/ami_tracker/signal_matcher.h>

namespace uvdar::ami {

/* SignalMatcher constructor //{ */
SignalMatcher::SignalMatcher(const std::vector<Sequence>& sequences, const int allowed_BER_per_seq)
    : SEQUENCE_SIZE_(sequences.at(0).size()), ALLOWED_BER_PER_SEQ_(allowed_BER_per_seq) {
  if (2 * SEQUENCE_SIZE_ >= 64) {
    throw std::runtime_error("[AmiTracker]: Maximum sequence size is bigger than 32 bits.");
  }
  initSequences_(sequences);
}
//}

/* matchSignal //{ */
int SignalMatcher::matchSignal(const Sequence& signal) const {
  const auto valid_size = checkSequenceSize_(signal);
  if (valid_size != MatchStatus::SIGNAL_SIZE_CORRECT) {
    return valid_size;
  }

  const uint32_t signal_value = packSignalPrefix_(signal);
  const uint32_t mask         = (1ULL << SEQUENCE_SIZE_) - 1;

  for (size_t seq_id = 0; seq_id < sequences_codes_.size(); ++seq_id) {
    const uint64_t seq = sequences_codes_[seq_id];

    for (size_t phase_offset = 0; phase_offset < SEQUENCE_SIZE_; ++phase_offset) {

      uint32_t current_window = static_cast<uint32_t>((seq >> phase_offset) & mask);

      const int match_errors = computeHammingDistance_(current_window, signal_value);
      if (match_errors <= ALLOWED_BER_PER_SEQ_) {
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
  for (size_t i = 0; i < SEQUENCE_SIZE_; ++i) {
    v |= (uint32_t(signal[i]) << i); // LSB-first
  }
  return v;
}
//}

/* initSequences_ //{ */
void SignalMatcher::initSequences_(const std::vector<Sequence>& seqs) {
  sequences_codes_.clear();
  for (const auto& seq : seqs) {
    uint64_t packed = 0;
    // Pack the sequence twice into a 64-bit integer
    // This allows us to "slide" a window of SEQUENCE_SIZE_ across it
    for (size_t i = 0; i < seq.size() * 2; ++i) {
      if (seq[i % seq.size()]) {
        packed |= (1ULL << i);
      }
    }
    sequences_codes_.push_back(packed);
  }
}
//}

/* checkSequenceSize_ //{ */
MatchStatus SignalMatcher::checkSequenceSize_(const Sequence& signal) const {
  // TODO: replace with enums, have no idea what those return values mean
  const auto& seq_size = signal.size();
  if (seq_size == 0) {
    return MatchStatus::SIGNAL_INVALID;
  } else if (seq_size < SEQUENCE_SIZE_) {
    return MatchStatus::SIGNAL_TOO_SHORT;
  }
  return MatchStatus::SIGNAL_SIZE_CORRECT;
}
//}

} // namespace uvdar::ami