#pragma once

#include <vector>
#include <bit>

#include <uvdar/ami_tracker/ami_tracker_types.h>
#include <uvdar/ami_tracker/signal_matcher_enums.h>

namespace uvdar::ami {

class SignalMatcher {
 public:
  SignalMatcher(const std::vector<Sequence>& sequences, const int allowed_BER_per_seq);
  int matchSignal(const Sequence& signal) const;

 private:
  void initSequences_(const std::vector<Sequence>& seqs);
  MatchStatus checkSequenceSize_(const Sequence& signal) const;
  int computeHammingDistance_(const uint32_t x, const uint32_t y) const;
  uint32_t packSignalPrefix_(const Sequence& signal) const;

 private:
  const size_t SEQUENCE_SIZE_;
  const int ALLOWED_BER_PER_SEQ_;

  std::vector<uint64_t> sequences_codes_;
};

} // namespace uvdar::ami