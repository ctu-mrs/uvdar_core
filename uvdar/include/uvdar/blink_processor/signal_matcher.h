#pragma once

#include <vector>
#include <bit>

#include <uvdar/blink_processor/blink_processor_types.h>
#include <uvdar/blink_processor/signal_matcher_enums.h>
#include <uvdar/blink_processor/marker_types.h>

namespace uvdar::blink_processor {

class SignalMatcher {
 public:
  SignalMatcher(const SignalMatcherConfig& config, const std::vector<Sequence>& sequences);

  int matchSignal(const Sequence& signal) const;

 private:
  void initReferenceSignalCodes_(const std::vector<Sequence>& seqs);
  MatchStatus checkSignalSequenceSize_(const Sequence& signal) const;
  int computeHammingDistance_(const uint32_t x, const uint32_t y) const;
  uint32_t packSignalPrefix_(const Sequence& signal) const;

 private:
  const SignalMatcherConfig cfg_;

  std::vector<uint64_t> reference_signal_codes_;
};

} // namespace uvdar::blink_processor