#pragma once

#include <uvdar/utils/i_logger.h>

#include <uvdar/blink_processor/blink_processor_types.h>
#include <uvdar/blink_processor/signal_matcher.h>
#include <uvdar/blink_processor/ami_tracker.h>

namespace uvdar::blink_processor {

class BlinkProcessor {
 public:
  explicit BlinkProcessor(BlinkProcessorConfig cfg, ILogger* logger);
  ~BlinkProcessor() = default;

  [[nodiscard]] bool setBlinkingPatterns(const std::vector<Sequence>& sequences);

  void processBuffer(std::vector<PointState>& unmatched_points);

  std::vector<TrackedMarker> getResults();

 private:
  BlinkProcessorConfig cfg_;
  ILogger* logger_;

  std::unique_ptr<AmiTracker> ami_tracker_;
  std::unique_ptr<SignalMatcher> signal_matcher_;

  std::vector<Sequence> blinking_patterns_;
};

} // namespace uvdar::blink_processor
