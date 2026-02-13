#pragma once

#include <uvdar/blink_processor/ami_tracker.h>

namespace uvdar::blink_processor {

class BlinkProcessor {
 public:
  explicit BlinkProcessor(const std::shared_ptr<AmiTrackerConfig> cfg, ILogger& logger);
  ~BlinkProcessor() = default;

  [[nodiscard]] bool setBlinkingPatterns(const std::vector<Sequence>& sequences);
  // void setFrameRate(const double input);

  void processBuffer(std::vector<PointState>& unmatched_points);

  std::vector<RetrievedSignal> getResults();

 private:
  std::vector<PointState> extractWindowForPatternMatch_(const SeqPtr& tseries);

 private:
  std::shared_ptr<AmiTrackerConfig> cfg_;
  ILogger& logger_;

  std::unique_ptr<AmiTracker> ami_tracker_;
  std::unique_ptr<SignalMatcher> signal_matcher_;

  std::vector<Sequence> blinking_patterns_;
  std::shared_ptr<TseriesBuffer> active_tseries_buffer_;
};

} // namespace uvdar::blink_processor
