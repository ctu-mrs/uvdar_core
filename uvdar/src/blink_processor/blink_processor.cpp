#include <uvdar/blink_processor/blink_processor.h>

namespace uvdar::blink_processor {

/* BlinkProcessor constructor //{ */
BlinkProcessor::BlinkProcessor(BlinkProcessorConfig cfg, ILogger& logger) : cfg_(std::move(cfg)), logger_(logger) {
}
//}

/* setBlinkingPatterns //{ */
bool BlinkProcessor::setBlinkingPatterns(const std::vector<Sequence>& sequences) {
  blinking_patterns_ = sequences;

  if (blinking_patterns_.size() == 0) {
    logger_.error("[UVDARBlinkProcessor]: Provided blinking patterns are empty!");
    return false;
  }

  cfg_.setPatternLength(static_cast<int>(blinking_patterns_[0].size()));

  if (!cfg_.isConfigValid(blinking_patterns_[0].size())) {
    logger_.error("[UVDARBlinkProcessor]: The wanted number of consecutive zeros is higher than the possible sequence "
                  "length in the buffer! Sequence cannot be set.");
    return false;
  }

  ami_tracker_    = std::make_unique<AmiTracker>(cfg_.ami_tracker, logger_);
  signal_matcher_ = std::make_unique<SignalMatcher>(cfg_.signal_matcher, blinking_patterns_);
  return true;
}
//}

/* processBuffer //{ */
void BlinkProcessor::processBuffer(std::vector<PointState>& unmatched_points) {
  ami_tracker_->processBuffer(unmatched_points);
}
//}

/* getResults //{ */
std::vector<TrackedMarker> BlinkProcessor::getResults() {
  const std::size_t raw_window_size = static_cast<std::size_t>(cfg_.seq.getMaxSequenceLength());
  auto tseries_window_buffer_copy   = ami_tracker_->getActiveTrackCopy(raw_window_size);

  logger_.info("[BlinkProcessor] getResults: " + std::to_string(tseries_window_buffer_copy.size()) +
               " tracks, patternLen=" + std::to_string(cfg_.getPatternLength()) +
               ", rawWindow=" + std::to_string(raw_window_size));

  std::vector<TrackedMarker> results;
  results.reserve(tseries_window_buffer_copy.size());

  for (size_t i = 0; i < tseries_window_buffer_copy.size(); ++i) {
    const auto& seq = tseries_window_buffer_copy[i];

    int id = signal_matcher_->matchSignal(seq.led_window);

    results.push_back({seq.last_point, id});
  }

  return results;
}
//}

} // namespace uvdar::blink_processor