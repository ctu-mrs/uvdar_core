#include <uvdar/blink_processor/blink_processor.h>

namespace uvdar::blink_processor {

/* BlinkProcessor constructor //{ */
BlinkProcessor::BlinkProcessor(BlinkProcessorConfig& cfg, ILogger& logger) : cfg_(std::move(cfg)), logger_(logger) {
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
  // TODO: not sure that this is correct, check whether the given size makes sense
  auto tseries_window_buffer_copy = ami_tracker_->getActiveTrackCopy(cfg_.getPatternLength());

  std::vector<TrackedMarker> results;
  results.reserve(tseries_window_buffer_copy.size());
  for (const auto& seq : tseries_window_buffer_copy) {
    int id = signal_matcher_->matchSignal(seq.led_window);

    results.push_back({seq.last_point, id});
  }

  return results;
}
//}

} // namespace uvdar::blink_processor