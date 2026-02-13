#include <uvdar/blink_processor/blink_processor.h>

namespace uvdar::blink_processor {

/* BlinkProcessor constructor //{ */
BlinkProcessor::BlinkProcessor(const std::shared_ptr<AmiTrackerConfig> cfg, ILogger& logger)
    : cfg_(cfg), logger_(logger) {
  ami_tracker_ = std::make_unique<AmiTracker>(cfg_, logger_);
}
//}

/* setBlinkingPatterns //{ */
bool BlinkProcessor::setBlinkingPatterns(const std::vector<Sequence>& sequences) {
  blinking_patterns_           = sequences;
  cfg_->blinking_patterns_size = blinking_patterns_.at(0).size();

  if (blinking_patterns_.size() == 0) {
    logger_.error("[UVDARBlinkProcessor]: Provided blinking patterns are empty!");
    return false;
  }

  if ((cfg_->stored_seq_len_factor * blinking_patterns_[0].size()) < cfg_->max_zeros_consecutive) {
    logger_.error("[UVDARBlinkProcessor]: The wanted number of consecutive zeros is higher than the possible sequence "
                  "length in the buffer! Sequence cannot be set.");
    return false;
  }

  signal_matcher_ = std::make_unique<SignalMatcher>(blinking_patterns_, cfg_->allowed_BER_per_seq);
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
  auto tseries_window_buffer_copy = ami_tracker_->getActiveTrackCopy(cfg_->blinking_patterns_size);

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