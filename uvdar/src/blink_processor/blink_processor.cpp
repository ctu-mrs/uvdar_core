#include <uvdar/blink_processor/blink_processor.h>

namespace uvdar::blink_processor {

/* BlinkProcessor constructor //{ */
BlinkProcessor::BlinkProcessor(const std::shared_ptr<AmiTrackerConfig> cfg, ILogger& logger)
    : cfg_(cfg), logger_(logger) {
  active_tseries_buffer_ = std::make_shared<TseriesBuffer>();
  ami_tracker_           = std::make_unique<AmiTracker>(cfg_, active_tseries_buffer_, logger_);
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
  std::scoped_lock lock(active_tseries_buffer_->mtx);

  std::vector<TrackedMarker> results;
  results.reserve(active_tseries_buffer_->buffer.size());

  for (const auto& seq : active_tseries_buffer_->buffer) {
    auto led_sequence = extractLedWindowForPatternMatch_(seq);

    int id = signal_matcher_->matchSignal(led_sequence);

    results.push_back({seq->back(), id});
  }

  return results;
}
//}

/* extractWindowForPatternMatch_ //{ */
std::vector<bool> BlinkProcessor::extractLedWindowForPatternMatch_(const SeqPtr& tseries) {
  std::vector<bool> led_sequence;

  if (!tseries || tseries->empty() || cfg_->blinking_patterns_size == 0) {
    return led_sequence;
  }

  const std::size_t n     = tseries->size();
  const std::size_t start = (n > cfg_->blinking_patterns_size) ? (n - cfg_->blinking_patterns_size) : 0;

  led_sequence.reserve(n - start);
  for (std::size_t i = start; i < n; ++i) {
    led_sequence.push_back((*tseries)[i].led_state);
  }

  return led_sequence;
}
//}

} // namespace uvdar::blink_processor