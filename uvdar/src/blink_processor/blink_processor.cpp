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
std::vector<RetrievedSignal> BlinkProcessor::getResults() {
  std::scoped_lock lock(active_tseries_buffer_->mtx);

  std::vector<RetrievedSignal> results;

  for (auto& seq : active_tseries_buffer_->buffer) {
    auto tseries_window = extractWindowForPatternMatch_(seq);

    std::vector<bool> led_states;
    led_states.reserve(tseries_window.size());
    for (const auto& point : tseries_window) {
      led_states.push_back(point.led_state);
    }

    int id = signal_matcher_->matchSignal(led_states);
    results.push_back({seq, id});
  }
  return results;
}
//}

/* extractWindowForPatternMatch_ //{ */
std::vector<PointState> BlinkProcessor::extractWindowForPatternMatch_(const SeqPtr& tseries) {
  std::vector<PointState> selected;
  if (tseries->size() > cfg_->blinking_patterns_size) {
    selected.insert(selected.end(), tseries->end() - cfg_->blinking_patterns_size, tseries->end());
  } else {
    selected.insert(selected.end(), tseries->begin(), tseries->end());
  }
  return selected;
}
//}

} // namespace uvdar::blink_processor