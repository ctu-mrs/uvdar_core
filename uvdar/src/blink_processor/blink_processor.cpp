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
  logger_.info("[BlinkProcessor] processBuffer: " + std::to_string(unmatched_points.size()) + " input points");
  ami_tracker_->processBuffer(unmatched_points);
  logger_.info("[BlinkProcessor] processBuffer done, " + std::to_string(unmatched_points.size()) +
               " unmatched remaining");
}
//}

/* getResults //{ */
std::vector<TrackedMarker> BlinkProcessor::getResults() {
  constexpr std::size_t kDownsampleWindow = 5;
  const std::size_t raw_window_size       = static_cast<std::size_t>(cfg_.getPatternLength()) * kDownsampleWindow;
  auto tseries_window_buffer_copy         = ami_tracker_->getActiveTrackCopy(raw_window_size);

  logger_.info("[BlinkProcessor] getResults: " + std::to_string(tseries_window_buffer_copy.size()) +
               " tracks, patternLen=" + std::to_string(cfg_.getPatternLength()) +
               ", rawWindow=" + std::to_string(raw_window_size));
  for (size_t i = 0; i < tseries_window_buffer_copy.size(); ++i) {
    const auto& seq = tseries_window_buffer_copy[i];
    std::string led_str;
    for (bool b : seq.led_window)
      led_str += b ? '1' : '0';
    logger_.info("  track[" + std::to_string(i) + "] led_window(" + std::to_string(seq.led_window.size()) +
                 ")=" + led_str + "  lastPt=(" + std::to_string(seq.last_point.point.x) + "," +
                 std::to_string(seq.last_point.point.y) + ")");
  }

  std::vector<TrackedMarker> results;
  results.reserve(tseries_window_buffer_copy.size());

  std::string out = "\n=== Matching Results (" + std::to_string(tseries_window_buffer_copy.size()) + " tracks) ===\n";
  for (size_t i = 0; i < tseries_window_buffer_copy.size(); ++i) {
    const auto& seq = tseries_window_buffer_copy[i];

    Sequence downsampled = downsampleSignal_(seq.led_window, kDownsampleWindow);
    int id               = signal_matcher_->matchSignal(downsampled);

    std::string raw_str, ds_str;
    for (bool b : downsampled)
      ds_str += b ? '1' : '0';
    out += "  [" + std::to_string(i) + "] ds=" + ds_str + "  id=" + std::to_string(id) + "\n";

    results.push_back({seq.last_point, id});
  }
  out += "================================";
  logger_.info(out);

  return results;
}
//}

/* downsampleSignal_ //{ */
Sequence BlinkProcessor::downsampleSignal_(const Sequence& signal, const std::size_t window_size) {
  if (signal.empty() || window_size == 0) {
    return {};
  }

  Sequence result;
  result.reserve((signal.size() + window_size - 1) / window_size);

  for (std::size_t i = 0; i < signal.size(); i += window_size) {
    const std::size_t end = std::min(i + window_size, signal.size());
    int true_count        = 0;
    for (std::size_t j = i; j < end; ++j) {
      if (signal[j]) {
        ++true_count;
      }
    }
    const int window_len = static_cast<int>(end - i);
    result.push_back(true_count > window_len / 2);
  }

  return result;
}
//}

/* printBuffer_ //{ */
void AmiVerification::printBuffer_() const {
  std::scoped_lock lock(active_tseries_buffer_->mtx);
  const auto& buf = active_tseries_buffer_->buffer;

  std::string out = "\n=== TseriesBuffer (" + std::to_string(buf.size()) + " rows) ===\n";
  for (size_t r = 0; r < buf.size(); ++r) {
    const auto& seq = buf[r];
    if (!seq || seq->empty()) {
      out += "  [" + std::to_string(r) + "] (empty)\n";
      continue;
    }
    out += "  [" + std::to_string(r) + "] (" + std::to_string(seq->size()) + " pts): LED=";
    for (size_t p = 0; p < seq->size(); ++p) {
      out += (*seq)[p].led_state ? '1' : '0';
    }
    const auto& last = seq->back();
    char coords[64];
    std::snprintf(coords, sizeof(coords), "  last=(%.1f,%.1f)", last.point.x, last.point.y);
    out += coords;
    out += "\n";
  }
  out += "================================";
  logger_.info(out);
}
//}

} // namespace uvdar::blink_processor