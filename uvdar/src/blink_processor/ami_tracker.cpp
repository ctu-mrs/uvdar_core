#include <uvdar/blink_processor/ami_tracker.h>

namespace uvdar::blink_processor {

/* AmiTracker constructor //{ */
AmiTracker::AmiTracker(const AmiTrackerConfig& cfg, ILogger* logger)
    : cfg_(cfg), logger_(logger), active_tseries_buffer_(std::make_shared<TseriesBuffer>()),
      local_search_(std::make_unique<LocalSearch>(cfg_.local)),
      extended_search_(std::make_unique<ExtendedSearch>(cfg_.extended)),
      verification_(std::make_unique<AmiVerification>(cfg_.verification, active_tseries_buffer_, logger_)) {
}
//}

/* processBuffer //{ */
void AmiTracker::processBuffer(std::vector<PointState>& unmatched_points) {
  std::vector<SeqPtr> copy_active_tseries_buffer;
  {
    std::scoped_lock lock(active_tseries_buffer_->mtx);
    copy_active_tseries_buffer = active_tseries_buffer_->buffer;
  }

  local_search_->run(unmatched_points, copy_active_tseries_buffer);

  extended_search_->run(unmatched_points, copy_active_tseries_buffer);

  verification_->run(unmatched_points, copy_active_tseries_buffer);
}
//}

/* getActiveTrackCopy //{ */
std::vector<TrackCopyWindow> AmiTracker::getActiveTrackCopy(std::size_t window_size) const {
  std::scoped_lock lock(active_tseries_buffer_->mtx);

  std::vector<TrackCopyWindow> copy_buffer;
  copy_buffer.reserve(active_tseries_buffer_->buffer.size());

  for (const auto& seq : active_tseries_buffer_->buffer) {
    if (!seq || seq->empty()) {
      continue;
    }

    TrackCopyWindow snapshot;
    snapshot.led_window = extractLedWindowForPatternMatch_(seq, window_size);
    snapshot.last_point = seq->back();
    copy_buffer.push_back(std::move(snapshot));
  }

  return copy_buffer;
}
//}

/* extractWindowForPatternMatch_ //{ */
std::vector<bool> AmiTracker::extractLedWindowForPatternMatch_(const SeqPtr& tseries, const size_t window_size) const {
  std::vector<bool> led_sequence;

  if (!tseries || tseries->empty() || window_size == 0) {
    return led_sequence;
  }

  const std::size_t n     = tseries->size();
  const std::size_t start = (n > window_size) ? (n - window_size) : 0;

  led_sequence.reserve(n - start);
  for (std::size_t i = start; i < n; ++i) {
    led_sequence.push_back((*tseries)[i].led_state);
  }

  return led_sequence;
}
//}

} // namespace uvdar::blink_processor