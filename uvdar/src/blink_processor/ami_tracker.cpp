#include <uvdar/blink_processor/ami_tracker.h>

namespace uvdar::blink_processor {

/* AmiTracker constructor //{ */
AmiTracker::AmiTracker(const std::shared_ptr<AmiTrackerConfig> cfg, ILogger& logger) : cfg_(cfg), logger_(logger) {

  cfg_->blinking_patterns_size = blinking_patterns_.at(0).size();
  active_tseries_buffer_       = std::make_shared<TseriesBuffer>();

  local_search_    = std::make_unique<LocalSearch>(cfg_);
  extended_search_ = std::make_unique<ExtendedSearch>(cfg_);
  verification_    = std::make_unique<AmiVerification>(cfg_, active_tseries_buffer_, logger_);
}
//}

/* setSequences //{ */
bool AmiTracker::setSequences(const std::vector<Sequence>& sequences) {
  blinking_patterns_ = sequences;

  signal_matcher_ = std::make_unique<SignalMatcher>(blinking_patterns_, cfg_->allowed_BER_per_seq);
  if (blinking_patterns_.size() == 0) {
    logger_.error("[UVDARAmiTracker]: Provided blinking patterns are empty!");
    return false;
  }

  if ((cfg_->stored_seq_len_factor * blinking_patterns_[0].size()) < cfg_->max_zeros_consecutive) {
    logger_.error("[UVDARAmiTracker]: The wanted number of consecutive zeros is higher than the possible sequence "
                  "length in the buffer! Sequence cannot be set.");
    return false;
  }
  return true;
}
//}

/* setFrameRate //{ */
void AmiTracker::setFrameRate(const double input) {
  if (input > 1.0) {
    frame_rate_ = input;
  } else {
    logger_.warn("[UVDARAmiTracker]: Cannot set non-positive frame rate! Ignoring the command");
  }
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

} // namespace uvdar::blink_processor