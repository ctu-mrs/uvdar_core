#include <uvdar/blink_processor/ami_tracker.h>

namespace uvdar::blink_processor {

/* AmiTracker constructor //{ */
AmiTracker::AmiTracker(const std::shared_ptr<AmiTrackerConfig> cfg,
                       std::shared_ptr<TseriesBuffer> active_tseries_buffer, ILogger& logger)
    : cfg_(cfg), logger_(logger), active_tseries_buffer_(active_tseries_buffer) {

  local_search_    = std::make_unique<LocalSearch>(cfg_);
  extended_search_ = std::make_unique<ExtendedSearch>(cfg_);
  verification_    = std::make_unique<AmiVerification>(cfg_, active_tseries_buffer_, logger_);
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