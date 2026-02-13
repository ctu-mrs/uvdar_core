#include <uvdar/blink_processor/ami_tracker.h>

namespace uvdar::blink_processor {

/* AmiTracker constructor //{ */
AmiTracker::AmiTracker(const std::shared_ptr<AmiTrackerConfig> cfg, ILogger& logger) : cfg_(cfg), logger_(logger) {

  cfg_->blinking_patterns_size = blinking_patterns_.at(0).size();
  local_search_                = std::make_unique<LocalSearch>(cfg_);
  extended_search_             = std::make_unique<ExtendedSearch>(cfg_);
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
void AmiTracker::processBuffer(std::vector<PointState>& current_frame) {
  findClosestPixelAndInsert_(current_frame);
  cleanPotentialBuffer_();
}
//}

/* findClosestPixelAndInsert_ //{ */
void AmiTracker::findClosestPixelAndInsert_(std::vector<PointState>& unmatched_points) {
  std::vector<SeqPtr> copy_active_tseries_buffer;
  {
    std::scoped_lock lock(tseries_buffer_mtx_);
    // this does not create a copy of data, it only copies the container
    copy_active_tseries_buffer = active_tseries_buffer_;
  }

  local_search_->run(unmatched_points, copy_active_tseries_buffer);
  extended_search_->run(unmatched_points, copy_active_tseries_buffer);

  addVirtualPointsToIdleSequences_(copy_active_tseries_buffer);

  enforceMaxBufferLength_(unmatched_points);

  startNewSequencesForUnmatchedPoints_(unmatched_points);
}

/* insertVirtualPointToSequence_ //{ */
void AmiTracker::insertVirtualPointToSequence_(std::vector<PointState>& sequence, const TimePoint& time) {
  // duplicate the last point and set the led_state to zero, so it is ignored by the extended search
  PointState pVirtual;
  pVirtual             = sequence.back();
  pVirtual.insert_time = time;
  pVirtual.led_state   = false;
  insertPointToSequence_(sequence, pVirtual);
}
//}

/* addVirtualPointsToIdleSequences_ //{ */
void AmiTracker::addVirtualPointsToIdleSequences_(std::vector<SeqPtr> copy_active_tseries_buffer) {
  for (auto seq : copy_active_tseries_buffer) {
    auto& tseries         = *seq;
    auto& last_point_time = tseries.back().insert_time;
    insertVirtualPointToSequence_(tseries, last_point_time);
  }
}
//}

/* enforceMaxBufferLength_ //{ */
void AmiTracker::enforceMaxBufferLength_(std::vector<PointState>& unmatched_points) {
  if (active_tseries_buffer_.size() > cfg_->max_buffer_length) {
    logger_.error("[AmiTracker]: The maximal excepted buffer length of " + std::to_string(cfg_->max_buffer_length) +
                  " is reached! " + std::to_string(unmatched_points.size()) +
                  " points will be discarded. Please consider to set the parameter \"max_buffer_length\" higher, if "
                  "the memory has the capacity.");
    auto diff = active_tseries_buffer_.size() - cfg_->max_buffer_length;

    active_tseries_buffer_.erase(active_tseries_buffer_.begin() + cfg_->max_buffer_length,
                                 active_tseries_buffer_.end());
  }
}
//}

/* startNewSequencesForUnmatchedPoints_ //{ */
void AmiTracker::startNewSequencesForUnmatchedPoints_(std::vector<PointState>& unmatched_points) {
  std::scoped_lock lock(tseries_buffer_mtx_);

  for (auto& point : unmatched_points) {
    std::vector<PointState> vect;
    vect.reserve(cfg_->stored_seq_len_factor * blinking_patterns_[0].size());
    vect.emplace_back(point);
    active_tseries_buffer_.emplace_back(std::make_shared<std::vector<PointState>>(vect));
  }
}
//}

void AmiTracker::insertPointToSequence_(std::vector<PointState>& sequence, const PointState signal) {
  sequence.push_back(signal);
  if (sequence.size() > (cfg_->blinking_patterns_size * cfg_->stored_seq_len_factor)) {
    sequence.erase(sequence.begin());
  }
}

/* cleanPotentialBuffer_ //{ */
/// @brief Remove sequences whose signal has been missing for too long. The original paper calls this part as a
/// verification method
void AmiTracker::cleanPotentialBuffer_() {
  std::scoped_lock lock(tseries_buffer_mtx_);

  auto it = active_tseries_buffer_.begin();
  while (it != active_tseries_buffer_.end()) {
    SeqPtr& tseries      = *it;
    int delete_criterion = cfg_->max_zeros_consecutive + cfg_->allowed_BER_per_seq;
    if (tseries->size() <= delete_criterion) {
      ++it;
      continue;
    }

    int zero_counter = countNumConsecutiveZerosInTseries_(tseries, delete_criterion);

    if (zero_counter > delete_criterion) {
      it = active_tseries_buffer_.erase(it);
    } else {
      ++it;
    }
  }
}
//}

/* countNumConsecutiveZerosInTseries_ //{ */
int AmiTracker::countNumConsecutiveZerosInTseries_(SeqPtr& tseries, const int max_num_zeros) {
  int zero_counter{0};
  for (const auto& point_state : (*tseries)) {
    if (point_state.led_state) {
      zero_counter = 0;
      continue;
    }

    zero_counter++;
    if (zero_counter > max_num_zeros) {
      break;
    }
  }
  return zero_counter;
}
//}

} // namespace uvdar::blink_processor