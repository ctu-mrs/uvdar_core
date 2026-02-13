#include <uvdar/blink_processor/ami_verification.h>

namespace uvdar::blink_processor {

/* AmiVerification constructor //{ */
AmiVerification::AmiVerification(const std::shared_ptr<AmiTrackerConfig> cfg,
                                 const std::shared_ptr<TseriesBuffer> active_buffer)
    : cfg_(cfg), active_tseries_buffer_(active_buffer) {
}
//}

/* run //{ */
void AmiVerification::run(std::vector<PointState>& unassigned_points, std::vector<SeqPtr>& buffer) {

  addVirtualPointsToIdleSequences_(buffer);

  enforceMaxBufferLength_(unassigned_points);

  startNewSequencesForUnmatchedPoints_(unassigned_points);

  cleanPotentialBuffer_();
}
//}

/* cleanPotentialBuffer_ //{ */
void AmiVerification::cleanPotentialBuffer_() {
  std::scoped_lock lock(active_tseries_buffer_->mtx);

  auto it = active_tseries_buffer_->buffer.begin();
  while (it != active_tseries_buffer_->buffer.end()) {
    SeqPtr& tseries      = *it;
    int delete_criterion = cfg_->max_zeros_consecutive + cfg_->allowed_BER_per_seq;
    if (tseries->size() <= delete_criterion) {
      ++it;
      continue;
    }

    int zero_counter = countNumConsecutiveZerosInTseries_(tseries, delete_criterion);

    if (zero_counter > delete_criterion) {
      it = active_tseries_buffer_->buffer.erase(it);
    } else {
      ++it;
    }
  }
}
//}

/* insertVirtualPointToSequence_ //{ */
void AmiVerification::insertVirtualPointToSequence_(std::vector<PointState>& sequence, const TimePoint& time) {
  // duplicate the last point and set the led_state to zero, so it is ignored by the extended search
  PointState pVirtual;
  pVirtual             = sequence.back();
  pVirtual.insert_time = time;
  pVirtual.led_state   = false;
  insertPointToSequence_(sequence, pVirtual);
}
//}

/* addVirtualPointsToIdleSequences_ //{ */
void AmiVerification::addVirtualPointsToIdleSequences_(std::vector<SeqPtr> copy_active_tseries_buffer) {
  for (auto seq : copy_active_tseries_buffer) {
    auto& tseries         = *seq;
    auto& last_point_time = tseries.back().insert_time;
    insertVirtualPointToSequence_(tseries, last_point_time);
  }
}
//}

/* enforceMaxBufferLength_ //{ */
void AmiVerification::enforceMaxBufferLength_(std::vector<PointState>& unmatched_points) {
  if (active_tseries_buffer_->buffer.size() > cfg_->max_buffer_length) {
    // logger_.error("[AmiVerification]: The maximal excepted buffer length of " +
    //               std::to_string(cfg_->max_buffer_length) + " is reached! " + std::to_string(unmatched_points.size())
    //               + " points will be discarded. Please consider to set the parameter \"max_buffer_length\" higher, if
    //               " "the memory has the capacity.");
    auto diff = active_tseries_buffer_->buffer.size() - cfg_->max_buffer_length;

    active_tseries_buffer_->buffer.erase(active_tseries_buffer_->buffer.begin() + cfg_->max_buffer_length,
                                         active_tseries_buffer_->buffer.end());
  }
}
//}

/* startNewSequencesForUnmatchedPoints_ //{ */
void AmiVerification::startNewSequencesForUnmatchedPoints_(std::vector<PointState>& unmatched_points) {
  std::scoped_lock lock(active_tseries_buffer_->mtx);

  for (auto& point : unmatched_points) {
    std::vector<PointState> vect;
    vect.reserve(cfg_->stored_seq_len_factor * cfg_->blinking_patterns_size);
    vect.emplace_back(point);
    active_tseries_buffer_->buffer.emplace_back(std::make_shared<std::vector<PointState>>(vect));
  }
}
//}

/* insertPointToSequence_ //{ */
void AmiVerification::insertPointToSequence_(std::vector<PointState>& sequence, const PointState signal) {
  sequence.push_back(signal);
  if (sequence.size() > (cfg_->blinking_patterns_size * cfg_->stored_seq_len_factor)) {
    sequence.erase(sequence.begin());
  }
}
//}

/* countNumConsecutiveZerosInTseries_ //{ */
int AmiVerification::countNumConsecutiveZerosInTseries_(SeqPtr& tseries, const int max_num_zeros) {
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