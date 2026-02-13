#pragma once

#include <uvdar/blink_processor/ami_tracker_types.h>
#include <uvdar/blink_processor/tseries_ops.h>
#include <uvdar/blink_processor/tseries_buffer.h>

namespace uvdar::blink_processor {

class AmiVerification {
 public:
  explicit AmiVerification(const std::shared_ptr<AmiTrackerConfig> cfg,
                           const std::shared_ptr<TseriesBuffer> active_buffer);
  void run(std::vector<PointState>& unassigned_points, std::vector<SeqPtr>& buffer);

 private:
  void cleanPotentialBuffer_();

  void insertVirtualPointToSequence_(std::vector<PointState>& sequence, const TimePoint& time);

  void addVirtualPointsToIdleSequences_(std::vector<SeqPtr> copy_active_tseries_buffer);
  void enforceMaxBufferLength_(std::vector<PointState>& unmatched_points);
  void startNewSequencesForUnmatchedPoints_(std::vector<PointState>& unmatched_points);
  void insertPointToSequence_(std::vector<PointState>& sequence, const PointState signal);
  int countNumConsecutiveZerosInTseries_(SeqPtr& tseries, const int max_num_zeros);

 private:
  std::shared_ptr<AmiTrackerConfig> cfg_;
  std::shared_ptr<TseriesBuffer> active_tseries_buffer_;
};

} // namespace uvdar::blink_processor