#pragma once

#include <uvdar/utils/i_logger.h>
#include <uvdar/blink_processor/blink_processor_types.h>
#include <uvdar/blink_processor/tseries_ops.h>
#include <uvdar/blink_processor/tseries_buffer.h>
#include <uvdar/blink_processor/marker_types.h>

namespace uvdar::blink_processor {

class AmiVerification {
 public:
  explicit AmiVerification(const VerificationConfig& cfg, std::shared_ptr<TseriesBuffer> active_buffer,
                           ILogger* logger);

  void run(std::vector<PointState>& unassigned_points, std::vector<SeqPtr>& buffer);

 private:
  void cleanPotentialBuffer_();

  void insertVirtualPointToSequence_(std::vector<PointState>& sequence, const TimePoint& time);

  void addVirtualPointsToIdleSequences_(std::vector<SeqPtr>& copy_active_tseries_buffer);

  void enforceMaxBufferLength_(std::vector<PointState>& unmatched_points);

  void startNewSequencesForUnmatchedPoints_(std::vector<PointState>& unmatched_points);

  void insertPointToSequence_(std::vector<PointState>& sequence, const PointState& signal);

  int countNumConsecutiveZerosInTseries_(SeqPtr& tseries, const int max_num_zeros);

  void printBuffer_() const;

 private:
  const VerificationConfig cfg_;
  std::shared_ptr<TseriesBuffer> active_tseries_buffer_;
  ILogger* logger_;
};

} // namespace uvdar::blink_processor