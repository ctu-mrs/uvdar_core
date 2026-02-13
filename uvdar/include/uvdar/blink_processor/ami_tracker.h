#pragma once

#include <memory>
#include <mutex>

#include <uvdar/utils/i_logger.h>
#include <uvdar/blink_processor/ami_tracker_types.h>
#include <uvdar/blink_processor/tseries_buffer.h>
#include <uvdar/blink_processor/signal_matcher.h>
#include <uvdar/blink_processor/local_search.h>
#include <uvdar/blink_processor/extended_search.h>
#include <uvdar/blink_processor/ami_verification.h>

namespace uvdar::blink_processor {

class AmiTracker {
 public:
  explicit AmiTracker(const std::shared_ptr<AmiTrackerConfig> cfg, ILogger& logger);
  ~AmiTracker() = default;

  [[nodiscard]] bool setSequences(const std::vector<Sequence>& sequences);
  void setFrameRate(const double input);

  void processBuffer(std::vector<PointState>& unmatched_points);

 private:
  // void findClosestPixelAndInsert_(std::vector<PointState>& unmatched_points);
  // void cleanPotentialBuffer_();

  // void insertVirtualPointToSequence_(std::vector<PointState>& sequence, const TimePoint& time);

  // void addVirtualPointsToIdleSequences_(std::vector<SeqPtr> copy_active_tseries_buffer);
  // void enforceMaxBufferLength_(std::vector<PointState>& unmatched_points);
  // void startNewSequencesForUnmatchedPoints_(std::vector<PointState>& unmatched_points);
  // void insertPointToSequence_(std::vector<PointState>& sequence, const PointState signal);
  // int countNumConsecutiveZerosInTseries_(SeqPtr& tseries, const int max_num_zeros);

 private:
  std::shared_ptr<AmiTrackerConfig> cfg_;
  ILogger& logger_;

  std::unique_ptr<SignalMatcher> signal_matcher_;

  std::vector<Sequence> blinking_patterns_;

  double frame_rate_;

  std::shared_ptr<TseriesBuffer> active_tseries_buffer_;

  std::unique_ptr<LocalSearch> local_search_;
  std::unique_ptr<ExtendedSearch> extended_search_;
  std::unique_ptr<AmiVerification> verification_;
};

} // namespace uvdar::blink_processor