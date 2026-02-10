#pragma once

#include <memory>
#include <mutex>

#include <uvdar/utils/i_logger.h>
#include <uvdar/ami_tracker/ami_tracker_types.h>
#include <uvdar/ami_tracker/signal_matcher.h>
#include <uvdar/ami_tracker/local_search.h>
#include <uvdar/ami_tracker/extended_search.h>

namespace uvdar::ami {

class AmiTracker {
 public:
  explicit AmiTracker(AmiTrackerConfig cfg, ILogger& logger);
  ~AmiTracker() = default;

  [[nodiscard]] bool setSequences(const std::vector<Sequence>& sequences);
  void setFrameRate(const double input);

  void processBuffer(std::vector<PointState>& current_frame);

 private:
  void findClosestPixelAndInsert_(std::vector<PointState>& unmatched_points);
  void cleanPotentialBuffer_();

  void insertVirtualPointToSequence_(std::vector<PointState>& sequence, const TimePoint& time);

  void addVirtualPointsToIdleSequences_(std::vector<SeqPtr> copy_active_tseries_buffer);
  void enforceMaxBufferLength_(std::vector<PointState>& unmatched_points);
  void startNewSequencesForUnmatchedPoints_(std::vector<PointState>& unmatched_points);
  void insertPointToSequence_(std::vector<PointState>& sequence, const PointState signal);
  int countNumConsequtiveZerosInTseries_(SeqPtr& tseries, const int max_num_zeros);

 private:
  AmiTrackerConfig cfg_;
  ILogger& logger_;

  std::unique_ptr<SignalMatcher> signal_matcher_;

  std::vector<Sequence> blinking_patterns_;

  double frame_rate_;

  std::vector<SeqPtr> active_tseries_buffer_;
  std::mutex tseries_buffer_mtx_;

  std::unique_ptr<LocalSearch> local_search_;
  std::unique_ptr<ExtendedSearch> extended_search_;
};

} // namespace uvdar::ami