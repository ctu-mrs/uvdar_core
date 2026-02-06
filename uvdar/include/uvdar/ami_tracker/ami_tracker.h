#pragma once

#include <memory>
#include <mutex>

#include <uvdar/utils/i_logger.h>
#include <uvdar/ami_tracker/ami_tracker_types.h>
#include <uvdar/ami_tracker/signal_matcher.h>
#include <uvdar/ami_tracker/local_search.h>

namespace uvdar::ami {

class AmiTracker {
 public:
  explicit AmiTracker(AmiTrackerConfig cfg, ILogger& logger);
  ~AmiTracker() = default;

  [[nodiscard]] bool setSequences(const std::vector<Sequence>& sequences);
  void setFrameRate(const double input);

  void processBuffer(std::vector<PointState>& current_frame);

 private:
  void findClosestPixelAndInsert_(std::vector<PointState>& current_frame);
  void cleanPotentialBuffer_();

 private:
  AmiTrackerConfig cfg_;
  ILogger& logger_;

  std::unique_ptr<SignalMatcher> signal_matcher_;

  std::vector<Sequence> blinking_patterns_;

  double frame_rate_;

  std::vector<SeqPtr> active_tseries_buffer_;
  std::mutex tseries_buffer_mtx_;

  std::unique_ptr<LocalSearch> local_search_;
};

} // namespace uvdar::ami