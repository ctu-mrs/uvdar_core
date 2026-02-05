#pragma once

#include <memory>

#include <uvdar/ami_tracker/ami_tracker_types.h>
#include <uvdar/ami_tracker/signal_matcher.h>
#include <uvdar/utils/i_logger.h>

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
};

} // namespace uvdar::ami