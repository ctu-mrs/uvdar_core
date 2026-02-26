#pragma once

#include <uvdar/utils/i_logger.h>
#include <uvdar/blink_processor/blink_processor_types.h>
#include <uvdar/blink_processor/tseries_buffer.h>
#include <uvdar/blink_processor/local_search.h>
#include <uvdar/blink_processor/extended_search.h>
#include <uvdar/blink_processor/ami_verification.h>

namespace uvdar::blink_processor {

class AmiTracker {
 public:
  explicit AmiTracker(const AmiTrackerConfig& cfg, ILogger* logger);

  void processBuffer(std::vector<PointState>& unmatched_points);

  [[nodiscard]] std::vector<TrackCopyWindow> getActiveTrackCopy(std::size_t window_size) const;

 private:
  std::vector<bool> extractLedWindowForPatternMatch_(const SeqPtr& tseries, const size_t window_size) const;

 private:
  AmiTrackerConfig cfg_;
  ILogger* logger_;

  std::shared_ptr<TseriesBuffer> active_tseries_buffer_;

  std::unique_ptr<LocalSearch> local_search_;
  std::unique_ptr<ExtendedSearch> extended_search_;
  std::unique_ptr<AmiVerification> verification_;
};

} // namespace uvdar::blink_processor