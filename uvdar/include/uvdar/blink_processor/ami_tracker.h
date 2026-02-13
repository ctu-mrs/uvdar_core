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
  explicit AmiTracker(const std::shared_ptr<AmiTrackerConfig> cfg, std::shared_ptr<TseriesBuffer> active_tseries_buffer,
                      ILogger& logger);
  ~AmiTracker() = default;

  void setFrameRate(const double input);

  void processBuffer(std::vector<PointState>& unmatched_points);

 private:
 private:
  std::shared_ptr<AmiTrackerConfig> cfg_;
  ILogger& logger_;

  double frame_rate_;

  std::shared_ptr<TseriesBuffer> active_tseries_buffer_;

  std::unique_ptr<LocalSearch> local_search_;
  std::unique_ptr<ExtendedSearch> extended_search_;
  std::unique_ptr<AmiVerification> verification_;
};

} // namespace uvdar::blink_processor