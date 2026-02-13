#pragma once

#include <memory>
#include <mutex>

#include <uvdar/blink_processor/ami_tracker_types.h>
#include <uvdar/blink_processor/tseries_ops.h>

namespace uvdar::blink_processor {

class LocalSearch {
 public:
  explicit LocalSearch(const std::shared_ptr<AmiTrackerConfig> cfg);

  void run(std::vector<PointState>& unassigned_points, std::vector<SeqPtr>& buffer);

 private:
  std::vector<PointState>::iterator findNearestPoint_(std::vector<PointState>& points, const PointState& reference);

  void insertPointToSequence_(std::vector<PointState>& sequence, const PointState signal);

 private:
  std::shared_ptr<AmiTrackerConfig> cfg_;
};

} // namespace uvdar::blink_processor