#pragma once

#include <memory>
#include <mutex>

#include <uvdar/ami_tracker/ami_tracker_types.h>
#include <uvdar/ami_tracker/tseries_ops.h>

namespace uvdar::ami {

class LocalSearch {
 public:
  explicit LocalSearch(const AmiTrackerConfig& cfg);

  void run(std::vector<PointState>& unassigned_points, std::vector<SeqPtr>& buffer);

 private:
  std::vector<PointState>::iterator findNearestPoint_(std::vector<PointState>& points, const PointState& reference);
  void insertPointToSequence_(std::vector<PointState>& sequence, const PointState signal);

 private:
  const AmiTrackerConfig& cfg_;
};

} // namespace uvdar::ami