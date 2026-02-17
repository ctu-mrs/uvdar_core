#pragma once

#include <memory>
#include <mutex>

#include <uvdar/blink_processor/blink_processor_types.h>
#include <uvdar/blink_processor/tseries_ops.h>
#include <uvdar/blink_processor/marker_types.h>

namespace uvdar::blink_processor {

class LocalSearch {
 public:
  explicit LocalSearch(const LocalSearchConfig& cfg);

  void run(std::vector<PointState>& unassigned_points, std::vector<SeqPtr>& buffer);

 private:
  std::vector<PointState>::iterator findNearestPoint_(std::vector<PointState>& points, const PointState& reference);

  void insertPointToSequence_(std::vector<PointState>& sequence, const PointState signal);

 private:
  const LocalSearchConfig cfg_;
};

} // namespace uvdar::blink_processor