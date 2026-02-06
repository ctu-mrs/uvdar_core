#include <uvdar/ami_tracker/local_search.h>

namespace uvdar::ami {

/* LocalSearch //{ */
LocalSearch::LocalSearch(const AmiTrackerConfig& cfg) : cfg_(cfg) {
}
//}

/* run //{ */
void LocalSearch::run(std::vector<PointState>& unassigned_points, std::vector<SeqPtr>& buffer) {
  auto it = buffer.begin();
  while (it != buffer.end()) {
    auto& tseries             = *it;
    PointState& last_inserted = tseries->back();

    auto bb_left_top     = last_inserted.point - cfg_.max_px_shift;
    auto bb_right_bottom = last_inserted.point + cfg_.max_px_shift;

    auto nearest_point_it = findNearestPoint_(unassigned_points, last_inserted);

    if (nearest_point_it == unassigned_points.end()) {
      ++it;
      continue;
    }
    if (!isInsideBoundingBox(nearest_point_it->point, bb_left_top, bb_right_bottom)) {
      ++it;
      continue;
    }

    insertPointSequence_(*tseries, *nearest_point_it);
    // Remove the point from current_frame so it is not reused
    unassigned_points.erase(nearest_point_it);
    // Remove sequence from active buffer
    it = buffer.erase(it);
  }
}
//}

/* findNearestPoint_ //{ */
std::vector<PointState>::iterator LocalSearch::findNearestPoint_(std::vector<PointState>& points,
                                                                 const PointState& reference) {
  if (points.empty()) {
    return points.end();
  }

  return std::min_element(points.begin(), points.end(), [&](const PointState& a, const PointState& b) {
    return euclideanDistanceSq(a.point, reference.point) < euclideanDistanceSq(b.point, reference.point);
  });
}
//}

/* insertPointSequence_ //{ */
void LocalSearch::insertPointSequence_(std::vector<PointState>& sequence, const PointState signal) {
  sequence.push_back(signal);
  if (sequence.size() > (cfg_.blinking_patterns_size * cfg_.stored_seq_len_factor)) {
    sequence.erase(sequence.begin());
  }
}
//}

} // namespace uvdar::ami