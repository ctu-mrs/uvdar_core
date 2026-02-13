#include <uvdar/blink_processor/tseries_ops.h>

namespace uvdar::blink_processor {

/* euclideanDistanceSq_ //{ */
double euclideanDistanceSq(const cv::Point2d& p1, const cv::Point2d& p2) {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return (dx * dx) + (dy * dy);
}
//}

/* isInsideBoundingBox //{ */
bool isInsideBoundingBox(const cv::Point2d& query_point, const cv::Point2d& left_top, const cv::Point2d& right_bottom) {
  return (left_top.x <= query_point.x && query_point.x <= right_bottom.x) &&
         (left_top.y <= query_point.y && query_point.y <= right_bottom.y);
}
//}

} // namespace uvdar::blink_processor