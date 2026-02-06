#pragma once

#include <uvdar/ami_tracker/ami_tracker_types.h>

namespace uvdar::ami {

double euclideanDistanceSq(const cv::Point2d& p1, const cv::Point2d& p2);

[[nodiscard]] bool isInsideBoundingBox(const cv::Point2d& query_point, const cv::Point2d& left_top,
                                       const cv::Point2d& right_bottom);

} // namespace uvdar::ami