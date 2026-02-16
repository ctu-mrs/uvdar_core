#pragma once

#include <uvdar/blink_processor/blink_processor_types.h>

namespace uvdar::blink_processor {

double euclideanDistanceSq(const cv::Point2d& p1, const cv::Point2d& p2);

[[nodiscard]] bool isInsideBoundingBox(const cv::Point2d& query_point, const cv::Point2d& left_top,
                                       const cv::Point2d& right_bottom);

} // namespace uvdar::blink_processor