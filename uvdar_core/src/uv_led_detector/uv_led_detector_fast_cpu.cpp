#include <uvdar_core/uv_led_detector/uv_led_detect_fast_cpu.h>

namespace uvdar {

/* UvdarLedDetectFastCpu constructor //{ */
UvdarLedDetectFastCpu::UvdarLedDetectFastCpu(UvLedDetectConfig cfg, ILogger& logger)
    : UvLedDetectFastBase(std::move(cfg), logger) {
  initFast_();
}
//}

/* initFast_ //{ */
void UvdarLedDetectFastCpu::initFast_() {
  initFastPointsSet_();
  initFastInteriorSet_();
}
//}

/* initFastPointsSet_ //{ */
void UvdarLedDetectFastCpu::initFastPointsSet_() {
  fast_points_set_.clear();

  // clang-format off
  fast_points_set_.push_back({
    {0, -3}, {0,  3}, { 3, 0}, {-3, 0},
    {2, -2}, {-2, 2}, {-2,-2}, { 2, 2},
    {-1,-3}, {1,  3}, { 3,-1}, {-3, 1},
    {1, -3}, {-1, 3}, { 3, 1}, {-3,-1},
  });

  fast_points_set_.push_back({
    {0, -4}, {0,  4}, { 4, 0}, {-4, 0},
    {3, -3}, {-3, 3}, {-3,-3}, { 3, 3},
    {-1,-4}, {1,  4}, { 4,-1}, {-4, 1},
    {1, -4}, {-1, 4}, { 4, 1}, {-4,-1},
    {-2,-4}, {2,  4}, { 4,-2}, {-4, 2},
    {2, -4}, {-2, 4}, { 4, 2}, {-4,-2},
  });
  // clang-format on
}
//}

/* initFastInteriorSet_ //{ */
void UvdarLedDetectFastCpu::initFastInteriorSet_() {
  fast_interior_set_.clear();

  // clang-format off
  fast_interior_set_.push_back({
    {0,0},  {1,0},  {2,0},
    {-2,1}, {-1,1},
    {0,1},  {1,1},  {2,1},
    {-1,2},
    {0,2},  {1,2},
  });

  fast_interior_set_.push_back({
    {0,0},  {1,0},  {2,0},  {3,0},
    {-3,1}, {-2,1}, {-1,1},
    {0,1},  {1,1},  {2,1},  {3,1},
    {-3,2}, {-2,2}, {-1,2},
    {0,2},  {1,2},  {2,2},  {3,2},
    {-2,3}, {-1,3},
    {0,3},  {1,3},  {2,3},
  });
  // clang-format on
}
//}

/* initOnFirstFrame_ //{ */
void UvdarLedDetectFastCpu::initOnFirstFrame_() {
  if (first_) {
    first_       = false;
    roi_         = cv::Rect(cv::Point(0, 0), image_curr_.size());
    image_check_ = cv::Mat(image_curr_.size(), CV_8UC1);
    image_check_ = cv::Scalar(0);
  }
}
//}

/* initDelayed //{ */
bool UvdarLedDetectFastCpu::initDelayed([[maybe_unused]] const cv::Mat i_image) {
  return false;
}
//}

/* clearMarks_ //{ */
void UvdarLedDetectFastCpu::clearMarks_() {
  for (int j = 0; j < image_curr_.rows; j++) {
    for (int i = 0; i < image_curr_.cols; i++) {
      if (image_check_.at<unsigned char>(j, i) == 255) {
        image_check_.at<unsigned char>(j, i) = 0;
      }
    }
  }
}
//}

/* validateMask_ //{ */
bool UvdarLedDetectFastCpu::validateMask_(const int mask_id) const noexcept {
  if (mask_id >= 0) {
    if (mask_id >= static_cast<int>(cfg_.masks.size())) {
      logger_.error("[UVDARDetectorFastCpu]: Mask index " + std::to_string(mask_id) +
                    " is greater than the current number of loaded masks!");
      return false;
    }
    if (image_curr_.size() != cfg_.masks[mask_id].size()) {
      logger_.error("[UVDARDetectorFastCpu]: The size of the selected mask does not match the current image!");
      return false;
    }
  }
  return true;
}
//}

/* processImage //{ */
bool UvdarLedDetectFastCpu::processImage(const cv::Mat image, std::vector<cv::Point2i>& detected_points,
                                         std::vector<cv::Point2i>& sun_points, int mask_id) {
  detected_points.clear();
  sun_points.clear();
  image_curr_ = image;

  if (!validateMask_(mask_id)) {
    return false;
  }

  if (cfg_.gui) {
    image_curr_.copyTo(image_view_);
  }

  initOnFirstFrame_();
  clearMarks_();

  scanImageForCandidates_(mask_id, detected_points, sun_points);

  rejectMarkersNearSun_(detected_points, sun_points);

  return true;
}
//}

/* isMaskedOut_ //{ */
inline bool UvdarLedDetectFastCpu::isMaskedOut_(const int point_idx, const int mask_id) const noexcept {
  if (mask_id >= 0) {
    if (cfg_.masks[mask_id].data[point_idx] == 0) {
      return true;
    }
  }
  return false;
}
//}

/* isAlreadyAssignedToCluster_ //{ */
inline bool UvdarLedDetectFastCpu::isAlreadyAssignedToCluster_(const int point_idx) const noexcept {
  return image_check_.data[point_idx] != 0;
}
//}

/* isBelowBrightnessThreshold_ //{ */
inline bool UvdarLedDetectFastCpu::isBelowBrightnessThreshold_(const int point_idx) const noexcept {
  return image_curr_.data[point_idx] <= cfg_.threshold;
}
//}

/* isInsideRoi_ //{ */
inline bool UvdarLedDetectFastCpu::isInsideRoi_(int x, int y) const noexcept {
  return static_cast<unsigned>(x) < static_cast<unsigned>(roi_.width) &&
         static_cast<unsigned>(y) < static_cast<unsigned>(roi_.height);
}
//}

/* isCenterBrighterThanNeighbor_ //{ */
inline bool UvdarLedDetectFastCpu::isCenterBrighterThanNeighbor_(int center, int neighbor) const noexcept {
  return (center - neighbor) >= cfg_.threshold_diff;
}
//}

/* addToCluster_ //{ */
inline void UvdarLedDetectFastCpu::addToCluster_(int idx) {
  image_check_.data[idx] = 255;
}
//}

/* isSunLikePixel_ //{ */
inline bool UvdarLedDetectFastCpu::isSunLikePixel_(const int i, const int j) const noexcept {
  return (image_curr_.data[index2d(i, j, image_curr_.cols)] > cfg_.threshold_sun);
}
//}

/* scanImageForCandidates_ //{ */
void UvdarLedDetectFastCpu::scanImageForCandidates_(const int mask_id, std::vector<cv::Point2i>& detected_points,
                                                    std::vector<cv::Point2i>& sun_points) {
  std::vector<SunCluster> sun_clusters;

  for (int j = 0; j < image_curr_.rows; j++) {
    for (int i = 0; i < image_curr_.cols; i++) {

      const int point_idx = index2d(i, j, image_curr_.cols);

      if (isMaskedOut_(point_idx, mask_id)) {
        continue;
      }
      if (isAlreadyAssignedToCluster_(point_idx)) {
        continue;
      }
      if (isBelowBrightnessThreshold_(point_idx)) {
        continue;
      }

      auto fast_result = evaluateFastRings_(i, j);

      if (fast_result.marker_candidate) {
        localizeMarkerPoint_(fast_result, detected_points);
      } else if (fast_result.sun_candidate) {
        localizeSunPoint_(fast_result, sun_points, sun_clusters);
      }
    }
  }
}
//}

/* evaluateFastRings_ //{ */
FastTestResult UvdarLedDetectFastCpu::evaluateFastRings_(const int i, const int j) {
  FastTestResult result;
  result.i             = i;
  result.j             = j;
  result.sun_candidate = isSunLikePixel_(i, j);
  result.ring_idx      = -1;

  for (const auto& fast_points : fast_points_set_) {
    result.marker_candidate = true;
    result.sun_test_points  = 0;
    result.ring_idx++;

    for (const auto& point : fast_points) {
      const int x = i + point.x;
      const int y = j + point.y;

      if (!isInsideRoi_(x, y)) {
        result.marker_candidate = false;
        break;
      }

      int center   = static_cast<int>(image_curr_.data[index2d(i, j, image_curr_.cols)]);
      int neighbor = static_cast<int>(image_curr_.data[index2d(x, y, image_curr_.cols)]);
      // if (!isCenterBrighterThanNeighbor_(image_curr_.data[center_idx], image_curr_.data[neighbor_idx])) {

      // center_bright = 250;
      // neightbor_bright = 100
      // threshold_diff = 100
      // if (250-100) < 100
      //     150 < 100: //FAlse
      //     marker_candidate = true

      // marker has bigger gradient than sun

      if ((center - neighbor) < cfg_.threshold_diff) {
        result.marker_candidate = false;

        if (!result.sun_candidate) {
          break;
        } else {
          result.sun_test_points++;
        }
      } else { //
        result.sun_candidate = false;
      }
    }

    // If marker is confirmed on smaller ring, no need for larger rings
    if (result.marker_candidate) {
      return result;
    }
  }

  return result;
}
//}

/* localizeMarkerPoint_ //{ */
void UvdarLedDetectFastCpu::localizeMarkerPoint_(const FastTestResult& fast_result,
                                                 std::vector<cv::Point2i>& detected_points) {
  unsigned char best_val = 0;
  cv::Point best_point(fast_result.i, fast_result.j);

  // use largest interior set
  size_t n             = fast_interior_set_.size() - 1;
  const auto& interior = fast_interior_set_[n];

  for (const auto& point : interior) {
    const int x = fast_result.i + point.x;
    const int y = fast_result.j + point.y;

    if (!isInsideRoi_(x, y)) {
      continue;
    }

    const int idx = index2d(x, y, image_curr_.cols);
    if (isAlreadyAssignedToCluster_(idx)) {
      continue;
    }

    // non-maxima suppression - select the brightest point inside the FAST neighborhood
    if (image_curr_.data[idx] > best_val) {
      best_val   = image_curr_.data[idx];
      best_point = cv::Point(x, y);
    }
    addToCluster_(idx);
  }
  detected_points.push_back(best_point);
}
//}

/* localizeSunPoint_ //{ */
void UvdarLedDetectFastCpu::localizeSunPoint_(const FastTestResult& fast_result, std::vector<cv::Point2i>& sun_points,
                                              std::vector<SunCluster>& sun_clusters) {
  if (fast_result.sun_test_points != static_cast<int>(fast_points_set_[fast_result.ring_idx].size())) {
    return;
  }
  const cv::Point point(fast_result.i, fast_result.j);
  bool found{false};

  for (size_t i = 0; i < sun_clusters.size(); ++i) {
    auto& cluster = sun_clusters[i];
    if (cv::norm(point - cluster.centroid()) < cfg_.threshold_sun_merge) {
      cluster.sum += point;
      cluster.count++;
      sun_points[i] = cluster.centroid();
      found         = true;
      break;
    }
  }

  if (!found) {
    sun_clusters.push_back(SunCluster(point, 1));
    sun_points.push_back(point);
  }
}
//}

/* rejectMarkersNearSun_ //{ */
void UvdarLedDetectFastCpu::rejectMarkersNearSun_(std::vector<cv::Point2i>& detected_points,
                                                  const std::vector<cv::Point2i>& sun_points) {

  auto is_glare = [&](const cv::Point2i& p) {
    for (const auto& s : sun_points) {
      if (cv::norm(p - s) < cfg_.threshold_sun_dist)
        return true;
    }
    return false;
  };

  detected_points.erase(std::remove_if(detected_points.begin(), detected_points.end(), is_glare),
                        detected_points.end());
}
//}

} // namespace uvdar
