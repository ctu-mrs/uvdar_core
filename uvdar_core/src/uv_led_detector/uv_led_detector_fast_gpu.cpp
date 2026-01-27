#include <uvdar_core/uv_led_detector/uv_led_detect_fast_gpu.h>

namespace uvdar {

/* UvdarLedDetectFastCpu constructor //{ */
UvdarLedDetectFastGpu::UvdarLedDetectFastGpu(UvLedDetectConfig cfg, ILogger& logger)
    : UvLedDetectFastBase(std::move(cfg), logger), gpu_mgr_(GpuContext::GetInstance()) {
  logGpuProperties_();
  initFastInteriorSet_();
  initGpuComputing_();
}
//}

/* processImage //{ */
bool UvdarLedDetectFastGpu::processImage(const cv::Mat image, std::vector<cv::Point2i>& detected_points,
                                         std::vector<cv::Point2i>& sun_points, int mask_id) {

  detected_points.clear();
  sun_points.clear();

  if (!validateMask_(image, mask_id)) {
    return false;
  }

  initOnFirstFrame_(image);
  clearMarks_(image);

  scanImageForCandidates_(image, mask_id, detected_points, sun_points);

  // rejectMarkersNearSun_(detected_points, sun_points);

  return true;
}
//}

/* scanImageForCandidates_ //{ */
void UvdarLedDetectFastGpu::scanImageForCandidates_(const cv::Mat image, const int mask_id,
                                                    std::vector<cv::Point2i>& detected_points,
                                                    std::vector<cv::Point2i>& sun_points) {
  if (!image_gpu_in_ || !marker_counter_gpu_ || !detected_markers_gpu_) {
    logger_.error("GPU Tensors not initialized! Skipping detection...");
    return;
  }

  updateGpuInputs_(image, mask_id);

  evaluateFastRingsGpu_();

  handleGpuResults_(image, detected_points, sun_points);
}
//}

/* updateGpuInputs_ //{ */
void UvdarLedDetectFastGpu::updateGpuInputs_(const cv::Mat& image, const int mask_id) {
  // reset counters
  marker_counter_gpu_->setData(std::vector<uint32_t>({0u, 0u}));

  // load camera frame
  greyToRgba_(image, image_pixels_in_);
  image_gpu_in_->setData(image_pixels_in_);

  // load mask
  if (cfg_.use_masks) {
    greyToRgba_(cfg_.masks[mask_id], image_mask_pixels_in_);
    image_gpu_mask_->setData(image_mask_pixels_in_);
  }
}
//}

/* evaluateFastRingsGpu_ //{ */
void UvdarLedDetectFastGpu::evaluateFastRingsGpu_() {
  // clang-format off
  gpu_mgr_.manager()
      .sequence()
      ->record<kp::OpSyncDevice>(
          {image_gpu_in_, 
           image_gpu_mask_, 
           marker_counter_gpu_, 
           detected_markers_gpu_, 
           detected_suns_gpu_})
      ->record<kp::OpAlgoDispatch>(eval_fast_ring_gpu_alg_)
      ->record<kp::OpSyncLocal>({detected_markers_gpu_, 
                                 marker_counter_gpu_, 
                                 detected_suns_gpu_})
      ->eval();
  // clang-format on
}
//}

/* handleGpuResults_ //{ */
void UvdarLedDetectFastGpu::handleGpuResults_(const cv::Mat& image, std::vector<cv::Point2i>& detected_points,
                                              std::vector<cv::Point2i>& sun_points) {
  const auto& counters        = marker_counter_gpu_->vector();
  const uint32_t marker_count = std::min(counters[0], MAX_MARKERS_);
  const uint32_t sun_count    = std::min(counters[1], MAX_MARKERS_);

  localizeMarkers_(image, marker_count, detected_markers_gpu_->vector(), detected_points);
  localizeSuns_(sun_count, detected_suns_gpu_->vector(), sun_points);
}
//}

/* localizeMarkers_ //{ */
void UvdarLedDetectFastGpu::localizeMarkers_(const cv::Mat& image, const uint32_t marker_count,
                                             const std::vector<uint32_t>& raw_detected_markers,
                                             std::vector<cv::Point2i>& detected_points) {
  detected_points.reserve(marker_count);

  for (uint32_t i = 0; i < marker_count; ++i) {
    const int x = static_cast<int>(raw_detected_markers[2 * i]);
    const int y = static_cast<int>(raw_detected_markers[2 * i + 1]);
    localizeMarkerPoint_(image, cv::Point2i(x, y), detected_points);
  }
}
//}

/* localizeMarkerPoint_ //{ */
void UvdarLedDetectFastGpu::localizeMarkerPoint_(const cv::Mat& image, const cv::Point point,
                                                 std::vector<cv::Point2i>& detected_points) {
  if (image.empty() || image.data == nullptr)
    return;
  if (fast_interior_set_.empty())
    return;

  cv::Point best_point   = point;
  unsigned char best_val = 0;

  const auto& interior = fast_interior_set_.back();

  for (const auto& dp : interior) {
    const int x = point.x + dp.x;
    const int y = point.y + dp.y;

    if (x < 0 || y < 0 || x >= image.cols || y >= image.rows) {
      continue;
    }

    const int idx = index2d(x, y, image.cols);

    if (isAlreadyAssignedToCluster_(idx)) {
      continue;
    }

    if (image.data[idx] > best_val) {
      best_val   = image.data[idx];
      best_point = cv::Point(x, y);
    }

    addToCluster_(idx);
  }

  detected_points.emplace_back(best_point);
}
//}

/* localizeSuns_ //{ */
void UvdarLedDetectFastGpu::localizeSuns_(const uint32_t sun_count, const std::vector<uint32_t>& raw_sun_markers,
                                          std::vector<cv::Point2i>& sun_points) {
  sun_points.reserve(sun_count);
  for (uint32_t i = 0; i < sun_count; ++i) {
    const int x = static_cast<int>(raw_sun_markers[2 * i]);
    const int y = static_cast<int>(raw_sun_markers[2 * i + 1]);
  }
}
//}

/* rejectMarkersNearSun_ //{ */
void UvdarLedDetectFastGpu::rejectMarkersNearSun_(std::vector<cv::Point2i>& detected_points,
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

/* initGpuComputing_ //{ */
void UvdarLedDetectFastGpu::initGpuComputing_() {
  auto& gpu = gpu_mgr_.manager();

  // clang-format off
  std::vector<int> pushConfigConst = {cfg_.threshold, 
                                      cfg_.threshold_diff, 
                                      cfg_.threshold_sun, 
                                      cfg_.fast_ring_size};
  // clang-format on
  std::vector<double> image_size{960, 600};

  cv::Mat img8uc1 = cv::Mat::zeros(cv::Size(image_size[0], image_size[1]), CV_8UC1);
  greyToRgba_(img8uc1, image_pixels_in_);
  image_gpu_in_ = gpu.imageT<uint8_t>(image_pixels_in_, image_size[0], image_size[1], 4);

  cv::Mat dummy_mask(cv::Size(image_size[0], image_size[1]), CV_8UC1, cv::Scalar(255));
  greyToRgba_(dummy_mask, image_mask_pixels_in_);
  image_gpu_mask_ = gpu.imageT<uint8_t>(image_mask_pixels_in_, image_size[0], image_size[1], 4);

  detected_markers_gpu_ = gpu.tensorT<uint32_t>(std::vector<uint32_t>(2 * MAX_MARKERS_, 0u));
  detected_suns_gpu_    = gpu.tensorT<uint32_t>(std::vector<uint32_t>(2 * MAX_MARKERS_, 0u));

  marker_counter_gpu_ = gpu.tensorT<uint32_t>(std::vector<uint32_t>(2, 0u));

  // clang-format off
  params_ = {image_gpu_in_, 
             image_gpu_mask_, 
             detected_markers_gpu_, 
             marker_counter_gpu_, 
             detected_suns_gpu_};
  // clang-format on

  // TODO: the most ugly part
  auto ceil_div = [](uint32_t a, uint32_t b) { return (a + b - 1) / b; };

  eval_fast_ring_gpu_alg_ = gpu.algorithm(params_, loadPrecompiledShader_("uvdar_core", "eval_fast_ring.spv"),
                                          kp::Workgroup({ceil_div(960, KERNEL_SIZE_), ceil_div(600, KERNEL_SIZE_), 1}),
                                          std::vector<float>{}, // optional
                                          pushConfigConst);
}
//}

/* initFastInteriorSet_ //{ */
void UvdarLedDetectFastGpu::initFastInteriorSet_() {
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

/* initDelayed //{ */
bool UvdarLedDetectFastGpu::initDelayed(const cv::Mat image) {
  return true;
}
//}

/* isAlreadyAssignedToCluster_ //{ */
inline bool UvdarLedDetectFastGpu::isAlreadyAssignedToCluster_(const int point_idx) const noexcept {
  return image_check_.data[point_idx] != 0;
}
//}

/* addToCluster_ //{ */
inline void UvdarLedDetectFastGpu::addToCluster_(int idx) {
  image_check_.data[idx] = 255;
}
//}

void UvdarLedDetectFastGpu::initOnFirstFrame_(const cv::Mat& image_curr) {
  if (first_) {
    first_       = false;
    roi_         = cv::Rect(cv::Point(0, 0), image_curr.size());
    image_check_ = cv::Mat(image_curr.size(), CV_8UC1);
    image_check_ = cv::Scalar(0);
  }
}
//}

/* clearMarks_ //{ */
void UvdarLedDetectFastGpu::clearMarks_(const cv::Mat& image_curr) {
  for (int j = 0; j < image_curr.rows; j++) {
    for (int i = 0; i < image_curr.cols; i++) {
      if (image_check_.at<unsigned char>(j, i) == 255) {
        image_check_.at<unsigned char>(j, i) = 0;
      }
    }
  }
}
//}

/* greyToRgba_ //{ */
void UvdarLedDetectFastGpu::greyToRgba_(const cv::Mat& gray, std::vector<uint8_t>& out) {
  CV_Assert(gray.type() == CV_8UC1);
  out.resize(gray.total() * 4);

  cv::Mat rgba(gray.rows, gray.cols, CV_8UC4, out.data());
  cv::cvtColor(gray, rgba, cv::COLOR_GRAY2RGBA);
}
//}

/* validateMask_ //{ */
bool UvdarLedDetectFastGpu::validateMask_(const cv::Mat& image_curr, const int mask_id) const noexcept {
  if (mask_id >= 0) {
    if (mask_id >= static_cast<int>(cfg_.masks.size())) {
      logger_.error("[UVDARDetectorFastGpu]: Mask index " + std::to_string(mask_id) +
                    " is greater than the current number of loaded masks!");
      return false;
    }
    if (image_curr.size() != cfg_.masks[mask_id].size()) {
      logger_.error("[UVDARDetectorFastGpu]: The size of the selected mask does not match the current image!");
      return false;
    }
  }
  return true;
}
//}

/* logGpuProperties_ //{ */
void UvdarLedDetectFastGpu::logGpuProperties_() {
  // logger_.info("[UVDARDetectorFastCpu]: Loaded shader, " + std::to_string(eval_fast_ring_shader_.size()) + " bytes");

  const auto& props = gpu_mgr_.manager().getDeviceProperties();
  std::string gpu_name(props.deviceName.data());

  auto deviceTypeToStr = [](vk::PhysicalDeviceType t) {
    switch (t) {
      case vk::PhysicalDeviceType::eDiscreteGpu:
        return "DISCRETE";
      case vk::PhysicalDeviceType::eIntegratedGpu:
        return "INTEGRATED";
      case vk::PhysicalDeviceType::eVirtualGpu:
        return "VIRTUAL";
      case vk::PhysicalDeviceType::eCpu:
        return "CPU";
      default:
        return "OTHER";
    }
  };

  logger_.info("[UVDARDetectorFastGpu]: Using GPU: " + gpu_name + " Type: " + deviceTypeToStr(props.deviceType));
}
//}

/* loadPrecompiledShader_ //{ */
std::vector<uint32_t> UvdarLedDetectFastGpu::loadPrecompiledShader_(const std::string& pkg, const std::string& rel) {
  auto path = std::filesystem::path(ament_index_cpp::get_package_share_directory(pkg)) / "gpu_shaders" / rel;

  std::ifstream shader_file(path, std::ios::binary);
  if (!shader_file) {
    throw std::runtime_error("Failed to open SPIR-V: " + path.string());
  }

  std::vector<char> buffer;
  buffer.insert(buffer.begin(), std::istreambuf_iterator<char>(shader_file), {});
  return {(uint32_t*)buffer.data(), (uint32_t*)(buffer.data() + buffer.size())};
}
//}

} // namespace uvdar
