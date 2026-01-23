#include <uvdar_core/uv_led_detector/uv_led_detect_fast_gpu.h>

namespace uvdar {

inline int index2d(int x, int y, int cols) noexcept {
  return cols * y + x;
}

static std::vector<uint8_t> mat8uc1_to_vector(const cv::Mat& m) {
  if (m.empty())
    throw std::runtime_error("Input mat is empty");
  if (m.type() != CV_8UC1)
    throw std::runtime_error("Expected CV_8UC1");
  if (m.channels() != 1)
    throw std::runtime_error("Expected single channel");

  cv::Mat contiguous = m.isContinuous() ? m : m.clone();

  const size_t n = contiguous.total(); // rows*cols for 8UC1
  std::vector<uint8_t> out(n);
  std::memcpy(out.data(), contiguous.data, n * sizeof(uint8_t));
  return out;
}

/* UvdarLedDetectFastCpu constructor //{ */
UvdarLedDetectFastGpu::UvdarLedDetectFastGpu(UvLedDetectConfig cfg, ILogger& logger)
    : UvLedDetectFastBase(std::move(cfg), logger), gpu_mgr_(GpuContext::GetInstance()) {
  logGpuProperties_();
  initFastInteriorSet_();
  auto& gpu = gpu_mgr_.manager();

  // clang-format off
  std::vector<int> pushConfigConst = {
    static_cast<int>(cfg.threshold), 
    static_cast<int>(cfg.threshold_diff)
  };

  cv::Mat img8uc1 = cv::Mat::zeros(cv::Size(960, 600), CV_8UC1);
  auto hostPixels = mat8uc1_to_vector(img8uc1);
  image_gpu_in_  =     gpu.imageT<uint8_t>(hostPixels, 960, 600, 1);
  image_gpu_mask_  =     gpu.imageT<uint8_t>(hostPixels, 960, 600,1);

  constexpr uint32_t MAX_MARKERS              = 100;
  std::vector<uint32_t> markers(2 * MAX_MARKERS, 0u);
  detected_markers_gpu_ = gpu.tensorT<uint32_t>(markers);

  std::vector<uint32_t> hostCtr = {0u};
  marker_counter_gpu_ = gpu.tensorT<uint32_t>(hostCtr);

  params_ = {image_gpu_in_, image_gpu_mask_, detected_markers_gpu_, marker_counter_gpu_};

  auto ceil_div = [](uint32_t a, uint32_t b){ return (a + b - 1) / b; };

  eval_fast_ring_gpu_alg_ = gpu.algorithm(params_, 
                                          loadPrecompiledShader_(
                                            "uvdar_core", 
                                            "eval_fast_ring.spv"),
                                          kp::Workgroup({ ceil_div(960,16), ceil_div(600,16), 1 }), // this was 16x16
                                          std::vector<float>{},     // optional
                                          pushConfigConst);
  // clang-format on
}
//}

/* processImage //{ */
bool UvdarLedDetectFastGpu::processImage(const cv::Mat image, std::vector<cv::Point2i>& detected_points,
                                         std::vector<cv::Point2i>& sun_points, int mask_id) {

  logger_.info("Size: " + std::to_string(image.cols) + ", " + std::to_string(image.rows));
  initOnFirstFrame_();

  sun_points.clear();
  image_curr_ = image;

  marker_counter_gpu_->setData(std::vector<uint32_t>{0u});

  const size_t nbytes = image.total() * image.elemSize(); // elemSize() = 1 for CV_8UC1
  image_pixels_in_.resize(nbytes);
  std::memcpy(image_pixels_in_.data(), image.data, nbytes);

  image_gpu_in_->setData(image_pixels_in_);

  gpu_mgr_.manager()
      .sequence()
      ->record<kp::OpSyncDevice>({image_gpu_in_, image_gpu_mask_, marker_counter_gpu_})
      ->record<kp::OpAlgoDispatch>(eval_fast_ring_gpu_alg_)
      ->record<kp::OpSyncLocal>({detected_markers_gpu_, marker_counter_gpu_})
      ->eval();

  const uint32_t count_raw = marker_counter_gpu_->vector()[0];
  uint32_t count           = std::min(count_raw, 100u);

  const std::vector<uint32_t> raw = detected_markers_gpu_->vector();

  localizeMarkers(raw, count_raw, detected_points);

  return true;
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

void UvdarLedDetectFastGpu::initOnFirstFrame_() {
  if (first_) {
    first_       = false;
    roi_         = cv::Rect(cv::Point(0, 0), image_curr_.size());
    image_check_ = cv::Mat(image_curr_.size(), CV_8UC1);
    image_check_ = cv::Scalar(0);
  }
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

/* scanImageForCandidates_ //{ */
void UvdarLedDetectFastGpu::scanImageForCandidates_(const int mask_id, std::vector<cv::Point2i>& detected_points,
                                                    std::vector<cv::Point2i>& sun_points) {
  std::vector<SunCluster> sun_clusters;

  // for (int j = 0; j < image_curr_.rows; j++) {
  //   for (int i = 0; i < image_curr_.cols; i++) {

  //     const int point_idx = index2d(i, j, image_curr_.cols);

  //     if (isMaskedOut_(point_idx, mask_id)) {
  //       continue;
  //     }
  //     if (isAlreadyAssignedToCluster_(point_idx)) {
  //       continue;
  //     }
  //     if (isBelowBrightnessThreshold_(point_idx)) {
  //       continue;
  //     }

  //     auto fast_result = evaluateFastRings_(i, j);

  //     if (fast_result.marker_candidate) {
  //       localizeMarkerPoint_(fast_result, detected_points);
  //     } else if (fast_result.sun_candidate) {
  //       localizeSunPoint_(fast_result, sun_points, sun_clusters);
  //     }
  //   }
  // }
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

/* getVectorFromImage_ //{ */
std::vector<uint8_t> UvdarLedDetectFastGpu::getVectorFromImage_(const cv::Mat image) {
}
//}

void UvdarLedDetectFastGpu::localizeMarkers(const std::vector<uint32_t> raw_points, const uint32_t raw_points_count,
                                            std::vector<cv::Point2i>& detected_points) {
  detected_points.clear();

  const uint32_t maxCount = static_cast<uint32_t>(raw_points.size() / 2);
  const uint32_t count    = std::min(raw_points_count, maxCount);

  for (uint32_t i = 0; i < count; ++i) {
    const int x = static_cast<int>(raw_points[2 * i + 0]);
    const int y = static_cast<int>(raw_points[2 * i + 1]);
    localizeMarkerPoint_(cv::Point(x, y), detected_points);
  }
}

void UvdarLedDetectFastGpu::localizeMarkerPoint_(const cv::Point point, std::vector<cv::Point2i>& detected_points) {
  if (image_curr_.empty() || image_curr_.data == nullptr)
    return;
  if (fast_interior_set_.empty())
    return;

  cv::Point best_point   = point;
  unsigned char best_val = 0;

  const auto& interior = fast_interior_set_.back();

  for (const auto& dp : interior) {
    const int x = point.x + dp.x;
    const int y = point.y + dp.y;

    if (x < 0 || y < 0 || x >= image_curr_.cols || y >= image_curr_.rows) {
      continue;
    }

    const int idx = index2d(x, y, image_curr_.cols);

    if (isAlreadyAssignedToCluster_(idx)) {
      continue;
    }

    // const unsigned char v = image_curr_.data[idx];
    // logger_.info("image data: " + std::to_string(image_curr_.data[idx]));

    if (image_curr_.data[idx] > best_val) {
      best_val   = image_curr_.data[idx];
      best_point = cv::Point(x, y);
    }

    addToCluster_(idx);
  }

  detected_points.push_back(best_point);
}

} // namespace uvdar
