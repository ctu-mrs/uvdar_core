#include <uvdar_core/uv_led_detector/uv_led_detect_fast_gpu.h>

namespace uvdar {

/* UvdarLedDetectFastCpu constructor //{ */
UvdarLedDetectFastGpu::UvdarLedDetectFastGpu(UvLedDetectConfig cfg, ILogger& logger)
    : UvLedDetectFastBase(std::move(cfg), logger), gpu_mgr_(GpuContext::GetInstance()) {
  logGpuProperties_();
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

  scanImageForCandidates_(image, mask_id, detected_points, sun_points);

  rejectMarkersNearSun_(detected_points, sun_points);

  return true;
}
//}

/* scanImageForCandidates_ //{ */
void UvdarLedDetectFastGpu::scanImageForCandidates_(const cv::Mat image, const int mask_id,
                                                    std::vector<cv::Point2i>& detected_points,
                                                    std::vector<cv::Point2i>& sun_points) {
  if (!gpu_resources_.valid()) {
    logger_.error("GPU Tensors not initialized! Skipping detection...");
    return;
  }

  updateGpuInputs_(image, mask_id);

  evaluateFastRingsGpu_();

  handleGpuResults_(detected_points, sun_points);
}
//}

/* updateGpuInputs_ //{ */
void UvdarLedDetectFastGpu::updateGpuInputs_(const cv::Mat& image, const int mask_id) {
  // reset counters
  gpu_resources_.marker_counter->setData(std::vector<uint32_t>({0u, 0u}));

  // load camera frame
  greyToRgba_(image, image_pixels_in_);
  gpu_resources_.image_in->setData(image_pixels_in_);

  // load mask
  if (cfg_.use_masks) {
    greyToRgba_(cfg_.masks[mask_id], image_mask_pixels_in_);
    gpu_resources_.image_mask->setData(image_mask_pixels_in_);
  }
}
//}

/* evaluateFastRingsGpu_ //{ */
void UvdarLedDetectFastGpu::evaluateFastRingsGpu_() {
  std::lock_guard<std::mutex> lk(gpu_mgr_.mutex());

  // clang-format off
  gpu_mgr_.manager()
      .sequence()
      ->record<kp::OpSyncDevice>(
          {gpu_resources_.image_in, 
           gpu_resources_.image_mask, 
           gpu_resources_.marker_counter})
      ->record<kp::OpAlgoDispatch>(gpu_resources_.eval_fast_ring_alg)
      ->record<kp::OpSyncLocal>({gpu_resources_.detected_markers, 
                                 gpu_resources_.marker_counter, 
                                 gpu_resources_.detected_suns})
      ->eval();
  // clang-format on
}
//}

/* handleGpuResults_ //{ */
void UvdarLedDetectFastGpu::handleGpuResults_(std::vector<cv::Point2i>& detected_points,
                                              std::vector<cv::Point2i>& sun_points) {
  const auto& counters        = gpu_resources_.marker_counter->vector();
  const uint32_t marker_count = std::min(counters[0], MAX_MARKERS_);
  const uint32_t sun_count    = std::min(counters[1], MAX_SUNS_);

  localizeMarkers_(marker_count, gpu_resources_.detected_markers->vector(), detected_points);
  localizeSuns_(sun_count, gpu_resources_.detected_suns->vector(), sun_points);
}
//}

/* localizeMarkers_ //{ */
void UvdarLedDetectFastGpu::localizeMarkers_(const uint32_t marker_count,
                                             const std::vector<uint32_t>& raw_detected_markers,
                                             std::vector<cv::Point2i>& detected_points) {
  if (marker_count == 0) {
    return;
  }

  // TODO: this parameter can be bigger than the fast_ring (something to discuss)
  const uint32_t distance_threshold = cfg_.fast_ring_size * cfg_.fast_ring_size;
  uint32_t cluster_count            = 0;

  for (uint32_t i = 0; i < marker_count; ++i) {
    const int x = static_cast<int>(raw_detected_markers[2 * i]);
    const int y = static_cast<int>(raw_detected_markers[2 * i + 1]);

    int best_cluster   = -1;
    uint32_t best_dist = distance_threshold + 1;

    for (uint32_t cluster_id = 0; cluster_id < cluster_count; ++cluster_id) {
      const int dx        = clusters_[cluster_id].avg_x - x;
      const int dy        = clusters_[cluster_id].avg_y - y;
      const uint32_t dist = static_cast<uint32_t>(dx * dx + dy * dy);

      if (dist <= best_dist) {
        best_cluster = cluster_id;
        best_dist    = dist;
        if (dist == 0) {
          break; // found the exact center
        }
      }
    }

    if (best_cluster != -1 && best_dist <= distance_threshold) {
      auto& c = clusters_[best_cluster];
      c.x_sum += x;
      c.y_sum += y;
      c.count += 1;
      c.avg_x = c.x_sum / c.count;
      c.avg_y = c.y_sum / c.count;
    } else if (cluster_count < MAX_MARKERS_) {
      auto& c = clusters_[cluster_count];
      c.x_sum = x;
      c.y_sum = y;
      c.count = 1;
      c.avg_x = x;
      c.avg_y = y;
      cluster_count++;
    }
  }

  detected_points.reserve(cluster_count);
  for (uint32_t i = 0; i < cluster_count; i++) {
    detected_points.emplace_back(clusters_[i].x_sum / clusters_[i].count, clusters_[i].y_sum / clusters_[i].count);
  }
}
//}

/* localizeSuns_ //{ */
void UvdarLedDetectFastGpu::localizeSuns_(const uint32_t sun_count, const std::vector<uint32_t>& raw_sun_markers,
                                          std::vector<cv::Point2i>& sun_points) {
  sun_points.reserve(sun_count);
  for (uint32_t i = 0; i < sun_count; ++i) {
    const int x = static_cast<int>(raw_sun_markers[2 * i]);
    const int y = static_cast<int>(raw_sun_markers[2 * i + 1]);
    sun_points.emplace_back(cv::Point2i(x, y));
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
void UvdarLedDetectFastGpu::initGpuComputing_(const int width, const int height) {
  if (width == 0 || height == 0) {
    throw std::runtime_error("[UVDARDetectorFastGpu]: Input image has zero dimension, cannot initialize GPU program.");
  }
  std::lock_guard<std::mutex> lk(gpu_mgr_.mutex());

  auto& gpu = gpu_mgr_.manager();
  // clang-format off
  std::vector<uint32_t> pushConfigConst = {
    static_cast<uint32_t>(cfg_.threshold), 
    static_cast<uint32_t>(cfg_.threshold_diff), 
    static_cast<uint32_t>(cfg_.threshold_sun), 
    static_cast<uint32_t>(cfg_.fast_ring_size),
    MAX_MARKERS_,
    MAX_SUNS_
  };
  // clang-format on
  std::vector<int> image_size{width, height};

  cv::Mat img8uc1 = cv::Mat::zeros(cv::Size(image_size[0], image_size[1]), CV_8UC1);
  greyToRgba_(img8uc1, image_pixels_in_);
  gpu_resources_.image_in = gpu.imageT<uint8_t>(image_pixels_in_, image_size[0], image_size[1], 4);

  cv::Mat dummy_mask(cv::Size(image_size[0], image_size[1]), CV_8UC1, cv::Scalar(255));
  greyToRgba_(dummy_mask, image_mask_pixels_in_);
  gpu_resources_.image_mask = gpu.imageT<uint8_t>(image_mask_pixels_in_, image_size[0], image_size[1], 4);

  gpu_resources_.detected_markers = gpu.tensorT<uint32_t>(std::vector<uint32_t>(2 * MAX_MARKERS_, 0u));
  gpu_resources_.detected_suns    = gpu.tensorT<uint32_t>(std::vector<uint32_t>(2 * MAX_SUNS_, 0u));
  gpu_resources_.marker_counter   = gpu.tensorT<uint32_t>(std::vector<uint32_t>(2, 0u));

  gpu_resources_.rebuild_params();
  if (gpu_resources_.params.empty()) {
    throw std::runtime_error("[UVDARDetectorFastGpu]: GPU parameters are never filled.");
  }

  // TODO: the most ugly part
  auto ceil_div = [](uint32_t a, uint32_t b) { return (a + b - 1) / b; };

  gpu_resources_.eval_fast_ring_alg =
      gpu.algorithm(gpu_resources_.params, loadPrecompiledShader_("uvdar_core", "eval_fast_ring.spv"),
                    kp::Workgroup({ceil_div(width, KERNEL_SIZE_), ceil_div(height, KERNEL_SIZE_), 1}),
                    std::vector<float>{}, // optional
                    pushConfigConst);
}
//}

/* initGpuProgram //{ */
void UvdarLedDetectFastGpu::initGpuProgram(const cv::Mat image) {
  initGpuComputing_(image.cols, image.rows);
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
  std::lock_guard<std::mutex> lk(gpu_mgr_.mutex());

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
