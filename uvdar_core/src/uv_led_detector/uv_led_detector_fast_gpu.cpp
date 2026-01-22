#include <uvdar_core/uv_led_detector/uv_led_detect_fast_gpu.h>

namespace uvdar {

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

  auto& gpu = gpu_mgr_.manager();
  // clang-format off
  std::vector<float> pushConfigConst = {
    static_cast<float>(cfg.threshold), 
    static_cast<float>(cfg.threshold_diff), 
    static_cast<float>(cfg.threshold_sun), 
    static_cast<float>(cfg.threshold_sun_dist),
    static_cast<float>(cfg.threshold_sun_merge)
  };

  cv::Mat img8uc1 = cv::Mat::zeros(cv::Size(960, 600), CV_8UC1);
  auto hostPixels = mat8uc1_to_vector(img8uc1);
  image_gpu_in_  =     gpu.imageT<uint8_t>(hostPixels, 960, 600,1);
  image_gpu_out_ =     gpu.imageT<uint8_t>(hostPixels, 960, 600, 1);



  params_                 = {image_gpu_in_, image_gpu_out_};
  eval_fast_ring_gpu_alg_ = gpu.algorithm(params_, 
                                          loadPrecompiledShader_(
                                            "uvdar_core", 
                                            "eval_fast_ring.spv"),
                                          kp::Workgroup({960, 600, 1}), // this was 16x16
                                          std::vector<float>{},     // optional
                                          pushConfigConst);
  // clang-format on
}
//}

/* processImage //{ */
bool UvdarLedDetectFastGpu::processImage(const cv::Mat image, std::vector<cv::Point2i>& detected_points,
                                         std::vector<cv::Point2i>& sun_points, int mask_id) {

  logger_.info("Callback");
  detected_points.clear();
  sun_points.clear();
  // image_curr_ = image;
  auto& gpu = gpu_mgr_.manager();

  // TODO: try to copy data directly, since image will outlive this part

  const size_t nbytes = image.total() * image.elemSize(); // elemSize() = 1 for CV_8UC1
  image_pixels_in_.resize(nbytes);
  std::memcpy(image_pixels_in_.data(), image.data, nbytes);

  image_gpu_in_->setData(image_pixels_in_);

  gpu_mgr_.manager()
      .sequence()
      ->record<kp::OpSyncDevice>({image_gpu_in_})           // upload input
      ->record<kp::OpAlgoDispatch>(eval_fast_ring_gpu_alg_) // run compute
      ->record<kp::OpSyncLocal>({image_gpu_out_})           // download output
      ->eval();

  const auto& outVec = image_gpu_out_->vector(); // adjust if your API name differs
  cv::Mat outMat(image.rows, image.cols, CV_8UC1, const_cast<uint8_t*>(outVec.data()));
  cv::imwrite("/tmp/uvdar_gpu_out.png", outMat.clone());

  // prints "Output {  0  4  12  }"
  logger_.info("Output: {  ");

  return true;
}
//}

/* initDelayed //{ */
bool UvdarLedDetectFastGpu::initDelayed(const cv::Mat image) {
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

} // namespace uvdar
