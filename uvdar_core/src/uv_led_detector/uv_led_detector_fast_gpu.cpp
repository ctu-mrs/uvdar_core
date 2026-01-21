#include <uvdar_core/uv_led_detector/uv_led_detect_fast_gpu.h>

namespace uvdar {

/* ParseShader //{ */
static std::string ParseShader(const std::string& filepath) {
  std::ifstream stream(filepath);
  if (!stream.is_open()) {
    throw std::runtime_error("Failed to open shader file: " + filepath);
  }

  std::string line;
  std::stringstream ss;
  while (getline(stream, line)) {
    ss << line << "\n"; // TODO: maybe "\n" is not needed
  }

  return ss.str();
}
//}

/* ShaderPathFromShare //{ */
static std::string ShaderPathFromShare(const std::string& package_name, const std::string& relative_under_share) {
  const auto share_dir       = ament_index_cpp::get_package_share_directory(package_name);
  std::filesystem::path path = std::filesystem::path(share_dir) / relative_under_share;

  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("Shader file does not exist: " + path.string());
  }

  return path.string();
}
//}

/* LoadEvalFastRingShader //{ */
static std::string LoadEvalFastRingShader() {
  auto filepath = ShaderPathFromShare("uvdar_core", "uv_led_detector/gpu_shaders/eval_fast_ring.shader");
  return ParseShader(filepath);
}
//}

/* UvdarLedDetectFastCpu constructor //{ */
UvdarLedDetectFastGpu::UvdarLedDetectFastGpu(UvLedDetectConfig cfg, ILogger& logger, kp::Manager& gpu_manager)
    : UvLedDetectFastBase(std::move(cfg), logger), gpu_mgr_(gpu_manager),
      eval_fast_ring_shader_(LoadEvalFastRingShader()) {
  logger.info("[UVDARDetectorFastCpu]: Loaded shader, " + std::to_string(eval_fast_ring_shader_.size()) + " bytes");

  //   const auto& props = mgr_->getDeviceProperties();

  //   logger.info("[UVDARDetectorFastCpu]: Using GPU: " + props.deviceName + " Type: " + props.deviceType);
}
//}

/* processImage //{ */
bool UvdarLedDetectFastGpu::processImage(const cv::Mat image, std::vector<cv::Point2i>& detected_points,
                                         std::vector<cv::Point2i>& sun_points, int mask_id) {
  //   logger_.info("som tu");
  return true;
}
//}

/* initDelayed //{ */
bool UvdarLedDetectFastGpu::initDelayed(const cv::Mat image) {
  return true;
}
//}

} // namespace uvdar
