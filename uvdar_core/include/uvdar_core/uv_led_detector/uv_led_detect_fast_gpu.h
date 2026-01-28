#pragma once

#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <chrono>

#include <kompute/Kompute.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <uvdar_core/uv_led_detector/uv_led_detect_fast_base.h>
#include <uvdar_core/uv_led_detector/gpu_context.h>

namespace uvdar {

struct Cluster {
  int x_sum;
  int y_sum;
  int count;
  int avg_x;
  int avg_y;
};

struct FastGpuResources {
  std::shared_ptr<kp::ImageT<uint8_t>> image_in;
  std::shared_ptr<kp::ImageT<uint8_t>> image_mask;

  std::shared_ptr<kp::TensorT<uint32_t>> detected_markers;
  std::shared_ptr<kp::TensorT<uint32_t>> detected_suns;
  std::shared_ptr<kp::TensorT<uint32_t>> marker_counter;

  std::shared_ptr<kp::Algorithm> eval_fast_ring_alg;

  std::vector<std::shared_ptr<kp::Memory>> params;

  bool valid() const {
    return image_in && image_mask && detected_markers && detected_suns && marker_counter && eval_fast_ring_alg;
  }

  void rebuild_params() {
    params.clear();
    params.reserve(5);

    // do not change the order
    params.push_back(image_in);
    params.push_back(image_mask);
    params.push_back(detected_markers);
    params.push_back(marker_counter);
    params.push_back(detected_suns);
  }
};

class UvdarLedDetectFastGpu : public UvLedDetectFastBase {
 public:
  explicit UvdarLedDetectFastGpu(UvLedDetectConfig cfg, ILogger& logger);
  bool processImage(const cv::Mat image, std::vector<cv::Point2i>& detected_points,
                    std::vector<cv::Point2i>& sun_points, int mask_id = -1) override;
  void initGpuProgram(const cv::Mat image) override;

 private:
  [[nodiscard]] bool validateMask_(const cv::Mat& image_curr, const int mask_id) const noexcept;

  void logGpuProperties_();
  void initGpuComputing_(const int width, const int height);

  void scanImageForCandidates_(const cv::Mat image, const int mask_id, std::vector<cv::Point2i>& detected_points,
                               std::vector<cv::Point2i>& sun_points);
  void rejectMarkersNearSun_(std::vector<cv::Point2i>& detected_points, const std::vector<cv::Point2i>& sun_points);

  std::vector<uint32_t> loadPrecompiledShader_(const std::string& pkg, const std::string& rel);

  void greyToRgba_(const cv::Mat& gray, std::vector<uint8_t>& out);

  void updateGpuInputs_(const cv::Mat& image, const int mask_id);
  void evaluateFastRingsGpu_();
  void handleGpuResults_(std::vector<cv::Point2i>& detected_points, std::vector<cv::Point2i>& sun_points);

  void localizeMarkers_(const uint32_t marker_count, const std::vector<uint32_t>& raw_detected_markers,
                        std::vector<cv::Point2i>& detected_points);

  void localizeSuns_(const uint32_t sun_count, const std::vector<uint32_t>& raw_sun_markers,
                     std::vector<cv::Point2i>& sun_points);

 private:
  static constexpr uint32_t MAX_MARKERS_{100};
  static constexpr uint32_t MAX_SUNS_{100};
  static constexpr uint32_t KERNEL_SIZE_{16};

  GpuContext& gpu_mgr_;

  FastGpuResources gpu_resources_;

  std::vector<uint8_t> image_pixels_in_;
  std::vector<uint8_t> image_mask_pixels_in_;

  std::array<Cluster, MAX_MARKERS_> clusters_;
};

} // namespace uvdar
