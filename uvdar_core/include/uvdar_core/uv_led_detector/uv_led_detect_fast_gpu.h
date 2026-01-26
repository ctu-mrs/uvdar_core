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

class UvdarLedDetectFastGpu : public UvLedDetectFastBase {
 public:
  explicit UvdarLedDetectFastGpu(UvLedDetectConfig cfg, ILogger& logger);
  bool processImage(const cv::Mat image, std::vector<cv::Point2i>& detected_points,
                    std::vector<cv::Point2i>& sun_points, int mask_id = -1) override;
  bool initDelayed(const cv::Mat image) override;

 private:
  void initOnFirstFrame_();

  void logGpuProperties_();
  void initGpuComputing_();

  void scanImageForCandidates_(const int mask_id, std::vector<cv::Point2i>& detected_points,
                               std::vector<cv::Point2i>& sun_points);
  void rejectMarkersNearSun_(std::vector<cv::Point2i>& detected_points, const std::vector<cv::Point2i>& sun_points);

  std::vector<uint32_t> loadPrecompiledShader_(const std::string& pkg, const std::string& rel);

  std::vector<uint8_t> getVectorFromImage_(const cv::Mat image);

  void localizeMarkers(const std::vector<uint32_t> raw_points, const uint32_t raw_points_count,
                       std::vector<cv::Point2i>& detected_points);
  void localizeMarkerPoint_(const cv::Point point, std::vector<cv::Point2i>& detected_points);

  void initFastInteriorSet_();
  [[nodiscard]] inline bool isAlreadyAssignedToCluster_(const int point_idx) const noexcept;
  inline void addToCluster_(int idx);

  void greyToRgba_(const cv::Mat& gray, std::vector<uint8_t>& out);

 private:
  static constexpr uint32_t MAX_MARKERS_{100};

  // const std::string eval_fast_ring_shader_;
  GpuContext& gpu_mgr_;

  std::vector<std::vector<cv::Point>> fast_interior_set_;
  cv::Mat image_curr_;
  cv::Mat image_check_;
  cv::Mat image_view_;
  cv::Rect roi_;
  bool first_{true};

  std::shared_ptr<kp::ImageT<uint8_t>> image_gpu_in_;
  std::shared_ptr<kp::ImageT<uint8_t>> image_gpu_out_;
  std::shared_ptr<kp::ImageT<uint8_t>> image_gpu_mask_;
  std::shared_ptr<kp::TensorT<uint32_t>> detected_markers_gpu_;
  std::shared_ptr<kp::TensorT<uint32_t>> marker_counter_gpu_;
  std::vector<uint8_t> image_pixels_in_;

  std::vector<std::shared_ptr<kp::Memory>> params_;
  std::shared_ptr<kp::Algorithm> eval_fast_ring_gpu_alg_;
};

} // namespace uvdar
