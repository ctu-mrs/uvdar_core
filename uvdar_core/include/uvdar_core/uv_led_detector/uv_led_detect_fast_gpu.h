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
  void initOnFirstFrame_(const cv::Mat& image_curr);
  void clearMarks_(const cv::Mat& image_curr);
  [[nodiscard]] bool validateMask_(const cv::Mat& image_curr, const int mask_id) const noexcept;

  void logGpuProperties_();
  void initGpuComputing_();

  void scanImageForCandidates_(const cv::Mat image, const int mask_id, std::vector<cv::Point2i>& detected_points,
                               std::vector<cv::Point2i>& sun_points);
  void rejectMarkersNearSun_(std::vector<cv::Point2i>& detected_points, const std::vector<cv::Point2i>& sun_points);

  std::vector<uint32_t> loadPrecompiledShader_(const std::string& pkg, const std::string& rel);

  void initFastInteriorSet_();
  [[nodiscard]] inline bool isAlreadyAssignedToCluster_(const int point_idx) const noexcept;
  inline void addToCluster_(int idx);

  void greyToRgba_(const cv::Mat& gray, std::vector<uint8_t>& out);

  void updateGpuInputs_(const cv::Mat& image, const int mask_id);
  void evaluateFastRingsGpu_();
  void handleGpuResults_(const cv::Mat& image, std::vector<cv::Point2i>& detected_points,
                         std::vector<cv::Point2i>& sun_points);

  void localizeMarkers_(const cv::Mat& image, const uint32_t marker_count,
                        const std::vector<uint32_t>& raw_detected_markers, std::vector<cv::Point2i>& detected_points);
  void localizeMarkerPoint_(const cv::Mat& image, const cv::Point point, std::vector<cv::Point2i>& detected_points);

  void localizeSuns_(const uint32_t sun_count, const std::vector<uint32_t>& raw_sun_markers,
                     std::vector<cv::Point2i>& sun_points);

 private:
  static constexpr uint32_t MAX_MARKERS_{100};
  static constexpr uint32_t KERNEL_SIZE_{16};
  // const std::string eval_fast_ring_shader_;
  GpuContext& gpu_mgr_;

  std::vector<std::vector<cv::Point>> fast_interior_set_;
  cv::Mat image_check_;
  cv::Mat image_view_;
  cv::Rect roi_;
  bool first_{true};

  std::shared_ptr<kp::ImageT<uint8_t>> image_gpu_in_;
  std::shared_ptr<kp::ImageT<uint8_t>> image_gpu_out_;
  std::shared_ptr<kp::ImageT<uint8_t>> image_gpu_mask_;
  std::shared_ptr<kp::TensorT<uint32_t>> detected_markers_gpu_;
  std::shared_ptr<kp::TensorT<uint32_t>> detected_suns_gpu_;
  std::shared_ptr<kp::TensorT<uint32_t>> marker_counter_gpu_;
  std::vector<uint8_t> image_pixels_in_;
  std::vector<uint8_t> image_mask_pixels_in_;

  std::vector<std::shared_ptr<kp::Memory>> params_;
  std::shared_ptr<kp::Algorithm> eval_fast_ring_gpu_alg_;
};

} // namespace uvdar
