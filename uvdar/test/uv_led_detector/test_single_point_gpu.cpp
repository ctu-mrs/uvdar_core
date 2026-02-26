#include <gtest/gtest.h>
#include "../dummy_logger.h"

#include <uvdar/uv_led_detector/uv_led_detector.h>

static const int W = 960;
static const int H = 600;

/* createSingleMarkerImage //{ */
cv::Mat createSingleMarkerImage(const int cx, const int cy) {

  cv::Mat image(H, W, CV_8UC1, cv::Scalar(0));
  // Single bright marker (tiny blob)
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      image.at<uint8_t>(cy + dy, cx + dx) = 255;
    }
  }
  return image;
}
//}

/* TEST(UvLedDetector, GPU_singlePoint) //{ */
TEST(UvLedDetector, GPU_singlePoint) {
  TestLogger logger;
  uvdar::UvLedDetectConfig cfg;
  cfg.gui                 = true;
  cfg.gpu                 = true;
  cfg.use_masks           = false;
  cfg.threshold           = 50;
  cfg.threshold_diff      = 25;
  cfg.fast_ring_size      = 3;
  cfg.threshold_sun       = 150;
  cfg.threshold_sun_dist  = 25;
  cfg.threshold_sun_merge = 20;
  uvdar::UvdarLedDetectFastGpu uv_detector(cfg, logger);

  std::vector<cv::Point2i> detected_points;
  std::vector<cv::Point2i> sun_points;

  auto dummy_image = createSingleMarkerImage(W / 2, H / 2); // center

  uv_detector.initGpuProgram(dummy_image);
  bool success_flag = uv_detector.processImage(dummy_image, detected_points, sun_points);

  EXPECT_TRUE(success_flag);
  EXPECT_EQ(detected_points.size(), 1);
  EXPECT_TRUE(sun_points.empty());
  EXPECT_NEAR(detected_points[0].x, W / 2, 1); // 1px tolerance
  EXPECT_NEAR(detected_points[0].y, H / 2, 1); // 1px tolerance
}
//}
