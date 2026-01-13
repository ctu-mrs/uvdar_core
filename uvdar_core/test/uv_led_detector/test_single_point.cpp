#include <gtest/gtest.h>
#include "../dummy_logger.h"

#include <uvdar_core/uv_led_detector/uv_led_detector.h>

static const int W = 64;
static const int H = 48;

/* createSinglePointImage //{ */
cv::Mat createSinglePointImage() {

  cv::Mat image(H, W, CV_8UC1, cv::Scalar(0));
  const int cx = W / 2;
  const int cy = H / 2;
  // image.at<uint8_t>(cy, cx) = 210;

  // Single bright marker (tiny blob)
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      image.at<uint8_t>(cy + dy, cx + dx) = 255;
    }
  }
  return image;
}
//}

/* TEST(UvLedDetector, CPU_singlePoint) //{ */
TEST(UvLedDetector, CPU_singlePoint) {
  DummyLogger logger;
  uvdar::UvLedDetectConfig cfg;
  cfg.gui                 = false;
  cfg.use_masks           = false;
  cfg.threshold           = 200;
  cfg.threshold_diff      = 100;
  cfg.threshold_sun       = 150;
  cfg.threshold_sun_dist  = 25;
  cfg.threshold_sun_merge = 20;
  uvdar::UvdarLedDetectFastCPU uv_detector(cfg, logger);

  std::vector<cv::Point2i> detected_points;
  std::vector<cv::Point2i> sun_points;

  auto dummy_image  = createSinglePointImage();
  bool success_flag = uv_detector.processImage(dummy_image, detected_points, sun_points);

  EXPECT_TRUE(success_flag);
  EXPECT_EQ(detected_points.size(), 1);
  EXPECT_TRUE(sun_points.empty());
  EXPECT_NEAR(detected_points[0].x, W / 2, 1); // 1px tolerance
  EXPECT_NEAR(detected_points[0].y, H / 2, 1); // 1px tolerance
}
//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  // initialize the random number generator
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}