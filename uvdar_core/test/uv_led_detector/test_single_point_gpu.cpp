#include <gtest/gtest.h>
#include "../dummy_logger.h"

#include <uvdar_core/uv_led_detector/uv_led_detector.h>

static const int W = 960;
static const int H = 600;

/* createSingleMarkerImage //{ */
cv::Mat createSingleMarkerImage(const int cx, const int cy) {

  cv::Mat image(H, W, CV_8UC1, cv::Scalar(0));
  image.at<uint8_t>(cy, cx) = 243;

  // // Single bright marker (tiny blob)
  // for (int dy = -1; dy <= 1; ++dy) {
  //   for (int dx = -1; dx <= 1; ++dx) {
  //     image.at<uint8_t>(cy + dy, cx + dx) = 255;
  //   }
  // }
  return image;
}
//}

/* TEST(UvLedDetector, GPU_loop) //{ */
TEST(UvLedDetector, GPU_loopMultiplePoints) {
  DummyLogger logger;
  uvdar::UvLedDetectConfig cfg;
  cfg.gui                 = true;
  cfg.use_masks           = false;
  cfg.threshold           = 200;
  cfg.threshold_diff      = 100;
  cfg.threshold_sun       = 150;
  cfg.threshold_sun_dist  = 25;
  cfg.threshold_sun_merge = 20;
  uvdar::UvdarLedDetectFastGpu uv_detector(cfg, logger);

  std::vector<cv::Point2i> detected_points;
  std::vector<cv::Point2i> sun_points;

  for (size_t i = 0; i < 5; ++i) {
    detected_points.clear();
    sun_points.clear();

    auto dummy_image                        = createSingleMarkerImage(W / 2 + i, H / 2 + i); // center
    dummy_image.at<uint8_t>(5 + i, 7 + i)   = 201;
    dummy_image.at<uint8_t>(21 + i, 11 + i) = 255;

    TEST_COUT << "[GPU]: original points:\n";
    TEST_COUT << H / 2 + i << ", " << W / 2 + i << "\n";
    TEST_COUT << 7 + i << ", " << 5 + i << "\n";
    TEST_COUT << 11 + i << ", " << 21 + i << "\n";

    bool success_flag = uv_detector.processImage(dummy_image, detected_points, sun_points);

    TEST_COUT << "[GPU]: detected points:\n";
    for (const auto& point : detected_points) {
      TEST_COUT << point.x << ", " << point.y << "\n";
    }
    TEST_COUT << std::endl;
  }

  // EXPECT_TRUE(success_flag);
  EXPECT_EQ(detected_points.size(), 3);
  EXPECT_TRUE(sun_points.empty());
  // EXPECT_NEAR(detected_points[0].x, W / 2, 1); // 1px tolerance
  // EXPECT_NEAR(detected_points[0].y, H / 2, 1); // 1px tolerance
}
//}

/* TEST(UvLedDetector, GPU_singlePoint) //{ */
TEST(UvLedDetector, GPU_singlePoint) {
  DummyLogger logger;
  uvdar::UvLedDetectConfig cfg;
  cfg.gui                 = true;
  cfg.use_masks           = false;
  cfg.threshold           = 200;
  cfg.threshold_diff      = 100;
  cfg.threshold_sun       = 150;
  cfg.threshold_sun_dist  = 25;
  cfg.threshold_sun_merge = 20;
  uvdar::UvdarLedDetectFastGpu uv_detector(cfg, logger);

  std::vector<cv::Point2i> detected_points;
  std::vector<cv::Point2i> sun_points;

  auto dummy_image = createSingleMarkerImage(W / 2, H / 2); // center

  bool success_flag = uv_detector.processImage(dummy_image, detected_points, sun_points);

  TEST_COUT << "[GPU]: original points:\n";
  TEST_COUT << H / 2 << ", " << W / 2 << " v:" << std::to_string(dummy_image.at<uint8_t>(H / 2, W / 2)) << "\n";

  TEST_COUT << "[GPU]: detected points:\n";
  for (const auto& point : detected_points) {
    TEST_COUT << point.y << ", " << point.x << "\n";
  }
  TEST_COUT << std::endl;

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