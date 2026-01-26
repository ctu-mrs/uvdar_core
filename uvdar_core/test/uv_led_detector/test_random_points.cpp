#include <gtest/gtest.h>
#include <random>
#include "../dummy_logger.h"

#include <uvdar_core/uv_led_detector/uv_led_detector.h>

/* createRandomMarkersImage //{ */
cv::Mat createRandomMarkersImage(int num_markers, uint8_t brightness, std::vector<cv::Point2i>& ground_truth) {
  cv::Mat image(H, W, CV_8UC1, cv::Scalar(0));
  ground_truth.clear();

  std::random_device rd;
  std::mt19937 gen(rd());
  // Keep markers away from the 4px ROI border
  std::uniform_int_distribution<> disX(10, W - 10);
  std::uniform_int_distribution<> disY(10, H - 10);

  while (ground_truth.size() < num_markers) {
    int cx = disX(gen);
    int cy = disY(gen);

    // Simple overlap check: ensure new marker is at least 15px from others
    bool overlap = false;
    for (const auto& pt : ground_truth) {
      if (cv::norm(pt - cv::Point2i(cx, cy)) < 15.0) {
        overlap = true;
        break;
      }
    }

    if (!overlap) {
      // Draw a 3x3 blob
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          image.at<uint8_t>(cy + dy, cx + dx) = brightness;
        }
      }
      ground_truth.push_back(cv::Point2i(cx, cy));
    }
  }
  return image;
}
//}

TEST(UvLedDetector, GPU_StressTest) {
  std::vector<cv::Point2i> ground_truth;
  std::vector<cv::Point2i> detected_points;
  std::vector<cv::Point2i> sun_points;

  int num_to_test   = 50; // Test with 50 random LEDs
  auto stress_image = createRandomMarkersImage(num_to_test, 255, ground_truth);

  uv_detector.processImage(stress_image, detected_points, sun_points);

  TEST_COUT << "Generated: " << num_to_test << " | Detected: " << detected_points.size() << "\n";

  // Since you don't have NMS yet, you might get > 50 points if blobs trigger twice.
  // This will reveal if your FAST ring is "double-triggering" on the blob edges.
  EXPECT_GE(detected_points.size(), num_to_test);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  // initialize the random number generator
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}