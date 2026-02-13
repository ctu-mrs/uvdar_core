#include <gtest/gtest.h>
#include <random>
#include "../dummy_logger.h"
#include "../timer.h"
#include <thread>

#include <uvdar/uv_led_detector/uv_led_detector.h>

static const int W           = 960;
static const int H           = 600;
static const int NUM_TESTS   = 1;
static const int NUM_MARKERS = 10;
// static const int W = 1920;
// static const int H = 1080;

/* compareGroundTruth //{ */
bool compareGroundTruth(std::vector<cv::Point2i> ground_truth, std::vector<cv::Point2i> detected_points,
                        int tol_px = 1) {
  if (ground_truth.size() != detected_points.size()) {
    TEST_COUT << "ground_truth.size() != detected_points.size()\n";
    return false;
  }

  std::vector<bool> used(detected_points.size(), false);

  for (const auto& gt : ground_truth) {
    bool matched = false;

    for (size_t i = 0; i < detected_points.size(); ++i) {
      if (used[i])
        continue;

      if (std::abs(detected_points[i].x - gt.x) <= tol_px && std::abs(detected_points[i].y - gt.y) <= tol_px) {
        used[i] = true;
        matched = true;
        break;
      }
    }

    if (!matched) {
      TEST_COUT << "No detected point matched ground-truth point (" << gt.x << ", " << gt.y << ")\n";
      return false;
    }
  }

  return true;
}
//}

/* createRandomMarkersImage //{ */
cv::Mat createRandomMarkersImage(int num_markers, uint8_t brightness, std::vector<cv::Point2i>& ground_truth) {
  cv::Mat image(H, W, CV_8UC1, cv::Scalar(0));
  ground_truth.clear();

  std::random_device rd;
  std::mt19937 gen(rd());
  // Keep markers away from the 4px ROI border
  std::uniform_int_distribution<> disX(10, W - 10);
  std::uniform_int_distribution<> disY(10, H - 10);

  while (ground_truth.size() < static_cast<size_t>(num_markers)) {
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

/* TEST(UvLedDetector, CPU_MultiplePoints_OneTrial) //{ */
TEST(UvLedDetector, CPU_MultiplePoints_OneTrial) {
  std::vector<cv::Point2i> ground_truth;
  std::vector<cv::Point2i> detected_points;
  std::vector<cv::Point2i> sun_points;

  auto dummy_image = createRandomMarkersImage(NUM_MARKERS, 255, ground_truth);

  TestLogger logger;
  uvdar::UvLedDetectConfig cfg;
  cfg.gui                 = true;
  cfg.gpu                 = false;
  cfg.use_masks           = false;
  cfg.threshold           = 50;
  cfg.threshold_diff      = 25;
  cfg.fast_ring_size      = 3;
  cfg.threshold_sun       = 150;
  cfg.threshold_sun_dist  = 25;
  cfg.threshold_sun_merge = 20;
  cfg.fast_ring_size      = 3; // this has no effect
  uvdar::UvdarLedDetectFastCpu uv_detector(cfg, logger);

  bool success_flag;
  { success_flag = uv_detector.processImage(dummy_image, detected_points, sun_points); }

  EXPECT_TRUE(success_flag);
  EXPECT_EQ(detected_points.size(), NUM_MARKERS);
  EXPECT_TRUE(compareGroundTruth(ground_truth, detected_points));
}
//}

/* TEST(UvLedDetector, CPU_MultiplePoints_3Cameras_10Tests) //{ */
TEST(UvLedDetector, CPU_MultiplePoints_3Cameras_10Tests) {
  constexpr int NUM_THREADS = 3;

  TestLogger logger;
  uvdar::UvLedDetectConfig cfg;
  cfg.gui                 = false;
  cfg.gpu                 = false;
  cfg.use_masks           = false;
  cfg.threshold           = 50;
  cfg.threshold_diff      = 25;
  cfg.fast_ring_size      = 3;
  cfg.threshold_sun       = 150;
  cfg.threshold_sun_dist  = 25;
  cfg.threshold_sun_merge = 20;
  cfg.fast_ring_size      = 3; // this has no effect

  // Initialize detectors for each thread
  std::vector<std::unique_ptr<uvdar::UvdarLedDetectFastCpu>> detectors;
  for (int i = 0; i < NUM_THREADS; ++i) {
    auto det = std::make_unique<uvdar::UvdarLedDetectFastCpu>(cfg, logger);

    // We provide an initial size for OpenCL/Cuda buffer allocation
    cv::Mat init_img = cv::Mat::zeros(cv::Size(W, H), CV_8UC1);
    detectors.push_back(std::move(det));
  }

  std::atomic<bool> all_ok{true};

  auto worker = [&](int tid) {
    for (int i = 0; i < NUM_TESTS; ++i) {
      std::vector<cv::Point2i> current_gt;
      std::vector<cv::Point2i> detected_points;
      std::vector<cv::Point2i> sun_points;

      cv::Mat trial_image = createRandomMarkersImage(NUM_MARKERS, 255, current_gt);
      bool ok             = detectors[tid]->processImage(trial_image, detected_points, sun_points);
      bool match = (detected_points.size() == (size_t)NUM_MARKERS) && compareGroundTruth(current_gt, detected_points);

      if (!ok || !match) {
        all_ok.store(false);
      }
    }
  };

  std::vector<std::thread> threads;
  threads.reserve(NUM_THREADS);

  for (int t = 0; t < NUM_THREADS; ++t) {
    threads.emplace_back(worker, t);
  }
  for (auto& th : threads) {
    th.join();
  }

  EXPECT_TRUE(all_ok.load()) << "One or more trials failed detection or ground truth comparison.";
}
//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}