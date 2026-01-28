#include <gtest/gtest.h>
#include <random>
#include "../dummy_logger.h"
#include "../timer.h"
#include <thread>

#include <uvdar_core/uv_led_detector/uv_led_detector.h>

static const int W = 960;
static const int H = 600;
// static const int W = 1920;
// static const int H = 1080;

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

TEST(UvLedDetector, GPU_CompTimeTest) {
  int NUM_TESTS   = 100;
  int NUM_MARKERS = 10;

  std::vector<cv::Point2i> ground_truth;
  std::vector<cv::Point2i> detected_points;
  std::vector<cv::Point2i> sun_points;

  auto dummy_image = createRandomMarkersImage(NUM_MARKERS, 255, ground_truth);

  DummyLogger logger;
  uvdar::UvLedDetectConfig cfg;
  cfg.gui                 = true;
  cfg.use_masks           = false;
  cfg.threshold           = 50;
  cfg.threshold_diff      = 25;
  cfg.fast_ring_size      = 3;
  cfg.threshold_sun       = 150;
  cfg.threshold_sun_dist  = 25;
  cfg.threshold_sun_merge = 20;
  uvdar::UvdarLedDetectFastGpu uv_detector(cfg, logger);
  uv_detector.initGpuProgram(dummy_image);

  int64_t total_time_ms{0};
  bool success_flag;
  for (int i = 0; i < NUM_TESTS; ++i) {
    uvdar::ScopeTimer timer(logger);
    success_flag = uv_detector.processImage(dummy_image, detected_points, sun_points);
    total_time_ms += timer.stop();
  }

  TEST_COUT << "[UvledDetectorGpu]: Average time: " << std::to_string(total_time_ms / NUM_TESTS) << " ms over "
            << std::to_string(NUM_TESTS) << " tests" << std::endl;

  EXPECT_TRUE(success_flag);
  EXPECT_EQ(detected_points.size(), NUM_MARKERS);
  EXPECT_TRUE(compareGroundTruth(ground_truth, detected_points));
}

TEST(UvLedDetector, CPU_CompTimeTest) {
  int NUM_TESTS   = 100;
  int NUM_MARKERS = 10;

  std::vector<cv::Point2i> ground_truth;
  std::vector<cv::Point2i> detected_points;
  std::vector<cv::Point2i> sun_points;

  auto dummy_image = createRandomMarkersImage(NUM_MARKERS, 255, ground_truth);

  DummyLogger logger;
  uvdar::UvLedDetectConfig cfg;
  cfg.gui                 = false;
  cfg.use_masks           = false;
  cfg.threshold           = 50;
  cfg.threshold_diff      = 25;
  cfg.fast_ring_size      = 3;
  cfg.threshold_sun       = 150;
  cfg.threshold_sun_dist  = 25;
  cfg.threshold_sun_merge = 20;
  uvdar::UvdarLedDetectFastCpu uv_detector(cfg, logger);

  int64_t total_time_ms{0};
  bool success_flag;
  for (int i = 0; i < NUM_TESTS; ++i) {
    uvdar::ScopeTimer timer(logger);
    success_flag = uv_detector.processImage(dummy_image, detected_points, sun_points);
    total_time_ms += timer.stop();
  }

  TEST_COUT << "[UvledDetectorCpu]: Average time: " << std::to_string(total_time_ms / NUM_TESTS) << " ms over "
            << std::to_string(NUM_TESTS) << " tests" << std::endl;

  EXPECT_TRUE(success_flag);
  EXPECT_EQ(detected_points.size(), NUM_MARKERS);
  EXPECT_TRUE(compareGroundTruth(ground_truth, detected_points));
}

TEST(UvLedDetector, GPU_CompTimeTest_3Threads) {
  constexpr int NUM_TESTS   = 100;
  constexpr int NUM_MARKERS = 10;
  constexpr int NUM_THREADS = 3;

  std::array<std::vector<cv::Point2i>, NUM_THREADS> ground_truths;
  std::array<cv::Mat, NUM_THREADS> dummy_images;
  std::array<std::vector<cv::Point2i>, NUM_THREADS> detected_points_per_thread;
  std::array<std::vector<cv::Point2i>, NUM_THREADS> sun_points_per_thread;
  std::array<bool, NUM_THREADS> success_flags{};

  for (int i = 0; i < NUM_THREADS; ++i) {
    dummy_images[i] = createRandomMarkersImage(NUM_MARKERS, 255, ground_truths[i]);
  }
  success_flags.fill(false);

  DummyLogger logger;
  uvdar::UvLedDetectConfig cfg;
  cfg.gui                 = false;
  cfg.use_masks           = false;
  cfg.threshold           = 50;
  cfg.threshold_diff      = 25;
  cfg.fast_ring_size      = 3;
  cfg.threshold_sun       = 150;
  cfg.threshold_sun_dist  = 25;
  cfg.threshold_sun_merge = 20;

  std::atomic<int64_t> total_time_ms{0};
  std::atomic<bool> all_ok{true};

  auto worker = [&](int tid) {
    DummyLogger thread_logger;
    uvdar::UvdarLedDetectFastCpu uv_detector(cfg, thread_logger);

    int64_t local_time_ms = 0;

    for (int i = 0; i < NUM_TESTS; ++i) {
      uvdar::ScopeTimer timer(thread_logger);
      bool ok =
          uv_detector.processImage(dummy_images[tid], detected_points_per_thread[tid], sun_points_per_thread[tid]);
      local_time_ms += timer.stop();
      all_ok.store(all_ok.load() && ok);
      success_flags[tid] = ok;
    }

    total_time_ms.fetch_add(local_time_ms);
  };

  std::vector<std::thread> threads;
  threads.reserve(NUM_THREADS);
  for (int t = 0; t < NUM_THREADS; ++t) {
    threads.emplace_back(worker, t);
  }
  for (auto& th : threads) {
    th.join();
  }

  TEST_COUT << "[UvledDetectorGPU 3 Threads]: Average time: " << (total_time_ms.load() / NUM_TESTS) << " ms over "
            << NUM_TESTS << " tests (across " << NUM_THREADS << " threads)\n";

  for (int t = 0; t < NUM_THREADS; ++t) {
    EXPECT_TRUE(success_flags[t]) << "Thread " << t << " failed";
    EXPECT_EQ(detected_points_per_thread[t].size(), NUM_MARKERS) << "Thread " << t;

    EXPECT_TRUE(compareGroundTruth(ground_truths[t], detected_points_per_thread[t]));
  }
  EXPECT_TRUE(all_ok.load());
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
  srand(time(NULL));

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}