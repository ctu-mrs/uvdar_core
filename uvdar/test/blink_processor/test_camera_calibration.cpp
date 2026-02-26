#include <gtest/gtest.h>
#include <random>
#include "../dummy_logger.h"
#include "../timer.h"

#include <uvdar/blink_processor/calibration.h>

using namespace uvdar::blink_processor;

/* TEST(CameraCalibration, ReadCalibrationFile) //{ */
TEST(CameraCalibration, ReadCalibrationFile) {
  std::filesystem::path calib_file(std::string(TEST_ASSETS_DIR) + "/cam_calib_results.txt");
  CameraCalibration calib = CameraCalibration::loadCalibration(calib_file);

  const std::vector<double> gt_poly_coeffs = {-2.254359e+02, 0.000000e+00, 2.826023e-03, -8.922588e-06, 1.995230e-08};
  const std::vector<double> gt_inv_poly_coeffs = {330.941354, 180.664646, -27.639261, 27.986217,  21.877833,
                                                  -20.121553, 9.248770,   22.643899,  -12.159559, -11.442729,
                                                  6.957154,   4.563095,   -0.988818,  -0.638005};
  const std::vector<double> center             = {316.134658, 457.251031};
  const double c                               = 1.000433;
  const double d                               = -0.000012;
  const double e                               = -0.000018;
  const int height                             = 600;
  const int width                              = 960;

  for (size_t i = 0; i < gt_poly_coeffs.size(); ++i) {
    EXPECT_EQ(gt_poly_coeffs.at(i), calib.model.poly_coeffs.at(i));
  }

  for (size_t i = 0; i < gt_inv_poly_coeffs.size(); ++i) {
    EXPECT_EQ(gt_inv_poly_coeffs.at(i), calib.model.inv_poly_coeffs.at(i));
  }

  EXPECT_EQ(calib.model.xc, center.at(0));
  EXPECT_EQ(calib.model.yc, center.at(1));

  EXPECT_EQ(calib.model.c, c);
  EXPECT_EQ(calib.model.d, d);
  EXPECT_EQ(calib.model.e, e);

  EXPECT_EQ(calib.model.height, height);
  EXPECT_EQ(calib.model.width, width);
}
//}

/* TEST(CameraCalibration, Transformations) //{ */
TEST(CameraCalibration, Transformations) {
  std::filesystem::path calib_file(std::string(TEST_ASSETS_DIR) + "/cam_calib_results.txt");
  CameraCalibration calib = CameraCalibration::loadCalibration(calib_file);

  cv::Point2d gt_pixel_coord(460, 512);

  auto world_coord = calib.camToWorld(gt_pixel_coord);
  auto pixel_coord = calib.worldToCam(world_coord);

  double TOL_PX = 2.0;
  EXPECT_NEAR(gt_pixel_coord.x, pixel_coord.x, TOL_PX);
  EXPECT_NEAR(gt_pixel_coord.y, pixel_coord.y, TOL_PX);
}
//}