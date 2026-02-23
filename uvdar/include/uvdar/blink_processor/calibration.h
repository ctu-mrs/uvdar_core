// Copyright (C) 2009 DAVIDE SCARAMUZZA
// Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org

#pragma once

#include <Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <filesystem>
#include <fstream>

namespace uvdar::blink_processor {

// clang-format off
enum class CalibrationFileState { 
  POLY_COEFFS = 1, 
  INV_POLY_COEFFS, 
  CENTER, 
  AFFINE_PARAMS, 
  IMAGE_SIZE,
  DONE
};
// clang-format on

struct CameraModel {
  std::vector<double> poly_coeffs;
  std::vector<double> inv_poly_coeffs;
  double xc;  // row coordinate of the center
  double yc;  // column coordinate of the center
  double c;   // affine parameter
  double d;   // affine parameter
  double e;   // affine parameter
  int width;  // image width
  int height; // image height
};

class CameraCalibration {
 public:
  static CameraCalibration loadCalibration(const std::filesystem::path& file_path);

  cv::Point3d camToWorld(cv::Point2d& point_px) const;
  cv::Point2d worldToCam(cv::Point3d& point) const;

 public:
  const CameraModel model;

 private:
  CameraCalibration(const CameraModel& model);

  static void extractPolyCoeffs_(std::string& line, CameraModel& camera);
  static void extractInvPolyCoeffs_(std::string& line, CameraModel& camera);
  static void extractCenter_(std::string& line, CameraModel& camera);
  static void extractAffineParameters_(std::string& line, CameraModel& camera);
  static void extractImageSize_(std::string& line, CameraModel& camera);
};

} // namespace uvdar::blink_processor