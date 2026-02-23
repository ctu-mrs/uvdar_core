#include <uvdar/blink_processor/calibration.h>

namespace uvdar::blink_processor {

/* CameraCalibration constructor //{ */
CameraCalibration::CameraCalibration(const CameraModel& model) : model(model) {
}
//}

/* loadCalibration //{ */
CameraCalibration CameraCalibration::loadCalibration(const std::filesystem::path& file_path) {
  if (file_path.empty()) {
    throw std::runtime_error("No camera calibration file has been specified!");
  }

  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open camera calibration file!");
  }

  CameraModel camera;
  auto calib_file_state = CalibrationFileState::POLY_COEFFS;

  std::string line;
  int line_num = 0;
  while (std::getline(file, line)) {
    ++line_num;

    if (line.empty() || line[0] == '#') {
      continue;
    }

    switch (calib_file_state) {
      case CalibrationFileState::POLY_COEFFS: {
        extractPolyCoeffs_(line, camera);
        calib_file_state = CalibrationFileState::INV_POLY_COEFFS;
        break;
      }
      case CalibrationFileState::INV_POLY_COEFFS: {
        extractInvPolyCoeffs_(line, camera);
        calib_file_state = CalibrationFileState::CENTER;
        break;
      }
      case CalibrationFileState::CENTER: {
        extractCenter_(line, camera);
        calib_file_state = CalibrationFileState::AFFINE_PARAMS;
        break;
      }
      case CalibrationFileState::AFFINE_PARAMS: {
        extractAffineParameters_(line, camera);
        calib_file_state = CalibrationFileState::IMAGE_SIZE;
        break;
      }
      case CalibrationFileState::IMAGE_SIZE: {
        extractImageSize_(line, camera);
        calib_file_state = CalibrationFileState::DONE;
        break;
      }
      case CalibrationFileState::DONE: {
        break;
      }
    }
  }

  if (calib_file_state != CalibrationFileState::DONE) {
    throw std::runtime_error("Camera calibration file is incomplete!");
  }

  return CameraCalibration(camera);
}
//}

/* extractPolyCoeffs_ //{ */
void CameraCalibration::extractPolyCoeffs_(std::string& line, CameraModel& camera) {
  std::istringstream iss(line);
  std::string token;

  // First token is the number of coefficients — skip it
  if (!std::getline(iss, token, ' ')) {
    throw std::runtime_error("Expected polynomial coefficient count but line is empty!");
  }

  while (std::getline(iss, token, ' ')) {
    if (token.empty())
      continue;
    double val = std::stod(token);
    camera.poly_coeffs.push_back(val);
  }
}
//}

/* extractInvPolyCoeffs_ //{ */
void CameraCalibration::extractInvPolyCoeffs_(std::string& line, CameraModel& camera) {
  std::istringstream iss(line);
  std::string token;

  // First token is the number of coefficients — skip it
  if (!std::getline(iss, token, ' ')) {
    throw std::runtime_error("Expected inverse polynomial coefficient count but line is empty!");
  }

  while (std::getline(iss, token, ' ')) {
    if (token.empty())
      continue;
    double val = std::stod(token);
    camera.inv_poly_coeffs.push_back(val);
  }
}
//}

/* extractCenter_ //{ */
void CameraCalibration::extractCenter_(std::string& line, CameraModel& camera) {
  std::istringstream iss(line);
  std::string token;
  std::array<double, 2> center_coord;

  size_t i{0};
  while (std::getline(iss, token, ' ')) {
    if (token.empty())
      continue;
    center_coord.at(i) = std::stod(token);
    ++i;
  }

  camera.xc = center_coord[0];
  camera.yc = center_coord[1];
}
//}

/* extractAffineParameters_ //{ */
void CameraCalibration::extractAffineParameters_(std::string& line, CameraModel& camera) {
  std::istringstream iss(line);
  std::string token;
  std::array<double, 3> affine_params;

  size_t i{0};
  while (std::getline(iss, token, ' ')) {
    if (token.empty())
      continue;
    affine_params.at(i) = std::stod(token);
    ++i;
  }

  camera.c = affine_params[0];
  camera.d = affine_params[1];
  camera.e = affine_params[2];
}
//}

/* extractImageSize_ //{ */
void CameraCalibration::extractImageSize_(std::string& line, CameraModel& camera) {
  std::istringstream iss(line);
  std::string token;
  std::array<double, 2> image_params;

  size_t i{0};
  while (std::getline(iss, token, ' ')) {
    if (token.empty())
      continue;
    image_params.at(i) = std::stod(token);
    ++i;
  }

  camera.height = image_params[0];
  camera.width  = image_params[1];
}
//}

/* camToWorld //{ */
cv::Point3d CameraCalibration::camToWorld(cv::Point2d& point_px) const {
  double inv_det = 1 / (model.c - model.d * model.e);

  // point_px.y = row, point_px.x = column; xc = row center, yc = column center
  double xp = inv_det * ((point_px.y - model.xc) - model.d * (point_px.x - model.yc));
  double yp = inv_det * (-model.e * (point_px.y - model.xc) + model.c * (point_px.x - model.yc));

  double r   = sqrt(xp * xp + yp * yp);
  double zp  = model.poly_coeffs.at(0);
  double r_i = 1;

  for (size_t i = 1; i < model.poly_coeffs.size(); ++i) {
    r_i *= r;
    zp += r_i * model.poly_coeffs.at(i);
  }

  // normalize to unit norm
  double inv_norm = 1 / sqrt(xp * xp + yp * yp + zp * zp);

  cv::Point3d world_coord;
  world_coord.x = inv_norm * xp;
  world_coord.y = inv_norm * yp;
  world_coord.z = inv_norm * zp;

  return world_coord;
}
//}

/* worldToCam //{ */
cv::Point2d CameraCalibration::worldToCam(cv::Point3d& point) const {
  double norm = sqrt(point.x * point.x + point.y * point.y);

  if (norm == 0) {
    return cv::Point2d(model.xc, model.yc);
  }

  double theta    = atan(point.z / norm);
  double inv_norm = 1 / norm;
  double rho      = model.inv_poly_coeffs.at(0);
  double t_i      = 1.0;
  for (size_t i = 1; i < model.inv_poly_coeffs.size(); ++i) {
    t_i *= theta;
    rho += t_i * model.inv_poly_coeffs.at(i);
  }

  double x = point.x * inv_norm * rho;
  double y = point.y * inv_norm * rho;

  cv::Point2d point_px;
  point_px.y = x * model.c + y * model.d + model.xc;
  point_px.x = x * model.e + y + model.yc;

  return point_px;
}
//}

} // namespace uvdar::blink_processor