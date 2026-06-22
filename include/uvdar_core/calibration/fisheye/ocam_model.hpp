#pragma once

#include <array>
#include <string>

#include <Eigen/Dense>

#include "uvdar_core/calibration/i_lens_model.hpp"

namespace uvdar_core::calibration::fisheye {

constexpr int max_polynomial_length = 64;

/**
 * @brief OCamCalib polynomial lens model.
 *
 * The stored coefficients mirror Davide Scaramuzza's calibration text format,
 * while the public methods implement the common lens API used by pose solvers.
 */
class OcamModel final : public uvdar_core::calibration::ILensModel {
public:
    /**
     * @brief Direct polynomial mapping image radius to z coordinate.
     */
    std::array<double, max_polynomial_length> pol {};
    int length_pol = 0;
    /**
     * @brief Inverse polynomial mapping ray angle to image radius.
     */
    std::array<double, max_polynomial_length> invpol {};
    int length_invpol = 0;
    /**
     * @brief OCamCalib affine image transform parameters.
     */
    double xc = 0.0;
    double yc = 0.0;
    double c = 1.0;
    double d = 0.0;
    double e = 0.0;
    int width = 0;
    int height = 0;

    /**
     * @brief Back-project [row, col] by the direct OCamCalib polynomial.
     */
    Eigen::Vector3d backProject(const Eigen::Vector2d& image_point) const override;

    /**
     * @brief Project a camera point by the inverse OCamCalib polynomial.
     */
    Eigen::Vector2d project(const Eigen::Vector3d& camera_point) const override;

    /**
     * @brief Analytic/finite-stable Jacobian d pixel / d camera point.
     */
    Eigen::Matrix<double, 2, 3> projectJacobian(const Eigen::Vector3d& camera_point) const override;

    /**
     * @brief Analytic/finite-stable Jacobian d bearing / d pixel.
     */
    Eigen::Matrix<double, 3, 2> backProjectJacobian(const Eigen::Vector2d& image_point) const override;
    int imageWidth() const override { return width; }
    int imageHeight() const override { return height; }
    std::string modelName() const override { return "ocamcalib"; }
};

/**
 * @brief Load an OCamCalib model from the standard text export.
 */
OcamModel loadModel(const std::string& filename);

/**
 * @brief Back-project an image point [row, col] to a unit bearing vector.
 */
Eigen::Vector3d cam2world(const Eigen::Vector2d& point_2d, const OcamModel& model);

/**
 * @brief Project a 3D camera-frame point to image coordinates [row, col].
 */
Eigen::Vector2d world2cam(const Eigen::Vector3d& point_3d, const OcamModel& model);

} // namespace uvdar_core::calibration::fisheye
