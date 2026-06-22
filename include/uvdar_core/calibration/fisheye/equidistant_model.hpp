#pragma once

#include "uvdar_core/calibration/i_lens_model.hpp"

namespace uvdar_core::calibration::fisheye {

/**
 * @brief OpenCV fisheye/equidistant camera model.
 *
 * Uses theta_d = theta * (1 + k1 theta^2 + k2 theta^4 + k3 theta^6 + k4 theta^8)
 * and u = [fx*x*theta_d/r + cx, fy*y*theta_d/r + cy].
 */
class EquidistantModel final : public ILensModel {
public:
    /**
     * @brief Intrinsics and OpenCV fisheye distortion coefficients.
     */
    struct Parameters {
        double fx = 1.0;
        double fy = 1.0;
        double cx = 0.0;
        double cy = 0.0;
        double k1 = 0.0;
        double k2 = 0.0;
        double k3 = 0.0;
        double k4 = 0.0;
        int width = 0;
        int height = 0;
    };

    /**
     * @brief Store equidistant intrinsics and distortion coefficients.
     */
    explicit EquidistantModel(Parameters parameters);

    /**
     * @brief Invert theta distortion by Newton iterations and return a bearing.
     */
    Eigen::Vector3d backProject(const Eigen::Vector2d& image_point) const override;

    /**
     * @brief Apply equidistant projection and theta polynomial distortion.
     */
    Eigen::Vector2d project(const Eigen::Vector3d& camera_point) const override;

    /**
     * @brief Analytic chain-rule Jacobian of project().
     */
    Eigen::Matrix<double, 2, 3> projectJacobian(const Eigen::Vector3d& camera_point) const override;

    /**
     * @brief Analytic chain-rule Jacobian of backProject().
     */
    Eigen::Matrix<double, 3, 2> backProjectJacobian(const Eigen::Vector2d& image_point) const override;
    int imageWidth() const override { return parameters_.width; }
    int imageHeight() const override { return parameters_.height; }
    std::string modelName() const override { return "fisheye_equidistant"; }

private:
    Parameters parameters_;
};

} // namespace uvdar_core::calibration::fisheye
