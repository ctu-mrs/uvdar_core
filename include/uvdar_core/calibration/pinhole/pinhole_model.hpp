#pragma once

#include "uvdar_core/calibration/i_lens_model.hpp"

namespace uvdar_core::calibration::pinhole {

/**
 * @brief Standard pinhole camera model with optional OpenCV radial-tangential distortion.
 *
 * Uses normalized coordinates x=X/Z, y=Y/Z and Brown-Conrady distortion:
 * radial k1,k2,k3 plus tangential p1,p2.
 */
class PinholeModel final : public ILensModel {
public:
    /**
     * @brief Intrinsics and OpenCV Brown-Conrady distortion coefficients.
     */
    struct Parameters {
        double fx = 1.0;
        double fy = 1.0;
        double cx = 0.0;
        double cy = 0.0;
        double k1 = 0.0;
        double k2 = 0.0;
        double p1 = 0.0;
        double p2 = 0.0;
        double k3 = 0.0;
        int width = 0;
        int height = 0;
    };

    /**
     * @brief Store pinhole intrinsics and distortion coefficients.
     */
    explicit PinholeModel(Parameters parameters);

    /**
     * @brief Iteratively undistort a pixel and return a unit bearing.
     */
    Eigen::Vector3d backProject(const Eigen::Vector2d& image_point) const override;

    /**
     * @brief Project a 3D point through perspective division and distortion.
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
    std::string modelName() const override { return "pinhole"; }

private:
    Parameters parameters_;
};

} // namespace uvdar_core::calibration::pinhole
