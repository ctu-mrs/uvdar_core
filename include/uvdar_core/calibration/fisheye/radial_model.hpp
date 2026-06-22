#pragma once

#include "uvdar_core/calibration/i_lens_model.hpp"

namespace uvdar_core::calibration::fisheye {

/**
 * @brief Common radial fisheye projection families with optional polynomial radius distortion.
 *
 * The input intrinsics are [fx, fy, cx, cy]. Distortion coefficients k1..k4
 * are applied as r_d = r * (1 + k1*r^2 + k2*r^4 + k3*r^6 + k4*r^8),
 * where r is the ideal normalized radius of the selected fisheye projection.
 */
class RadialModel final : public ILensModel {
public:
    /**
     * @brief Ideal fisheye radius law r(theta).
     */
    enum class Projection {
        EquisolidAngle,
        Stereographic,
        Orthographic,
    };

    /**
     * @brief Intrinsics, selected fisheye projection family, and radial distortion.
     */
    struct Parameters {
        Projection projection = Projection::EquisolidAngle;
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
     * @brief Store radial fisheye intrinsics and projection family.
     */
    explicit RadialModel(Parameters parameters);

    /**
     * @brief Convert a distorted image radius back to theta and a bearing.
     */
    Eigen::Vector3d backProject(const Eigen::Vector2d& image_point) const override;

    /**
     * @brief Project by theta-to-radius law, polynomial distortion, and intrinsics.
     */
    Eigen::Vector2d project(const Eigen::Vector3d& camera_point) const override;

    /**
     * @brief Analytic chain-rule Jacobian d pixel / d camera point.
     */
    Eigen::Matrix<double, 2, 3> projectJacobian(const Eigen::Vector3d& camera_point) const override;

    /**
     * @brief Analytic chain-rule Jacobian d bearing / d pixel.
     */
    Eigen::Matrix<double, 3, 2> backProjectJacobian(const Eigen::Vector2d& image_point) const override;
    int imageWidth() const override { return parameters_.width; }
    int imageHeight() const override { return parameters_.height; }
    std::string modelName() const override;

private:
    /**
     * @brief Ideal projection radius r(theta).
     */
    double thetaToRadius(double theta) const;

    /**
     * @brief Derivative dr/dtheta of the ideal projection law.
     */
    double thetaToRadiusDerivative(double theta) const;

    /**
     * @brief Invert ideal projection radius to theta.
     */
    double radiusToTheta(double radius) const;

    /**
     * @brief Derivative dtheta/dr of the inverse ideal law.
     */
    double radiusToThetaDerivative(double radius) const;

    /**
     * @brief Apply polynomial radial distortion r_d = r(1 + k1 r^2 + ...).
     */
    double distortRadius(double radius) const;

    /**
     * @brief Derivative of polynomial radial distortion.
     */
    double distortRadiusDerivative(double radius) const;

    /**
     * @brief Invert polynomial radius distortion by Newton iterations.
     */
    double undistortRadius(double distorted_radius) const;

    Parameters parameters_;
};

} // namespace uvdar_core::calibration::fisheye
