#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>

namespace uvdar_core::calibration {

/**
 * @brief Common projection API for all camera/lens models used by pose solvers.
 *
 * The pose-estimation code depends only on this interface. Implementations may
 * be pinhole, OCamCalib, or any fisheye model as long as they provide analytic
 * projection and inverse-projection Jacobians.
 */
class ILensModel {
public:
    virtual ~ILensModel() = default;

    /**
     * @brief Convert image coordinates to a unit camera-frame bearing.
     */
    virtual Eigen::Vector3d backProject(const Eigen::Vector2d& image_point) const = 0;

    /**
     * @brief Project a camera-frame 3D point to image coordinates.
     */
    virtual Eigen::Vector2d project(const Eigen::Vector3d& camera_point) const = 0;

    /**
     * @brief Analytic Jacobian d(project(camera_point))/d(camera_point).
     */
    virtual Eigen::Matrix<double, 2, 3> projectJacobian(const Eigen::Vector3d& camera_point) const = 0;
    /**
     * @brief Analytic Jacobian d(backProject(image_point))/d(image_point).
     */
    virtual Eigen::Matrix<double, 3, 2> backProjectJacobian(const Eigen::Vector2d& image_point) const = 0;

    /**
     * @brief Image width in pixels.
     */
    virtual int imageWidth() const = 0;

    /**
     * @brief Image height in pixels.
     */
    virtual int imageHeight() const = 0;

    /**
     * @brief Stable string used in YAML configuration and diagnostics.
     */
    virtual std::string modelName() const = 0;
};

using LensModelPtr = std::shared_ptr<ILensModel>;

} // namespace uvdar_core::calibration
