#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "uvdar_core/calibration/i_lens_model.hpp"

namespace uvdar_core::pose_estimation {

/**
 * @brief Generic calibrated camera used by pose-estimation backends.
 *
 * Pose estimators depend only on the lens interface: project, back-project,
 * and their Jacobians. No solver should require a concrete lens model type.
 */
struct CameraModel {
    calibration::LensModelPtr lens;
    int image_width = 0;
    int image_height = 0;

    /** @brief Whether this camera has a calibrated lens model. */
    bool valid() const { return static_cast<bool>(lens); }

    /** @brief Effective image width, preferring explicit configuration. */
    int width() const { return image_width > 0 ? image_width : (lens ? lens->imageWidth() : 0); }

    /** @brief Effective image height, preferring explicit configuration. */
    int height() const { return image_height > 0 ? image_height : (lens ? lens->imageHeight() : 0); }

    /** @brief Project a camera-frame point through the configured lens. */
    Eigen::Vector2d project(const Eigen::Vector3d& camera_point) const
    {
        return lens ? lens->project(camera_point) : Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    /** @brief Return d(pixel)/d(camera point) at a camera-frame point. */
    Eigen::Matrix<double, 2, 3> projectionJacobian(const Eigen::Vector3d& camera_point) const
    {
        return lens ? lens->projectJacobian(camera_point) : Eigen::Matrix<double, 2, 3>::Zero();
    }

    /** @brief Back-project a pixel to the calibrated camera ray. */
    Eigen::Vector3d backProject(const Eigen::Vector2d& pixel) const
    {
        return lens ? lens->backProject(pixel) : Eigen::Vector3d::Zero();
    }

    /** @brief Back-project and normalize a finite pixel ray. */
    std::optional<Eigen::Vector3d> bearingForPixel(const Eigen::Vector2d& pixel) const
    {
        const Eigen::Vector3d bearing = backProject(pixel);
        if (!bearing.allFinite() || bearing.squaredNorm() <= std::numeric_limits<double>::epsilon()) {
            return std::nullopt;
        }
        return bearing.normalized();
    }

    /** @brief Test whether a pixel lies inside this image with an optional margin. */
    bool containsPixel(const Eigen::Vector2d& pixel, double margin = 0.0) const
    {
        return pixel.x() >= margin && pixel.y() >= margin
            && pixel.x() < static_cast<double>(width()) - margin
            && pixel.y() < static_cast<double>(height()) - margin;
    }

    /** @brief Largest pairwise angle between valid camera rays. */
    static double largestBearingAngle(const std::vector<Eigen::Vector3d>& bearings)
    {
        double largest_angle = 0.0;
        for (std::size_t first = 0U; first + 1U < bearings.size(); ++first) {
            if (!bearings[first].allFinite() || bearings[first].squaredNorm() <= std::numeric_limits<double>::epsilon()) {
                continue;
            }
            for (std::size_t second = first + 1U; second < bearings.size(); ++second) {
                if (!bearings[second].allFinite() || bearings[second].squaredNorm() <= std::numeric_limits<double>::epsilon()) {
                    continue;
                }
                const double cosine = std::clamp(
                    bearings[first].normalized().dot(bearings[second].normalized()),
                    -1.0,
                    1.0);
                largest_angle = std::max(largest_angle, std::acos(cosine));
            }
        }
        return largest_angle;
    }

    /** @brief Unit vector in the mean direction of valid camera rays. */
    static std::optional<Eigen::Vector3d> meanBearing(const std::vector<Eigen::Vector3d>& bearings)
    {
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (const Eigen::Vector3d& bearing : bearings) {
            if (bearing.allFinite() && bearing.squaredNorm() > std::numeric_limits<double>::epsilon()) {
                mean += bearing.normalized();
            }
        }
        if (mean.squaredNorm() <= std::numeric_limits<double>::epsilon()) {
            return std::nullopt;
        }
        return mean.normalized();
    }
};

} // namespace uvdar_core::pose_estimation
