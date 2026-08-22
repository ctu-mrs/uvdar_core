#include "uvdar_core/calibration/fisheye/radial_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "uvdar_core/helpers/polynomial.hpp"

namespace uvdar_core::calibration::fisheye {

namespace {

constexpr double epsilon = 1.0e-12;
constexpr double pi = 3.14159265358979323846;

double square(double value)
{
    return value * value;
}

std::array<double, 10> distortionPolynomial(
    const RadialModel::Parameters& parameters)
{
    return {{
        0.0, 1.0, 0.0, parameters.k1, 0.0, parameters.k2,
        0.0, parameters.k3, 0.0, parameters.k4,
    }};
}

} // namespace

RadialModel::RadialModel(Parameters parameters)
    : parameters_(parameters)
{
}

Eigen::Vector3d RadialModel::backProject(const Eigen::Vector2d& image_point) const
{
    const double xd = (image_point.x() - parameters_.cx) / parameters_.fx;
    const double yd = (image_point.y() - parameters_.cy) / parameters_.fy;
    const double distorted_radius = std::hypot(xd, yd);
    if (distorted_radius < epsilon) {
        return Eigen::Vector3d::UnitZ();
    }

    // Pixel radius -> undistorted model radius -> ray angle theta.
    const double radius = undistortRadius(distorted_radius);
    const double theta = radiusToTheta(radius);
    const double sin_theta = std::sin(theta);
    const double scale = sin_theta / distorted_radius;
    return Eigen::Vector3d(xd * scale, yd * scale, std::cos(theta)).normalized();
}

Eigen::Vector2d RadialModel::project(const Eigen::Vector3d& camera_point) const
{
    const double xy_norm = std::hypot(camera_point.x(), camera_point.y());
    if (xy_norm < epsilon) {
        return {parameters_.cx, parameters_.cy};
    }

    // Project through the selected ideal fisheye law, then distort the radius.
    const double theta = std::atan2(xy_norm, camera_point.z());
    const double radius = distortRadius(thetaToRadius(theta));
    const double scale = radius / xy_norm;
    return {parameters_.fx * camera_point.x() * scale + parameters_.cx, parameters_.fy * camera_point.y() * scale + parameters_.cy};
}

Eigen::Matrix<double, 2, 3> RadialModel::projectJacobian(const Eigen::Vector3d& camera_point) const
{
    const double x = camera_point.x();
    const double y = camera_point.y();
    const double z = camera_point.z();
    const double xy_norm = std::hypot(x, y);
    if (xy_norm < epsilon) {
        Eigen::Matrix<double, 2, 3> jacobian = Eigen::Matrix<double, 2, 3>::Zero();
        const double z_safe = std::abs(z) < epsilon ? epsilon : z;
        const double local_scale = thetaToRadiusDerivative(0.0) / z_safe;
        jacobian(0, 0) = parameters_.fx * local_scale;
        jacobian(1, 1) = parameters_.fy * local_scale;
        return jacobian;
    }

    // Chain rule: camera point -> theta -> ideal radius -> distorted radius -> pixel.
    const double theta = std::atan2(xy_norm, z);
    const double ideal_radius = thetaToRadius(theta);
    const double radius = distortRadius(ideal_radius);
    const double dradius_dtheta = distortRadiusDerivative(ideal_radius) * thetaToRadiusDerivative(theta);
    const double denom = xy_norm * xy_norm + z * z;
    const Eigen::Vector3d dxy_norm(x / xy_norm, y / xy_norm, 0.0);
    Eigen::Vector3d dtheta;
    dtheta << z / denom * dxy_norm.x(), z / denom * dxy_norm.y(), -xy_norm / denom;
    const Eigen::Vector3d dradius = dradius_dtheta * dtheta;
    const Eigen::Vector3d dscale = (xy_norm * dradius - radius * dxy_norm) / (xy_norm * xy_norm);

    Eigen::Matrix<double, 2, 3> jacobian;
    jacobian.row(0) = parameters_.fx * (Eigen::RowVector3d(1.0, 0.0, 0.0) * (radius / xy_norm) + x * dscale.transpose());
    jacobian.row(1) = parameters_.fy * (Eigen::RowVector3d(0.0, 1.0, 0.0) * (radius / xy_norm) + y * dscale.transpose());
    return jacobian;
}

Eigen::Matrix<double, 3, 2> RadialModel::backProjectJacobian(const Eigen::Vector2d& image_point) const
{
    const double xd = (image_point.x() - parameters_.cx) / parameters_.fx;
    const double yd = (image_point.y() - parameters_.cy) / parameters_.fy;
    const double distorted_radius = std::hypot(xd, yd);
    if (distorted_radius < epsilon) {
        Eigen::Matrix<double, 3, 2> jacobian = Eigen::Matrix<double, 3, 2>::Zero();
        jacobian(0, 0) = 1.0 / parameters_.fx;
        jacobian(1, 1) = 1.0 / parameters_.fy;
        return jacobian;
    }

    // Inverse chain for d bearing / d pixel.
    const double radius = undistortRadius(distorted_radius);
    const double theta = radiusToTheta(radius);
    const double dtheta_drd = radiusToThetaDerivative(radius) / std::max(distortRadiusDerivative(radius), epsilon);
    const double inv_radius = 1.0 / distorted_radius;
    const Eigen::Vector2d radial_unit(xd * inv_radius, yd * inv_radius);
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);

    Eigen::Matrix<double, 3, 2> d_bearing_d_distorted;
    for (int axis = 0; axis < 2; ++axis) {
        const double dtheta_d_axis = dtheta_drd * (axis == 0 ? xd : yd) * inv_radius;
        Eigen::Vector2d dunit;
        if (axis == 0) {
            dunit << yd * yd, -xd * yd;
        } else {
            dunit << -xd * yd, xd * xd;
        }
        dunit *= inv_radius * inv_radius * inv_radius;
        d_bearing_d_distorted(0, axis) = dunit.x() * sin_theta + radial_unit.x() * cos_theta * dtheta_d_axis;
        d_bearing_d_distorted(1, axis) = dunit.y() * sin_theta + radial_unit.y() * cos_theta * dtheta_d_axis;
        d_bearing_d_distorted(2, axis) = -sin_theta * dtheta_d_axis;
    }

    Eigen::Matrix2d pixel_to_distorted = Eigen::Matrix2d::Zero();
    pixel_to_distorted(0, 0) = 1.0 / parameters_.fx;
    pixel_to_distorted(1, 1) = 1.0 / parameters_.fy;
    return d_bearing_d_distorted * pixel_to_distorted;
}

std::string RadialModel::modelName() const
{
    switch (parameters_.projection) {
        case Projection::EquisolidAngle:
            return "fisheye_equisolid";
        case Projection::Stereographic:
            return "fisheye_stereographic";
        case Projection::Orthographic:
            return "fisheye_orthographic";
    }
    return "fisheye_radial";
}

double RadialModel::thetaToRadius(double theta) const
{
    // Ideal fisheye projection families.
    switch (parameters_.projection) {
        case Projection::EquisolidAngle:
            return 2.0 * std::sin(theta * 0.5);
        case Projection::Stereographic:
            return 2.0 * std::tan(std::min(theta, pi - 1.0e-9) * 0.5);
        case Projection::Orthographic:
            return std::sin(theta);
    }
    return theta;
}

double RadialModel::thetaToRadiusDerivative(double theta) const
{
    switch (parameters_.projection) {
        case Projection::EquisolidAngle:
            return std::cos(theta * 0.5);
        case Projection::Stereographic: {
            const double value = std::tan(std::min(theta, pi - 1.0e-9) * 0.5);
            return 1.0 + value * value;
        }
        case Projection::Orthographic:
            return std::cos(theta);
    }
    return 1.0;
}

double RadialModel::radiusToTheta(double radius) const
{
    switch (parameters_.projection) {
        case Projection::EquisolidAngle:
            return 2.0 * std::asin(std::clamp(radius * 0.5, -1.0, 1.0));
        case Projection::Stereographic:
            return 2.0 * std::atan(radius * 0.5);
        case Projection::Orthographic:
            return std::asin(std::clamp(radius, -1.0, 1.0));
    }
    return radius;
}

double RadialModel::radiusToThetaDerivative(double radius) const
{
    switch (parameters_.projection) {
        case Projection::EquisolidAngle: {
            const double clamped = std::clamp(radius * 0.5, -1.0 + epsilon, 1.0 - epsilon);
            return 1.0 / std::sqrt(1.0 - clamped * clamped);
        }
        case Projection::Stereographic:
            return 1.0 / (1.0 + 0.25 * radius * radius);
        case Projection::Orthographic: {
            const double clamped = std::clamp(radius, -1.0 + epsilon, 1.0 - epsilon);
            return 1.0 / std::sqrt(1.0 - clamped * clamped);
        }
    }
    return 1.0;
}

double RadialModel::distortRadius(double radius) const
{
    const auto coefficients = distortionPolynomial(parameters_);
    return uvdar_core::helpers::evaluatePolynomialAscending(
        coefficients.begin(), coefficients.end(), radius);
}

double RadialModel::distortRadiusDerivative(double radius) const
{
    const auto coefficients = distortionPolynomial(parameters_);
    return uvdar_core::helpers::
        evaluatePolynomialAndDerivativeAscending(
            coefficients.begin(), coefficients.end(), radius).second;
}

double RadialModel::undistortRadius(double distorted_radius) const
{
    double radius = distorted_radius;
    for (int i = 0; i < 8; ++i) {
        // Newton solve distortRadius(radius) = distorted_radius.
        const double f = distortRadius(radius) - distorted_radius;
        const double df = distortRadiusDerivative(radius);
        if (std::abs(df) < epsilon) {
            break;
        }
        radius = std::max(0.0, radius - f / df);
    }
    return radius;
}

} // namespace uvdar_core::calibration::fisheye
