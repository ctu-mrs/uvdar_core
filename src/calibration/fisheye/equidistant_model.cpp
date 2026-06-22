#include "uvdar_core/calibration/fisheye/equidistant_model.hpp"

#include <cmath>

namespace uvdar_core::calibration::fisheye {

namespace {

constexpr double epsilon = 1.0e-12;

double thetaScaleDerivative(const EquidistantModel::Parameters& parameters, double theta)
{
    // d/dtheta of theta * (1 + k1 theta^2 + ... + k4 theta^8).
    const double t2 = theta * theta;
    const double t4 = t2 * t2;
    const double t6 = t4 * t2;
    const double t8 = t4 * t4;
    return 1.0 + 3.0 * parameters.k1 * t2 + 5.0 * parameters.k2 * t4 + 7.0 * parameters.k3 * t6 + 9.0 * parameters.k4 * t8;
}

double distortTheta(const EquidistantModel::Parameters& parameters, double theta)
{
    // OpenCV fisheye angular distortion polynomial.
    const double t2 = theta * theta;
    const double t4 = t2 * t2;
    const double t6 = t4 * t2;
    const double t8 = t4 * t4;
    return theta * (1.0 + parameters.k1 * t2 + parameters.k2 * t4 + parameters.k3 * t6 + parameters.k4 * t8);
}

} // namespace

EquidistantModel::EquidistantModel(Parameters parameters)
    : parameters_(parameters)
{
}

Eigen::Vector3d EquidistantModel::backProject(const Eigen::Vector2d& image_point) const
{
    const double xd = (image_point.x() - parameters_.cx) / parameters_.fx;
    const double yd = (image_point.y() - parameters_.cy) / parameters_.fy;
    const double theta_d = std::sqrt(xd * xd + yd * yd);
    if (theta_d < 1.0e-12) {
        return Eigen::Vector3d::UnitZ();
    }

    double theta = theta_d;
    for (int i = 0; i < 6; ++i) {
        // Newton solve theta_d(theta) = observed_radius.
        const double f = distortTheta(parameters_, theta) - theta_d;
        const double df = thetaScaleDerivative(parameters_, theta);
        theta -= f / df;
    }

    const double scale = std::tan(theta) / theta_d;
    return Eigen::Vector3d(xd * scale, yd * scale, 1.0).normalized();
}

Eigen::Vector2d EquidistantModel::project(const Eigen::Vector3d& camera_point) const
{
    const double r = std::hypot(camera_point.x(), camera_point.y());
    if (r < 1.0e-12) {
        return {parameters_.cx, parameters_.cy};
    }

    // Equidistant projection maps ray angle theta to image radius theta_d.
    const double theta = std::atan2(r, camera_point.z());
    const double theta_d = distortTheta(parameters_, theta);
    const double scale = theta_d / r;
    return {parameters_.fx * camera_point.x() * scale + parameters_.cx, parameters_.fy * camera_point.y() * scale + parameters_.cy};
}

Eigen::Matrix<double, 2, 3> EquidistantModel::projectJacobian(const Eigen::Vector3d& camera_point) const
{
    const double x = camera_point.x();
    const double y = camera_point.y();
    const double z = camera_point.z();
    const double r = std::hypot(x, y);
    if (r < epsilon) {
        Eigen::Matrix<double, 2, 3> jacobian = Eigen::Matrix<double, 2, 3>::Zero();
        const double z_safe = std::abs(z) < epsilon ? epsilon : z;
        jacobian(0, 0) = parameters_.fx / z_safe;
        jacobian(1, 1) = parameters_.fy / z_safe;
        return jacobian;
    }

    // Chain rule: camera point -> r,theta -> distorted theta -> pixel.
    const double theta = std::atan2(r, z);
    const double theta_d = distortTheta(parameters_, theta);
    const double dtheta_dtheta = thetaScaleDerivative(parameters_, theta);
    const double denom = r * r + z * z;
    const Eigen::Vector3d dr(x / r, y / r, 0.0);
    Eigen::Vector3d dtheta;
    dtheta << z / denom * dr.x(), z / denom * dr.y(), -r / denom;
    const Eigen::Vector3d dtheta_d = dtheta_dtheta * dtheta;
    const Eigen::Vector3d dscale = (r * dtheta_d - theta_d * dr) / (r * r);

    Eigen::Matrix<double, 2, 3> jacobian;
    jacobian.row(0) = parameters_.fx * (Eigen::RowVector3d(1.0, 0.0, 0.0) * (theta_d / r) + x * dscale.transpose());
    jacobian.row(1) = parameters_.fy * (Eigen::RowVector3d(0.0, 1.0, 0.0) * (theta_d / r) + y * dscale.transpose());
    return jacobian;
}

Eigen::Matrix<double, 3, 2> EquidistantModel::backProjectJacobian(const Eigen::Vector2d& image_point) const
{
    const double xd = (image_point.x() - parameters_.cx) / parameters_.fx;
    const double yd = (image_point.y() - parameters_.cy) / parameters_.fy;
    const double theta_d = std::sqrt(xd * xd + yd * yd);
    if (theta_d < epsilon) {
        Eigen::Matrix<double, 3, 2> jacobian = Eigen::Matrix<double, 3, 2>::Zero();
        jacobian(0, 0) = 1.0 / parameters_.fx;
        jacobian(1, 1) = 1.0 / parameters_.fy;
        return jacobian;
    }

    double theta = theta_d;
    for (int i = 0; i < 6; ++i) {
        // Same inverse angular distortion as backProject().
        const double df = thetaScaleDerivative(parameters_, theta);
        theta -= (distortTheta(parameters_, theta) - theta_d) / df;
    }

    const double inv_distorted = 1.0 / theta_d;
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);
    const Eigen::Vector2d radial_unit(xd * inv_distorted, yd * inv_distorted);
    const double dtheta_dtheta_d = 1.0 / thetaScaleDerivative(parameters_, theta);

    Eigen::Matrix<double, 3, 2> d_bearing_d_distorted;
    for (int axis = 0; axis < 2; ++axis) {
        const double dtheta_d_axis = dtheta_dtheta_d * (axis == 0 ? xd : yd) * inv_distorted;
        Eigen::Vector2d dunit;
        if (axis == 0) {
            dunit << yd * yd, -xd * yd;
        } else {
            dunit << -xd * yd, xd * xd;
        }
        dunit *= inv_distorted * inv_distorted * inv_distorted;
        d_bearing_d_distorted(0, axis) = dunit.x() * sin_theta + radial_unit.x() * cos_theta * dtheta_d_axis;
        d_bearing_d_distorted(1, axis) = dunit.y() * sin_theta + radial_unit.y() * cos_theta * dtheta_d_axis;
        d_bearing_d_distorted(2, axis) = -sin_theta * dtheta_d_axis;
    }

    // Final chain: bearing(distorted normalized coords) * d normalized / d pixel.
    Eigen::Matrix2d pixel_to_distorted = Eigen::Matrix2d::Zero();
    pixel_to_distorted(0, 0) = 1.0 / parameters_.fx;
    pixel_to_distorted(1, 1) = 1.0 / parameters_.fy;
    return d_bearing_d_distorted * pixel_to_distorted;
}

} // namespace uvdar_core::calibration::fisheye
