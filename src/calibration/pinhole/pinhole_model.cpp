#include "uvdar_core/calibration/pinhole/pinhole_model.hpp"

#include <cmath>

namespace uvdar_core::calibration::pinhole {

namespace {

constexpr double epsilon = 1.0e-12;

Eigen::Matrix3d normalizedVectorJacobian(const Eigen::Vector3d& vector)
{
    // d(v / ||v||) / dv = I/||v|| - vv^T/||v||^3.
    const double norm = vector.norm();
    if (norm < epsilon) {
        return Eigen::Matrix3d::Zero();
    }
    return Eigen::Matrix3d::Identity() / norm - (vector * vector.transpose()) / (norm * norm * norm);
}

} // namespace

PinholeModel::PinholeModel(Parameters parameters)
    : parameters_(parameters)
{
}

Eigen::Vector3d PinholeModel::backProject(const Eigen::Vector2d& image_point) const
{
    double x = (image_point.x() - parameters_.cx) / parameters_.fx;
    double y = (image_point.y() - parameters_.cy) / parameters_.fy;

    // Iteratively invert the Brown-Conrady distortion used by OpenCV pinhole calibration.
    for (int i = 0; i < 5; ++i) {
        const double r2 = x * x + y * y;
        const double radial = 1.0 + parameters_.k1 * r2 + parameters_.k2 * r2 * r2 + parameters_.k3 * r2 * r2 * r2;
        const double dx = 2.0 * parameters_.p1 * x * y + parameters_.p2 * (r2 + 2.0 * x * x);
        const double dy = parameters_.p1 * (r2 + 2.0 * y * y) + 2.0 * parameters_.p2 * x * y;
        x = ((image_point.x() - parameters_.cx) / parameters_.fx - dx) / radial;
        y = ((image_point.y() - parameters_.cy) / parameters_.fy - dy) / radial;
    }

    return Eigen::Vector3d(x, y, 1.0).normalized();
}

Eigen::Vector2d PinholeModel::project(const Eigen::Vector3d& camera_point) const
{
    // Perspective division followed by Brown-Conrady radial/tangential distortion.
    const double z = std::abs(camera_point.z()) < epsilon ? std::copysign(epsilon, camera_point.z() == 0.0 ? 1.0 : camera_point.z()) : camera_point.z();
    const double x = camera_point.x() / z;
    const double y = camera_point.y() / z;
    const double r2 = x * x + y * y;
    const double radial = 1.0 + parameters_.k1 * r2 + parameters_.k2 * r2 * r2 + parameters_.k3 * r2 * r2 * r2;
    const double xd = x * radial + 2.0 * parameters_.p1 * x * y + parameters_.p2 * (r2 + 2.0 * x * x);
    const double yd = y * radial + parameters_.p1 * (r2 + 2.0 * y * y) + 2.0 * parameters_.p2 * x * y;
    return {parameters_.fx * xd + parameters_.cx, parameters_.fy * yd + parameters_.cy};
}

Eigen::Matrix<double, 2, 3> PinholeModel::projectJacobian(const Eigen::Vector3d& camera_point) const
{
    const double z = std::abs(camera_point.z()) < epsilon ? std::copysign(epsilon, camera_point.z() == 0.0 ? 1.0 : camera_point.z()) : camera_point.z();
    const double x = camera_point.x() / z;
    const double y = camera_point.y() / z;
    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double radial = 1.0 + parameters_.k1 * r2 + parameters_.k2 * r4 + parameters_.k3 * r4 * r2;
    const double dradial_dx = 2.0 * x * (parameters_.k1 + 2.0 * parameters_.k2 * r2 + 3.0 * parameters_.k3 * r4);
    const double dradial_dy = 2.0 * y * (parameters_.k1 + 2.0 * parameters_.k2 * r2 + 3.0 * parameters_.k3 * r4);

    // d(x_distorted,y_distorted)/d(x,y).
    Eigen::Matrix2d distortion_jacobian;
    distortion_jacobian(0, 0) = radial + x * dradial_dx + 2.0 * parameters_.p1 * y + 6.0 * parameters_.p2 * x;
    distortion_jacobian(0, 1) = x * dradial_dy + 2.0 * parameters_.p1 * x + 2.0 * parameters_.p2 * y;
    distortion_jacobian(1, 0) = y * dradial_dx + 2.0 * parameters_.p1 * x + 2.0 * parameters_.p2 * y;
    distortion_jacobian(1, 1) = radial + y * dradial_dy + 6.0 * parameters_.p1 * y + 2.0 * parameters_.p2 * x;

    Eigen::Matrix<double, 2, 3> normalized_jacobian;
    normalized_jacobian << 1.0 / z, 0.0, -camera_point.x() / (z * z),
        0.0, 1.0 / z, -camera_point.y() / (z * z);

    Eigen::Matrix2d intrinsics = Eigen::Matrix2d::Zero();
    intrinsics(0, 0) = parameters_.fx;
    intrinsics(1, 1) = parameters_.fy;
    return intrinsics * distortion_jacobian * normalized_jacobian;
}

Eigen::Matrix<double, 3, 2> PinholeModel::backProjectJacobian(const Eigen::Vector2d& image_point) const
{
    double x = (image_point.x() - parameters_.cx) / parameters_.fx;
    double y = (image_point.y() - parameters_.cy) / parameters_.fy;
    for (int i = 0; i < 5; ++i) {
        const double r2 = x * x + y * y;
        const double radial = 1.0 + parameters_.k1 * r2 + parameters_.k2 * r2 * r2 + parameters_.k3 * r2 * r2 * r2;
        const double dx = 2.0 * parameters_.p1 * x * y + parameters_.p2 * (r2 + 2.0 * x * x);
        const double dy = parameters_.p1 * (r2 + 2.0 * y * y) + 2.0 * parameters_.p2 * x * y;
        x = ((image_point.x() - parameters_.cx) / parameters_.fx - dx) / radial;
        y = ((image_point.y() - parameters_.cy) / parameters_.fy - dy) / radial;
    }

    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double radial = 1.0 + parameters_.k1 * r2 + parameters_.k2 * r4 + parameters_.k3 * r4 * r2;
    const double dradial_dx = 2.0 * x * (parameters_.k1 + 2.0 * parameters_.k2 * r2 + 3.0 * parameters_.k3 * r4);
    const double dradial_dy = 2.0 * y * (parameters_.k1 + 2.0 * parameters_.k2 * r2 + 3.0 * parameters_.k3 * r4);

    Eigen::Matrix2d distortion_jacobian;
    distortion_jacobian(0, 0) = radial + x * dradial_dx + 2.0 * parameters_.p1 * y + 6.0 * parameters_.p2 * x;
    distortion_jacobian(0, 1) = x * dradial_dy + 2.0 * parameters_.p1 * x + 2.0 * parameters_.p2 * y;
    distortion_jacobian(1, 0) = y * dradial_dx + 2.0 * parameters_.p1 * x + 2.0 * parameters_.p2 * y;
    distortion_jacobian(1, 1) = radial + y * dradial_dy + 6.0 * parameters_.p1 * y + 2.0 * parameters_.p2 * x;

    Eigen::Matrix2d pixel_to_distorted = Eigen::Matrix2d::Zero();
    pixel_to_distorted(0, 0) = 1.0 / parameters_.fx;
    pixel_to_distorted(1, 1) = 1.0 / parameters_.fy;
    // Inverse-function theorem for the iterative undistortion map.
    const Eigen::Matrix2d undistorted_jacobian = distortion_jacobian.inverse() * pixel_to_distorted;

    Eigen::Matrix<double, 3, 2> ray_jacobian;
    ray_jacobian << 1.0, 0.0,
        0.0, 1.0,
        0.0, 0.0;
    return normalizedVectorJacobian(Eigen::Vector3d(x, y, 1.0)) * ray_jacobian * undistorted_jacobian;
}

} // namespace uvdar_core::calibration::pinhole
