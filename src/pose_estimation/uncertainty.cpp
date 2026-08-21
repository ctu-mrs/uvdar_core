#include "uvdar_core/pose_estimation/uncertainty.hpp"

#include <algorithm>
#include <cmath>

namespace uvdar_core::pose_estimation::uncertainty {

PoseTangent relativePoseTangent(const CameraPose& base, const CameraPose& candidate)
{
    PoseTangent delta;
    delta.head<3>() = candidate.translation - base.translation;
    const Eigen::AngleAxisd angle_axis(candidate.rotation * base.rotation.transpose());
    if (angle_axis.angle() < 1.0e-12) {
        delta.tail<3>() = Eigen::Vector3d::Zero();
    } else {
        delta.tail<3>() = angle_axis.axis() * angle_axis.angle();
    }
    return delta;
}

double poseTangentDistance(const CameraPose& base, const CameraPose& candidate)
{
    const PoseTangent delta = relativePoseTangent(base, candidate);
    return delta.head<3>().norm() + delta.tail<3>().norm();
}

PoseCovariance covarianceFromPoseSamples(const std::vector<PoseTangent>& samples, const double scale)
{
    if (samples.empty()) {
        return PoseCovariance::Zero();
    }

    PoseTangent mean = PoseTangent::Zero();
    for (const PoseTangent& sample : samples) {
        mean += sample;
    }
    mean /= static_cast<double>(samples.size());

    PoseCovariance covariance = PoseCovariance::Zero();
    for (const PoseTangent& sample : samples) {
        const PoseTangent difference = sample - mean;
        covariance += difference * difference.transpose();
    }
    return scale * covariance;
}

Eigen::Matrix2d regularizedCovariance(const Eigen::Matrix2d& covariance, double regularization)
{
    Eigen::Matrix2d output = 0.5 * (covariance + covariance.transpose());
    if (!output.allFinite()) {
        output = Eigen::Matrix2d::Identity();
    }
    output.diagonal().array() += std::max(regularization, 1.0e-9);
    return output;
}

PoseCovariance covarianceFromInformation(const PoseCovariance& information, double eps)
{
    // Use the symmetric eigensystem instead of a direct inverse so rank-deficient
    // point sets leave unobservable pose directions with large/zero information.
    Eigen::SelfAdjointEigenSolver<PoseCovariance> solver(0.5 * (information + information.transpose()) + eps * PoseCovariance::Identity());
    if (solver.info() != Eigen::Success) {
        return PoseCovariance::Identity() * 1.0e6;
    }

    const double tolerance = std::max(eps, 1.0e-10 * solver.eigenvalues().cwiseAbs().maxCoeff());
    Eigen::Matrix<double, 6, 1> inverse_values = Eigen::Matrix<double, 6, 1>::Zero();
    for (int i = 0; i < 6; ++i) {
        if (solver.eigenvalues()(i) > tolerance) {
            inverse_values(i) = 1.0 / solver.eigenvalues()(i);
        }
    }
    return solver.eigenvectors() * inverse_values.asDiagonal() * solver.eigenvectors().transpose();
}

Eigen::Matrix<double, 2, 6> imageProjectionJacobian(
    const CameraModel& camera,
    const CameraPose& pose,
    const Eigen::Vector3d& world_point)
{
    const Eigen::Vector3d camera_point = transformPoint(pose, world_point);
    const Eigen::Matrix<double, 2, 3> project_jacobian = camera.projectionJacobian(camera_point);

    // For a left-multiplied small rotation, d(RX+t)/dtheta = -[X_c]x.
    Eigen::Matrix<double, 2, 6> jacobian;
    jacobian << project_jacobian, -project_jacobian * uvdar_core::helpers::skew(camera_point);
    return jacobian;
}

PoseCovariance poseInformationMatrixFromPixelsLinearized(
    const CameraPose& pose,
    const std::vector<Eigen::Vector3d>& world_points,
    const std::vector<Eigen::Matrix2d>& pixel_covariances,
    const CameraModel& camera,
    double covariance_regularization)
{
    PoseCovariance information = PoseCovariance::Zero();
    const std::size_t count = std::min(world_points.size(), pixel_covariances.size());
    for (std::size_t i = 0; i < count; ++i) {
        const Eigen::Matrix<double, 2, 6> jacobian = imageProjectionJacobian(camera, pose, world_points[i]);
        const Eigen::Matrix2d covariance = regularizedCovariance(pixel_covariances[i], covariance_regularization);
        // Linear Gaussian residual model: Lambda += J^T R^-1 J.
        information += jacobian.transpose() * covariance.inverse() * jacobian;
    }
    return information;
}

PoseCovariance poseCovarianceFromPixelsLinearized(
    const CameraPose& pose,
    const std::vector<Eigen::Vector3d>& world_points,
    const std::vector<Eigen::Matrix2d>& pixel_covariances,
    const CameraModel& camera,
    double covariance_regularization,
    double eps)
{
    return covarianceFromInformation(
        poseInformationMatrixFromPixelsLinearized(pose, world_points, pixel_covariances, camera, covariance_regularization),
        eps);
}

} // namespace uvdar_core::pose_estimation::uncertainty
