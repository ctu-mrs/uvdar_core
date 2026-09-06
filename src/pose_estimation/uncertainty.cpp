#include "uvdar_core/pose_estimation/uncertainty.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>

namespace uvdar_core::pose_estimation::uncertainty {

namespace {

Eigen::Vector3d rotationLog(const Eigen::Matrix3d& rotation)
{
    const Eigen::AngleAxisd angle_axis(rotation);
    if (!std::isfinite(angle_axis.angle()) || angle_axis.angle() <= 1.0e-12) {
        return Eigen::Vector3d::Zero();
    }
    return angle_axis.axis() * angle_axis.angle();
}

} // namespace

std::optional<PoseDistributionMoments> momentMatchPoseDistribution(
    const std::vector<PoseDistributionComponent>& components)
{
    double total_weight = 0.0;
    Eigen::Matrix4d quaternion_moment = Eigen::Matrix4d::Zero();
    PoseDistributionMoments moments;
    std::vector<const PoseDistributionComponent*> valid_components;
    valid_components.reserve(components.size());
    for (const PoseDistributionComponent& component : components) {
        if (!component.pose.translation.allFinite()
            || !component.pose.rotation.allFinite()
            || !component.covariance.allFinite()
            || !std::isfinite(component.weight)
            || component.weight <= 0.0) {
            continue;
        }
        Eigen::Quaterniond quaternion(component.pose.rotation);
        if (!quaternion.coeffs().allFinite()
            || quaternion.squaredNorm() <= std::numeric_limits<double>::epsilon()) {
            continue;
        }
        quaternion.normalize();
        valid_components.push_back(&component);
        total_weight += component.weight;
        moments.pose.translation += component.weight * component.pose.translation;
        const Eigen::Vector4d coefficients = quaternion.coeffs();
        quaternion_moment += component.weight
            * coefficients * coefficients.transpose();
    }
    if (!(total_weight > 1.0e-12) || !std::isfinite(total_weight)) {
        return std::nullopt;
    }
    moments.pose.translation /= total_weight;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> quaternion_solver(
        quaternion_moment);
    if (quaternion_solver.info() != Eigen::Success) {
        return std::nullopt;
    }
    Eigen::Quaterniond mean_quaternion;
    mean_quaternion.coeffs() = quaternion_solver.eigenvectors().col(3);
    if (mean_quaternion.w() < 0.0) {
        mean_quaternion.coeffs() *= -1.0;
    }
    if (!mean_quaternion.coeffs().allFinite()
        || mean_quaternion.squaredNorm() <= std::numeric_limits<double>::epsilon()) {
        return std::nullopt;
    }
    mean_quaternion.normalize();
    moments.pose.rotation = mean_quaternion.toRotationMatrix();

    // All increments are left tangents expressed in the same destination
    // frame, so component covariances can be combined without a basis change.
    for (int iteration = 0; iteration < 20; ++iteration) {
        Eigen::Vector3d increment = Eigen::Vector3d::Zero();
        for (const PoseDistributionComponent* component : valid_components) {
            increment += component->weight * rotationLog(
                component->pose.rotation * moments.pose.rotation.transpose());
        }
        increment /= total_weight;
        if (!increment.allFinite()) {
            return std::nullopt;
        }
        if (increment.norm() <= 1.0e-11) {
            break;
        }
        moments.pose.rotation = uvdar_core::helpers::expSO3(increment)
            * moments.pose.rotation;
    }

    PoseTangent tangent_mean = PoseTangent::Zero();
    for (const PoseDistributionComponent* component : valid_components) {
        tangent_mean += component->weight
            * relativePoseTangent(moments.pose, component->pose);
    }
    tangent_mean /= total_weight;
    for (const PoseDistributionComponent* component : valid_components) {
        const PoseTangent offset =
            relativePoseTangent(moments.pose, component->pose) - tangent_mean;
        moments.covariance += component->weight
            * (component->covariance + offset * offset.transpose());
    }
    moments.covariance /= total_weight;
    moments.covariance = 0.5
        * (moments.covariance + moments.covariance.transpose());
    if (!moments.pose.translation.allFinite()
        || !moments.pose.rotation.allFinite()
        || !moments.covariance.allFinite()) {
        return std::nullopt;
    }
    return moments;
}

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
    const Eigen::Vector3d rotated_point = pose.rotation * world_point;
    const Eigen::Matrix<double, 2, 3> project_jacobian = camera.projectionJacobian(camera_point);

    // applyLeftCameraPoseIncrement uses R' = Exp(theta)R and t' = t, hence
    // d(RX+t)/dtheta = -[RX]x (not -[RX+t]x).
    Eigen::Matrix<double, 2, 6> jacobian;
    jacobian << project_jacobian, -project_jacobian * uvdar_core::helpers::skew(rotated_point);
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
