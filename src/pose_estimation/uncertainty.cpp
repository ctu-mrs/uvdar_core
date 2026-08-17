#include "uvdar_core/pose_estimation/uncertainty.hpp"

#include <algorithm>
#include <cmath>

namespace uvdar_core::pose_estimation::uncertainty {

Eigen::Matrix2d regularizedCovariance(const Eigen::Matrix2d& covariance, double regularization)
{
    Eigen::Matrix2d output = 0.5 * (covariance + covariance.transpose());
    if (!output.allFinite()) {
        output = Eigen::Matrix2d::Identity();
    }
    output.diagonal().array() += std::max(regularization, 1.0e-9);
    return output;
}

Eigen::Matrix<double, 6, 6> covarianceFromInformation(const Eigen::Matrix<double, 6, 6>& information, double eps, double max_variance)
{
    // Use the symmetric eigensystem instead of a direct inverse so rank-deficient
    // point sets can be handled explicitly.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(0.5 * (information + information.transpose()) + eps * Eigen::Matrix<double, 6, 6>::Identity());
    if (solver.info() != Eigen::Success) {
        return Eigen::Matrix<double, 6, 6>::Identity() * max_variance;
    }

    // A direction the LED geometry cannot constrain has near-zero information,
    // which as a covariance has to become a large variance, not a small one.
    // Truncating the inverse to zero would instead report total certainty about
    // exactly the axis that was never observed, so bound the inverse from above
    // rather than zeroing it.
    const double tolerance = std::max(eps, 1.0e-10 * solver.eigenvalues().cwiseAbs().maxCoeff());
    Eigen::Matrix<double, 6, 1> inverse_values = Eigen::Matrix<double, 6, 1>::Zero();
    for (int i = 0; i < 6; ++i) {
        inverse_values(i) = std::min(max_variance, 1.0 / std::max(solver.eigenvalues()(i), tolerance));
    }
    return solver.eigenvectors() * inverse_values.asDiagonal() * solver.eigenvectors().transpose();
}

Eigen::Matrix<double, 2, 6> imageProjectionJacobian(
    const CameraModel& camera,
    const CameraPose& pose,
    const Eigen::Vector3d& world_point)
{
    const Eigen::Vector3d rotated_point = pose.rotation * world_point;
    const Eigen::Vector3d camera_point = rotated_point + pose.translation;
    const Eigen::Matrix<double, 2, 3> project_jacobian = camera.lens->projectJacobian(camera_point);

    // The rotation tangent must match how the solver actually steps the pose:
    // GeometricSolver::refinePose applies R <- expSO3(theta) * R with t held
    // fixed, so with X_c = R X_b + t,
    //     X_c' = expSO3(theta) R X_b + t ~= X_c + theta x (R X_b)
    // and therefore d(X_c)/d(theta) = -[R X_b]x -- the rotated body point, not
    // the full camera point. Using [X_c]x here adds a spurious -[t]x term; at
    // 5 m range against a 0.28 m body that is ~19x too large, and it makes the
    // rotation columns nearly parallel to the tangential translation columns,
    // which corrupts the translation block of the marginal covariance.
    Eigen::Matrix<double, 2, 6> jacobian;
    jacobian << project_jacobian, -project_jacobian * skew(rotated_point);
    return jacobian;
}

Eigen::Matrix<double, 6, 6> poseInformationMatrixFromPixelsLinearized(
    const CameraPose& pose,
    const std::vector<Eigen::Vector3d>& world_points,
    const std::vector<Eigen::Matrix2d>& pixel_covariances,
    const CameraModel& camera,
    double covariance_regularization)
{
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
    const std::size_t count = std::min(world_points.size(), pixel_covariances.size());
    for (std::size_t i = 0; i < count; ++i) {
        const Eigen::Matrix<double, 2, 6> jacobian = imageProjectionJacobian(camera, pose, world_points[i]);
        const Eigen::Matrix2d covariance = regularizedCovariance(pixel_covariances[i], covariance_regularization);
        // Linear Gaussian residual model: Lambda += J^T R^-1 J.
        information += jacobian.transpose() * covariance.inverse() * jacobian;
    }
    return information;
}

Eigen::Matrix<double, 6, 6> poseCovarianceFromPixelsLinearized(
    const CameraPose& pose,
    const std::vector<Eigen::Vector3d>& world_points,
    const std::vector<Eigen::Matrix2d>& pixel_covariances,
    const CameraModel& camera,
    double covariance_regularization,
    double eps,
    double max_variance)
{
    return covarianceFromInformation(
        poseInformationMatrixFromPixelsLinearized(pose, world_points, pixel_covariances, camera, covariance_regularization),
        eps,
        max_variance);
}

} // namespace uvdar_core::pose_estimation::uncertainty
