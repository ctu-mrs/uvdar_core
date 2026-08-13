#include "uvdar_core/pose_estimation/geometric_solver/geometric_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "uvdar_core/pose_estimation/geometric_solver/p2p.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/p3p.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/p4p.hpp"
#include "uvdar_core/pose_estimation/uncertainty.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

namespace unc = uvdar_core::pose_estimation::uncertainty;

namespace {

constexpr double epsilon = 1.0e-12;

} // namespace

GeometricSolver::GeometricSolver(GeometricSolverConfig config, BodyModel body, std::vector<CameraModel> cameras)
    : config_(std::move(config))
    , body_(std::move(body))
    , cameras_(std::move(cameras))
{
    latest_measurements_.frame_id = config_.output_frame;
}

void GeometricSolver::processFrame(
    std::size_t camera_index,
    const std::vector<TrackedPoint>& points,
    int,
    int,
    double stamp,
    const Eigen::Isometry3d& camera_to_output,
    const Eigen::Isometry3d&)
{
    if (camera_index >= cameras_.size() || !cameras_[camera_index].lens) {
        return;
    }

    const CameraModel& camera = cameras_[camera_index];
    std::map<int, std::vector<Observation>> by_target;
    for (const TrackedPoint& point : points) {
        if (point.id < 0) {
            continue;
        }
        const int target = classifyMatch(point.id);
        if (target < 0) {
            continue;
        }
        if (auto observation = makeObservation(point, camera); observation) {
            by_target[target].push_back(*observation);
        }
    }

    TimedPoseMeasurements measurements;
    measurements.stamp = stamp;
    measurements.frame_id = config_.output_frame;

    for (const auto& [target, observations] : by_target) {
        if (observations.size() < 2U) {
            continue;
        }

        // Minimal solvers can return multiple algebraic candidates. Rank them
        // after pixel-space refinement by the same reprojection objective.
        const std::vector<CameraPose> candidates = solveCameraPoses(observations, camera);
        if (candidates.empty()) {
            continue;
        }

        double best_error = std::numeric_limits<double>::infinity();
        CameraPose best_pose;
        bool found = false;
        for (const CameraPose& candidate : candidates) {
            const CameraPose refined = refinePose(candidate, observations, camera);
            const double error = reprojectionError(refined, observations, camera);
            if (std::isfinite(error) && error < best_error) {
                best_error = error;
                best_pose = refined;
                found = true;
            }
        }
        if (!found) {
            continue;
        }

        const Eigen::Matrix<double, 6, 6> covariance = poseCovariance(best_pose, observations, camera, camera_to_output.rotation());
        measurements.poses.push_back(toMeasurement(target, best_pose, camera_to_output, covariance));
    }

    std::scoped_lock lock(mutex_);
    latest_measurements_ = std::move(measurements);
}

TimedPoseMeasurements GeometricSolver::scatterAndMeasure(double, double stamp)
{
    std::scoped_lock lock(mutex_);
    if (latest_measurements_.stamp <= 0.0) {
        latest_measurements_.stamp = stamp;
    }
    return latest_measurements_;
}

std::vector<PoseMeasurement> GeometricSolver::verifiedHypotheses() const
{
    std::scoped_lock lock(mutex_);
    return latest_measurements_.poses;
}

std::vector<PoseMeasurement> GeometricSolver::tentativeHypotheses() const
{
    return {};
}

std::optional<GeometricSolver::Observation> GeometricSolver::makeObservation(const TrackedPoint& point, const CameraModel& camera) const
{
    const int local_signal = point.id % config_.signals_per_target;
    const auto marker = markerForSignal(local_signal);
    if (!marker) {
        return std::nullopt;
    }

    Observation observation;
    observation.point = point;
    observation.marker = *marker;
    // Use tracker prediction when available; its covariance is propagated later.
    observation.image_point = point.has_prediction ? point.predicted_position : Eigen::Vector2d(point.x, point.y);
    observation.bearing = camera.lens->backProject(observation.image_point).normalized();
    if (!observation.bearing.allFinite() || observation.bearing.norm() < epsilon) {
        return std::nullopt;
    }
    return observation;
}

std::vector<GeometricSolver::CameraPose> GeometricSolver::solveCameraPoses(const std::vector<Observation>& observations, const CameraModel& camera) const
{
    if (observations.size() == 2U) {
        return solveP2P(observations, camera);
    }
    if (observations.size() == 3U) {
        return solveP3P(observations);
    }
    if (observations.size() == 4U) {
        std::vector<CameraPose> poses = solveP4P(observations);
        if (!poses.empty()) {
            return poses;
        }
    }
    // General PnP handles five-or-more points, and is also a fallback when P4P
    // rejects all algebraic candidates.
    if (auto pose = solvePnP(observations); pose) {
        return {*pose};
    }
    return {};
}

std::vector<GeometricSolver::CameraPose> GeometricSolver::solveP2P(const std::vector<Observation>& observations, const CameraModel&) const
{
    Eigen::Matrix<double, 3, 2> pw;
    Eigen::Matrix<double, 3, 2> pi;
    for (int i = 0; i < 2; ++i) {
        pw.col(i) = observations[static_cast<std::size_t>(i)].marker.pose.position;
        pi.col(i) = observations[static_cast<std::size_t>(i)].bearing;
    }

    // Two correspondences define a camera ray plane. The third body marker
    // supplies the matching world-plane normal needed to resolve P2P.
    Eigen::Vector3d v_cam = pi.col(0).cross(pi.col(1));
    if (v_cam.squaredNorm() < 1.0e-18) {
        return {};
    }
    v_cam.normalize();

    Eigen::Vector3d baseline = pw.col(1) - pw.col(0);
    Eigen::Vector3d v_world = Eigen::Vector3d::UnitY();
    for (const LEDMarker& marker : body_) {
        if ((marker.pose.position - pw.col(0)).norm() > 1.0e-9 && (marker.pose.position - pw.col(1)).norm() > 1.0e-9) {
            v_world = -baseline.cross(marker.pose.position - pw.col(0));
            break;
        }
    }
    if (v_world.squaredNorm() < 1.0e-18) {
        v_world = Eigen::Vector3d::UnitY();
    }
    v_world.normalize();

    const auto solutions = P2P::solve(pw, pi, v_cam, v_world, P2P::Method::Sweeney);
    std::vector<CameraPose> output;
    output.reserve(solutions.size());
    for (const auto& solution : solutions) {
        output.push_back({solution.R, solution.t});
    }
    return output;
}

std::vector<GeometricSolver::CameraPose> GeometricSolver::solveP3P(const std::vector<Observation>& observations) const
{
    Eigen::Matrix3d pw;
    Eigen::Matrix3d pi;
    for (int i = 0; i < 3; ++i) {
        pw.col(i) = observations[static_cast<std::size_t>(i)].marker.pose.position;
        pi.col(i) = observations[static_cast<std::size_t>(i)].bearing;
    }
    std::vector<CameraPose> output;
    for (const auto& solution : P3P::solve(pw, pi, 1)) {
        output.push_back({solution.R, solution.t});
    }
    return output;
}

std::vector<GeometricSolver::CameraPose> GeometricSolver::solveP4P(const std::vector<Observation>& observations) const
{
    Eigen::Matrix<double, 3, 4> pw;
    Eigen::Matrix<double, 3, 4> pi;
    for (int i = 0; i < 4; ++i) {
        pw.col(i) = observations[static_cast<std::size_t>(i)].marker.pose.position;
        pi.col(i) = observations[static_cast<std::size_t>(i)].bearing;
    }
    std::vector<CameraPose> output;
    for (const auto& solution : P4P::solve(pw, pi, config_.p4p_reprojection_threshold_rad)) {
        output.push_back({solution.R, solution.t});
    }
    return output;
}

std::optional<GeometricSolver::CameraPose> GeometricSolver::solvePnP(const std::vector<Observation>& observations) const
{
    if (observations.size() < 3U) {
        return std::nullopt;
    }

    std::optional<CameraPose> seed;
    if (observations.size() >= 3U) {
        // seed iterative PnP from the deterministic first P3P triplet when possible.
        const std::vector<Observation> first_triplet(observations.begin(), observations.begin() + 3);
        const std::vector<CameraPose> candidates = solveP3P(first_triplet);
        double best_cost = std::numeric_limits<double>::infinity();
        for (const CameraPose& candidate : candidates) {
            if (auto residual = bearingResidualVector(observations, candidate); residual) {
                const double cost = residual->squaredNorm() / static_cast<double>(observations.size());
                if (std::isfinite(cost) && cost < best_cost) {
                    best_cost = cost;
                    seed = candidate;
                }
            }
        }
    }

    CameraPose pose;
    if (seed) {
        pose = *seed;
    } else {
        pose.rotation = Eigen::Matrix3d::Identity();
        pose.translation = Eigen::Vector3d(0.0, 0.0, 3.0);
    }

    for (int iteration = 0; iteration < config_.pnp_max_iterations; ++iteration) {
        const auto residual = bearingResidualVector(observations, pose);
        if (!residual) {
            break;
        }
        if (residual->norm() / static_cast<double>(observations.size()) < config_.pnp_residual_tolerance) {
            break;
        }

        // Finite-difference LM on tangent [omega, translation]
        Eigen::MatrixXd jacobian(residual->size(), 6);
        for (int parameter = 0; parameter < 6; ++parameter) {
            Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
            delta(parameter) = config_.pnp_finite_difference_eps;

            CameraPose perturbed = pose;
            if (parameter < 3) {
                perturbed.rotation = expSO3(delta.head<3>()) * pose.rotation;
            } else {
                perturbed.translation(parameter - 3) += config_.pnp_finite_difference_eps;
            }

            const auto perturbed_residual = bearingResidualVector(observations, perturbed);
            jacobian.col(parameter) = ((perturbed_residual ? *perturbed_residual : *residual) - *residual) / config_.pnp_finite_difference_eps;
        }

        const Eigen::MatrixXd hessian = jacobian.transpose() * jacobian + config_.pnp_damping * Eigen::Matrix<double, 6, 6>::Identity();
        const Eigen::VectorXd gradient = jacobian.transpose() * *residual;
        const Eigen::VectorXd step = -hessian.ldlt().solve(gradient);
        if (!step.allFinite() || step.norm() < config_.pnp_step_tolerance) {
            break;
        }

        // Left-multiplicative SO(3) update, additive translation update.
        pose.rotation = expSO3(step.head<3>()) * pose.rotation;
        pose.translation += step.tail<3>();
    }

    return pose;
}

std::optional<Eigen::VectorXd> GeometricSolver::bearingResidualVector(const std::vector<Observation>& observations, const CameraPose& pose) const
{
    Eigen::VectorXd residual(static_cast<int>(observations.size() * 3U));
    for (std::size_t i = 0; i < observations.size(); ++i) {
        const Eigen::Vector3d predicted_raw = pose.rotation * observations[i].marker.pose.position + pose.translation;
        const double predicted_norm = predicted_raw.norm();
        if (predicted_norm < epsilon) {
            return std::nullopt;
        }
        const Eigen::Vector3d predicted = predicted_raw / predicted_norm;
        const Eigen::Vector3d measured = observations[i].bearing.normalized();
        // Bearing residual lives on the embedded unit sphere in R^3.
        residual.segment<3>(static_cast<int>(3U * i)) = predicted - measured;
    }
    return residual;
}

GeometricSolver::CameraPose GeometricSolver::refinePose(const CameraPose& seed, const std::vector<Observation>& observations, const CameraModel& camera) const
{
    CameraPose pose = seed;
    for (int iteration = 0; iteration < config_.refinement_iterations; ++iteration) {
        // Weighted Gauss-Newton normal equations:
        // (sum J^T R^-1 J) delta = sum J^T R^-1 r.
        Eigen::MatrixXd normal = Eigen::MatrixXd::Zero(6, 6);
        Eigen::VectorXd rhs = Eigen::VectorXd::Zero(6);
        for (const Observation& observation : observations) {
            const Eigen::Vector3d camera_point = pose.rotation * observation.marker.pose.position + pose.translation;
            const Eigen::Vector2d residual = observation.image_point - camera.lens->project(camera_point);
            const Eigen::Matrix<double, 2, 6> jacobian = unc::imageProjectionJacobian(camera, pose, observation.marker.pose.position);
            const Eigen::Matrix2d covariance = unc::regularizedCovariance(
                observation.point.has_prediction ? observation.point.prediction_covariance : observation.point.covariance,
                config_.covariance_regularization_px);
            const Eigen::Matrix2d information = covariance.inverse();
            normal += jacobian.transpose() * information * jacobian;
            rhs += jacobian.transpose() * information * residual;
        }

        const Eigen::VectorXd delta = normal.ldlt().solve(rhs);
        if (!delta.allFinite() || delta.norm() < 1.0e-10) {
            break;
        }
        pose.translation += delta.head<3>();
        pose.rotation = expSO3(delta.tail<3>()) * pose.rotation;
    }
    return pose;
}

double GeometricSolver::reprojectionError(const CameraPose& pose, const std::vector<Observation>& observations, const CameraModel& camera) const
{
    double error = 0.0;
    for (const Observation& observation : observations) {
        const Eigen::Vector3d camera_point = pose.rotation * observation.marker.pose.position + pose.translation;
        if (camera_point.z() <= epsilon) {
            return std::numeric_limits<double>::infinity();
        }
        error += (camera.lens->project(camera_point) - observation.image_point).squaredNorm();
    }
    return error;
}

Eigen::Matrix<double, 6, 6> GeometricSolver::poseCovariance(
    const CameraPose& pose,
    const std::vector<Observation>& observations,
    const CameraModel& camera,
    const Eigen::Matrix3d& camera_to_output_rotation) const
{
    std::vector<Eigen::Vector3d> world_points;
    std::vector<Eigen::Matrix2d> pixel_covariances;
    world_points.reserve(observations.size());
    pixel_covariances.reserve(observations.size());
    for (const Observation& observation : observations) {
        world_points.push_back(observation.marker.pose.position);
        pixel_covariances.push_back(observation.point.has_prediction ? observation.point.prediction_covariance : observation.point.covariance);
    }

    // Linearize each pixel residual around the refined pose and invert the
    // accumulated Fisher information matrix.
    Eigen::Matrix<double, 6, 6> covariance_camera = unc::poseCovarianceFromPixelsLinearized(
        pose,
        world_points,
        pixel_covariances,
        camera,
        config_.covariance_regularization_px);
    // Rotate both position and small-angle covariance blocks into output frame.
    Eigen::Matrix<double, 6, 6> transform = Eigen::Matrix<double, 6, 6>::Zero();
    transform.topLeftCorner<3, 3>() = camera_to_output_rotation;
    transform.bottomRightCorner<3, 3>() = camera_to_output_rotation;
    return transform * covariance_camera * transform.transpose();
}

PoseMeasurement GeometricSolver::toMeasurement(
    int target,
    const CameraPose& camera_pose,
    const Eigen::Isometry3d& camera_to_output,
    const Eigen::Matrix<double, 6, 6>& covariance) const
{
    Eigen::Isometry3d body_to_camera = Eigen::Isometry3d::Identity();
    body_to_camera.linear() = camera_pose.rotation;
    body_to_camera.translation() = camera_pose.translation;
    const Eigen::Isometry3d body_to_output = camera_to_output * body_to_camera;

    PoseMeasurement measurement;
    measurement.id = target;
    measurement.pose.position = body_to_output.translation();
    measurement.pose.orientation = Eigen::Quaterniond(body_to_output.rotation()).normalized();
    measurement.covariance = covariance;
    return measurement;
}

int GeometricSolver::classifyMatch(int signal_id) const
{
    if (std::find(config_.signal_ids.begin(), config_.signal_ids.end(), signal_id) == config_.signal_ids.end()) {
        return -1;
    }
    return signal_id / std::max(1, config_.signals_per_target);
}

std::optional<LEDMarker> GeometricSolver::markerForSignal(int signal_id) const
{
    for (const LEDMarker& marker : body_) {
        if (marker.signal_id == signal_id) {
            return marker;
        }
    }
    return std::nullopt;
}

} // namespace uvdar_core::pose_estimation::geometric_solver
