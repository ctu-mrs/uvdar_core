#include "uvdar_core/pose_estimation/geometric_solver/geometric_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <limits>
#include <random>

#include "uvdar_core/pose_estimation/geometric_solver/p2p.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/p3p.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/p4p.hpp"
#include "uvdar_core/pose_estimation/uncertainty.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

namespace unc = uvdar_core::pose_estimation::uncertainty;

namespace {

constexpr double epsilon = 1.0e-12;
constexpr double ellipse_branch_distance_threshold = 1.5;
constexpr double monte_carlo_branch_distance_threshold = 2.5;
constexpr double ellipse_transform_scale = 0.5;
constexpr double fallback_covariance = 1.0e-6;
constexpr double monte_carlo_covariance_scale = 1.0;

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

        std::vector<CameraPose> refined_candidates;
        std::vector<double> refined_errors;
        refined_candidates.reserve(candidates.size());
        refined_errors.reserve(candidates.size());

        for (const CameraPose& candidate : candidates) {
            const CameraPose refined = refinePose(candidate, observations, camera);
            const double error = reprojectionError(refined, observations, camera);
            if (std::isfinite(error)) {
                refined_candidates.push_back(refined);
                refined_errors.push_back(error);
            }
        }
        if (refined_candidates.empty()) {
            continue;
        }

        const auto best_it = std::min_element(refined_errors.begin(), refined_errors.end());
        const std::size_t best_index = static_cast<std::size_t>(best_it - refined_errors.begin());
        const CameraPose best_pose = refined_candidates[best_index];

        Eigen::Matrix<double, 6, 6> covariance_camera = Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
        switch (config_.uncertainty_solver) {
            case UncertaintySolver::JacobianPropagation:
                covariance_camera = poseCovarianceByJacobianPropagation(best_pose, observations, camera);
                break;
            case UncertaintySolver::MonteCarlo:
                covariance_camera = poseCovarianceByMonteCarlo(refined_candidates, refined_errors, observations, camera, static_cast<int>(best_index));
                break;
            case UncertaintySolver::EllipseTransform:
                covariance_camera = poseCovarianceByEllipseTransform(
                    refined_candidates,
                    refined_errors,
                    observations,
                    camera,
                    static_cast<int>(best_index));
                break;
        }

        const Eigen::Matrix<double, 6, 6> covariance = rotateCovarianceToOutput(covariance_camera, camera_to_output.rotation());
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

GeometricSolver::Tangent GeometricSolver::poseTangent(const CameraPose& pose) const
{
    Tangent tangent;
    tangent.head<3>() = pose.translation;
    const Eigen::AngleAxisd angle_axis(pose.rotation);
    const double angle = angle_axis.angle();
    if (angle < epsilon) {
        tangent.tail<3>() = Eigen::Vector3d::Zero();
    } else {
        tangent.tail<3>() = angle_axis.axis() * angle;
    }
    return tangent;
}

GeometricSolver::Tangent GeometricSolver::tangentFromBase(const CameraPose& base_pose, const CameraPose& candidate_pose) const
{
    Tangent delta;
    delta.head<3>() = candidate_pose.translation - base_pose.translation;
    const Eigen::Matrix3d relative_rotation = candidate_pose.rotation * base_pose.rotation.transpose();
    const Eigen::AngleAxisd angle_axis(relative_rotation);
    const double angle = angle_axis.angle();
    if (angle < epsilon) {
        delta.tail<3>() = Eigen::Vector3d::Zero();
    } else {
        delta.tail<3>() = angle_axis.axis() * angle;
    }
    return delta;
}

Eigen::VectorXd GeometricSolver::observationVector(const std::vector<Observation>& observations) const
{
    Eigen::VectorXd output(static_cast<Eigen::Index>(observations.size() * 2U));
    for (std::size_t i = 0; i < observations.size(); ++i) {
        output(static_cast<Eigen::Index>(2 * i)) = observations[i].image_point.x();
        output(static_cast<Eigen::Index>(2 * i + 1)) = observations[i].image_point.y();
    }
    return output;
}

Eigen::MatrixXd GeometricSolver::detectorCovariance(const std::vector<Observation>& observations) const
{
    const std::size_t dimensions = observations.size() * 2U;
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(
        static_cast<Eigen::Index>(dimensions),
        static_cast<Eigen::Index>(dimensions));
    for (std::size_t i = 0; i < observations.size(); ++i) {
        const std::size_t index = 2U * i;
        const Eigen::Matrix2d point_covariance = unc::regularizedCovariance(
            observations[i].point.has_prediction ? observations[i].point.prediction_covariance : observations[i].point.covariance,
            config_.covariance_regularization_px);
        covariance.block<2, 2>(static_cast<Eigen::Index>(index), static_cast<Eigen::Index>(index)) = point_covariance;
    }
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(dimensions); ++i) {
        covariance(i, i) += config_.covariance_regularization_px;
    }
    return covariance;
}

std::vector<GeometricSolver::Observation> GeometricSolver::observationsFromVector(
    const std::vector<Observation>& base_observations,
    const Eigen::VectorXd& sample,
    const CameraModel& camera) const
{
    if (sample.size() != static_cast<Eigen::Index>(base_observations.size() * 2U)) {
        return {};
    }

    std::vector<Observation> output = base_observations;
    for (std::size_t i = 0; i < output.size(); ++i) {
        output[i].image_point = sample.segment<2>(static_cast<Eigen::Index>(2 * i));
        const Eigen::Vector3d bearing = camera.lens->backProject(output[i].image_point);
        if (!bearing.allFinite() || bearing.squaredNorm() < epsilon) {
            return {};
        }
        output[i].bearing = bearing.normalized();
    }
    return output;
}

Eigen::Matrix<double, 6, 6> GeometricSolver::covarianceFromPoseSamples(const TangentCollection& samples, double covariance_scale) const
{
    if (samples.empty()) {
        return Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
    }

    Tangent mean = Tangent::Zero();
    for (const Tangent& sample : samples) {
        mean += sample;
    }
    mean /= static_cast<double>(samples.size());

    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
    for (const Tangent& sample : samples) {
        const Tangent diff = sample - mean;
        covariance += diff * diff.transpose();
    }

    return covariance_scale * covariance;
}

Eigen::Matrix<double, 6, 6> GeometricSolver::rotateCovarianceToOutput(
    const Eigen::Matrix<double, 6, 6>& covariance,
    const Eigen::Matrix3d& camera_to_output_rotation) const
{
    Eigen::Matrix<double, 6, 6> transform = Eigen::Matrix<double, 6, 6>::Zero();
    transform.topLeftCorner<3, 3>() = camera_to_output_rotation;
    transform.bottomRightCorner<3, 3>() = camera_to_output_rotation;
    return transform * covariance * transform.transpose();
}

    Eigen::Matrix<double, 6, 6> GeometricSolver::poseCovarianceByEllipseTransform(
        const std::vector<CameraPose>& base_poses,
        const std::vector<double>&,
        const std::vector<Observation>& observations,
        const CameraModel& camera,
        int selected_index) const
{
    if (base_poses.empty() || selected_index < 0 || static_cast<std::size_t>(selected_index) >= base_poses.size()) {
        return Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
    }

    const Eigen::VectorXd mean_pixel = observationVector(observations);
    if (mean_pixel.size() == 0) {
        return Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
    }

    Eigen::MatrixXd covariance_pixel = detectorCovariance(observations);
    Eigen::LLT<Eigen::MatrixXd> cholesky(covariance_pixel);
    if (cholesky.info() != Eigen::Success) {
        return Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
    }
    const Eigen::MatrixXd cholesky_factor = cholesky.matrixL();

    const std::size_t selected_pose_index = static_cast<std::size_t>(selected_index);
    TangentCollection selected_samples;

    for (Eigen::Index axis = 0; axis < cholesky_factor.cols(); ++axis) {
        const Eigen::VectorXd perturbation = cholesky_factor.col(axis);
        const std::array<Eigen::VectorXd, 2> sigma_points{
            mean_pixel + perturbation,
            mean_pixel - perturbation,
        };

        for (const Eigen::VectorXd& sigma_point : sigma_points) {
            const auto sample_observations = observationsFromVector(observations, sigma_point, camera);
            if (sample_observations.empty()) {
                selected_samples.push_back(Tangent::Zero());
                continue;
            }

            const std::vector<CameraPose> sample_candidates = solveCameraPoses(sample_observations, camera);
            if (sample_candidates.empty()) {
                selected_samples.push_back(Tangent::Zero());
                continue;
            }

            std::vector<CameraPose> refined_candidates;
            refined_candidates.reserve(sample_candidates.size());
            for (const CameraPose& sample_candidate : sample_candidates) {
                refined_candidates.push_back(refinePose(sample_candidate, sample_observations, camera));
            }
            if (refined_candidates.empty()) {
                selected_samples.push_back(Tangent::Zero());
                continue;
            }

            const CameraPose& base_pose = base_poses[selected_pose_index];
            double best_distance = std::numeric_limits<double>::infinity();
            Tangent best_delta = Tangent::Zero();
            for (const CameraPose& candidate : refined_candidates) {
                const Tangent delta = tangentFromBase(base_pose, candidate);
                const double distance = delta.head<3>().norm() + delta.tail<3>().norm();
                if (distance < best_distance) {
                    best_distance = distance;
                    best_delta = delta;
                }
            }
            if (best_distance < ellipse_branch_distance_threshold) {
                selected_samples.push_back(best_delta);
            } else {
                selected_samples.push_back(Tangent::Zero());
            }
        }
    }

    if (selected_samples.size() < 2U) {
        return Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
    }
    return covarianceFromPoseSamples(selected_samples, ellipse_transform_scale);
}

Eigen::Matrix<double, 6, 6> GeometricSolver::poseCovarianceByMonteCarlo(
    const std::vector<CameraPose>& base_poses,
    const std::vector<double>&,
    const std::vector<Observation>& observations,
    const CameraModel& camera,
    int selected_index) const
{
    if (base_poses.empty() || selected_index < 0 || static_cast<std::size_t>(selected_index) >= base_poses.size()) {
        return Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
    }

    const Eigen::VectorXd mean_pixel = observationVector(observations);
    if (mean_pixel.size() == 0) {
        return Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
    }

    const int sample_count = std::max(1, config_.uncertainty_samples);
    if (sample_count <= 0) {
        return Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
    }
    const std::size_t selected_pose_index = static_cast<std::size_t>(selected_index);

    Eigen::MatrixXd covariance_pixel = detectorCovariance(observations);
    Eigen::LLT<Eigen::MatrixXd> cholesky(covariance_pixel);
    if (cholesky.info() != Eigen::Success) {
        return Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
    }
    const Eigen::MatrixXd cholesky_factor = cholesky.matrixL();

    const std::size_t dimension = static_cast<std::size_t>(covariance_pixel.rows());
    std::mt19937_64 random_generator(
        static_cast<std::mt19937_64::result_type>(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
    std::normal_distribution<double> normal_distribution(0.0, 1.0);

    TangentCollection selected_samples;

    for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
        Eigen::VectorXd noise(static_cast<Eigen::Index>(dimension));
        for (std::size_t i = 0; i < dimension; ++i) {
            noise(static_cast<Eigen::Index>(i)) = normal_distribution(random_generator);
        }
        const Eigen::VectorXd sample_point = mean_pixel + cholesky_factor * noise;
        const auto sample_observations = observationsFromVector(observations, sample_point, camera);
        if (sample_observations.empty()) {
            continue;
        }

        const std::vector<CameraPose> sample_candidates = solveCameraPoses(sample_observations, camera);
        if (sample_candidates.empty()) {
            continue;
        }
        std::vector<CameraPose> refined_candidates;
        refined_candidates.reserve(sample_candidates.size());
        for (const CameraPose& sample_candidate : sample_candidates) {
            refined_candidates.push_back(refinePose(sample_candidate, sample_observations, camera));
        }
        if (refined_candidates.empty()) {
            continue;
        }

        int best_branch = -1;
        double best_branch_distance = std::numeric_limits<double>::infinity();
        Tangent best_delta = Tangent::Zero();
        for (std::size_t branch_index = 0U; branch_index < base_poses.size(); ++branch_index) {
            const CameraPose& base_pose = base_poses[branch_index];
            for (const CameraPose& candidate : refined_candidates) {
                const Tangent delta = tangentFromBase(base_pose, candidate);
                const double distance = delta.head<3>().norm() + delta.tail<3>().norm();
                if (distance < best_branch_distance) {
                    best_branch_distance = distance;
                    best_delta = delta;
                    best_branch = static_cast<int>(branch_index);
                }
            }
        }
        if (best_branch == static_cast<int>(selected_pose_index) && best_branch_distance < monte_carlo_branch_distance_threshold) {
            selected_samples.push_back(best_delta);
        }
    }

    if (selected_samples.empty() || selected_samples.size() == 1U) {
        return Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
    }
    const double covariance_scale = monte_carlo_covariance_scale / static_cast<double>(selected_samples.size() - 1U);
    return covarianceFromPoseSamples(selected_samples, covariance_scale);
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

Eigen::Matrix<double, 6, 6> GeometricSolver::poseCovarianceByJacobianPropagation(
    const CameraPose& pose,
    const std::vector<Observation>& observations,
    const CameraModel& camera) const
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
    return unc::poseCovarianceFromPixelsLinearized(
        pose,
        world_points,
        pixel_covariances,
        camera,
        config_.covariance_regularization_px);
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
