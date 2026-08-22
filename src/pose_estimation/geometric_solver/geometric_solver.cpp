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
#include "uvdar_core/pose_estimation/geometric_solver/pnp.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/gp3p.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/gp4_5p.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/gp6p.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/gpnp.hpp"
// Known-axis equations are isolated from the central P2P implementation.
#include "uvdar_core/pose_estimation/geometric_solver/generalized_known_axis.hpp"
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
constexpr double observability_score_tolerance = 1.0e-6;

} // namespace

GeometricSolver::GeometricSolver(GeometricSolverConfig config, BodyModel body, std::vector<CameraModel> cameras)
    : config_(std::move(config))
    , body_(std::move(body))
    , cameras_(std::move(cameras))
{
    latest_measurements_.frame_id = config_.output_frame;
    camera_up_axes_.resize(cameras_.size());
    latest_camera_frames_.resize(cameras_.size());
    latest_input_measurements_.resize(cameras_.size());
    for (TimedPoseMeasurements& measurements : latest_input_measurements_) {
        measurements.frame_id = config_.output_frame;
    }
}

void GeometricSolver::setCameraUpAxis(
    const std::size_t camera_index,
    std::optional<Eigen::Vector3d> camera_up_axis)
{
    if (camera_index >= camera_up_axes_.size()) {
        return;
    }
    if (camera_up_axis
        && (!camera_up_axis->allFinite() || camera_up_axis->squaredNorm() < epsilon)) {
        camera_up_axis = std::nullopt;
    }
    if (camera_up_axis) {
        camera_up_axis->normalize();
    }

    std::scoped_lock lock(mutex_);
    camera_up_axes_[camera_index] = std::move(camera_up_axis);
}

void GeometricSolver::processFrame(
    std::size_t camera_index,
    const std::vector<TrackedPoint>& points,
    int,
    int,
    double stamp,
    const Eigen::Isometry3d& camera_to_output,
    const Eigen::Isometry3d& output_to_camera)
{
    if (camera_index >= cameras_.size() || !cameras_[camera_index].valid()) {
        return;
    }

    const CameraModel& camera = cameras_[camera_index];
    std::optional<Eigen::Vector3d> camera_up_axis;
    {
        std::scoped_lock lock(mutex_);
        camera_up_axis = camera_up_axes_[camera_index];
    }
    std::map<int, std::vector<Observation>> by_target;
    for (const TrackedPoint& point : points) {
        if (point.id < 0) {
            continue;
        }
        const int target = targetForSignal(config_.signal_ids, config_.signals_per_target, point.id);
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
    std::map<std::pair<std::size_t, int>, CameraPose> updated_camera_poses;
    std::map<int, CameraPose> updated_rig_poses;

    std::vector<BufferedCameraFrame> rig_candidate_frames;
    if (config_.multi_cam_rig_enable) {
        BufferedCameraFrame current_frame;
        current_frame.valid = true;
        current_frame.camera_index = camera_index;
        current_frame.stamp = stamp;
        current_frame.camera_to_output = camera_to_output;
        current_frame.output_to_camera = output_to_camera;
        current_frame.camera_up_axis = camera_up_axis;
        current_frame.observations_by_target = by_target;

        std::scoped_lock lock(mutex_);
        latest_camera_frames_[camera_index] = std::move(current_frame);
        rig_candidate_frames.reserve(latest_camera_frames_.size());
        for (const BufferedCameraFrame& frame : latest_camera_frames_) {
            if (frame.valid) {
                // Per-observation filtering below keeps real detections inside
                // the synchronization interval while allowing tracker
                // virtual predictions to bridge asynchronous camera gaps.
                rig_candidate_frames.push_back(frame);
            }
        }
    }

    for (const auto& [target, observations] : by_target) {
        bool rig_solution_published = false;
        if (config_.multi_cam_rig_enable) {
            const std::vector<RigObservation> rig_observations = makeRigObservations(
                target, rig_candidate_frames, stamp);
            std::vector<bool> represented_cameras(cameras_.size(), false);
            std::size_t represented_camera_count = 0U;
            for (const RigObservation& observation : rig_observations) {
                if (!represented_cameras[observation.camera_index]) {
                    represented_cameras[observation.camera_index] = true;
                    ++represented_camera_count;
                }
            }

            const bool standard_rig_geometry = rig_observations.size() >= 3U
                && hasObservableRigMarkerGeometry(rig_observations);
            const bool known_axis_rig_geometry = !standard_rig_geometry
                && config_.odometry_ref_enable
                && rig_observations.size() >= 2U
                && hasDistinctRigMarkerPair(rig_observations)
                && rigOutputUpAxis(rig_observations).has_value();
            if (represented_camera_count >= static_cast<std::size_t>(config_.multi_cam_min_cameras)
                && (standard_rig_geometry || known_axis_rig_geometry)) {
                std::string rig_method;
                const std::vector<ScoredRigPose> refined_rig_candidates = refineRigCandidates(
                    solveRigPoses(rig_observations, &rig_method), rig_observations);
                std::optional<CameraPose> previous_rig_pose;
                {
                    std::scoped_lock lock(mutex_);
                    if (const auto previous = latest_rig_poses_.find(target);
                        previous != latest_rig_poses_.end()) {
                        previous_rig_pose = previous->second;
                    }
                }
                const auto selected_rig_index = selectRigCandidate(
                    refined_rig_candidates, rig_observations, previous_rig_pose);
                if (selected_rig_index) {
                    const CameraPose best_pose = refined_rig_candidates[*selected_rig_index].pose;
                    std::vector<CameraPose> base_poses;
                    base_poses.reserve(refined_rig_candidates.size());
                    for (const ScoredRigPose& candidate : refined_rig_candidates) {
                        base_poses.push_back(candidate.pose);
                    }

                    Eigen::Matrix<double, 6, 6> covariance =
                        Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
                    switch (config_.uncertainty_solver) {
                        case UncertaintySolver::JacobianPropagation:
                            covariance = poseCovarianceByRigJacobianPropagation(
                                best_pose, rig_observations);
                            break;
                        case UncertaintySolver::MonteCarlo:
                            covariance = poseCovarianceByRigMonteCarlo(
                                base_poses,
                                rig_observations,
                                static_cast<int>(*selected_rig_index));
                            break;
                        case UncertaintySolver::EllipseTransform:
                            covariance = poseCovarianceByRigEllipseTransform(
                                base_poses,
                                rig_observations,
                                static_cast<int>(*selected_rig_index));
                            break;
                    }
                    measurements.poses.push_back(toRigMeasurement(
                        target,
                        best_pose,
                        covariance,
                        rig_method));
                    updated_rig_poses.emplace(target, best_pose);
                    rig_solution_published = true;
                }
            }
        }

        // A synchronized generalized solve is used only when at least two
        // cameras contribute a valid observable ray set. Otherwise retain the
        // legacy solve for the camera that triggered this callback.
        if (rig_solution_published || observations.size() < 2U) {
            continue;
        }

        // Minimal solvers can return multiple algebraic candidates.  Keep them
        // through refinement, then let the body model resolve branches from
        // configured LED visibility instead of numerical residual tie-breaks.
        std::string method;
        const std::vector<ScoredCameraPose> refined_candidates = refineCandidates(
            solveCameraPoses(observations, camera, camera_up_axis, &method),
            observations,
            camera);
        if (refined_candidates.empty()) {
            continue;
        }

        const std::pair<std::size_t, int> pose_key{camera_index, target};
        std::optional<CameraPose> previous_pose;
        {
            std::scoped_lock lock(mutex_);
            if (const auto previous = latest_camera_poses_.find(pose_key); previous != latest_camera_poses_.end()) {
                previous_pose = previous->second;
            }
        }
        const auto selected_index = selectObservabilityAwareCandidate(refined_candidates, observations, previous_pose);
        if (!selected_index) {
            continue;
        }
        const std::size_t best_index = *selected_index;
        const CameraPose best_pose = refined_candidates[best_index].pose;
        std::vector<CameraPose> base_poses;
        base_poses.reserve(refined_candidates.size());
        for (const ScoredCameraPose& candidate : refined_candidates) {
            base_poses.push_back(candidate.pose);
        }

        Eigen::Matrix<double, 6, 6> covariance_camera = Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
        switch (config_.uncertainty_solver) {
            case UncertaintySolver::JacobianPropagation:
                covariance_camera = poseCovarianceByJacobianPropagation(
                    best_pose, observations, camera, camera_up_axis);
                break;
            case UncertaintySolver::MonteCarlo:
                covariance_camera = poseCovarianceByMonteCarlo(
                    base_poses,
                    observations,
                    camera,
                    camera_up_axis,
                    static_cast<int>(best_index));
                break;
            case UncertaintySolver::EllipseTransform:
                covariance_camera = poseCovarianceByEllipseTransform(
                    base_poses,
                    observations,
                    camera,
                    camera_up_axis,
                    static_cast<int>(best_index));
                break;
        }

        const Eigen::Matrix<double, 6, 6> covariance = uvdar_core::helpers::rotatePoseCovariance(
            covariance_camera,
            camera_to_output.rotation());
        measurements.poses.push_back(toMeasurement(
            target, best_pose, camera_to_output, covariance, method));
        updated_camera_poses.emplace(pose_key, best_pose);
    }

    std::scoped_lock lock(mutex_);
    latest_input_measurements_[camera_index] = std::move(measurements);

    // Merge independent input batches without publishing duplicate body IDs.
    // A camera clears only its own previous batch.  For each body, the newest
    // valid input wins until that input supplies its next frame, eliminating
    // callback ordering as a source of dropped measurements.
    TimedPoseMeasurements merged_measurements;
    merged_measurements.frame_id = config_.output_frame;
    std::map<int, std::pair<double, PoseMeasurement>> freshest_by_target;
    for (const TimedPoseMeasurements& input_measurements : latest_input_measurements_) {
        merged_measurements.stamp = std::max(
            merged_measurements.stamp, input_measurements.stamp);
        for (const PoseMeasurement& measurement : input_measurements.poses) {
            const auto existing = freshest_by_target.find(measurement.id);
            if (existing == freshest_by_target.end()
                || input_measurements.stamp >= existing->second.first) {
                freshest_by_target[measurement.id] = {
                    input_measurements.stamp, measurement};
            }
        }
    }
    merged_measurements.poses.reserve(freshest_by_target.size());
    for (const auto& [target, stamped_measurement] : freshest_by_target) {
        (void)target;
        merged_measurements.poses.push_back(stamped_measurement.second);
    }
    latest_measurements_ = std::move(merged_measurements);
    for (const auto& [key, pose] : updated_camera_poses) {
        latest_camera_poses_[key] = pose;
    }
    for (const auto& [target, pose] : updated_rig_poses) {
        latest_rig_poses_[target] = pose;
    }
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
    const auto marker = body_.markerForSignal(local_signal);
    if (!marker) {
        return std::nullopt;
    }

    Observation observation;
    observation.point = point;
    observation.marker = *marker;
    // Use tracker prediction when available; its covariance is propagated later.
    observation.image_point = point.has_prediction ? point.predicted_position : Eigen::Vector2d(point.x, point.y);
    const auto bearing = camera.bearingForPixel(observation.image_point);
    if (!bearing) {
        return std::nullopt;
    }
    observation.bearing = *bearing;
    return observation;
}

std::vector<GeometricSolver::CameraPose> GeometricSolver::solveCameraPoses(
    const std::vector<Observation>& observations,
    const CameraModel& camera,
    const std::optional<Eigen::Vector3d>& camera_up_axis,
    std::string* method) const
{
    if (observations.size() == 2U) {
        if (method != nullptr) {
            *method = "P2P";
        }
        return config_.odometry_ref_enable ? solveP2P(observations, camera_up_axis) : std::vector<CameraPose> {};
    }
    if (observations.size() == 3U) {
        if (method != nullptr) {
            *method = "P3P";
        }
        return solveP3P(observations);
    }
    if (observations.size() == 4U) {
        std::vector<CameraPose> poses = solveP4P(observations);
        if (!poses.empty()) {
            if (method != nullptr) {
                *method = "P4P";
            }
            return poses;
        }
    }
    // General PnP handles five-or-more points, and is also a fallback when P4P
    // rejects all algebraic candidates.
    if (method != nullptr) {
        *method = "PnP";
    }
    PnP::PointMatrix pw(3, static_cast<Eigen::Index>(observations.size()));
    PnP::PointMatrix pi(3, static_cast<Eigen::Index>(observations.size()));
    for (std::size_t i = 0; i < observations.size(); ++i) {
        pw.col(static_cast<Eigen::Index>(i)) = observations[i].marker.pose.position;
        pi.col(static_cast<Eigen::Index>(i)) = observations[i].bearing;
    }
    PnP::Options options;
    options.max_iterations = config_.pnp_max_iterations;
    options.damping = config_.pnp_damping;
    options.finite_difference_epsilon = config_.pnp_finite_difference_eps;
    options.step_tolerance = config_.pnp_step_tolerance;
    options.residual_tolerance = config_.pnp_residual_tolerance;
    options.p4p_reprojection_threshold_rad = config_.p4p_reprojection_threshold_rad;

    std::vector<CameraPose> poses;
    for (const PnP::Solution& solution : PnP::solve(pw, pi, options)) {
        poses.push_back(toCameraPose(solution));
    }
    return poses;
}

std::vector<GeometricSolver::ScoredCameraPose> GeometricSolver::refineCandidates(
    const std::vector<CameraPose>& candidates,
    const std::vector<Observation>& observations,
    const CameraModel& camera) const
{
    std::vector<ScoredCameraPose> refined_candidates;
    refined_candidates.reserve(candidates.size());
    for (const CameraPose& candidate : candidates) {
        // Two image points provide only four constraints.  Refining their P2P
        // solution with an unconstrained six-DOF optimizer would discard the
        // navigation-derived up-axis constraint that made it observable.
        CameraPose refined_pose = observations.size() == 2U ? candidate : refinePose(candidate, observations, camera);
        const double error = reprojectionError(refined_pose, observations, camera);
        if (std::isfinite(error)) {
            refined_candidates.push_back({std::move(refined_pose), error});
        }
    }
    return refined_candidates;
}

std::vector<GeometricSolver::RigObservation> GeometricSolver::makeRigObservations(
    const int target,
    const std::vector<BufferedCameraFrame>& frames,
    const double reference_stamp) const
{
    std::vector<RigObservation> output;
    for (const BufferedCameraFrame& frame : frames) {
        if (frame.camera_index >= cameras_.size()) {
            continue;
        }
        const auto target_observations = frame.observations_by_target.find(target);
        if (target_observations == frame.observations_by_target.end()) {
            continue;
        }
        for (const Observation& observation : target_observations->second) {
            const bool synchronized = std::abs(frame.stamp - reference_stamp)
                <= config_.multi_cam_sync_tolerance_sec;
            const bool usable_virtual_prediction = observation.point.virtual_point
                && observation.point.has_prediction
                && observation.point.predicted_position.allFinite()
                && observation.point.prediction_covariance.allFinite();
            if (!synchronized && !usable_virtual_prediction) {
                continue;
            }
            RigObservation rig_observation;
            rig_observation.observation = observation;
            rig_observation.camera_index = frame.camera_index;
            rig_observation.camera_to_output = frame.camera_to_output;
            rig_observation.output_to_camera = frame.output_to_camera;
            rig_observation.ray_origin = frame.camera_to_output.translation();
            rig_observation.ray_direction =
                frame.camera_to_output.rotation() * observation.bearing;
            if (frame.camera_up_axis) {
                rig_observation.output_up_axis =
                    frame.camera_to_output.rotation() * *frame.camera_up_axis;
                if (!rig_observation.output_up_axis->allFinite()
                    || rig_observation.output_up_axis->squaredNorm() <= epsilon) {
                    rig_observation.output_up_axis = std::nullopt;
                } else {
                    rig_observation.output_up_axis->normalize();
                }
            }
            if (rig_observation.ray_origin.allFinite()
                && rig_observation.ray_direction.allFinite()
                && rig_observation.ray_direction.squaredNorm() > epsilon) {
                rig_observation.ray_direction.normalize();
                output.push_back(std::move(rig_observation));
            }
        }
    }
    return output;
}

std::vector<GeometricSolver::CameraPose> GeometricSolver::solveRigPoses(
    const std::vector<RigObservation>& observations,
    std::string* method) const
{
    if (observations.size() < 2U) {
        return {};
    }
    GeneralizedPointMatrix world_points(3, static_cast<Eigen::Index>(observations.size()));
    GeneralizedPointMatrix ray_origins(3, static_cast<Eigen::Index>(observations.size()));
    GeneralizedPointMatrix ray_directions(3, static_cast<Eigen::Index>(observations.size()));
    for (std::size_t i = 0U; i < observations.size(); ++i) {
        const Eigen::Index column = static_cast<Eigen::Index>(i);
        world_points.col(column) = observations[i].observation.marker.pose.position;
        ray_origins.col(column) = observations[i].ray_origin;
        ray_directions.col(column) = observations[i].ray_direction;
    }

    GeneralizedSolverOptions options;
    options.max_iterations = config_.multi_cam_refinement_iterations;
    options.damping = config_.multi_cam_damping;
    options.step_tolerance = config_.multi_cam_step_tolerance;
    options.residual_tolerance = config_.multi_cam_residual_tolerance;
    options.maximum_angular_error_rad = config_.multi_cam_max_angular_error_rad;
    options.gp3p_depth_seed_levels = config_.multi_cam_gp3p_depth_seed_levels;
    options.gp3p_depth_iterations = config_.multi_cam_gp3p_depth_iterations;
    options.gp3p_root_tolerance = config_.multi_cam_gp3p_root_tolerance;

    auto signatureForCount = [](const std::size_t count) {
        if (count == 2U) {
            return std::string("gP2P");
        }
        if (count == 3U) {
            return std::string("gP3P");
        }
        if (count == 4U) {
            return std::string("gP4P");
        }
        if (count == 5U) {
            return std::string("gP5P");
        }
        if (count == 6U) {
            return std::string("gP6P");
        }
        return std::string("gPnP");
    };

    bool used_gpnp_fallback = false;
    auto solveUnconstrained = [&]() {
        std::vector<PoseSolution> output;
        if (observations.size() == 3U) {
            output = GP3P::solve(
                world_points, ray_origins, ray_directions, options);
        } else if (observations.size() <= 5U) {
            output = GP4_5P::solve(
                world_points, ray_origins, ray_directions, options);
        } else if (observations.size() == 6U) {
            output = GP6P::solve(
                world_points, ray_origins, ray_directions, options);
            // Repeated physical markers can violate the six-ray DLT rank even
            // though a non-collinear generalized problem remains observable.
            if (output.empty()) {
                used_gpnp_fallback = true;
                output = GPnP::solve(
                    world_points, ray_origins, ray_directions, options);
            }
        } else {
            output = GPnP::solve(
                world_points, ray_origins, ray_directions, options);
        }
        return output;
    };

    const std::optional<Eigen::Vector3d> output_up_axis =
        config_.odometry_ref_enable
        ? rigOutputUpAxis(observations) : std::nullopt;
    const bool use_known_axis = output_up_axis
        && !hasObservableRigMarkerGeometry(observations)
        && hasDistinctRigMarkerPair(observations);
    std::vector<PoseSolution> solutions;
    if (use_known_axis) {
        Eigen::Vector3d model_axis =
            config_.odometry_ref_model_gravity_axis;
        if (!model_axis.allFinite() || model_axis.squaredNorm() <= epsilon) {
            return {};
        }
        model_axis.normalize();
        solutions = GeneralizedKnownAxis::solve(
            world_points,
            ray_origins,
            ray_directions,
            *output_up_axis,
            model_axis,
            options);

        if (method != nullptr) {
            *method = signatureForCount(observations.size()) + "o";
        }
    } else {
        if (observations.size() < 3U
            || !hasObservableRigMarkerGeometry(observations)) {
            return {};
        }
        solutions = solveUnconstrained();
        if (method != nullptr) {
            *method = used_gpnp_fallback
                ? "gPnP" : signatureForCount(observations.size());
        }
    }

    std::vector<CameraPose> output;
    output.reserve(solutions.size());
    for (const PoseSolution& solution : solutions) {
        output.push_back(toCameraPose(solution));
    }
    return output;
}

bool GeometricSolver::hasObservableRigMarkerGeometry(
    const std::vector<RigObservation>& observations) const
{
    constexpr double same_marker_position_tolerance = 1.0e-9;
    std::vector<Eigen::Vector3d> unique_marker_positions;
    unique_marker_positions.reserve(observations.size());
    for (const RigObservation& observation : observations) {
        const Eigen::Vector3d& marker_position =
            observation.observation.marker.pose.position;
        const bool already_present = std::any_of(
            unique_marker_positions.begin(),
            unique_marker_positions.end(),
            [&](const Eigen::Vector3d& existing_position) {
                return (existing_position - marker_position).norm()
                    <= same_marker_position_tolerance;
            });
        if (!already_present) {
            unique_marker_positions.push_back(marker_position);
        }
    }

    if (unique_marker_positions.size() < 3U) {
        return false;
    }
    for (std::size_t first = 0U;
         first + 2U < unique_marker_positions.size(); ++first) {
        for (std::size_t second = first + 1U;
             second + 1U < unique_marker_positions.size(); ++second) {
            for (std::size_t third = second + 1U;
                 third < unique_marker_positions.size(); ++third) {
                if (BodyModel::hasObservableTriangle(
                        unique_marker_positions[first],
                        unique_marker_positions[second],
                        unique_marker_positions[third])) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool GeometricSolver::hasDistinctRigMarkerPair(
    const std::vector<RigObservation>& observations) const
{
    constexpr double same_marker_position_tolerance = 1.0e-9;
    for (std::size_t first = 0U; first + 1U < observations.size(); ++first) {
        for (std::size_t second = first + 1U;
             second < observations.size(); ++second) {
            if ((observations[first].observation.marker.pose.position
                    - observations[second].observation.marker.pose.position).norm()
                > same_marker_position_tolerance) {
                return true;
            }
        }
    }
    return false;
}

std::optional<Eigen::Vector3d> GeometricSolver::rigOutputUpAxis(
    const std::vector<RigObservation>& observations) const
{
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    std::optional<Eigen::Vector3d> reference;
    for (const RigObservation& observation : observations) {
        if (!observation.output_up_axis
            || !observation.output_up_axis->allFinite()
            || observation.output_up_axis->squaredNorm() <= epsilon) {
            continue;
        }
        const Eigen::Vector3d axis = observation.output_up_axis->normalized();
        if (!reference) {
            reference = axis;
        } else if (reference->dot(axis) < 0.99) {
            // The same odometry direction transformed through known rigid TFs
            // must agree across an arbitrary-N rig. Do not silently fuse stale
            // or inconsistent transforms as a hard orientation constraint.
            return std::nullopt;
        }
        sum += axis;
    }
    if (!reference || !sum.allFinite() || sum.squaredNorm() <= epsilon) {
        return std::nullopt;
    }
    return sum.normalized();
}

std::vector<GeometricSolver::ScoredRigPose> GeometricSolver::refineRigCandidates(
    const std::vector<CameraPose>& candidates,
    const std::vector<RigObservation>& observations) const
{
    std::vector<ScoredRigPose> output;
    output.reserve(candidates.size());
    const std::optional<Eigen::Vector3d> output_up_axis =
        config_.odometry_ref_enable
        ? rigOutputUpAxis(observations) : std::nullopt;
    const bool use_known_axis = output_up_axis
        && !hasObservableRigMarkerGeometry(observations)
        && hasDistinctRigMarkerPair(observations);
    for (const CameraPose& candidate : candidates) {
        CameraPose refined = use_known_axis
            ? refineKnownAxisRigPose(
                candidate, observations, *output_up_axis)
            : refineRigPose(candidate, observations);
        const double error = rigReprojectionError(refined, observations);
        if (std::isfinite(error)
            && hasPositiveRigDepths(refined, observations)
            && rigVisibilityScore(refined, observations)) {
            output.push_back({std::move(refined), error});
        }
    }
    return output;
}

GeometricSolver::CameraPose GeometricSolver::refineRigPose(
    const CameraPose& seed,
    const std::vector<RigObservation>& observations) const
{
    CameraPose pose = seed;
    double previous_error = rigReprojectionError(pose, observations);
    for (int iteration = 0; iteration < std::max(0, config_.multi_cam_refinement_iterations); ++iteration) {
        Eigen::Matrix<double, 6, 6> normal = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> rhs = Eigen::Matrix<double, 6, 1>::Zero();
        for (const RigObservation& observation : observations) {
            const CameraModel& camera = cameras_[observation.camera_index];
            const Eigen::Vector3d rotated_output_point =
                pose.rotation * observation.observation.marker.pose.position;
            const Eigen::Vector3d output_point = rotated_output_point + pose.translation;
            const Eigen::Vector3d camera_point = observation.output_to_camera * output_point;
            if (camera_point.z() <= epsilon) {
                continue;
            }
            const Eigen::Vector2d residual =
                observation.observation.image_point - camera.project(camera_point);
            const Eigen::Matrix<double, 2, 3> projection_jacobian =
                camera.projectionJacobian(camera_point);
            Eigen::Matrix<double, 2, 6> jacobian;
            jacobian.leftCols<3>() = projection_jacobian * observation.output_to_camera.rotation();
            jacobian.rightCols<3>() = projection_jacobian
                * observation.output_to_camera.rotation()
                * (-uvdar_core::helpers::skew(rotated_output_point));
            const Eigen::Matrix2d covariance = unc::regularizedCovariance(
                observation.observation.point.has_prediction
                    ? observation.observation.point.prediction_covariance
                    : observation.observation.point.covariance,
                config_.covariance_regularization_px);
            const Eigen::Matrix2d information = covariance.inverse();
            normal += jacobian.transpose() * information * jacobian;
            rhs += jacobian.transpose() * information * residual;
        }
        normal.diagonal().array() += std::max(config_.multi_cam_damping, 1.0e-15);
        const Eigen::Matrix<double, 6, 1> delta = normal.ldlt().solve(rhs);
        if (!delta.allFinite()
            || delta.norm() < std::max(0.0, config_.multi_cam_step_tolerance)) {
            break;
        }

        bool accepted = false;
        double scale = 1.0;
        for (int line_search = 0; line_search < 10; ++line_search) {
            CameraPose candidate = pose;
            applyLeftCameraPoseIncrement(
                candidate,
                scale * delta.head<3>(),
                scale * delta.tail<3>());
            const double candidate_error = rigReprojectionError(candidate, observations);
            if (std::isfinite(candidate_error) && candidate_error < previous_error) {
                pose = candidate;
                previous_error = candidate_error;
                accepted = true;
                break;
            }
            scale *= 0.5;
        }
        if (!accepted) {
            break;
        }
    }
    return pose;
}

GeometricSolver::CameraPose GeometricSolver::refineKnownAxisRigPose(
    const CameraPose& seed,
    const std::vector<RigObservation>& observations,
    const Eigen::Vector3d& output_up_axis) const
{
    Eigen::Vector3d output_up = output_up_axis;
    Eigen::Vector3d model_up = config_.odometry_ref_model_gravity_axis;
    if (!output_up.allFinite() || !model_up.allFinite()
        || output_up.squaredNorm() <= epsilon
        || model_up.squaredNorm() <= epsilon) {
        return seed;
    }
    output_up.normalize();
    model_up.normalize();

    // Remove the two rotational components that contradict the hard odometry
    // reference while retaining the seed's yaw about gravity.
    const Eigen::Matrix3d output_to_aligned =
        uvdar_core::helpers::rotationBetween(
            output_up, Eigen::Vector3d::UnitZ());
    const Eigen::Matrix3d model_to_aligned =
        uvdar_core::helpers::rotationBetween(
            model_up, Eigen::Vector3d::UnitZ());
    const Eigen::Matrix3d aligned = output_to_aligned * seed.rotation
        * model_to_aligned.transpose();
    const double initial_yaw = std::atan2(
        aligned(1, 0) - aligned(0, 1),
        aligned(0, 0) + aligned(1, 1));

    CameraPose pose = seed;
    pose.rotation = output_to_aligned.transpose()
        * uvdar_core::helpers::rotationZ(initial_yaw)
        * model_to_aligned;
    double previous_error = rigReprojectionError(pose, observations);
    for (int iteration = 0;
         iteration < std::max(0, config_.multi_cam_refinement_iterations);
         ++iteration) {
        Eigen::Matrix4d normal = Eigen::Matrix4d::Zero();
        Eigen::Vector4d rhs = Eigen::Vector4d::Zero();
        for (const RigObservation& observation : observations) {
            if (observation.camera_index >= cameras_.size()) {
                continue;
            }
            const CameraModel& camera = cameras_[observation.camera_index];
            const Eigen::Vector3d rotated_output_point =
                pose.rotation * observation.observation.marker.pose.position;
            const Eigen::Vector3d camera_point = observation.output_to_camera
                * (rotated_output_point + pose.translation);
            if (camera_point.z() <= epsilon) {
                continue;
            }
            const Eigen::Vector2d residual =
                observation.observation.image_point - camera.project(camera_point);
            const Eigen::Matrix<double, 2, 3> projection_jacobian =
                camera.projectionJacobian(camera_point);
            Eigen::Matrix<double, 2, 4> jacobian;
            jacobian.leftCols<3>() = projection_jacobian
                * observation.output_to_camera.rotation();
            jacobian.col(3) = projection_jacobian
                * observation.output_to_camera.rotation()
                * output_up.cross(rotated_output_point);
            const Eigen::Matrix2d covariance = unc::regularizedCovariance(
                observation.observation.point.has_prediction
                    ? observation.observation.point.prediction_covariance
                    : observation.observation.point.covariance,
                config_.covariance_regularization_px);
            const Eigen::Matrix2d information = covariance.inverse();
            normal += jacobian.transpose() * information * jacobian;
            rhs += jacobian.transpose() * information * residual;
        }
        normal.diagonal().array() +=
            std::max(config_.multi_cam_damping, 1.0e-15);
        const Eigen::Vector4d delta = normal.ldlt().solve(rhs);
        if (!delta.allFinite()
            || delta.norm() < std::max(0.0, config_.multi_cam_step_tolerance)) {
            break;
        }

        bool accepted = false;
        double scale = 1.0;
        for (int line_search = 0; line_search < 10; ++line_search) {
            CameraPose candidate = pose;
            candidate.translation += scale * delta.head<3>();
            candidate.rotation = Eigen::AngleAxisd(
                scale * delta(3), output_up).toRotationMatrix()
                * candidate.rotation;
            const double candidate_error =
                rigReprojectionError(candidate, observations);
            if (std::isfinite(candidate_error)
                && candidate_error < previous_error) {
                pose = candidate;
                previous_error = candidate_error;
                accepted = true;
                break;
            }
            scale *= 0.5;
        }
        if (!accepted) {
            break;
        }
    }
    return pose;
}

double GeometricSolver::rigReprojectionError(
    const CameraPose& pose,
    const std::vector<RigObservation>& observations) const
{
    double error = 0.0;
    for (const RigObservation& observation : observations) {
        if (observation.camera_index >= cameras_.size()) {
            return std::numeric_limits<double>::infinity();
        }
        const Eigen::Vector3d output_point =
            transformPoint(pose, observation.observation.marker.pose.position);
        const Eigen::Vector3d camera_point = observation.output_to_camera * output_point;
        if (!camera_point.allFinite() || camera_point.z() <= epsilon) {
            return std::numeric_limits<double>::infinity();
        }
        const Eigen::Vector2d projected = cameras_[observation.camera_index].project(camera_point);
        if (!projected.allFinite()) {
            return std::numeric_limits<double>::infinity();
        }
        error += (projected - observation.observation.image_point).squaredNorm();
    }
    return error;
}

bool GeometricSolver::hasPositiveRigDepths(
    const CameraPose& pose,
    const std::vector<RigObservation>& observations) const
{
    return std::all_of(
        observations.begin(), observations.end(), [&](const RigObservation& observation) {
            const Eigen::Vector3d output_point =
                transformPoint(pose, observation.observation.marker.pose.position);
            return (observation.output_to_camera * output_point).z() > epsilon;
        });
}

std::optional<GeometricSolver::RigVisibilityScore> GeometricSolver::rigVisibilityScore(
    const CameraPose& pose,
    const std::vector<RigObservation>& observations) const
{
    std::vector<std::vector<Eigen::Vector3d>> marker_positions_by_camera(cameras_.size());
    std::vector<const RigObservation*> representative_observation(cameras_.size(), nullptr);
    for (const RigObservation& observation : observations) {
        if (observation.camera_index >= cameras_.size()) {
            return std::nullopt;
        }
        marker_positions_by_camera[observation.camera_index].push_back(
            observation.observation.marker.pose.position);
        representative_observation[observation.camera_index] = &observation;
    }

    RigVisibilityScore aggregate;
    std::size_t contributing_cameras = 0U;
    for (std::size_t camera_index = 0U; camera_index < cameras_.size(); ++camera_index) {
        const RigObservation* camera_observation = representative_observation[camera_index];
        if (camera_observation == nullptr) {
            continue;
        }

        CameraPose body_to_camera;
        body_to_camera.rotation = camera_observation->output_to_camera.rotation() * pose.rotation;
        body_to_camera.translation = camera_observation->output_to_camera * pose.translation;
        const BodyModel::VisibilityScore visibility = body_.visibilityScore(
            body_to_camera, marker_positions_by_camera[camera_index]);

        // A generalized algebraic branch is physically admissible only when
        // every blob observed by this camera has at least one colocated LED
        // whose emission axis points toward this camera.  Do not compensate a
        // violation in one camera with a high visibility score in another.
        if (!visibility.observed_leds_face_camera
            || !std::isfinite(visibility.visibility_margin)
            || !std::isfinite(visibility.observed_view_cosine_mean)) {
            return std::nullopt;
        }
        aggregate.visibility_margin += visibility.visibility_margin;
        aggregate.observed_view_cosine_mean += visibility.observed_view_cosine_mean;
        ++contributing_cameras;
    }

    if (contributing_cameras == 0U) {
        return std::nullopt;
    }
    const double normalization = 1.0 / static_cast<double>(contributing_cameras);
    aggregate.visibility_margin *= normalization;
    aggregate.observed_view_cosine_mean *= normalization;
    return aggregate;
}

std::optional<std::size_t> GeometricSolver::selectRigCandidate(
    const std::vector<ScoredRigPose>& candidates,
    const std::vector<RigObservation>& observations,
    const std::optional<CameraPose>& previous_pose) const
{
    std::optional<std::size_t> best_index;
    double best_margin = -std::numeric_limits<double>::infinity();
    double best_view_cosine = -std::numeric_limits<double>::infinity();
    double best_continuity = std::numeric_limits<double>::infinity();

    for (std::size_t candidate_index = 0U; candidate_index < candidates.size(); ++candidate_index) {
        const ScoredRigPose& candidate = candidates[candidate_index];
        if (!hasPositiveRigDepths(candidate.pose, observations)) {
            continue;
        }

        const std::optional<RigVisibilityScore> visibility =
            rigVisibilityScore(candidate.pose, observations);
        if (!visibility) {
            continue;
        }
        const double margin = visibility->visibility_margin;
        const double view_cosine = visibility->observed_view_cosine_mean;
        const double continuity = previous_pose
            ? unc::poseTangentDistance(*previous_pose, candidate.pose)
            : std::numeric_limits<double>::infinity();

        const bool same_margin = std::abs(margin - best_margin) <= observability_score_tolerance;
        const bool same_view = std::abs(view_cosine - best_view_cosine) <= observability_score_tolerance;
        const bool better = !best_index
            || margin > best_margin + observability_score_tolerance
            || (same_margin
                && view_cosine > best_view_cosine + observability_score_tolerance)
            || (same_margin && same_view && previous_pose
                && continuity < best_continuity - observability_score_tolerance)
            || (same_margin && same_view
                && (!previous_pose
                    || std::abs(continuity - best_continuity) <= observability_score_tolerance)
                && candidate.reprojection_error < candidates[*best_index].reprojection_error);
        if (better) {
            best_index = candidate_index;
            best_margin = margin;
            best_view_cosine = view_cosine;
            best_continuity = continuity;
        }
    }
    return best_index;
}

std::vector<GeometricSolver::CameraPose> GeometricSolver::solveP2P(
    const std::vector<Observation>& observations,
    const std::optional<Eigen::Vector3d>& camera_up_axis) const
{
    if (observations.size() != 2U || !camera_up_axis) {
        return {};
    }

    Eigen::Matrix<double, 3, 2> pw;
    Eigen::Matrix<double, 3, 2> pi;
    for (int i = 0; i < 2; ++i) {
        pw.col(i) = observations[static_cast<std::size_t>(i)].marker.pose.position;
        pi.col(i) = observations[static_cast<std::size_t>(i)].bearing;
    }

    // Two correspondences leave one rotational degree of freedom.  The target
    // body-frame gravity axis and the navigation-derived camera gravity axis
    // remove it, regardless of the positions of the two LEDs.
    Eigen::Vector3d body_up_axis = config_.odometry_ref_model_gravity_axis;
    if (!body_up_axis.allFinite() || body_up_axis.squaredNorm() < 1.0e-18) {
        return {};
    }
    body_up_axis.normalize();

    Eigen::Vector3d v_cam = *camera_up_axis;
    if (!v_cam.allFinite() || v_cam.squaredNorm() < 1.0e-18) {
        return {};
    }
    v_cam.normalize();

    // Li takes the known body-up and camera-up directions directly.  The ray
    // plane normal is not a gravity direction and must not be supplied here.
    const auto solutions = P2P::solve(
        pw,
        pi,
        v_cam,
        body_up_axis,
        P2P::Method::Li,
        config_.odometry_ref_min_axis_observability);
    std::vector<CameraPose> candidates;
    candidates.reserve(solutions.size());
    for (const auto& solution : solutions) {
        candidates.push_back(toCameraPose(solution));
    }

    return candidates;
}

std::vector<GeometricSolver::CameraPose> GeometricSolver::solveP3P(const std::vector<Observation>& observations) const
{
    if (observations.size() != 3U
        || !BodyModel::hasObservableTriangle(
            observations[0].marker.pose.position,
            observations[1].marker.pose.position,
            observations[2].marker.pose.position)) {
        return {};
    }

    Eigen::Matrix3d pw;
    Eigen::Matrix3d pi;
    for (int i = 0; i < 3; ++i) {
        pw.col(i) = observations[static_cast<std::size_t>(i)].marker.pose.position;
        pi.col(i) = observations[static_cast<std::size_t>(i)].bearing;
    }
    std::vector<CameraPose> candidates;
    for (const auto& solution : P3P::solve(pw, pi, 1)) {
        candidates.push_back(toCameraPose(solution));
    }

    return candidates;
}

bool GeometricSolver::hasPositiveDepths(const CameraPose& pose, const std::vector<Observation>& observations) const
{
    return std::all_of(observations.begin(), observations.end(), [&pose](const Observation& observation) {
        return transformPoint(pose, observation.marker.pose.position).z() > epsilon;
    });
}

std::vector<Eigen::Vector3d> GeometricSolver::observedMarkerPositions(const std::vector<Observation>& observations) const
{
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(observations.size());
    for (const Observation& observation : observations) {
        positions.push_back(observation.marker.pose.position);
    }
    return positions;
}

std::optional<std::size_t> GeometricSolver::selectObservabilityAwareCandidate(
    const std::vector<ScoredCameraPose>& candidates,
    const std::vector<Observation>& observations,
    const std::optional<CameraPose>& previous_pose) const
{
    const std::vector<Eigen::Vector3d> observed_positions = observedMarkerPositions(observations);
    std::optional<std::size_t> best_index;
    BodyModel::VisibilityScore best_visibility;
    double best_continuity_distance = std::numeric_limits<double>::infinity();

    for (std::size_t index = 0U; index < candidates.size(); ++index) {
        const ScoredCameraPose& candidate = candidates[index];
        if (!hasPositiveDepths(candidate.pose, observations)) {
            continue;
        }

        const BodyModel::VisibilityScore visibility = body_.visibilityScore(candidate.pose, observed_positions);
        if (!visibility.observed_leds_face_camera) {
            continue;
        }
        const double continuity_distance = previous_pose
            ? unc::poseTangentDistance(*previous_pose, candidate.pose)
            : std::numeric_limits<double>::infinity();
        if (!best_index) {
            best_index = index;
            best_visibility = visibility;
            best_continuity_distance = continuity_distance;
            continue;
        }

        const ScoredCameraPose& best = candidates[*best_index];
        const bool same_visibility_support = visibility.observed_leds_face_camera == best_visibility.observed_leds_face_camera;
        const bool equivalent_observability = same_visibility_support
            && std::abs(visibility.visibility_margin - best_visibility.visibility_margin) <= observability_score_tolerance
            && std::abs(visibility.observed_view_cosine_mean - best_visibility.observed_view_cosine_mean) <= observability_score_tolerance;
        // Once a physically visible P2P branch has been established, temporal
        // continuity is the meaningful tie-breaker. Tiny visibility-margin
        // changes near a branch boundary must not flip the pose each frame.
        const bool p2p_continuity_better = observations.size() == 2U
            && previous_pose
            && continuity_distance
                < best_continuity_distance - observability_score_tolerance;
        const bool p2p_continuity_worse_or_equal = observations.size() == 2U
            && previous_pose
            && !p2p_continuity_better;
        const bool better = p2p_continuity_better
            || (!p2p_continuity_worse_or_equal
                && same_visibility_support
                && visibility.visibility_margin > best_visibility.visibility_margin + observability_score_tolerance)
            || (!p2p_continuity_worse_or_equal && same_visibility_support
                && std::abs(visibility.visibility_margin - best_visibility.visibility_margin) <= observability_score_tolerance
                && visibility.observed_view_cosine_mean > best_visibility.observed_view_cosine_mean + observability_score_tolerance)
            // When the model says the branches are equally observable, retain
            // the existing physical branch rather than alternating because of
            // sub-pixel reprojection noise.
            || (!p2p_continuity_worse_or_equal && equivalent_observability && previous_pose
                && continuity_distance < best_continuity_distance - observability_score_tolerance)
            || (!p2p_continuity_worse_or_equal && equivalent_observability
                && (!previous_pose
                    || std::abs(continuity_distance - best_continuity_distance) <= observability_score_tolerance)
                && candidate.reprojection_error < best.reprojection_error);
        if (better) {
            best_index = index;
            best_visibility = visibility;
            best_continuity_distance = continuity_distance;
        }
    }
    return best_index;
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
        output.push_back(toCameraPose(solution));
    }
    return output;
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
        const auto bearing = camera.bearingForPixel(output[i].image_point);
        if (!bearing) {
            return {};
        }
        output[i].bearing = *bearing;
    }
    return output;
}

Eigen::VectorXd GeometricSolver::rigObservationVector(
    const std::vector<RigObservation>& observations) const
{
    Eigen::VectorXd output(static_cast<Eigen::Index>(2U * observations.size()));
    for (std::size_t i = 0U; i < observations.size(); ++i) {
        output.segment<2>(static_cast<Eigen::Index>(2U * i)) =
            observations[i].observation.image_point;
    }
    return output;
}

Eigen::MatrixXd GeometricSolver::rigDetectorCovariance(
    const std::vector<RigObservation>& observations) const
{
    const Eigen::Index dimensions = static_cast<Eigen::Index>(2U * observations.size());
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(dimensions, dimensions);
    for (std::size_t i = 0U; i < observations.size(); ++i) {
        const Observation& observation = observations[i].observation;
        covariance.block<2, 2>(
            static_cast<Eigen::Index>(2U * i),
            static_cast<Eigen::Index>(2U * i)) = unc::regularizedCovariance(
            observation.point.has_prediction
                ? observation.point.prediction_covariance
                : observation.point.covariance,
            config_.covariance_regularization_px);
    }
    return covariance;
}

std::vector<GeometricSolver::RigObservation> GeometricSolver::rigObservationsFromVector(
    const std::vector<RigObservation>& base_observations,
    const Eigen::VectorXd& sample) const
{
    if (sample.size() != static_cast<Eigen::Index>(2U * base_observations.size())) {
        return {};
    }
    std::vector<RigObservation> output = base_observations;
    for (std::size_t i = 0U; i < output.size(); ++i) {
        RigObservation& observation = output[i];
        if (observation.camera_index >= cameras_.size()) {
            return {};
        }
        observation.observation.image_point =
            sample.segment<2>(static_cast<Eigen::Index>(2U * i));
        const auto bearing = cameras_[observation.camera_index].bearingForPixel(
            observation.observation.image_point);
        if (!bearing) {
            return {};
        }
        observation.observation.bearing = *bearing;
        observation.ray_direction = observation.camera_to_output.rotation() * *bearing;
        if (!observation.ray_direction.allFinite()
            || observation.ray_direction.squaredNorm() <= epsilon) {
            return {};
        }
        observation.ray_direction.normalize();
    }
    return output;
}

Eigen::Matrix<double, 6, 6> GeometricSolver::poseCovarianceByRigJacobianPropagation(
    const CameraPose& pose,
    const std::vector<RigObservation>& observations) const
{
    const std::optional<Eigen::Vector3d> constrained_up =
        config_.odometry_ref_enable
        && hasDistinctRigMarkerPair(observations)
        && !hasObservableRigMarkerGeometry(observations)
        ? rigOutputUpAxis(observations) : std::nullopt;
    if (constrained_up) {
        const Eigen::Vector3d output_up = constrained_up->normalized();
        Eigen::Matrix4d information = Eigen::Matrix4d::Zero();
        for (const RigObservation& observation : observations) {
            if (observation.camera_index >= cameras_.size()) {
                continue;
            }
            const CameraModel& camera = cameras_[observation.camera_index];
            const Eigen::Vector3d rotated_output_point =
                pose.rotation * observation.observation.marker.pose.position;
            const Eigen::Vector3d camera_point = observation.output_to_camera
                * (rotated_output_point + pose.translation);
            if (!camera_point.allFinite() || camera_point.z() <= epsilon) {
                continue;
            }
            const Eigen::Matrix<double, 2, 3> projection_jacobian =
                camera.projectionJacobian(camera_point);
            Eigen::Matrix<double, 2, 4> jacobian;
            jacobian.leftCols<3>() = projection_jacobian
                * observation.output_to_camera.rotation();
            jacobian.col(3) = projection_jacobian
                * observation.output_to_camera.rotation()
                * output_up.cross(rotated_output_point);
            const Eigen::Matrix2d covariance = unc::regularizedCovariance(
                observation.observation.point.has_prediction
                    ? observation.observation.point.prediction_covariance
                    : observation.observation.point.covariance,
                config_.covariance_regularization_px);
            information += jacobian.transpose()
                * covariance.inverse() * jacobian;
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(
            0.5 * (information + information.transpose()));
        if (eigensolver.info() == Eigen::Success
            && eigensolver.eigenvalues().maxCoeff() > epsilon) {
            const double floor = std::max(
                epsilon,
                1.0e-12 * eigensolver.eigenvalues().maxCoeff());
            const Eigen::Vector4d inverse_eigenvalues =
                eigensolver.eigenvalues().unaryExpr(
                    [&](const double value) {
                        return 1.0 / std::max(value, floor);
                    });
            const Eigen::Matrix4d state_covariance =
                eigensolver.eigenvectors()
                * inverse_eigenvalues.asDiagonal()
                * eigensolver.eigenvectors().transpose();
            // Published tangent order is [translation, rotation]. The known
            // axis is deterministic here; only yaw about it is estimated.
            Eigen::Matrix<double, 6, 4> lift =
                Eigen::Matrix<double, 6, 4>::Zero();
            lift.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
            lift.block<3, 1>(3, 3) = output_up;
            const Eigen::Matrix<double, 6, 6> covariance =
                lift * state_covariance * lift.transpose();
            if (covariance.allFinite()) {
                return 0.5 * (covariance + covariance.transpose());
            }
        }
    }

    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
    for (const RigObservation& observation : observations) {
        if (observation.camera_index >= cameras_.size()) {
            continue;
        }
        const CameraModel& camera = cameras_[observation.camera_index];
        const Eigen::Vector3d rotated_output_point =
            pose.rotation * observation.observation.marker.pose.position;
        const Eigen::Vector3d camera_point = observation.output_to_camera
            * (rotated_output_point + pose.translation);
        if (!camera_point.allFinite() || camera_point.z() <= epsilon) {
            continue;
        }
        const Eigen::Matrix<double, 2, 3> projection_jacobian =
            camera.projectionJacobian(camera_point);
        Eigen::Matrix<double, 2, 6> jacobian;
        jacobian.leftCols<3>() = projection_jacobian
            * observation.output_to_camera.rotation();
        jacobian.rightCols<3>() = projection_jacobian
            * observation.output_to_camera.rotation()
            * (-uvdar_core::helpers::skew(rotated_output_point));
        const Eigen::Matrix2d covariance = unc::regularizedCovariance(
            observation.observation.point.has_prediction
                ? observation.observation.point.prediction_covariance
                : observation.observation.point.covariance,
            config_.covariance_regularization_px);
        information += jacobian.transpose() * covariance.inverse() * jacobian;
    }
    return unc::covarianceFromInformation(information);
}

Eigen::Matrix<double, 6, 6> GeometricSolver::poseCovarianceByRigEllipseTransform(
    const std::vector<CameraPose>& base_poses,
    const std::vector<RigObservation>& observations,
    const int selected_index) const
{
    if (base_poses.empty() || selected_index < 0
        || static_cast<std::size_t>(selected_index) >= base_poses.size()) {
        return Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
    }
    const Eigen::VectorXd mean_pixel = rigObservationVector(observations);
    const Eigen::MatrixXd covariance_pixel = rigDetectorCovariance(observations);
    Eigen::LLT<Eigen::MatrixXd> cholesky(covariance_pixel);
    if (mean_pixel.size() == 0 || cholesky.info() != Eigen::Success) {
        return poseCovarianceByRigJacobianPropagation(
            base_poses[static_cast<std::size_t>(selected_index)], observations);
    }

    const CameraPose& base_pose = base_poses[static_cast<std::size_t>(selected_index)];
    const Eigen::MatrixXd factor = cholesky.matrixL();
    std::vector<unc::PoseTangent> samples;
    samples.reserve(static_cast<std::size_t>(2 * factor.cols()));
    for (Eigen::Index axis = 0; axis < factor.cols(); ++axis) {
        const std::array<Eigen::VectorXd, 2> sigma_points {
            mean_pixel + factor.col(axis),
            mean_pixel - factor.col(axis),
        };
        for (const Eigen::VectorXd& sigma_point : sigma_points) {
            const std::vector<RigObservation> sample_observations =
                rigObservationsFromVector(observations, sigma_point);
            if (sample_observations.empty()) {
                continue;
            }
            const std::vector<ScoredRigPose> candidates = refineRigCandidates(
                solveRigPoses(sample_observations), sample_observations);
            double best_distance = std::numeric_limits<double>::infinity();
            unc::PoseTangent best_delta = unc::PoseTangent::Zero();
            for (const ScoredRigPose& candidate : candidates) {
                const double distance = unc::poseTangentDistance(base_pose, candidate.pose);
                if (distance < best_distance) {
                    best_distance = distance;
                    best_delta = unc::relativePoseTangent(base_pose, candidate.pose);
                }
            }
            if (best_distance < ellipse_branch_distance_threshold) {
                samples.push_back(best_delta);
            }
        }
    }
    if (samples.size() < 2U) {
        return poseCovarianceByRigJacobianPropagation(base_pose, observations);
    }
    return unc::covarianceFromPoseSamples(samples, ellipse_transform_scale);
}

Eigen::Matrix<double, 6, 6> GeometricSolver::poseCovarianceByRigMonteCarlo(
    const std::vector<CameraPose>& base_poses,
    const std::vector<RigObservation>& observations,
    const int selected_index) const
{
    if (base_poses.empty() || selected_index < 0
        || static_cast<std::size_t>(selected_index) >= base_poses.size()) {
        return Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
    }
    const Eigen::VectorXd mean_pixel = rigObservationVector(observations);
    const Eigen::MatrixXd covariance_pixel = rigDetectorCovariance(observations);
    Eigen::LLT<Eigen::MatrixXd> cholesky(covariance_pixel);
    if (mean_pixel.size() == 0 || cholesky.info() != Eigen::Success) {
        return poseCovarianceByRigJacobianPropagation(
            base_poses[static_cast<std::size_t>(selected_index)], observations);
    }

    const int sample_count = std::max(1, config_.uncertainty_samples);
    const Eigen::MatrixXd factor = cholesky.matrixL();
    std::mt19937_64 random_generator(
        static_cast<std::mt19937_64::result_type>(
            std::chrono::high_resolution_clock::now().time_since_epoch().count()));
    std::normal_distribution<double> normal_distribution(0.0, 1.0);
    std::vector<unc::PoseTangent> selected_samples;
    selected_samples.reserve(static_cast<std::size_t>(sample_count));

    for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
        Eigen::VectorXd noise(mean_pixel.size());
        for (Eigen::Index i = 0; i < noise.size(); ++i) {
            noise(i) = normal_distribution(random_generator);
        }
        const std::vector<RigObservation> sample_observations =
            rigObservationsFromVector(observations, mean_pixel + factor * noise);
        if (sample_observations.empty()) {
            continue;
        }
        const std::vector<ScoredRigPose> candidates = refineRigCandidates(
            solveRigPoses(sample_observations), sample_observations);
        int best_branch = -1;
        double best_distance = std::numeric_limits<double>::infinity();
        unc::PoseTangent best_delta = unc::PoseTangent::Zero();
        for (std::size_t branch = 0U; branch < base_poses.size(); ++branch) {
            for (const ScoredRigPose& candidate : candidates) {
                const double distance = unc::poseTangentDistance(
                    base_poses[branch], candidate.pose);
                if (distance < best_distance) {
                    best_distance = distance;
                    best_branch = static_cast<int>(branch);
                    best_delta = unc::relativePoseTangent(base_poses[branch], candidate.pose);
                }
            }
        }
        if (best_branch == selected_index
            && best_distance < monte_carlo_branch_distance_threshold) {
            selected_samples.push_back(best_delta);
        }
    }
    if (selected_samples.size() < 2U) {
        return poseCovarianceByRigJacobianPropagation(
            base_poses[static_cast<std::size_t>(selected_index)], observations);
    }
    return unc::covarianceFromPoseSamples(
        selected_samples,
        monte_carlo_covariance_scale
            / static_cast<double>(selected_samples.size() - 1U));
}

Eigen::Matrix<double, 6, 6> GeometricSolver::poseCovarianceByEllipseTransform(
    const std::vector<CameraPose>& base_poses,
    const std::vector<Observation>& observations,
    const CameraModel& camera,
    const std::optional<Eigen::Vector3d>& camera_up_axis,
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
    std::vector<unc::PoseTangent> selected_samples;

    for (Eigen::Index axis = 0; axis < cholesky_factor.cols(); ++axis) {
        const Eigen::VectorXd perturbation = cholesky_factor.col(axis);
        const std::array<Eigen::VectorXd, 2> sigma_points{
            mean_pixel + perturbation,
            mean_pixel - perturbation,
        };

        for (const Eigen::VectorXd& sigma_point : sigma_points) {
            const auto sample_observations = observationsFromVector(observations, sigma_point, camera);
            if (sample_observations.empty()) {
                selected_samples.push_back(unc::PoseTangent::Zero());
                continue;
            }

            const std::vector<ScoredCameraPose> refined_candidates = refineCandidates(
                solveCameraPoses(sample_observations, camera, camera_up_axis),
                sample_observations,
                camera);
            if (refined_candidates.empty()) {
                selected_samples.push_back(unc::PoseTangent::Zero());
                continue;
            }

            const CameraPose& base_pose = base_poses[selected_pose_index];
            double best_distance = std::numeric_limits<double>::infinity();
            unc::PoseTangent best_delta = unc::PoseTangent::Zero();
            for (const ScoredCameraPose& candidate : refined_candidates) {
                const unc::PoseTangent delta = unc::relativePoseTangent(base_pose, candidate.pose);
                const double distance = unc::poseTangentDistance(base_pose, candidate.pose);
                if (distance < best_distance) {
                    best_distance = distance;
                    best_delta = delta;
                }
            }
            if (best_distance < ellipse_branch_distance_threshold) {
                selected_samples.push_back(best_delta);
            } else {
                selected_samples.push_back(unc::PoseTangent::Zero());
            }
        }
    }

    if (selected_samples.size() < 2U) {
        return Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
    }
    return unc::covarianceFromPoseSamples(selected_samples, ellipse_transform_scale);
}

Eigen::Matrix<double, 6, 6> GeometricSolver::poseCovarianceByMonteCarlo(
    const std::vector<CameraPose>& base_poses,
    const std::vector<Observation>& observations,
    const CameraModel& camera,
    const std::optional<Eigen::Vector3d>& camera_up_axis,
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

    std::vector<unc::PoseTangent> selected_samples;

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

        const std::vector<ScoredCameraPose> refined_candidates = refineCandidates(
            solveCameraPoses(sample_observations, camera, camera_up_axis),
            sample_observations,
            camera);
        if (refined_candidates.empty()) {
            continue;
        }

        int best_branch = -1;
        double best_branch_distance = std::numeric_limits<double>::infinity();
        unc::PoseTangent best_delta = unc::PoseTangent::Zero();
        for (std::size_t branch_index = 0U; branch_index < base_poses.size(); ++branch_index) {
            const CameraPose& base_pose = base_poses[branch_index];
            for (const ScoredCameraPose& candidate : refined_candidates) {
                const unc::PoseTangent delta = unc::relativePoseTangent(base_pose, candidate.pose);
                const double distance = unc::poseTangentDistance(base_pose, candidate.pose);
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
    return unc::covarianceFromPoseSamples(selected_samples, covariance_scale);
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
            const Eigen::Vector3d camera_point = transformPoint(pose, observation.marker.pose.position);
            const Eigen::Vector2d residual = observation.image_point - camera.project(camera_point);
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
        applyLeftCameraPoseIncrement(pose, delta.head<3>(), delta.tail<3>());
    }
    return pose;
}

double GeometricSolver::reprojectionError(const CameraPose& pose, const std::vector<Observation>& observations, const CameraModel& camera) const
{
    double error = 0.0;
    for (const Observation& observation : observations) {
        const Eigen::Vector3d camera_point = transformPoint(pose, observation.marker.pose.position);
        if (camera_point.z() <= epsilon) {
            return std::numeric_limits<double>::infinity();
        }
        error += (camera.project(camera_point) - observation.image_point).squaredNorm();
    }
    return error;
}

Eigen::Matrix<double, 6, 6> GeometricSolver::poseCovarianceByJacobianPropagation(
    const CameraPose& pose,
    const std::vector<Observation>& observations,
    const CameraModel& camera,
    const std::optional<Eigen::Vector3d>& camera_up_axis) const
{
    if (observations.size() == 2U && camera_up_axis) {
        Eigen::Matrix<double, 3, 2> world_points;
        Eigen::Matrix<double, 3, 2> bearings;
        Eigen::Matrix<double, 6, 6> bearing_covariance =
            Eigen::Matrix<double, 6, 6>::Zero();
        for (int index = 0; index < 2; ++index) {
            const Observation& observation =
                observations[static_cast<std::size_t>(index)];
            world_points.col(index) = observation.marker.pose.position;
            bearings.col(index) = observation.bearing;
            const Eigen::Matrix2d pixel_covariance =
                unc::regularizedCovariance(
                    observation.point.has_prediction
                        ? observation.point.prediction_covariance
                        : observation.point.covariance,
                    config_.covariance_regularization_px);
            const Eigen::Matrix<double, 3, 2> bearing_jacobian =
                camera.backProjectionJacobian(observation.image_point);
            bearing_covariance.block<3, 3>(3 * index, 3 * index) =
                bearing_jacobian * pixel_covariance
                * bearing_jacobian.transpose();
        }

        Eigen::Vector3d body_up_axis = config_.odometry_ref_model_gravity_axis;
        Eigen::Vector3d camera_axis = *camera_up_axis;
        if (body_up_axis.allFinite() && camera_axis.allFinite()
            && body_up_axis.squaredNorm() > epsilon
            && camera_axis.squaredNorm() > epsilon) {
            body_up_axis.normalize();
            camera_axis.normalize();
            const std::vector<P2P::PoseJacobian> jacobians =
                P2P::jacobianPoseWrtBearings(
                    world_points,
                    bearings,
                    camera_axis,
                    body_up_axis,
                    P2P::Method::Li);
            const auto closest = std::min_element(
                jacobians.begin(),
                jacobians.end(),
                [&](const P2P::PoseJacobian& first,
                    const P2P::PoseJacobian& second) {
                    return unc::poseTangentDistance(
                               pose, toCameraPose(first.sol))
                        < unc::poseTangentDistance(
                               pose, toCameraPose(second.sol));
                });
            if (closest != jacobians.end()
                && closest->dpose_dpi.allFinite()
                && bearing_covariance.allFinite()) {
                // P2P uses [rotation, translation], while published pose
                // covariance is [translation, rotation]. The up/gravity
                // constraint is treated as known; detector and tracker
                // uncertainty enters through the two bearing Jacobians.
                const Eigen::Matrix<double, 6, 6> solver_covariance =
                    closest->dpose_dpi * bearing_covariance
                    * closest->dpose_dpi.transpose();
                Eigen::Matrix<double, 6, 6> covariance;
                covariance.topLeftCorner<3, 3>() =
                    solver_covariance.bottomRightCorner<3, 3>();
                covariance.topRightCorner<3, 3>() =
                    solver_covariance.bottomLeftCorner<3, 3>();
                covariance.bottomLeftCorner<3, 3>() =
                    solver_covariance.topRightCorner<3, 3>();
                covariance.bottomRightCorner<3, 3>() =
                    solver_covariance.topLeftCorner<3, 3>();
                if (covariance.allFinite()) {
                    return 0.5 * (covariance + covariance.transpose());
                }
            }
        }
    }

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
    const Eigen::Matrix<double, 6, 6>& covariance,
    const std::string& method) const
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
    measurement.method = method;
    return measurement;
}

PoseMeasurement GeometricSolver::toRigMeasurement(
    const int target,
    const CameraPose& output_pose,
    const Eigen::Matrix<double, 6, 6>& covariance,
    const std::string& method) const
{
    PoseMeasurement measurement;
    measurement.id = target;
    measurement.pose.position = output_pose.translation;
    measurement.pose.orientation = Eigen::Quaterniond(output_pose.rotation).normalized();
    measurement.covariance = covariance;
    measurement.method = method;
    return measurement;
}

} // namespace uvdar_core::pose_estimation::geometric_solver
