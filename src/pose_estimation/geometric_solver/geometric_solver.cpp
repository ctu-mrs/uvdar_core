#include "uvdar_core/pose_estimation/geometric_solver/geometric_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <limits>
#include <random>

#include "uvdar_core/helpers/levenberg_marquardt.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/p3p.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/p4p.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/pnp.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/gp3p.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/gp4_5p.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/gp6p.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/gpnp.hpp"
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
constexpr double duplicate_pose_threshold = 1.0e-5;

} // namespace

GeometricSolver::GeometricSolver(GeometricSolverConfig config, BodyModel body, std::vector<CameraModel> cameras)
    : config_(std::move(config))
    , body_(std::move(body))
    , cameras_(std::move(cameras))
{
    latest_measurements_.frame_id = config_.output_frame;
    latest_camera_frames_.resize(cameras_.size());
    latest_input_measurements_.resize(cameras_.size());
    for (TimedPoseMeasurements& measurements : latest_input_measurements_) {
        measurements.frame_id = config_.output_frame;
    }
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

    std::vector<BufferedCameraFrame> rig_candidate_frames;
    if (config_.multi_cam_rig_enable) {
        BufferedCameraFrame current_frame;
        current_frame.valid = true;
        current_frame.camera_index = camera_index;
        current_frame.stamp = stamp;
        current_frame.camera_to_output = camera_to_output;
        current_frame.output_to_camera = output_to_camera;
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
            if (represented_camera_count >= static_cast<std::size_t>(config_.multi_cam_min_cameras)
                && standard_rig_geometry) {
                std::string rig_method;
                const std::vector<ScoredRigPose> refined_rig_candidates = refineRigCandidates(
                    solveRigPoses(rig_observations, &rig_method), rig_observations);
                if (!refined_rig_candidates.empty()) {
                    std::vector<CameraPose> base_poses;
                    base_poses.reserve(refined_rig_candidates.size());
                    for (const ScoredRigPose& candidate : refined_rig_candidates) {
                        base_poses.push_back(candidate.pose);
                    }

                    std::vector<unc::PoseDistributionComponent> components;
                    components.reserve(refined_rig_candidates.size());
                    for (std::size_t index = 0U;
                         index < refined_rig_candidates.size(); ++index) {
                        Eigen::Matrix<double, 6, 6> branch_covariance =
                            Eigen::Matrix<double, 6, 6>::Identity()
                            * fallback_covariance;
                        switch (config_.uncertainty_solver) {
                            case UncertaintySolver::JacobianPropagation:
                                branch_covariance = poseCovarianceByRigJacobianPropagation(
                                    refined_rig_candidates[index].pose,
                                    rig_observations);
                                break;
                            case UncertaintySolver::MonteCarlo:
                                branch_covariance = poseCovarianceByRigMonteCarlo(
                                    base_poses,
                                    rig_observations,
                                    static_cast<int>(index));
                                break;
                            case UncertaintySolver::EllipseTransform:
                                branch_covariance = poseCovarianceByRigEllipseTransform(
                                    base_poses,
                                    rig_observations,
                                    static_cast<int>(index));
                                break;
                        }
                        components.push_back({
                            refined_rig_candidates[index].pose,
                            branch_covariance,
                            1.0});
                    }
                    const auto moments = unc::momentMatchPoseDistribution(components);
                    if (moments) {
                        measurements.poses.push_back(toRigMeasurement(
                            target,
                            moments->pose,
                            moments->covariance,
                            rig_method));
                        rig_solution_published = true;
                    }
                }
            }
        }

        // Use a synchronized generalized solve only when at least two cameras
        // contribute observable ray sets. Otherwise solve the triggering
        // camera independently.
        if (rig_solution_published || observations.size() < 2U) {
            continue;
        }

        // Minimal solvers can return multiple algebraic candidates. Retain all
        // visibility-admissible branches and moment-match them below; no
        // residual or temporal tie-break is allowed to hide ambiguity.
        std::string method;
        std::vector<ScoredCameraPose> refined_candidates;
        if (observations.size() == 2U) {
            method = "V2";
            for (VisibilityPoseEstimate& estimate : solveVisibilityPoses(observations)) {
                if (estimate.pose.translation.allFinite()
                    && estimate.pose.rotation.allFinite()
                    && estimate.covariance.allFinite()
                    && std::isfinite(estimate.probability)
                    && estimate.probability > 0.0) {
                    refined_candidates.push_back({
                        std::move(estimate.pose),
                        0.0,
                        std::move(estimate.covariance),
                        estimate.probability});
                }
            }
        } else {
            refined_candidates = refineCandidates(
                solveCameraPoses(observations, camera, &method),
                observations,
                camera);
        }
        if (refined_candidates.empty()) {
            continue;
        }

        std::vector<CameraPose> base_poses;
        base_poses.reserve(refined_candidates.size());
        for (const ScoredCameraPose& candidate : refined_candidates) {
            base_poses.push_back(candidate.pose);
        }

        std::vector<unc::PoseDistributionComponent> components;
        components.reserve(refined_candidates.size());
        for (std::size_t index = 0U; index < refined_candidates.size(); ++index) {
            Eigen::Matrix<double, 6, 6> branch_covariance =
                Eigen::Matrix<double, 6, 6>::Identity() * fallback_covariance;
            if (refined_candidates[index].visibility_covariance) {
                branch_covariance = *refined_candidates[index].visibility_covariance;
            } else {
                switch (config_.uncertainty_solver) {
                    case UncertaintySolver::JacobianPropagation:
                        branch_covariance = poseCovarianceByJacobianPropagation(
                            refined_candidates[index].pose, observations, camera);
                        break;
                    case UncertaintySolver::MonteCarlo:
                        branch_covariance = poseCovarianceByMonteCarlo(
                            base_poses,
                            observations,
                            camera,
                            static_cast<int>(index));
                        break;
                    case UncertaintySolver::EllipseTransform:
                        branch_covariance = poseCovarianceByEllipseTransform(
                            base_poses,
                            observations,
                            camera,
                            static_cast<int>(index));
                        break;
                }
            }
            // Visibility is a feasibility test, not a calibrated likelihood,
            // so discrete algebraic branches receive equal prior weight. V2
            // supplies its continuous feasible-measure weight explicitly.
            components.push_back({
                refined_candidates[index].pose,
                branch_covariance,
                refined_candidates[index].distribution_weight});
        }
        const auto moments = unc::momentMatchPoseDistribution(components);
        if (!moments) {
            continue;
        }

        const Eigen::Matrix<double, 6, 6> covariance = uvdar_core::helpers::rotatePoseCovariance(
            moments->covariance,
            camera_to_output.rotation());
        measurements.poses.push_back(toMeasurement(
            target, moments->pose, camera_to_output, covariance, method));
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
    std::string* method) const
{
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
    const std::vector<Eigen::Vector3d> observed_positions =
        observedMarkerPositions(observations);
    const double minimum_view_cosine =
        std::cos(config_.visibility_pose.visibility_half_angle_rad);
    for (const CameraPose& candidate : candidates) {
        CameraPose refined_pose = refinePose(candidate, observations, camera);
        const double error = reprojectionError(refined_pose, observations, camera);
        const BodyModel::VisibilityScore visibility = body_.visibilityScore(
            refined_pose, observed_positions);
        if (!std::isfinite(error)
            || !hasPositiveDepths(refined_pose, observations)
            || !std::isfinite(visibility.observed_view_cosine_minimum)
            || visibility.observed_view_cosine_minimum
                < minimum_view_cosine) {
            continue;
        }
        const auto duplicate = std::find_if(
            refined_candidates.begin(),
            refined_candidates.end(),
            [&](const ScoredCameraPose& existing) {
                return unc::poseTangentDistance(existing.pose, refined_pose)
                    <= duplicate_pose_threshold;
            });
        if (duplicate == refined_candidates.end()) {
            refined_candidates.push_back({std::move(refined_pose), error});
        } else if (error < duplicate->reprojection_error) {
            duplicate->pose = std::move(refined_pose);
            duplicate->reprojection_error = error;
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
    if (observations.size() < 3U
        || !hasObservableRigMarkerGeometry(observations)) {
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

    std::vector<PoseSolution> solutions = solveUnconstrained();
    if (method != nullptr) {
        *method = used_gpnp_fallback
            ? "gPnP" : signatureForCount(observations.size());
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

std::vector<GeometricSolver::ScoredRigPose> GeometricSolver::refineRigCandidates(
    const std::vector<CameraPose>& candidates,
    const std::vector<RigObservation>& observations) const
{
    std::vector<ScoredRigPose> output;
    output.reserve(candidates.size());
    for (const CameraPose& candidate : candidates) {
        CameraPose refined = refineRigPose(candidate, observations);
        const double error = rigReprojectionError(refined, observations);
        if (std::isfinite(error)
            && hasPositiveRigDepths(refined, observations)
            && rigVisibilityScore(refined, observations)) {
            const auto duplicate = std::find_if(
                output.begin(), output.end(),
                [&](const ScoredRigPose& existing) {
                    return unc::poseTangentDistance(existing.pose, refined)
                        <= duplicate_pose_threshold;
                });
            if (duplicate == output.end()) {
                output.push_back({std::move(refined), error});
            } else if (error < duplicate->reprojection_error) {
                duplicate->pose = std::move(refined);
                duplicate->reprojection_error = error;
            }
        }
    }
    return output;
}

GeometricSolver::CameraPose GeometricSolver::refineRigPose(
    const CameraPose& seed,
    const std::vector<RigObservation>& observations) const
{
    uvdar_core::helpers::LevenbergMarquardtOptions options;
    options.max_iterations = std::max(0, config_.multi_cam_refinement_iterations);
    options.initial_damping = std::max(config_.multi_cam_damping, 1.0e-15);
    options.step_tolerance = std::max(0.0, config_.multi_cam_step_tolerance);
    options.residual_tolerance = std::max(0.0, config_.multi_cam_residual_tolerance);
    const auto optimized = uvdar_core::helpers::levenbergMarquardt(
        seed,
        6,
        [&](const CameraPose& pose, const bool with_jacobian)
            -> std::optional<uvdar_core::helpers::LeastSquaresLinearization> {
            uvdar_core::helpers::LeastSquaresLinearization output;
            output.residual.resize(2 * observations.size());
            if (with_jacobian) {
                output.jacobian.resize(2 * observations.size(), 6);
            }
            for (std::size_t index = 0U; index < observations.size(); ++index) {
                const RigObservation& observation = observations[index];
                if (observation.camera_index >= cameras_.size()) {
                    return std::nullopt;
                }
                const CameraModel& camera = cameras_[observation.camera_index];
                const Eigen::Vector3d rotated = pose.rotation
                    * observation.observation.marker.pose.position;
                const Eigen::Vector3d camera_point = observation.output_to_camera
                    * (rotated + pose.translation);
                if (camera_point.z() <= epsilon) {
                    return std::nullopt;
                }
                const Eigen::Matrix2d covariance = unc::regularizedCovariance(
                    observation.observation.point.has_prediction
                        ? observation.observation.point.prediction_covariance
                        : observation.observation.point.covariance,
                    config_.covariance_regularization_px);
                Eigen::LLT<Eigen::Matrix2d> llt(covariance);
                if (llt.info() != Eigen::Success) {
                    return std::nullopt;
                }
                const Eigen::Matrix2d whitening = llt.matrixL().solve(
                    Eigen::Matrix2d::Identity());
                output.residual.segment<2>(2 * index) = whitening
                    * (camera.project(camera_point)
                        - observation.observation.image_point);
                if (with_jacobian) {
                    const Eigen::Matrix<double, 2, 3> projection =
                        camera.projectionJacobian(camera_point)
                        * observation.output_to_camera.rotation();
                    output.jacobian.block<2, 3>(2 * index, 0) =
                        whitening * projection;
                    output.jacobian.block<2, 3>(2 * index, 3) = whitening
                        * projection * (-uvdar_core::helpers::skew(rotated));
                }
            }
            return output;
        },
        [](const CameraPose& pose, const Eigen::VectorXd& delta) {
            CameraPose candidate = pose;
            applyLeftCameraPoseIncrement(
                candidate, delta.head<3>(), delta.tail<3>());
            return candidate;
        },
        options);
    return optimized.state;
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
        if (!std::isfinite(visibility.observed_view_cosine_minimum)
            || visibility.observed_view_cosine_minimum
                < std::cos(config_.visibility_pose.visibility_half_angle_rad)
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

std::vector<VisibilityPoseEstimate> GeometricSolver::solveVisibilityPoses(
    const std::vector<Observation>& observations) const
{
    if (observations.size() != 2U) {
        return {};
    }
    const Observation& first = observations[0];
    const Observation& second = observations[1];
    return VisibilityPoseSolver::solve(
        first.marker.pose.position,
        second.marker.pose.position,
        first.marker.pose.orientation * Eigen::Vector3d::UnitX(),
        second.marker.pose.orientation * Eigen::Vector3d::UnitX(),
        first.bearing,
        second.bearing,
        config_.visibility_pose);
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
                solveCameraPoses(sample_observations, camera),
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
            solveCameraPoses(sample_observations, camera),
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
    uvdar_core::helpers::LevenbergMarquardtOptions options;
    options.max_iterations = std::max(0, config_.refinement_iterations);
    options.initial_damping = std::max(config_.pnp_damping, 1.0e-15);
    options.step_tolerance = std::max(0.0, config_.pnp_step_tolerance);
    options.residual_tolerance = std::max(0.0, config_.pnp_residual_tolerance);
    const auto optimized = uvdar_core::helpers::levenbergMarquardt(
        seed,
        6,
        [&](const CameraPose& pose, const bool with_jacobian)
            -> std::optional<uvdar_core::helpers::LeastSquaresLinearization> {
            uvdar_core::helpers::LeastSquaresLinearization output;
            output.residual.resize(2 * observations.size());
            if (with_jacobian) {
                output.jacobian.resize(2 * observations.size(), 6);
            }
            for (std::size_t index = 0U; index < observations.size(); ++index) {
                const Observation& observation = observations[index];
                const Eigen::Vector3d camera_point = transformPoint(
                    pose, observation.marker.pose.position);
                if (camera_point.z() <= epsilon) {
                    return std::nullopt;
                }
                const Eigen::Matrix2d covariance = unc::regularizedCovariance(
                    observation.point.has_prediction
                        ? observation.point.prediction_covariance
                        : observation.point.covariance,
                    config_.covariance_regularization_px);
                Eigen::LLT<Eigen::Matrix2d> llt(covariance);
                if (llt.info() != Eigen::Success) {
                    return std::nullopt;
                }
                const Eigen::Matrix2d whitening = llt.matrixL().solve(
                    Eigen::Matrix2d::Identity());
                output.residual.segment<2>(2 * index) = whitening
                    * (camera.project(camera_point) - observation.image_point);
                if (with_jacobian) {
                    output.jacobian.block<2, 6>(2 * index, 0) = whitening
                        * unc::imageProjectionJacobian(
                            camera, pose, observation.marker.pose.position);
                }
            }
            return output;
        },
        [](const CameraPose& pose, const Eigen::VectorXd& delta) {
            CameraPose candidate = pose;
            applyLeftCameraPoseIncrement(
                candidate, delta.head<3>(), delta.tail<3>());
            return candidate;
        },
        options);
    return optimized.state;
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
