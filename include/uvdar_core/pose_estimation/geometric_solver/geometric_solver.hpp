#pragma once

#include <map>
#include <mutex>
#include <optional>
#include <vector>

#include "uvdar_core/pose_estimation/body_model.hpp"
#include "uvdar_core/pose_estimation/camera_model.hpp"
#include "uvdar_core/pose_estimation/i_pose_estimator.hpp"
#include "uvdar_core/pose_estimation/uncertainty.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

/**
 * @brief Supported uncertainty propagation methods.
 */
enum class UncertaintySolver {
    JacobianPropagation,
    MonteCarlo,
    EllipseTransform,
};

/**
 * @brief Parameters for the deterministic geometric pose-estimation backend.
 */
struct GeometricSolverConfig {
    UncertaintySolver uncertainty_solver = UncertaintySolver::JacobianPropagation;
    int uncertainty_samples = 5000;
    bool debug = false;
    std::string output_frame = "local_origin";
    std::vector<int> signal_ids;
    int signals_per_target = 1;
    bool enable_p2p = true;
    // Target model-frame direction opposite gravity. +Z is the UVDAR model
    // convention; configure a different normalized direction when needed.
    Eigen::Vector3d p2p_model_gravity_axis = Eigen::Vector3d::UnitZ();
    double p4p_reprojection_threshold_rad = 0.01;
    double covariance_regularization_px = 1.0e-6;
    int refinement_iterations = 8;
    int pnp_max_iterations = 40;
    double pnp_damping = 1.0e-8;
    double pnp_finite_difference_eps = 1.0e-6;
    double pnp_step_tolerance = 1.0e-10;
    double pnp_residual_tolerance = 1.0e-10;
};

/**
 * @brief Direct geometric pose estimator using P2P/P3P/P4P/PnP by marker count.
 *
 * The solver first associates tracked signal ids to body LEDs, converts image
 * points to unit bearings, chooses a minimal solver by observation count, and
 * then refines the selected candidate by weighted Gauss-Newton reprojection
 * minimization.
 */
class GeometricSolver final : public IPoseEstimator {
public:
    GeometricSolver(GeometricSolverConfig config, BodyModel body, std::vector<CameraModel> cameras);

    /**
     * @brief Set the navigation-derived body-up direction expressed in a camera frame.
     *
     * P2P uses this as its additional orientation constraint.  Clearing the
     * value disables only P2P for that camera; P3P and larger solvers remain
     * unaffected.
     */
    void setCameraUpAxis(std::size_t camera_index, std::optional<Eigen::Vector3d> camera_up_axis);

    /**
     * @brief Solve poses for all targets visible in one camera frame.
     */
    void processFrame(
        std::size_t camera_index,
        const std::vector<TrackedPoint>& points,
        int image_width,
        int image_height,
        double stamp,
        const Eigen::Isometry3d& camera_to_output,
        const Eigen::Isometry3d& output_to_camera) override;

    /**
     * @brief Return the latest deterministic solution batch.
     */
    TimedPoseMeasurements scatterAndMeasure(double now, double stamp) override;

    /**
     * @brief Direct-solver outputs are treated as verified hypotheses.
     */
    std::vector<PoseMeasurement> verifiedHypotheses() const override;

    /**
     * @brief Direct solver has no tentative particle state.
     */
    std::vector<PoseMeasurement> tentativeHypotheses() const override;

private:
    /**
     * @brief One 2D-3D correspondence after signal association.
     */
    struct Observation {
        TrackedPoint point;
        LEDMarker marker;
        Eigen::Vector2d image_point = Eigen::Vector2d::Zero();
        Eigen::Vector3d bearing = Eigen::Vector3d::UnitZ();
    };

    using CameraPose = uvdar_core::pose_estimation::CameraPose;

    /**
     * @brief A candidate pose after pixel-space refinement and scoring.
     */
    struct ScoredCameraPose {
        CameraPose pose;
        double reprojection_error = 0.0;
    };

    /**
     * @brief Convert a tracked point into a marker-bearing correspondence.
     */
    std::optional<Observation> makeObservation(const TrackedPoint& point, const CameraModel& camera) const;

    /**
     * @brief Dispatch to P2P, P3P, P4P, or iterative PnP by correspondence count.
     */
    std::vector<CameraPose> solveCameraPoses(
        const std::vector<Observation>& observations,
        const CameraModel& camera,
        const std::optional<Eigen::Vector3d>& camera_up_axis) const;

    /**
     * @brief Refine solver candidates and discard invalid pixel reprojections.
     */
    std::vector<ScoredCameraPose> refineCandidates(
        const std::vector<CameraPose>& candidates,
        const std::vector<Observation>& observations,
        const CameraModel& camera) const;

    /**
     * @brief Two-point pose from two 3D points, two bearings, and gravity axes.
     */
    std::vector<CameraPose> solveP2P(
        const std::vector<Observation>& observations,
        const std::optional<Eigen::Vector3d>& camera_up_axis) const;

    /**
     * @brief Perspective-3-point solver using quartic depth constraints.
     */
    std::vector<CameraPose> solveP3P(const std::vector<Observation>& observations) const;

    /**
     * @brief Verify that every observed marker is in front of the candidate camera.
     */
    bool hasPositiveDepths(const CameraPose& pose, const std::vector<Observation>& observations) const;

    /**
     * @brief Select a pose by generic body-model observability and, if needed, continuity.
     */
    std::optional<std::size_t> selectObservabilityAwareCandidate(
        const std::vector<ScoredCameraPose>& candidates,
        const std::vector<Observation>& observations,
        const std::optional<CameraPose>& previous_pose) const;

    /**
     * @brief Return the blended physical marker positions detected in this frame.
     */
    std::vector<Eigen::Vector3d> observedMarkerPositions(const std::vector<Observation>& observations) const;

    /**
     * @brief Perspective-4-point algebraic solver with angular residual gating.
     */
    std::vector<CameraPose> solveP4P(const std::vector<Observation>& observations) const;

    /**
     * @brief General PnP by finite-difference Levenberg-Marquardt on bearings.
     */
    std::optional<CameraPose> solvePnP(const std::vector<Observation>& observations) const;

    /**
     * @brief Residual vector r_i = normalize(R X_i + t) - bearing_i.
     */
    std::optional<Eigen::VectorXd> bearingResidualVector(const std::vector<Observation>& observations, const CameraPose& pose) const;

    /**
     * @brief Weighted Gauss-Newton refinement on pixel reprojection residuals.
     */
    CameraPose refinePose(const CameraPose& seed, const std::vector<Observation>& observations, const CameraModel& camera) const;

    /**
     * @brief Sum of squared image reprojection residuals for candidate ranking.
     */
    double reprojectionError(const CameraPose& pose, const std::vector<Observation>& observations, const CameraModel& camera) const;

    /**
     * @brief Propagate 2D tracker covariance through projection Jacobians to 6D pose.
     */
    Eigen::Matrix<double, 6, 6> poseCovarianceByJacobianPropagation(
        const CameraPose& pose,
        const std::vector<Observation>& observations,
        const CameraModel& camera) const;

    /**
     * @brief Propagate detector uncertainty by Monte-Carlo sampling.
     */
    Eigen::Matrix<double, 6, 6> poseCovarianceByMonteCarlo(
        const std::vector<CameraPose>& base_poses,
        const std::vector<Observation>& observations,
        const CameraModel& camera,
        const std::optional<Eigen::Vector3d>& camera_up_axis,
        int selected_index) const;

    /**
     * @brief Propagate detector uncertainty with deterministic ellipse-transform points.
     */
    Eigen::Matrix<double, 6, 6> poseCovarianceByEllipseTransform(
        const std::vector<CameraPose>& base_poses,
        const std::vector<Observation>& observations,
        const CameraModel& camera,
        const std::optional<Eigen::Vector3d>& camera_up_axis,
        int selected_index) const;

    /**
     * @brief Build 2N x 2N detector pixel covariance from observations.
     */
    Eigen::MatrixXd detectorCovariance(const std::vector<Observation>& observations) const;

    /**
     * @brief Return stacked pixel coordinates for all observations.
     */
    Eigen::VectorXd observationVector(const std::vector<Observation>& observations) const;

    /**
     * @brief Build observations from a sampled detector vector.
     */
    std::vector<Observation> observationsFromVector(
        const std::vector<Observation>& base_observations,
        const Eigen::VectorXd& sample,
        const CameraModel& camera) const;

    /**
     * @brief Transform body-to-camera pose into the configured output frame.
     */
    PoseMeasurement toMeasurement(int target, const CameraPose& camera_pose, const Eigen::Isometry3d& camera_to_output, const Eigen::Matrix<double, 6, 6>& covariance) const;

    GeometricSolverConfig config_;
    BodyModel body_;
    std::vector<CameraModel> cameras_;
    mutable std::mutex mutex_;
    TimedPoseMeasurements latest_measurements_;
    std::map<std::pair<std::size_t, int>, CameraPose> latest_camera_poses_;
    std::vector<std::optional<Eigen::Vector3d>> camera_up_axes_;
};

} // namespace uvdar_core::pose_estimation::geometric_solver
