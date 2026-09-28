#pragma once

#include <map>
#include <mutex>
#include <optional>
#include <vector>

#include "uvdar_core/pose_estimation/body_model.hpp"
#include "uvdar_core/pose_estimation/camera_model.hpp"
#include "uvdar_core/pose_estimation/geometric_solver/visibility_pose.hpp"
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
    VisibilityPoseConfig visibility_pose;
    double p4p_reprojection_threshold_rad = 0.01;
    double covariance_regularization_px = 1.0e-6;
    int refinement_iterations = 8;
    int pnp_max_iterations = 40;
    double pnp_damping = 1.0e-8;
    double pnp_finite_difference_eps = 1.0e-6;
    double pnp_step_tolerance = 1.0e-10;
    double pnp_residual_tolerance = 1.0e-10;
    // When enabled, combine approximately synchronized observations from
    // rigidly connected cameras as non-central rays in output_frame.
    bool multi_cam_rig_enable = false;
    double multi_cam_sync_tolerance_sec = 0.03;
    int multi_cam_min_cameras = 2;
    double multi_cam_max_angular_error_rad = 0.05;
    int multi_cam_refinement_iterations = 40;
    double multi_cam_damping = 1.0e-8;
    double multi_cam_step_tolerance = 1.0e-10;
    double multi_cam_residual_tolerance = 1.0e-10;
    int multi_cam_gp3p_depth_seed_levels = 7;
    int multi_cam_gp3p_depth_iterations = 60;
    double multi_cam_gp3p_root_tolerance = 1.0e-9;
};

/**
 * @brief Direct geometric pose estimator using visibility/P3P/P4P/PnP by marker count.
 *
 * The solver first associates tracked signal ids to body LEDs, converts image
 * points to unit bearings, chooses a solver by observation count, refines all
 * admissible algebraic candidates by weighted analytic LM reprojection, and
 * moment-matches any remaining pose ambiguity.
 */
class GeometricSolver final : public IPoseEstimator {
public:
    GeometricSolver(GeometricSolverConfig config, BodyModel body, std::vector<CameraModel> cameras);

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
        std::optional<Eigen::Matrix<double, 6, 6>> visibility_covariance;
        double distribution_weight = 1.0;
    };

    /** @brief One observation expressed as a non-central ray in output_frame. */
    struct RigObservation {
        Observation observation;
        std::size_t camera_index = 0U;
        Eigen::Isometry3d camera_to_output = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d output_to_camera = Eigen::Isometry3d::Identity();
        Eigen::Vector3d ray_origin = Eigen::Vector3d::Zero();
        Eigen::Vector3d ray_direction = Eigen::Vector3d::UnitZ();
    };

    /** @brief Last timestamped observation map retained for one camera. */
    struct BufferedCameraFrame {
        bool valid = false;
        std::size_t camera_index = 0U;
        double stamp = 0.0;
        Eigen::Isometry3d camera_to_output = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d output_to_camera = Eigen::Isometry3d::Identity();
        std::map<int, std::vector<Observation>> observations_by_target;
    };

    /** @brief A body-to-output candidate refined against all rig cameras. */
    struct ScoredRigPose {
        CameraPose pose;
        double reprojection_error = 0.0;
    };

    /** @brief Aggregate visibility score valid only when every rig camera agrees. */
    struct RigVisibilityScore {
        double visibility_margin = 0.0;
        double observed_view_cosine_mean = 0.0;
    };

    /**
     * @brief Convert a tracked point into a marker-bearing correspondence.
     */
    std::optional<Observation> makeObservation(const TrackedPoint& point, const CameraModel& camera) const;

    /**
     * @brief Dispatch to P3P, P4P, or iterative PnP by correspondence count.
     */
    std::vector<CameraPose> solveCameraPoses(
        const std::vector<Observation>& observations,
        const CameraModel& camera,
        std::string* method = nullptr) const;

    /**
     * @brief Refine solver candidates and discard invalid pixel reprojections.
     */
    std::vector<ScoredCameraPose> refineCandidates(
        const std::vector<CameraPose>& candidates,
        const std::vector<Observation>& observations,
        const CameraModel& camera) const;

    /** @brief Convert synchronized camera observations into output-frame rays. */
    std::vector<RigObservation> makeRigObservations(
        int target,
        const std::vector<BufferedCameraFrame>& frames,
        double reference_stamp) const;

    /** @brief Dispatch by ray count to GP3P, GP4_5P, GP6P, or GPnP. */
    std::vector<CameraPose> solveRigPoses(
        const std::vector<RigObservation>& observations,
        std::string* method = nullptr) const;

    /**
     * @brief Require three distinct non-collinear physical body markers.
     *
     * Seeing the same two LEDs from many cameras improves triangulation but
     * does not constrain rotation about their body-frame baseline.
     */
    bool hasObservableRigMarkerGeometry(
        const std::vector<RigObservation>& observations) const;

    /** @brief Jointly refine generalized candidates in all cameras' pixel spaces. */
    std::vector<ScoredRigPose> refineRigCandidates(
        const std::vector<CameraPose>& candidates,
        const std::vector<RigObservation>& observations) const;

    /** @brief Weighted multi-camera pixel refinement of a body-to-output pose. */
    CameraPose refineRigPose(
        const CameraPose& seed,
        const std::vector<RigObservation>& observations) const;

    /** @brief Joint squared pixel reprojection error over all cameras. */
    double rigReprojectionError(
        const CameraPose& pose,
        const std::vector<RigObservation>& observations) const;

    /** @brief Verify positive camera-frame depth for every rig observation. */
    bool hasPositiveRigDepths(
        const CameraPose& pose,
        const std::vector<RigObservation>& observations) const;

    /**
     * @brief Require every observed physical LED group to face its source camera.
     *
     * Generalized minimal solvers know only ray geometry. This applies the
     * directional LED constraints independently in every contributing camera.
     */
    std::optional<RigVisibilityScore> rigVisibilityScore(
        const CameraPose& pose,
        const std::vector<RigObservation>& observations) const;

    /**
     * @brief Visibility-constrained distribution from two markers and bearings.
     */
    std::vector<VisibilityPoseEstimate> solveVisibilityPoses(
        const std::vector<Observation>& observations) const;

    /**
     * @brief Perspective-3-point solver using quartic depth constraints.
     */
    std::vector<CameraPose> solveP3P(const std::vector<Observation>& observations) const;

    /**
     * @brief Verify that every observed marker is in front of the candidate camera.
     */
    bool hasPositiveDepths(const CameraPose& pose, const std::vector<Observation>& observations) const;

    /**
     * @brief Return the blended physical marker positions detected in this frame.
     */
    std::vector<Eigen::Vector3d> observedMarkerPositions(const std::vector<Observation>& observations) const;

    /**
     * @brief Perspective-4-point algebraic solver with angular residual gating.
     */
    std::vector<CameraPose> solveP4P(const std::vector<Observation>& observations) const;

    /**
     * @brief Weighted analytic LM refinement on pixel reprojection residuals.
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

    /** @brief Joint Fisher-information covariance from every rig camera. */
    Eigen::Matrix<double, 6, 6> poseCovarianceByRigJacobianPropagation(
        const CameraPose& pose,
        const std::vector<RigObservation>& observations) const;

    /** @brief Joint multi-camera Monte-Carlo uncertainty propagation. */
    Eigen::Matrix<double, 6, 6> poseCovarianceByRigMonteCarlo(
        const std::vector<CameraPose>& base_poses,
        const std::vector<RigObservation>& observations,
        int selected_index) const;

    /** @brief Joint deterministic ellipse-transform uncertainty propagation. */
    Eigen::Matrix<double, 6, 6> poseCovarianceByRigEllipseTransform(
        const std::vector<CameraPose>& base_poses,
        const std::vector<RigObservation>& observations,
        int selected_index) const;

    /**
     * @brief Propagate detector uncertainty by Monte-Carlo sampling.
     */
    Eigen::Matrix<double, 6, 6> poseCovarianceByMonteCarlo(
        const std::vector<CameraPose>& base_poses,
        const std::vector<Observation>& observations,
        const CameraModel& camera,
        int selected_index) const;

    /**
     * @brief Propagate detector uncertainty with deterministic ellipse-transform points.
     */
    Eigen::Matrix<double, 6, 6> poseCovarianceByEllipseTransform(
        const std::vector<CameraPose>& base_poses,
        const std::vector<Observation>& observations,
        const CameraModel& camera,
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

    /** @brief Stack all synchronized rig pixels and their block covariance. */
    Eigen::VectorXd rigObservationVector(const std::vector<RigObservation>& observations) const;
    Eigen::MatrixXd rigDetectorCovariance(const std::vector<RigObservation>& observations) const;

    /** @brief Rebuild calibrated output-frame rays after pixel perturbation. */
    std::vector<RigObservation> rigObservationsFromVector(
        const std::vector<RigObservation>& base_observations,
        const Eigen::VectorXd& sample) const;

    /**
     * @brief Transform body-to-camera pose into the configured output frame.
     */
    PoseMeasurement toMeasurement(
        int target,
        const CameraPose& camera_pose,
        const Eigen::Isometry3d& camera_to_output,
        const Eigen::Matrix<double, 6, 6>& covariance,
        const std::string& method) const;

    /** @brief Publish a body-to-output generalized pose without another transform. */
    PoseMeasurement toRigMeasurement(
        int target,
        const CameraPose& output_pose,
        const Eigen::Matrix<double, 6, 6>& covariance,
        const std::string& method) const;

    GeometricSolverConfig config_;
    BodyModel body_;
    std::vector<CameraModel> cameras_;
    mutable std::mutex mutex_;
    TimedPoseMeasurements latest_measurements_;
    // Each input owns its latest batch so an empty callback from one camera
    // cannot erase a valid result produced by another camera.
    std::vector<TimedPoseMeasurements> latest_input_measurements_;
    std::vector<BufferedCameraFrame> latest_camera_frames_;
};

} // namespace uvdar_core::pose_estimation::geometric_solver
