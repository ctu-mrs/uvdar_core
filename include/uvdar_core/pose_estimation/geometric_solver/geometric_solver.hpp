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
 * @brief Parameters for the deterministic geometric pose-estimation backend.
 */
struct GeometricSolverConfig {
    bool debug = false;
    std::string output_frame = "local_origin";
    std::vector<int> signal_ids;
    int signals_per_target = 1;
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

    using CameraPose = uvdar_core::pose_estimation::uncertainty::CameraPose;

    /**
     * @brief Convert a tracked point into a marker-bearing correspondence.
     */
    std::optional<Observation> makeObservation(const TrackedPoint& point, const CameraModel& camera) const;

    /**
     * @brief Dispatch to P2P, P3P, P4P, or iterative PnP by correspondence count.
     */
    std::vector<CameraPose> solveCameraPoses(const std::vector<Observation>& observations, const CameraModel& camera) const;

    /**
     * @brief Two-point pose from two 3D points, two bearings, and a plane normal.
     */
    std::vector<CameraPose> solveP2P(const std::vector<Observation>& observations, const CameraModel& camera) const;

    /**
     * @brief Perspective-3-point solver using quartic depth constraints.
     */
    std::vector<CameraPose> solveP3P(const std::vector<Observation>& observations) const;

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
    Eigen::Matrix<double, 6, 6> poseCovariance(const CameraPose& pose, const std::vector<Observation>& observations, const CameraModel& camera, const Eigen::Matrix3d& camera_to_output_rotation) const;

    /**
     * @brief Transform body-to-camera pose into the configured output frame.
     */
    PoseMeasurement toMeasurement(int target, const CameraPose& camera_pose, const Eigen::Isometry3d& camera_to_output, const Eigen::Matrix<double, 6, 6>& covariance) const;

    /**
     * @brief Map global signal id to target id, or reject unknown signals.
     */
    int classifyMatch(int signal_id) const;

    /**
     * @brief Find the body LED carrying a local signal id.
     */
    std::optional<LEDMarker> markerForSignal(int signal_id) const;

    GeometricSolverConfig config_;
    BodyModel body_;
    std::vector<CameraModel> cameras_;
    mutable std::mutex mutex_;
    TimedPoseMeasurements latest_measurements_;
};

} // namespace uvdar_core::pose_estimation::geometric_solver
