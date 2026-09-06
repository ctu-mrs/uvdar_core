#pragma once

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "uvdar_core/pose_estimation/types.hpp"

namespace uvdar_core::pose_estimation {

/**
 * @brief One 6D pose measurement consumed by the Kalman filter.
 *
 * x is ordered as [position(3), roll-pitch-yaw(3)]. The covariance is supplied
 * by the pose estimator and is not recomputed inside this filter.
 */
struct KfPoseMeasurement {
    int id = -1;
    Eigen::VectorXd x;
    // Supplied by a pose estimator. This filter does not inspect the LED body
    // model or camera model to derive measurement uncertainty.
    Eigen::MatrixXd covariance;
    double stamp = 0.0;
    // Wall-clock receive time used for short anonymous-track prediction padding
    // when multiple cameras publish nearly simultaneously.
    double receipt_stamp = 0.0;
    std::string camera_frame;
};

/**
 * @brief Internal/output KF state.
 *
 * With velocity disabled x is [p, rpy]. With identity-based velocity enabled
 * x is [p, v, rpy], and H selects [p, rpy] for measurement correction.
 */
struct KfPoseState {
    int id = -1;
    Eigen::VectorXd x;
    Eigen::MatrixXd covariance;
    int update_count = 0;
};

/**
 * @brief Parameters of the relative-pose Kalman filter.
 */
struct KfPoseConfig {
    bool debug = false;
    bool anonymous_measurements = false;
    bool indoor = false;
    bool use_velocity = false;
    int min_measurements_to_validation = 10;
    double decay_age_normal = 3.0;
    double decay_age_unvalidated = 1.0;
    double match_level_threshold_associate = 0.3;
    double match_level_threshold_remove = 0.5;
    std::string output_frame = "local_origin";
    std::function<bool(const Eigen::Vector3d&, const std::string&, double)> accepts_correction;
};

/**
 * @brief Kalman filter for UVDAR relative poses.
 *
 * Measurements already contain full pose covariance, position overlap gates
 * association, and process noise is a fixed heuristic rather than a full target
 * dynamics model.
 */
class KfPose {
public:
    explicit KfPose(KfPoseConfig config);

    /**
     * @brief Associate and correct tracks with new pose measurements.
     *
     * Anonymous mode uses Gaussian overlap matching; identified mode uses the
     * measurement id modulo 1000.
     */
    void applyMeasurements(const std::vector<KfPoseMeasurement>& measurements);

    /**
     * @brief Predict all states to now and remove stale/invalid tracks.
     */
    void spin(double now);

    /**
     * @brief States with enough corrections to be considered validated.
     */
    std::vector<KfPoseState> validatedStates() const;

    /**
     * @brief States below the validation update-count threshold.
     */
    std::vector<KfPoseState> tentativeStates() const;

private:
    struct FilterData {
        KfPoseState state;
        double latest_update = 0.0;
        double latest_measurement = 0.0;
    };

    /**
     * @brief Initialize a new Gaussian state from a measurement.
     */
    void initiateNew(const KfPoseMeasurement& measurement, int id);

    /**
     * @brief Perform greedy Gaussian-overlap association for anonymous targets.
     */
    void applyMeasurementsAnonymous(const std::vector<KfPoseMeasurement>& measurements);

    /**
     * @brief Correct tracks by explicit target id.
     */
    void applyMeasurementsWithIdentity(const std::vector<KfPoseMeasurement>& measurements);

    /**
     * @brief Predict x and P with x'=A(dt)x, P'=A P A^T + Q(dt).
     */
    KfPoseState predictTillTime(FilterData& data, double target_time, bool apply_update);

    /**
     * @brief Standard linear Kalman correction with position-overlap covariance inflation.
     */
    KfPoseState correctWithMeasurement(FilterData& data, const KfPoseMeasurement& measurement, double& match_level, bool prior_predict, bool apply_update);

    /**
     * @brief Gaussian product peak used as a positional association score.
     */
    double gaussJointMaxVal(const Eigen::MatrixXd& sigma0, const Eigen::MatrixXd& sigma1, const Eigen::VectorXd& mu0, const Eigen::VectorXd& mu1) const;

    /**
     * @brief Drop tracks containing non-finite state or covariance entries.
     */
    void removeNans();

    /**
     * @brief Merge anonymous tracks whose Gaussian position ellipsoids overlap.
     */
    void removeOverlaps();

    /**
     * @brief State-transition matrix A(dt).
     */
    Eigen::MatrixXd aDt(double dt) const;

    /**
     * @brief Measurement matrix H mapping state to [p, rpy].
     */
    Eigen::MatrixXd h() const;

    /**
     * @brief Process-noise covariance Q(dt) from UVDAR motion heuristics.
     */
    Eigen::MatrixXd qDt(double dt) const;

    /**
     * @brief Choose the 2pi-equivalent angle nearest to a reference measurement.
     */
    double fixAngle(double original, double measurement) const;

    KfPoseConfig config_;
    double vl_ = 2.0;
    double vv_ = 1.0;
    double sn_ = 4.0;
    int next_id_ = 0;
    std::vector<FilterData> states_;
};

} // namespace uvdar_core::pose_estimation
