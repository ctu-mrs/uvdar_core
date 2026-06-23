#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Geometry>

#include "uvdar_core/pose_estimation/types.hpp"

namespace uvdar_core::pose_estimation {

/**
 * @brief Common ROS-independent interface for pose-estimation backends.
 *
 * Both the stochastic particle filter and deterministic geometric solver
 * consume tracked image points and camera transforms, then publish compatible
 * PoseMeasurement batches.
 */
class IPoseEstimator {
public:
    virtual ~IPoseEstimator() = default;

    /**
     * @brief Process one camera frame of tracked image points.
     *
     * camera_to_output and output_to_camera are inverse rigid transforms used
     * to evaluate the body-to-camera projection equation x = pi(T_co^-1 T_ob X).
     */
    virtual void processFrame(
        std::size_t camera_index,
        const std::vector<TrackedPoint>& points,
        int image_width,
        int image_height,
        double stamp,
        const Eigen::Isometry3d& camera_to_output,
        const Eigen::Isometry3d& output_to_camera) = 0;

    /**
     * @brief Advance estimator state and return the current pose measurements.
     *
     * Stateful filters may diffuse/propagate hypotheses here. Stateless solvers
     * return the latest frame result with the requested publication stamp.
     */
    virtual TimedPoseMeasurements scatterAndMeasure(double now, double stamp) = 0;

    /**
     * @brief Return accepted backend hypotheses as pose measurements.
     *
     * Particle filtering exposes verified particles; direct solvers expose the
     * solved poses as verified hypotheses for visualization consistency.
     */
    virtual std::vector<PoseMeasurement> verifiedHypotheses() const = 0;

    /**
     * @brief Return tentative backend hypotheses as pose measurements.
     *
     * Backends without a tentative state may return an empty vector.
     */
    virtual std::vector<PoseMeasurement> tentativeHypotheses() const = 0;
};

} // namespace uvdar_core::pose_estimation
