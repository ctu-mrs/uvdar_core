#pragma once

#include <mutex>
#include <random>

#include "uvdar_core/pose_estimation/i_pose_estimator.hpp"
#include "uvdar_core/pose_estimation/particle_filter/reprojection_model.hpp"

namespace uvdar_core::pose_estimation::particle_filter {

struct ParticleFilterConfig {
    bool debug = false;
    bool separate_by_distance = true;
    double max_cluster_distance = 100.0;
    double scatter_time_step = 0.1;
    double mutation_position_max_step = 1.0;
    double mutation_orientation_max_step = 1.0;
    double mutation_velocity_max_step = 1.0;
    int max_hypothesis_count = 1000;
    double max_hypothesis_age = 1.5;
    std::string output_frame;
};

/**
 * @brief Particle-filter pose estimator core.
 *
 * The class owns only estimator state. ROS2 nodes provide tracker observations,
 * calibrated camera transforms, and publish the returned measurements. The
 * algorithm samples candidate poses from image clusters, scores them by
 * reprojection residual, mutates verified particles, and publishes the hull of
 * the verified particle cloud.
 */
class ParticleFilter : public uvdar_core::pose_estimation::IPoseEstimator {
public:
    ParticleFilter(ParticleFilterConfig config, uvdar_core::pose_estimation::BodyModel body, std::vector<int> signal_ids, ReprojectionModelPtr reprojection_model);

    /**
     * @brief Add hypotheses from one tracker frame and verify existing ones.
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
     * @brief Mutate/prune particles, propagate velocity, and publish measurement hulls.
     */
    TimedPoseMeasurements scatterAndMeasure(double now, double stamp) override;

    /**
     * @brief Return particles that passed the verification reprojection gate.
     */
    std::vector<PoseMeasurement> verifiedHypotheses() const override;

    /**
     * @brief Return neutral particles that are not yet verified or rejected.
     */
    std::vector<PoseMeasurement> tentativeHypotheses() const override;

private:
    /**
     * @brief Group points by target id and optionally split distant image clusters.
     */
    std::vector<ImageCluster> separateBySignals(const std::vector<TrackedPoint>& points) const;

    /**
     * @brief Map a global signal id to target index using signals_per_target.
     */
    int classifyMatch(int signal_id) const;

    /**
     * @brief Update particle flags by comparing reprojection error to two gates.
     */
    void checkHypothesisFitness(
        AssociatedHypotheses& hypotheses,
        std::size_t camera_index,
        double threshold_unfit,
        double threshold_verified,
        const std::vector<ImageCluster>& clusters,
        const Eigen::Isometry3d& output_to_camera,
        double stamp);
    /**
     * @brief Remove stale particles and randomly thin oversized hypothesis sets.
     */
    void removeExtraHypotheses(AssociatedHypotheses& hypotheses, double now);

    /**
     * @brief Draw parents and generate pose/velocity mutations.
     */
    std::vector<Hypothesis> mutateHypotheses(const AssociatedHypotheses& hypotheses, int count, double now) const;

    /**
     * @brief Perturb pose with isotropic position and SO(3) angle-axis noise.
     */
    std::vector<Hypothesis> generateMutations(const Hypothesis& source, int count, double now, double position_max_step, double angle_max_step) const;

    /**
     * @brief Perturb linear velocity by bounded random vectors.
     */
    std::vector<Hypothesis> generateVelocityMutations(const Hypothesis& source, int count, double velocity_max_step) const;

    /**
     * @brief Constant-velocity propagation p(t) = p0 + v dt.
     */
    void propagate(double now);

    /**
     * @brief Convert verified particles to one pose/covariance measurement.
     *
     * Position and orientation covariance come from enclosing ellipsoids of the
     * verified particle spread.
     */
    std::optional<PoseMeasurement> measurementHull(const AssociatedHypotheses& hypotheses) const;

    /**
     * @brief Minimum-volume enclosing ellipsoid approximation for 3D samples.
     */
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> enclosingEllipsoid(const std::vector<Eigen::Vector3d>& points) const;

    /**
     * @brief Markley-style quaternion averaging via dominant eigenvector/SVD.
     */
    Eigen::Quaterniond averageOrientation(const std::vector<Hypothesis>& hypotheses) const;

    /**
     * @brief Uniform random scalar in [0, 1].
     */
    double random01() const;

    /**
     * @brief Uniform-ish random unit vector from a normalized cube sample.
     */
    Eigen::Vector3d randomUnitVector() const;

    ParticleFilterConfig config_;
    uvdar_core::pose_estimation::BodyModel body_;
    std::vector<int> signal_ids_;
    int signals_per_target_ = 1;
    ReprojectionModelPtr reprojection_model_;

    mutable std::mutex mutex_;
    mutable std::mt19937 rng_;
    std::vector<AssociatedHypotheses> hypothesis_buffer_;
};

} // namespace uvdar_core::pose_estimation::particle_filter
