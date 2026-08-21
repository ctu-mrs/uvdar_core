#pragma once

#include <memory>
#include <random>

#include "uvdar_core/pose_estimation/body_model.hpp"
#include "uvdar_core/pose_estimation/camera_model.hpp"
#include "uvdar_core/pose_estimation/particle_filter/types.hpp"

namespace uvdar_core::pose_estimation::particle_filter {

using uvdar_core::pose_estimation::CameraModel;

/**
 * @brief Data needed to score one hypothesis against one camera frame.
 *
 * The body is already transformed into the candidate output-frame pose before
 * projection into the selected camera.
 */
struct ReprojectionContext {
    int target = -1;
    std::vector<ImagePointIdentified> observed_points;
    std::size_t camera_index = 0;
    Eigen::Isometry3d output_to_camera = Eigen::Isometry3d::Identity();
    uvdar_core::pose_estimation::BodyModel body;
};

/**
 * @brief Reprojection and sampling model used by the particle-filter estimator.
 *
 * This class implements stochastic initialization: sample candidate body poses,
 * project directional LEDs through the calibrated lens model, compare signal ids
 * in image space, then refine viable hypotheses by random mutation.
 */
class ReprojectionModel final {
public:
    /**
     * @brief Sampling and reprojection thresholds for the particle model.
     */
    struct Options {
        bool debug = false;
        std::vector<int> signal_ids;
        int signals_per_target = 1;
        double max_diameter = 0.0;
        int edge_detection_margin = 10;
        int initial_rough_hypothesis_count = 200;
        int initial_hypothesis_count = 10;
        int max_init_iterations = 10000;
        int max_mutation_refine_iterations = 1000;
    };

    /**
     * @brief Store camera/lens models and the target LED geometry.
     */
    ReprojectionModel(std::vector<CameraModel> cameras, uvdar_core::pose_estimation::BodyModel body, Options options);

    /**
     * @brief Back-project image point to a camera-frame bearing.
     */
    Eigen::Vector3d directionFromImagePoint(const Eigen::Vector2d& point, std::size_t camera_index) const;

    /**
     * @brief Score a particle with sum-of-squared nearest-neighbor reprojection residuals.
     */
    double hypothesisError(const Hypothesis& hypothesis, const ReprojectionContext& context) const;

    /**
     * @brief Generate initial and refined pose hypotheses from one image cluster.
     *
     * Range is initialized from apparent angular spread, then hypotheses are
     * accepted by reprojection thresholds and locally explored by random pose
     * mutations.
     */
    std::vector<Hypothesis> extractHypotheses(
        const std::vector<ImagePointIdentified>& points,
        int target,
        std::size_t camera_index,
        const Eigen::Isometry3d& camera_to_output,
        const Eigen::Isometry3d& output_to_camera,
        double stamp) const;

    /**
     * @brief Initial acceptance threshold in squared pixels.
     */
    double reprojectionThresholdInitial(std::size_t camera_index) const;

    /**
     * @brief First mutation-stage acceptance threshold in squared pixels.
     */
    double reprojectionThresholdMutation1(std::size_t camera_index) const;

    /**
     * @brief Second mutation-stage acceptance threshold in squared pixels.
     */
    double reprojectionThresholdMutation2(std::size_t camera_index) const;

    /**
     * @brief Third mutation-stage acceptance threshold in squared pixels.
     */
    double reprojectionThresholdMutation3(std::size_t camera_index) const;

    /**
     * @brief Verification threshold used to accept existing particles.
     */
    double reprojectionThresholdVerified(std::size_t camera_index) const;

    /**
     * @brief Rejection threshold used to remove unfit particles.
     */
    double reprojectionThresholdUnfit(std::size_t camera_index) const;

    /**
     * @brief Range heuristic proportional to image width.
     */
    double uvdarRange(std::size_t camera_index) const;

private:
    struct ProjectedMarker {
        Eigen::Vector2d position = Eigen::Vector2d::Zero();
        int signal_id = -1;
        double cos_view_angle = 0.0;
        double distance = 0.0;
    };

    /**
     * @brief Project all visible LEDs and compute nearest-neighbor residual cost.
     *
     * The LED brightness term uses the empirical formula
     * max(0, cos(theta)) * (a + b / (range + c)^2).
     */
    double modelError(const ReprojectionContext& context) const;

    /**
     * @brief Transform one marker to the camera and return projected pixel/range/view cosine.
     */
    std::tuple<ImagePointIdentified, double, double> projectGlobalMarker(
        const LEDMarker& marker,
        std::size_t camera_index,
        const Eigen::Isometry3d& output_to_camera) const;
    /**
     * @brief Project a camera-frame point using the configured lens model.
     */
    Eigen::Vector2d projectCameraPoint(const Eigen::Vector3d& point, std::size_t camera_index) const;

    /**
     * @brief Estimate range from bearing spread via diameter / tan(angle / 2).
     */
    Eigen::Vector3d roughInit(const std::vector<Eigen::Vector3d>& directions, std::size_t camera_index) const;

    /**
     * @brief Largest pairwise angle between normalized image bearings.
     */
    double largestAngle(const std::vector<Eigen::Vector3d>& directions) const;

    /**
     * @brief Test whether the cluster centroid lies near the image boundary.
     */
    bool averageIsNearEdge(const std::vector<ImagePointIdentified>& points, int margin, std::size_t camera_index) const;

    /**
     * @brief Randomly sample candidate poses inside the rough bearing cone.
     */
    std::pair<std::vector<Hypothesis>, std::vector<double>> viableInitialHypotheses(
        const std::vector<ImagePointIdentified>& observed_points,
        const Eigen::Vector3d& furthest_position,
        int target,
        std::size_t camera_index,
        const Eigen::Isometry3d& camera_to_output,
        const Eigen::Isometry3d& output_to_camera,
        int desired_count,
        double stamp) const;
    /**
     * @brief Keep already-good hypotheses and add random pose mutations below threshold.
     */
    std::vector<Hypothesis> refineByMutation(
        const std::vector<ImagePointIdentified>& observed_points,
        const std::vector<Hypothesis>& hypotheses,
        std::size_t camera_index,
        const Eigen::Isometry3d& output_to_camera,
        int target,
        double threshold_local,
        double position_max_step,
        double angle_max_step,
        unsigned desired_count) const;
    std::vector<CameraModel> cameras_;
    uvdar_core::pose_estimation::BodyModel body_;
    Options options_;
    mutable std::mt19937 rng_;
};

using ReprojectionModelPtr = std::shared_ptr<ReprojectionModel>;

} // namespace uvdar_core::pose_estimation::particle_filter
