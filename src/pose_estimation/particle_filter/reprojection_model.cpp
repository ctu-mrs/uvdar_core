#include "uvdar_core/pose_estimation/particle_filter/reprojection_model.hpp"
#include "uvdar_core/pose_estimation/particle_filter/sampling.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace uvdar_core::pose_estimation::particle_filter {

namespace {

constexpr double unmatched_observed_point_penalty = 15.0 * 15.0;
constexpr double unmatched_projected_point_penalty = 0.0;
constexpr double max_initial_velocity = 1.0;
// Empirical LED brightness model: a + b / (range + c)^2, scaled by cos(view).
constexpr double led_projection_coefs[3] = {1.3398, 31.4704, 0.0154};

} // namespace

ReprojectionModel::ReprojectionModel(std::vector<CameraModel> cameras, uvdar_core::pose_estimation::BodyModel body, Options options)
    : cameras_(std::move(cameras))
    , body_(std::move(body))
    , options_(std::move(options))
    , rng_(std::random_device {}())
{
}

Eigen::Vector3d ReprojectionModel::directionFromImagePoint(const Eigen::Vector2d& point, std::size_t camera_index) const
{
    return cameras_.at(camera_index).backProject(point);
}

double ReprojectionModel::hypothesisError(const Hypothesis& hypothesis, const ReprojectionContext& context) const
{
    if (context.target != hypothesis.index) {
        return -1.0;
    }

    ReprojectionContext transformed = context;
    transformed.body = context.body.rotate(hypothesis.pose.orientation).translate(hypothesis.pose.position);
    return modelError(transformed);
}

std::vector<Hypothesis> ReprojectionModel::extractHypotheses(
    const std::vector<ImagePointIdentified>& points,
    int target,
    std::size_t camera_index,
    const Eigen::Isometry3d& camera_to_output,
    const Eigen::Isometry3d& output_to_camera,
    double stamp) const
{
    if (points.empty()) {
        return {};
    }

    std::vector<Eigen::Vector3d> directions;
    directions.reserve(points.size());
    if (points.size() != 1U) {
        for (const auto& point : points) {
            directions.push_back(directionFromImagePoint(point.position.cast<double>(), camera_index));
        }
    }
    Eigen::Vector2d cluster_center = Eigen::Vector2d::Zero();
    for (const auto& point : points) {
        cluster_center += point.position.cast<double>();
    }
    cluster_center /= static_cast<double>(points.size());

    Eigen::Vector3d furthest_position = Eigen::Vector3d::Zero();
    if (points.size() == 1U
        || CameraModel::largestBearingAngle(directions) < 0.01
        || !cameras_.at(camera_index).containsPixel(cluster_center, static_cast<double>(options_.edge_detection_margin))) {
        // With weak angular baseline, fall back to the far-range ray prior.
        furthest_position = directionFromImagePoint(points.front().position.cast<double>(), camera_index) * uvdarRange(camera_index);
    } else {
        // Otherwise estimate range from angular diameter: r ~= D / (2 tan(alpha/2)).
        furthest_position = roughInit(directions, camera_index);
    }

    auto [initial, initial_errors] = viableInitialHypotheses(
        points,
        furthest_position,
        target,
        camera_index,
        camera_to_output,
        output_to_camera,
        options_.initial_rough_hypothesis_count,
        stamp);

    const double rough_count = static_cast<double>(options_.initial_rough_hypothesis_count);
    const double ratio_found = rough_count > 0.0 ? static_cast<double>(initial.size()) / rough_count : 0.0;
    const int desired_count = static_cast<int>(ratio_found * static_cast<double>(options_.initial_hypothesis_count));
    if (desired_count <= 0 || initial.empty()) {
        return {};
    }

    // Three mutation stages progressively tighten the reprojection gate.
    std::vector<Hypothesis> refined = refineByMutation(
        points, initial, camera_index, output_to_camera, target, reprojectionThresholdMutation1(camera_index), 1.0, 1.0, desired_count);
    if (refined.empty()) {
        return refined;
    }

    const double ratio_found_2 = static_cast<double>(refined.size()) / static_cast<double>(desired_count);
    const int desired_count_2 = static_cast<int>(ratio_found_2 * static_cast<double>(desired_count));
    refined = refineByMutation(points, refined, camera_index, output_to_camera, target, reprojectionThresholdMutation2(camera_index), 1.0, 1.0, desired_count_2);
    if (refined.empty()) {
        return refined;
    }

    const double ratio_found_3 = static_cast<double>(refined.size()) / static_cast<double>(std::max(1, desired_count_2));
    const int desired_count_3 = static_cast<int>(ratio_found_3 * static_cast<double>(desired_count_2));
    refined = refineByMutation(points, refined, camera_index, output_to_camera, target, reprojectionThresholdMutation3(camera_index), 1.0, 1.0, desired_count_3);

    const int static_count = static_cast<int>(refined.size());
    for (int i = 0; i < static_count; ++i) {
        const auto mutations = generateVelocityMutations(
            refined[static_cast<std::size_t>(i)],
            5,
            max_initial_velocity,
            rng_);
        refined.insert(refined.end(), mutations.begin(), mutations.end());
    }

    return refined;
}

double ReprojectionModel::reprojectionThresholdInitial(std::size_t camera_index) const
{
    return uvdar_core::helpers::squared(cameras_.at(camera_index).width() / 50.0);
}

double ReprojectionModel::reprojectionThresholdMutation1(std::size_t camera_index) const
{
    return uvdar_core::helpers::squared(cameras_.at(camera_index).width() / 75.0);
}

double ReprojectionModel::reprojectionThresholdMutation2(std::size_t camera_index) const
{
    return uvdar_core::helpers::squared(cameras_.at(camera_index).width() / 100.0);
}

double ReprojectionModel::reprojectionThresholdMutation3(std::size_t camera_index) const
{
    return uvdar_core::helpers::squared(cameras_.at(camera_index).width() / 150.0);
}

double ReprojectionModel::reprojectionThresholdVerified(std::size_t camera_index) const
{
    return uvdar_core::helpers::squared(cameras_.at(camera_index).width() / 150.0);
}

double ReprojectionModel::reprojectionThresholdUnfit(std::size_t camera_index) const
{
    return uvdar_core::helpers::squared(cameras_.at(camera_index).width() / 50.0);
}

double ReprojectionModel::uvdarRange(std::size_t camera_index) const
{
    return cameras_.at(camera_index).width() / 50.0;
}

double ReprojectionModel::modelError(const ReprojectionContext& context) const
{
    std::vector<ProjectedMarker> projected_markers;
    for (const auto& marker : context.body) {
        const auto [projected, distance, cos_view_angle] = projectGlobalMarker(marker, context.camera_index, context.output_to_camera);
        const auto& camera = cameras_.at(context.camera_index);
        if (camera.containsPixel(projected.position.cast<double>(), -0.5)) {
            projected_markers.push_back({projected.position.cast<double>(), marker.signal_id, cos_view_angle, distance});
        }
    }

    std::vector<BodyModel::ProjectedSignal> selected_markers;
    for (const auto& marker : projected_markers) {
        // Directional LEDs fade with viewing angle and inverse-square range.
        const double intensity = std::round(std::max(0.0, marker.cos_view_angle)
            * (led_projection_coefs[0] + led_projection_coefs[1] / uvdar_core::helpers::squared(marker.distance + led_projection_coefs[2])));
        if (intensity > 0.0) {
            selected_markers.push_back({marker.position, marker.signal_id});
        }
    }

    selected_markers = BodyModel::mergeProjectedSignalBlobs(selected_markers, 3.0);

    // Cost is nearest-neighbor squared pixel distance per expected signal, with
    // a fixed penalty for observed points not explained by any projected LED.
    double total_error = 0.0;
    for (const auto& observed : context.observed_points) {
        double closest_distance = std::numeric_limits<double>::max();
        bool found = false;
        for (const auto& projected : selected_markers) {
            const auto signal_index = static_cast<std::size_t>((context.target % 1000) * options_.signals_per_target + projected.signal_id);
            if (signal_index >= options_.signal_ids.size() || options_.signal_ids[signal_index] != observed.id) {
                continue;
            }
            const double distance = (projected.position - observed.position.cast<double>()).norm();
            if (distance < closest_distance) {
                closest_distance = distance;
                found = true;
            }
        }
        total_error += found ? uvdar_core::helpers::squared(closest_distance) : unmatched_observed_point_penalty;
    }

    total_error += unmatched_projected_point_penalty
        * static_cast<double>(std::max(0, static_cast<int>(selected_markers.size()) - static_cast<int>(context.observed_points.size())));
    return total_error;
}

std::tuple<ImagePointIdentified, double, double> ReprojectionModel::projectGlobalMarker(
    const LEDMarker& marker,
    std::size_t camera_index,
    const Eigen::Isometry3d& output_to_camera) const
{
    LEDMarker local = marker;
    local.pose = transformPose(marker.pose, output_to_camera);
    const Eigen::Vector2d position = projectCameraPoint(local.pose.position, camera_index);
    return {
        {local.signal_id, position.cast<int>()},
        local.pose.position.norm(),
        BodyModel::ledViewCosine(local, Eigen::Vector3d::Zero()),
    };
}

Eigen::Vector2d ReprojectionModel::projectCameraPoint(const Eigen::Vector3d& point, std::size_t camera_index) const
{
    return cameras_.at(camera_index).project(point);
}

Eigen::Vector3d ReprojectionModel::roughInit(const std::vector<Eigen::Vector3d>& directions, std::size_t camera_index) const
{
    const Eigen::Vector3d average = CameraModel::meanBearing(directions).value_or(Eigen::Vector3d::UnitZ());

    double max_length = uvdarRange(camera_index);
    if (directions.size() > 1U) {
        // Approximate the body as a diameter subtending the largest bearing angle.
        max_length = (options_.max_diameter / 2.0) / std::tan(CameraModel::largestBearingAngle(directions) / 2.0);
    }
    return average * max_length * 1.25;
}

std::pair<std::vector<Hypothesis>, std::vector<double>> ReprojectionModel::viableInitialHypotheses(
    const std::vector<ImagePointIdentified>& observed_points,
    const Eigen::Vector3d& furthest_position,
    int target,
    std::size_t camera_index,
    const Eigen::Isometry3d& camera_to_output,
    const Eigen::Isometry3d& output_to_camera,
    int desired_count,
    double stamp) const
{
    Eigen::FullPivLU<Eigen::MatrixXd> lu(furthest_position.normalized().transpose());
    const Eigen::MatrixXd null_space = lu.kernel();
    const Eigen::Vector3d side_shift_init = null_space.topLeftCorner(3, 1).normalized() * options_.max_diameter;

    ReprojectionContext context;
    context.camera_index = camera_index;
    context.output_to_camera = output_to_camera;
    context.target = target;
    context.observed_points = observed_points;
    context.body = body_;

    const double threshold = static_cast<double>(observed_points.size()) * reprojectionThresholdInitial(camera_index);
    std::vector<Hypothesis> hypotheses;
    std::vector<double> errors;
    int iterations = 0;
    while (static_cast<int>(hypotheses.size()) < desired_count && iterations <= options_.max_init_iterations) {
        // Sample along the rough bearing ray, then side-step in its null space.
        const double d = randomUniform01(rng_);
        const double sidestep_direction = randomUniform01(rng_);
        const double sidestep_distance = randomUniform01(rng_);
        const Eigen::Vector3d side_shift = Eigen::AngleAxisd(2.0 * M_PI * sidestep_direction, furthest_position.normalized())
            * side_shift_init * sidestep_distance;
        const Eigen::Vector3d camera_position = furthest_position * d + side_shift;

        Hypothesis hypothesis;
        hypothesis.unique_id = static_cast<int>(rng_());
        hypothesis.index = target;
        hypothesis.pose.position = camera_to_output * camera_position;
        hypothesis.pose.orientation = Eigen::AngleAxisd(
            randomUniform01(rng_) * 2.0 * M_PI,
            randomUnitVector(rng_));
        hypothesis.flag = HypothesisFlag::Neutral;
        hypothesis.observed = stamp;
        hypothesis.propagated = stamp;

        const double error = hypothesisError(hypothesis, context);
        if (error < threshold) {
            hypotheses.push_back(hypothesis);
            errors.push_back(error);
        }
        ++iterations;
    }
    return {hypotheses, errors};
}

std::vector<Hypothesis> ReprojectionModel::refineByMutation(
    const std::vector<ImagePointIdentified>& observed_points,
    const std::vector<Hypothesis>& hypotheses,
    std::size_t camera_index,
    const Eigen::Isometry3d& output_to_camera,
    int target,
    double threshold_local,
    double position_max_step,
    double angle_max_step,
    unsigned desired_count) const
{
    if (desired_count == 0U) {
        return {};
    }

    ReprojectionContext context;
    context.camera_index = camera_index;
    context.output_to_camera = output_to_camera;
    context.target = target;
    context.observed_points = observed_points;
    context.body = body_;

    std::vector<Hypothesis> output;
    const double threshold = static_cast<double>(observed_points.size()) * threshold_local;
    for (const auto& hypothesis : hypotheses) {
        if (hypothesisError(hypothesis, context) < threshold) {
            output.push_back(hypothesis);
            output.back().flag = HypothesisFlag::Neutral;
        }
    }

    int iterations = 0;
    while (output.size() < desired_count && iterations <= options_.max_mutation_refine_iterations) {
        // Rejection sampling around currently viable particles.
        for (const auto& hypothesis : hypotheses) {
            for (auto mutation : generatePoseMutations(
                hypothesis,
                1,
                position_max_step,
                angle_max_step,
                rng_)) {
                if (hypothesisError(mutation, context) < threshold) {
                    mutation.flag = HypothesisFlag::Neutral;
                    output.push_back(mutation);
                }
            }
            if (output.size() >= desired_count) {
                break;
            }
        }
        ++iterations;
    }

    return output;
}

} // namespace uvdar_core::pose_estimation::particle_filter
