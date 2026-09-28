#include "uvdar_core/pose_estimation/geometric_solver/visibility_pose.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>
#include <utility>

#include <Eigen/Geometry>

#include "uvdar_core/helpers/math.hpp"
#include "uvdar_core/pose_estimation/uncertainty.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

namespace {

constexpr double kEpsilon = 1.0e-12;
constexpr double kTwoPi = 2.0 * std::numbers::pi;

struct Interval {
    double lower = 0.0;
    double upper = 0.0;
};

struct QuadraturePoint {
    double value = 0.0;
    double weight = 0.0;
};

struct WeightedPose {
    CameraPose pose;
    double spin = 0.0;
    double weight = 0.0;
};

double wrapPositive(double angle)
{
    angle = std::fmod(angle, kTwoPi);
    return angle < 0.0 ? angle + kTwoPi : angle;
}

std::vector<QuadraturePoint> gaussLegendre(const int requested_order)
{
    const int order = std::clamp(requested_order, 2, 256);
    std::vector<QuadraturePoint> output(static_cast<std::size_t>(order));
    const int positive_root_count = (order + 1) / 2;
    for (int root = 0; root < positive_root_count; ++root) {
        double x = std::cos(
            std::numbers::pi * (static_cast<double>(root) + 0.75)
            / (static_cast<double>(order) + 0.5));
        double derivative = 0.0;
        for (int iteration = 0; iteration < 32; ++iteration) {
            double previous = 1.0;
            double current = x;
            for (int degree = 2; degree <= order; ++degree) {
                const double next =
                    ((2.0 * static_cast<double>(degree) - 1.0) * x * current
                        - (static_cast<double>(degree) - 1.0) * previous)
                    / static_cast<double>(degree);
                previous = current;
                current = next;
            }
            derivative = static_cast<double>(order) * (x * current - previous)
                / (x * x - 1.0);
            const double step = current / derivative;
            x -= step;
            if (std::abs(step) <= 2.0e-15) {
                break;
            }
        }
        const double weight = 2.0 / ((1.0 - x * x) * derivative * derivative);
        output[static_cast<std::size_t>(root)] = {-x, weight};
        output[static_cast<std::size_t>(order - 1 - root)] = {x, weight};
    }
    return output;
}

std::vector<Interval> visibilityIntervals(
    const Eigen::Vector3d& body_normal,
    const Eigen::Matrix3d& base_rotation,
    const Eigen::Vector3d& camera_baseline,
    const Eigen::Vector3d& camera_bearing,
    const double cosine_half_angle)
{
    const Eigen::Vector3d normal_zero = base_rotation * body_normal;
    const Eigen::Vector3d toward_camera = -camera_bearing;
    const double axial_normal = camera_baseline.dot(normal_zero);
    const double axial_view = camera_baseline.dot(toward_camera);
    const double cosine_coefficient = normal_zero.dot(toward_camera)
        - axial_normal * axial_view;
    const double sine_coefficient = camera_baseline.cross(normal_zero).dot(toward_camera);
    const double threshold = cosine_half_angle - axial_normal * axial_view;
    const double amplitude = std::hypot(cosine_coefficient, sine_coefficient);

    if (amplitude <= kEpsilon) {
        return threshold <= kEpsilon
            ? std::vector<Interval>{{0.0, kTwoPi}}
            : std::vector<Interval>{};
    }
    const double normalized_threshold = threshold / amplitude;
    if (normalized_threshold > 1.0 + kEpsilon) {
        return {};
    }
    if (normalized_threshold <= -1.0 + kEpsilon) {
        return {{0.0, kTwoPi}};
    }

    const double center = wrapPositive(std::atan2(sine_coefficient, cosine_coefficient));
    const double half_width = std::acos(std::clamp(normalized_threshold, -1.0, 1.0));
    const double lower = wrapPositive(center - half_width);
    const double upper = wrapPositive(center + half_width);
    if (lower <= upper) {
        return {{lower, upper}};
    }
    return {{0.0, upper}, {lower, kTwoPi}};
}

std::vector<Interval> intersectIntervals(
    const std::vector<Interval>& first,
    const std::vector<Interval>& second)
{
    std::vector<Interval> intersections;
    for (const Interval& a : first) {
        for (const Interval& b : second) {
            const Interval overlap{std::max(a.lower, b.lower), std::min(a.upper, b.upper)};
            if (overlap.upper - overlap.lower > kEpsilon) {
                intersections.push_back(overlap);
            }
        }
    }
    std::sort(intersections.begin(), intersections.end(), [](const Interval& a, const Interval& b) {
        return a.lower < b.lower;
    });
    std::vector<Interval> merged;
    for (const Interval& interval : intersections) {
        if (!merged.empty() && interval.lower <= merged.back().upper + kEpsilon) {
            merged.back().upper = std::max(merged.back().upper, interval.upper);
        } else {
            merged.push_back(interval);
        }
    }
    return merged;
}

std::vector<std::vector<std::size_t>> angularModes(
    const std::vector<WeightedPose>& samples,
    const double requested_gap)
{
    if (samples.empty()) {
        return {};
    }
    if (samples.size() == 1U) {
        return {{0U}};
    }

    std::vector<std::size_t> order(samples.size());
    std::iota(order.begin(), order.end(), 0U);
    std::sort(order.begin(), order.end(), [&](const std::size_t first, const std::size_t second) {
        return samples[first].spin < samples[second].spin;
    });

    const double gap_threshold = std::clamp(requested_gap, kEpsilon, std::numbers::pi);
    std::vector<double> gaps(order.size());
    for (std::size_t index = 0U; index + 1U < order.size(); ++index) {
        gaps[index] = samples[order[index + 1U]].spin - samples[order[index]].spin;
    }
    gaps.back() = samples[order.front()].spin + kTwoPi - samples[order.back()].spin;
    const auto largest = std::max_element(gaps.begin(), gaps.end());
    if (*largest <= gap_threshold) {
        return {std::move(order)};
    }

    const std::size_t start =
        (static_cast<std::size_t>(std::distance(gaps.begin(), largest)) + 1U) % order.size();
    std::vector<std::vector<std::size_t>> modes(1U);
    for (std::size_t offset = 0U; offset < order.size(); ++offset) {
        const std::size_t ordered_index = (start + offset) % order.size();
        if (offset > 0U) {
            const std::size_t previous_ordered_index = (start + offset - 1U) % order.size();
            const double gap = ordered_index > previous_ordered_index
                ? samples[order[ordered_index]].spin - samples[order[previous_ordered_index]].spin
                : samples[order[ordered_index]].spin + kTwoPi
                    - samples[order[previous_ordered_index]].spin;
            if (gap > gap_threshold) {
                modes.emplace_back();
            }
        }
        modes.back().push_back(order[ordered_index]);
    }
    return modes;
}

std::optional<VisibilityPoseEstimate> summarizeMode(
    const std::vector<WeightedPose>& samples,
    const std::vector<std::size_t>& indices,
    const double total_weight)
{
    VisibilityPoseEstimate estimate;
    double mode_weight = 0.0;
    std::vector<uncertainty::PoseDistributionComponent> components;
    components.reserve(indices.size());
    for (const std::size_t index : indices) {
        const WeightedPose& sample = samples[index];
        mode_weight += sample.weight;
        components.push_back({sample.pose, uncertainty::PoseCovariance::Zero(), sample.weight});
    }
    if (!(mode_weight > kEpsilon) || !std::isfinite(mode_weight)) {
        return std::nullopt;
    }
    estimate.probability = mode_weight / total_weight;
    const auto moments = uncertainty::momentMatchPoseDistribution(components);
    if (!moments) {
        return std::nullopt;
    }
    estimate.pose = moments->pose;
    estimate.covariance = moments->covariance;
    return estimate;
}

} // namespace

std::vector<VisibilityPoseEstimate> VisibilityPoseSolver::solve(
    const Eigen::Vector3d& first_body_point,
    const Eigen::Vector3d& second_body_point,
    const Eigen::Vector3d& first_body_normal,
    const Eigen::Vector3d& second_body_normal,
    const Eigen::Vector3d& first_camera_bearing,
    const Eigen::Vector3d& second_camera_bearing,
    const VisibilityPoseConfig& config)
{
    if (!first_body_point.allFinite() || !second_body_point.allFinite()
        || !first_body_normal.allFinite() || !second_body_normal.allFinite()
        || !first_camera_bearing.allFinite() || !second_camera_bearing.allFinite()
        || first_body_normal.squaredNorm() <= kEpsilon
        || second_body_normal.squaredNorm() <= kEpsilon
        || first_camera_bearing.squaredNorm() <= kEpsilon
        || second_camera_bearing.squaredNorm() <= kEpsilon
        || config.visibility_half_angle_rad <= 0.0
        || config.visibility_half_angle_rad >= std::numbers::pi
        || config.minimum_bearing_separation_rad <= 0.0
        || config.mode_gap_rad <= 0.0
        || config.depth_quadrature_order < 2
        || config.spin_quadrature_order < 2) {
        return {};
    }

    const Eigen::Vector3d body_delta = second_body_point - first_body_point;
    const double marker_distance = body_delta.norm();
    if (!std::isfinite(marker_distance) || marker_distance <= kEpsilon) {
        return {};
    }
    const Eigen::Vector3d body_baseline = body_delta / marker_distance;
    const Eigen::Vector3d first_normal = first_body_normal.normalized();
    const Eigen::Vector3d second_normal = second_body_normal.normalized();
    const Eigen::Vector3d first_bearing = first_camera_bearing.normalized();
    const Eigen::Vector3d second_bearing = second_camera_bearing.normalized();
    const double bearing_cosine = std::clamp(first_bearing.dot(second_bearing), -1.0, 1.0);
    const double bearing_angle = std::acos(bearing_cosine);
    const double bearing_sine = std::sin(bearing_angle);
    if (!std::isfinite(bearing_angle)
        || bearing_angle < config.minimum_bearing_separation_rad
        || std::numbers::pi - bearing_angle < config.minimum_bearing_separation_rad
        || bearing_sine <= kEpsilon) {
        return {};
    }

    // In the ray-plane basis, beta in (alpha, pi) parameterizes every and only
    // positive-depth solution exactly once:
    // lambda1 = d sin(beta-alpha)/sin(alpha), lambda2 = d sin(beta)/sin(alpha).
    const Eigen::Vector3d ray_plane_axis =
        (second_bearing - bearing_cosine * first_bearing) / bearing_sine;
    const double cosine_half_angle = std::cos(config.visibility_half_angle_rad);
    const std::vector<QuadraturePoint> depth_quadrature =
        gaussLegendre(config.depth_quadrature_order);
    const std::vector<QuadraturePoint> spin_quadrature =
        gaussLegendre(config.spin_quadrature_order);
    const double beta_center = 0.5 * (bearing_angle + std::numbers::pi);
    const double beta_radius = 0.5 * (std::numbers::pi - bearing_angle);

    std::vector<WeightedPose> samples;
    samples.reserve(
        static_cast<std::size_t>(config.depth_quadrature_order)
        * static_cast<std::size_t>(config.spin_quadrature_order) * 2U);
    for (const QuadraturePoint& depth_node : depth_quadrature) {
        const double beta = beta_center + beta_radius * depth_node.value;
        const double lambda_first = marker_distance
            * std::sin(beta - bearing_angle) / bearing_sine;
        const double lambda_second = marker_distance * std::sin(beta) / bearing_sine;
        if (!(lambda_first > 0.0) || !(lambda_second > 0.0)) {
            continue;
        }
        const Eigen::Vector3d camera_baseline =
            std::cos(beta) * first_bearing + std::sin(beta) * ray_plane_axis;
        const Eigen::Matrix3d base_rotation =
            uvdar_core::helpers::rotationBetween(body_baseline, camera_baseline);
        const std::vector<Interval> feasible_intervals = intersectIntervals(
            visibilityIntervals(
                first_normal,
                base_rotation,
                camera_baseline,
                first_bearing,
                cosine_half_angle),
            visibilityIntervals(
                second_normal,
                base_rotation,
                camera_baseline,
                second_bearing,
                cosine_half_angle));

        // Uniform arc length is the correspondence-order-invariant measure on
        // the exact depth curve (lambda1(beta), lambda2(beta)).
        const double arc_length_jacobian = marker_distance / bearing_sine
            * std::sqrt(
                std::pow(std::cos(beta - bearing_angle), 2)
                + std::pow(std::cos(beta), 2));
        const double beta_weight = beta_radius * depth_node.weight
            * arc_length_jacobian;
        for (const Interval& interval : feasible_intervals) {
            const double spin_radius = 0.5 * (interval.upper - interval.lower);
            const double spin_center = 0.5 * (interval.upper + interval.lower);
            for (const QuadraturePoint& spin_node : spin_quadrature) {
                const double spin = spin_center + spin_radius * spin_node.value;
                const Eigen::Matrix3d rotation = Eigen::AngleAxisd(
                    spin, camera_baseline).toRotationMatrix() * base_rotation;
                CameraPose pose;
                pose.rotation = rotation;
                pose.translation = lambda_first * first_bearing
                    - rotation * first_body_point;
                const double weight = beta_weight * spin_radius * spin_node.weight;
                if (pose.translation.allFinite() && pose.rotation.allFinite()
                    && std::isfinite(weight) && weight > 0.0) {
                    samples.push_back({std::move(pose), spin, weight});
                }
            }
        }
    }
    if (samples.empty()) {
        return {};
    }

    const double total_weight = std::accumulate(
        samples.begin(), samples.end(), 0.0,
        [](const double sum, const WeightedPose& sample) {
            return sum + sample.weight;
        });
    if (!(total_weight > kEpsilon) || !std::isfinite(total_weight)) {
        return {};
    }

    std::vector<VisibilityPoseEstimate> estimates;
    for (const std::vector<std::size_t>& mode : angularModes(samples, config.mode_gap_rad)) {
        auto estimate = summarizeMode(samples, mode, total_weight);
        if (estimate
            && estimate->probability > 0.0
            && estimate->pose.translation.allFinite()
            && estimate->pose.rotation.allFinite()
            && estimate->covariance.allFinite()) {
            estimates.push_back(std::move(*estimate));
        }
    }
    std::sort(estimates.begin(), estimates.end(), [](const auto& first, const auto& second) {
        return first.probability > second.probability;
    });
    return estimates;
}

} // namespace uvdar_core::pose_estimation::geometric_solver
