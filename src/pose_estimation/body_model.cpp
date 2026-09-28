#include "uvdar_core/pose_estimation/body_model.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace uvdar_core::pose_estimation {

namespace {

// Directional LEDs are treated as visible only inside this angular cone.
constexpr double directional_led_view_angle = 120.0 * M_PI / 180.0;
// LEDs mounted at almost the same physical position form one diameter group.
constexpr double led_group_distance = 0.03;
constexpr double minimum_relative_triangle_area = 1.0e-4;

} // namespace

BodyModel::BodyModel(const std::string& model_file)
{
    parseModelFile(model_file);
    prepareGroups();
}

BodyModel BodyModel::translate(const Eigen::Vector3d& position) const
{
    BodyModel output = *this;
    for (auto& marker : output.markers_) {
        marker.pose.position += position;
    }
    return output;
}

BodyModel BodyModel::rotate(const Eigen::Quaterniond& orientation) const
{
    BodyModel output = *this;
    for (auto& marker : output.markers_) {
        marker.pose.position = orientation * marker.pose.position;
        marker.pose.orientation = orientation * marker.pose.orientation;
    }
    return output;
}

BodyModel BodyModel::rotate(const Eigen::AngleAxisd& orientation) const
{
    return rotate(Eigen::Quaterniond(orientation));
}

std::pair<double, double> BodyModel::maxMinVisibleDiameter() const
{
    if (groups_.empty()) {
        return {-1.0, -1.0};
    }
    if (groups_.size() == 1U) {
        return {std::numeric_limits<double>::max(), 0.0};
    }

    // The apparent-size range prior should use only pairs that can be observed
    // from a common viewing direction, so opposing directional LEDs are skipped.
    double max_dist = 0.0;
    double min_dist = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i + 1 < groups_.size(); ++i) {
        for (std::size_t j = i + 1; j < groups_.size(); ++j) {
            if (!areGroupsSimultaneouslyVisible(groups_[i], groups_[j])) {
                continue;
            }
            const double distance = (groupCenter(groups_[i]) - groupCenter(groups_[j])).norm();
            max_dist = std::max(max_dist, distance);
            min_dist = std::min(min_dist, distance);
        }
    }
    return {max_dist, min_dist};
}

int BodyModel::maxSignalId() const
{
    int output = -1;
    for (const auto& marker : markers_) {
        output = std::max(output, marker.signal_id);
    }
    return output;
}

std::optional<LEDMarker> BodyModel::markerForSignal(int signal_id) const
{
    const std::vector<int>* matching_group = nullptr;
    const LEDMarker* matching_marker = nullptr;
    for (const std::vector<int>& group : groups_) {
        const auto marker = std::find_if(group.begin(), group.end(), [this, signal_id](int index) {
            return markers_[static_cast<std::size_t>(index)].signal_id == signal_id;
        });
        if (marker == group.end()) {
            continue;
        }
        if (matching_group != nullptr) {
            // One blink code occurring on distinct physical boards is not a
            // unique 2D-3D correspondence for a deterministic solver.
            return std::nullopt;
        }
        matching_group = &group;
        matching_marker = &markers_[static_cast<std::size_t>(*marker)];
    }

    if (matching_group == nullptr || matching_marker == nullptr) {
        return std::nullopt;
    }
    LEDMarker marker = *matching_marker;
    marker.pose.position = groupCenter(*matching_group);
    return marker;
}

bool BodyModel::hasObservableTriangle(
    const Eigen::Vector3d& first,
    const Eigen::Vector3d& second,
    const Eigen::Vector3d& third)
{
    const double longest_side_squared = std::max({
        (first - second).squaredNorm(),
        (second - third).squaredNorm(),
        (third - first).squaredNorm(),
    });
    const double twice_triangle_area = (second - first).cross(third - first).norm();
    return std::isfinite(longest_side_squared)
        && std::isfinite(twice_triangle_area)
        && longest_side_squared > std::numeric_limits<double>::epsilon()
        && twice_triangle_area > minimum_relative_triangle_area * longest_side_squared;
}

BodyModel::VisibilityScore BodyModel::visibilityScore(
    const CameraPose& pose,
    const std::vector<Eigen::Vector3d>& observed_marker_positions) const
{
    VisibilityScore score;
    const Eigen::Vector3d camera_in_body = -pose.rotation.transpose() * pose.translation;
    double observed_view_sum = 0.0;
    double hidden_view_maximum = -std::numeric_limits<double>::infinity();
    std::size_t observed_count = 0U;

    for (const std::vector<int>& group : groups_) {
        const Eigen::Vector3d center = groupCenter(group);
        const bool observed = std::any_of(
            observed_marker_positions.begin(),
            observed_marker_positions.end(),
            [&center](const Eigen::Vector3d& observed_position) {
                return (center - observed_position).norm() < led_group_distance;
            });

        double maximum_view_cosine = -std::numeric_limits<double>::infinity();
        for (int index : group) {
            maximum_view_cosine = std::max(
                maximum_view_cosine,
                ledViewCosine(markers_[static_cast<std::size_t>(index)], camera_in_body));
        }
        if (!std::isfinite(maximum_view_cosine)) {
            continue;
        }

        if (observed) {
            score.observed_view_cosine_minimum = std::min(score.observed_view_cosine_minimum, maximum_view_cosine);
            observed_view_sum += maximum_view_cosine;
            ++observed_count;
        } else {
            hidden_view_maximum = std::max(hidden_view_maximum, maximum_view_cosine);
        }
    }

    if (observed_count == 0U) {
        return score;
    }
    score.observed_view_cosine_mean = observed_view_sum / static_cast<double>(observed_count);
    score.observed_leds_face_camera = score.observed_view_cosine_minimum > 0.0;
    if (!std::isfinite(hidden_view_maximum)) {
        hidden_view_maximum = -1.0;
    }
    score.visibility_margin = score.observed_view_cosine_minimum - hidden_view_maximum;
    return score;
}

double BodyModel::ledViewCosine(const LEDMarker& marker, const Eigen::Vector3d& camera_position)
{
    const Eigen::Vector3d camera_direction = camera_position - marker.pose.position;
    const Eigen::Vector3d led_direction = marker.pose.orientation * Eigen::Vector3d::UnitX();
    if (!camera_direction.allFinite() || !led_direction.allFinite()
        || camera_direction.squaredNorm() < std::numeric_limits<double>::epsilon()
        || led_direction.squaredNorm() < std::numeric_limits<double>::epsilon()) {
        return -std::numeric_limits<double>::infinity();
    }
    return led_direction.normalized().dot(camera_direction.normalized());
}

std::vector<BodyModel::ProjectedSignal> BodyModel::mergeProjectedSignalBlobs(
    const std::vector<ProjectedSignal>& projections,
    const double maximum_distance_px)
{
    if (maximum_distance_px <= 0.0) {
        return projections;
    }

    std::vector<bool> merged(projections.size(), false);
    std::vector<ProjectedSignal> output;
    output.reserve(projections.size());
    for (std::size_t seed = 0U; seed < projections.size(); ++seed) {
        if (merged[seed]) {
            continue;
        }

        const int signal_id = projections[seed].signal_id;
        Eigen::Vector2d position_sum = Eigen::Vector2d::Zero();
        std::size_t component_count = 0U;
        std::vector<std::size_t> pending{seed};
        merged[seed] = true;

        // Grow one connected same-signal image-space component. This models
        // an unresolved LED pair (or chain of pairs) independently of row
        // ordering in the model file.
        while (!pending.empty()) {
            const std::size_t current = pending.back();
            pending.pop_back();
            position_sum += projections[current].position;
            ++component_count;
            for (std::size_t candidate = 0U; candidate < projections.size(); ++candidate) {
                if (merged[candidate] || projections[candidate].signal_id != signal_id
                    || (projections[candidate].position - projections[current].position).norm() >= maximum_distance_px) {
                    continue;
                }
                merged[candidate] = true;
                pending.push_back(candidate);
            }
        }
        output.push_back({position_sum / static_cast<double>(component_count), signal_id});
    }
    return output;
}

void BodyModel::prepareGroups()
{
    groups_.clear();
    for (std::size_t i = 0; i < markers_.size(); ++i) {
        // A body can have multiple blink codes on a single physical LED board.
        // Group by position to avoid counting colocated codes as body diameter.
        bool found = false;
        for (auto& group : groups_) {
            if ((markers_[i].pose.position - groupCenter(group)).norm() < led_group_distance) {
                group.push_back(static_cast<int>(i));
                found = true;
                break;
            }
        }
        if (!found) {
            groups_.push_back({static_cast<int>(i)});
        }
    }
}

void BodyModel::parseModelFile(const std::string& model_file)
{
    std::ifstream input(model_file);
    if (!input.is_open()) {
        throw std::runtime_error("Could not open UVDAR LED model file '" + model_file + "'.");
    }

    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        int type = -1;
        double pitch = 0.0;
        double yaw = 0.0;
        int signal_id = -1;
        ss >> x >> y >> z >> type >> pitch >> yaw >> signal_id;
        if (!ss) {
            throw std::runtime_error("Invalid line in UVDAR LED model file '" + model_file + "': " + line);
        }

        LEDMarker marker;
        marker.pose.position = Eigen::Vector3d(x, y, z);
        // Model files store LED pointing direction as yaw then pitch in the
        // body frame.
        marker.pose.orientation =
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        marker.type = type;
        marker.signal_id = signal_id;
        markers_.push_back(marker);
    }
}

bool BodyModel::areSimultaneouslyVisible(const LEDMarker& a, const LEDMarker& b) const
{
    return a.pose.orientation.angularDistance(b.pose.orientation) < directional_led_view_angle;
}

Eigen::Vector3d BodyModel::groupCenter(const std::vector<int>& group) const
{
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (int index : group) {
        center += markers_[static_cast<std::size_t>(index)].pose.position;
    }
    return group.empty() ? center : center / static_cast<double>(group.size());
}

bool BodyModel::areGroupsSimultaneouslyVisible(const std::vector<int>& first, const std::vector<int>& second) const
{
    for (int first_index : first) {
        for (int second_index : second) {
            if (areSimultaneouslyVisible(
                    markers_[static_cast<std::size_t>(first_index)],
                    markers_[static_cast<std::size_t>(second_index)])) {
                return true;
            }
        }
    }
    return false;
}

} // namespace uvdar_core::pose_estimation
