#include "uvdar_core/pose_estimation/body_model.hpp"

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
            const auto& a = markers_[groups_[i][0]];
            const auto& b = markers_[groups_[j][0]];
            if (!areSimultaneouslyVisible(a, b)) {
                continue;
            }
            const double distance = (a.pose.position - b.pose.position).norm();
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

void BodyModel::prepareGroups()
{
    groups_.clear();
    for (std::size_t i = 0; i < markers_.size(); ++i) {
        // A body can have multiple blink codes on a single physical LED board.
        // Group by position to avoid counting colocated codes as body diameter.
        bool found = false;
        for (auto& group : groups_) {
            if ((markers_[i].pose.position - markers_[group[0]].pose.position).norm() < led_group_distance) {
                group.push_back(static_cast<int>(i));
                found = true;
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
        // body frame. This matches the ROS1 model-file convention.
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
    if (a.type == 0 && b.type == 0) {
        return a.pose.orientation.angularDistance(b.pose.orientation) < directional_led_view_angle;
    }
    return false;
}

} // namespace uvdar_core::pose_estimation
