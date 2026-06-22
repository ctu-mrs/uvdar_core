#pragma once

#include <string>
#include <utility>
#include <vector>

#include "uvdar_core/pose_estimation/types.hpp"

namespace uvdar_core::pose_estimation {

/**
 * @brief Rigid 3D marker layout loaded from a UVDAR body model text file.
 *
 * Marker coordinates are expressed in the target body frame. LED orientation
 * encodes the directional emission axis used by visibility and reprojection
 * scoring.
 */
class BodyModel {
public:
    BodyModel() = default;

    /**
     * @brief Load LED positions, orientations, types, and signal ids.
     */
    explicit BodyModel(const std::string& model_file);

    /**
     * @brief Return a copy translated by p' = p + position.
     */
    BodyModel translate(const Eigen::Vector3d& position) const;

    /**
     * @brief Return a copy rotated by p' = R p and q' = R q.
     */
    BodyModel rotate(const Eigen::Quaterniond& orientation) const;

    /**
     * @brief Return a copy rotated by an axis-angle SO(3) element.
     */
    BodyModel rotate(const Eigen::AngleAxisd& orientation) const;

    /**
     * @brief Compute maximum/minimum distance between simultaneously visible LED groups.
     *
     * The particle-filter rough initializer uses this as an apparent-diameter
     * prior when converting bearing spread into range.
     */
    std::pair<double, double> maxMinVisibleDiameter() const;

    /**
     * @brief Largest local signal id in the body model.
     */
    int maxSignalId() const;

    /**
     * @brief Number of LED entries, including colocated LEDs with different signals.
     */
    std::size_t size() const { return markers_.size(); }

    const LEDMarker& operator[](std::size_t index) const { return markers_.at(index); }
    std::vector<LEDMarker>::const_iterator begin() const { return markers_.begin(); }
    std::vector<LEDMarker>::const_iterator end() const { return markers_.end(); }

private:
    /**
     * @brief Group colocated LEDs so body diameter is based on physical points.
     */
    void prepareGroups();

    /**
     * @brief Parse model-file rows: x y z type pitch yaw signal_id.
     */
    void parseModelFile(const std::string& model_file);

    /**
     * @brief Directional LED visibility test based on angular separation.
     */
    bool areSimultaneouslyVisible(const LEDMarker& a, const LEDMarker& b) const;

    std::vector<LEDMarker> markers_;
    std::vector<std::vector<int>> groups_;
};

} // namespace uvdar_core::pose_estimation
