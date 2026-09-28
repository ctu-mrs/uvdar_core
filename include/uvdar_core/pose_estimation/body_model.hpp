#pragma once

#include <limits>
#include <optional>
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
    /**
     * @brief Observability support for a camera pose hypothesis.
     *
     * Each physical group represents one detected blob.  Its view cosine is
     * the best cosine among all of the group's LED emission axes.
     */
    struct VisibilityScore {
        bool observed_leds_face_camera = false;
        // Accumulated with std::min(); start at the identity element so the
        // first observed LED supplies the actual minimum.
        double observed_view_cosine_minimum = std::numeric_limits<double>::infinity();
        double observed_view_cosine_mean = -std::numeric_limits<double>::infinity();
        double visibility_margin = -std::numeric_limits<double>::infinity();
    };

    /**
     * @brief A projected blink signal before association with a detected blob.
     */
    struct ProjectedSignal {
        Eigen::Vector2d position = Eigen::Vector2d::Zero();
        int signal_id = -1;
    };

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
     * @brief Return the blended physical marker uniquely carrying a signal id.
     *
     * Close LEDs on one arm are represented by their group centroid because a
     * distant camera normally detects them as a single blob.  An id occurring
     * on multiple physical groups is ambiguous and therefore has no unique
     * geometric correspondence.
     */
    std::optional<LEDMarker> markerForSignal(int signal_id) const;

    /**
     * @brief Test whether three blended marker positions form an observable triangle.
     */
    static bool hasObservableTriangle(
        const Eigen::Vector3d& first,
        const Eigen::Vector3d& second,
        const Eigen::Vector3d& third);

    /**
     * @brief Score a pose from LED orientation and the physical blobs observed in a frame.
     */
    VisibilityScore visibilityScore(
        const CameraPose& pose,
        const std::vector<Eigen::Vector3d>& observed_marker_positions) const;

    /**
     * @brief Cosine between an LED emission axis and the line toward a camera.
     */
    static double ledViewCosine(const LEDMarker& marker, const Eigen::Vector3d& camera_position);

    /**
     * @brief Merge same-signal projections that form one unresolved physical blob.
     */
    static std::vector<ProjectedSignal> mergeProjectedSignalBlobs(
        const std::vector<ProjectedSignal>& projections,
        double maximum_distance_px);

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

    /**
     * @brief Centroid of one physical LED group.
     */
    Eigen::Vector3d groupCenter(const std::vector<int>& group) const;

    /**
     * @brief Whether some LED orientation in each physical group can be jointly visible.
     */
    bool areGroupsSimultaneouslyVisible(const std::vector<int>& first, const std::vector<int>& second) const;

    std::vector<LEDMarker> markers_;
    std::vector<std::vector<int>> groups_;
};

} // namespace uvdar_core::pose_estimation
