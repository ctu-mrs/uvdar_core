#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "uvdar_core/helpers/math.hpp"

namespace uvdar_core::pose_estimation {

/**
 * @brief Rigid body pose in a ROS-style frame: translation plus unit quaternion.
 */
struct Pose {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

/**
 * @brief Rigid transform from a body/model frame to a camera frame.
 *
 * It represents X_c = R X_b + t and is shared by geometric solving,
 * reprojection, and uncertainty propagation. It is deliberately not owned by
 * the uncertainty module.
 */
struct CameraPose {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

/**
 * @brief Transform a body/model point into the camera frame with CameraPose.
 */
inline Eigen::Vector3d transformPoint(const CameraPose& pose, const Eigen::Vector3d& body_point)
{
    return pose.rotation * body_point + pose.translation;
}

/**
 * @brief Apply a translation and left-multiplied SO(3) camera-pose increment.
 */
inline void applyLeftCameraPoseIncrement(
    CameraPose& pose,
    const Eigen::Vector3d& translation_increment,
    const Eigen::Vector3d& rotation_increment)
{
    pose.translation += translation_increment;
    pose.rotation = uvdar_core::helpers::expSO3(rotation_increment) * pose.rotation;
}

/**
 * @brief Minimal velocity state used by the particle filter: linear part only.
 */
struct Twist {
    Eigen::Vector3d linear = Eigen::Vector3d::Zero();
};

/**
 * @brief One LED in the body model, including its local pose and emitted signal.
 */
struct LEDMarker {
    Pose pose;
    int type = -1;
    int signal_id = -1;
};

/**
 * @brief Tracker output converted to pose-estimation input.
 *
 * The covariance fields are 2D image-plane covariance matrices. When a tracker
 * predicts through a missed detection, the predicted position and covariance
 * are used by covariance-aware solvers.
 */
struct TrackedPoint {
    double x = 0.0;
    double y = 0.0;
    int id = -1;
    bool virtual_point = false;
    bool associated_with_detection = false;
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Identity();
    bool has_prediction = false;
    Eigen::Vector2d predicted_position = Eigen::Vector2d::Zero();
    Eigen::Matrix2d prediction_covariance = Eigen::Matrix2d::Identity();
};

/**
 * @brief Map an allowed global signal id to its target index.
 */
inline int targetForSignal(
    const std::vector<int>& signal_ids,
    const int signals_per_target,
    const int signal_id)
{
    if (std::find(signal_ids.begin(), signal_ids.end(), signal_id) == signal_ids.end()) {
        return -1;
    }
    return signal_id / std::max(1, signals_per_target);
}

/**
 * @brief Pose estimate with 6D covariance in [position, orientation] order.
 */
struct PoseMeasurement {
    int id = -1;
    Pose pose;
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
    /** Compact estimator/solver signature used by diagnostics and visualization. */
    std::string method;
};

/**
 * @brief Timestamped batch of target pose estimates in one output frame.
 */
struct TimedPoseMeasurements {
    double stamp = 0.0;
    std::string frame_id;
    std::vector<PoseMeasurement> poses;
};

/**
 * @brief Apply a rigid transform to a pose.
 *
 * Formula: p' = T p, R' = R_T R.
 */
inline Pose transformPose(const Pose& pose, const Eigen::Isometry3d& transform)
{
    return {
        transform * pose.position,
        Eigen::Quaterniond(transform.rotation()) * pose.orientation,
    };
}

} // namespace uvdar_core::pose_estimation
