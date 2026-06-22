#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "uvdar_core/pose_estimation/math.hpp"

namespace uvdar_core::pose_estimation {

/**
 * @brief Rigid body pose in a ROS-style frame: translation plus unit quaternion.
 */
struct Pose {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

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
 * @brief Pose estimate with 6D covariance in [position, orientation] order.
 */
struct PoseMeasurement {
    int id = -1;
    Pose pose;
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
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
