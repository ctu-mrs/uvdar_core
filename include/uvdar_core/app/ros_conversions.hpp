#pragma once

#include <array>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "uvdar_core/pose_estimation/types.hpp"

namespace uvdar_core::app {

/**
 * @brief Convert a ROS 2 timestamp to seconds for estimator internals.
 */
inline double toSeconds(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

/**
 * @brief Convert floating-point seconds back to a ROS 2 timestamp.
 */
inline builtin_interfaces::msg::Time toRosTime(double seconds)
{
    if (seconds <= 0.0) {
        return builtin_interfaces::msg::Time {};
    }

    const double integral_seconds = std::floor(seconds);
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(integral_seconds);
    stamp.nanosec = static_cast<std::uint32_t>(std::llround((seconds - integral_seconds) * 1.0e9));
    if (stamp.nanosec >= 1000000000U) {
        ++stamp.sec;
        stamp.nanosec -= 1000000000U;
    }
    return stamp;
}

/**
 * @brief Convert a ROS transform message to Eigen isometry.
 */
inline Eigen::Isometry3d toEigen(const geometry_msgs::msg::TransformStamped& transform)
{
    const auto& translation = transform.transform.translation;
    const auto& quaternion = transform.transform.rotation;
    Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
    output.translation() = Eigen::Vector3d(translation.x, translation.y, translation.z);
    output.linear() = Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z).normalized().toRotationMatrix();
    return output;
}

/**
 * @brief Convert the internal pose representation to a ROS pose message.
 */
inline geometry_msgs::msg::Pose toMsg(const pose_estimation::Pose& pose)
{
    geometry_msgs::msg::Pose output;
    output.position.x = pose.position.x();
    output.position.y = pose.position.y();
    output.position.z = pose.position.z();
    output.orientation.w = pose.orientation.w();
    output.orientation.x = pose.orientation.x();
    output.orientation.y = pose.orientation.y();
    output.orientation.z = pose.orientation.z();
    return output;
}

/**
 * @brief Convert ROS row-major 6x6 pose covariance storage to Eigen.
 */
inline Eigen::Matrix<double, 6, 6> covarianceFromMsg(const std::array<double, 36>& input)
{
    Eigen::Matrix<double, 6, 6> output;
    for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 6; ++column) {
            output(column, row) = input[static_cast<std::size_t>(6 * column + row)];
        }
    }
    return output;
}

/**
 * @brief Convert Eigen 6x6 pose covariance to ROS message storage.
 */
inline std::array<double, 36> covarianceToMsg(const Eigen::Matrix<double, 6, 6>& covariance)
{
    std::array<double, 36> output {};
    for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 6; ++column) {
            output[static_cast<std::size_t>(6 * column + row)] = covariance(column, row);
        }
    }
    return output;
}

/**
 * @brief Extract the published 6D covariance from a 6D or 9D DKF state covariance.
 */
inline std::array<double, 36> stateCovarianceToMsg(const Eigen::MatrixXd& input, bool velocity_state)
{
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
    if (velocity_state) {
        covariance.topLeftCorner<3, 3>() = input.topLeftCorner<3, 3>();
        covariance.bottomRightCorner<3, 3>() = input.bottomRightCorner<3, 3>();
    } else {
        covariance = input.topLeftCorner<6, 6>();
    }
    return covarianceToMsg(covariance);
}

/**
 * @brief Rotate the translational and angular 3x3 blocks of a 6D pose covariance.
 */
inline Eigen::Matrix<double, 6, 6> rotatePoseCovariance(
    const Eigen::Matrix<double, 6, 6>& covariance,
    const Eigen::Matrix3d& rotation)
{
    Eigen::Matrix<double, 6, 6> output = covariance;
    output.topLeftCorner<3, 3>() = rotation * covariance.topLeftCorner<3, 3>() * rotation.transpose();
    output.bottomRightCorner<3, 3>() = rotation * covariance.bottomRightCorner<3, 3>() * rotation.transpose();
    output.topRightCorner<3, 3>() = rotation * covariance.topRightCorner<3, 3>() * rotation.transpose();
    output.bottomLeftCorner<3, 3>() = rotation * covariance.bottomLeftCorner<3, 3>() * rotation.transpose();
    return output;
}

} // namespace uvdar_core::app
