#pragma once
#ifndef GP6P_HPP
#define GP6P_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "uvdar_core/pose_estimation/geometric_solver/generalized_solver_types.hpp"

namespace uvdar_core::pose_estimation::geometric_solver {

/**
 * @brief Linear generalized pose initializer for six or more calibrated rays.
 *
 * Each correspondence contributes two independent equations from
 * n^T(RX+t-o)=0, where n spans the plane normal to the ray direction. The
 * resulting generalized DLT system estimates the twelve entries of [R|t],
 * projects the raw rotation onto SO(3), recomputes translation, and finishes
 * with angular-residual refinement. This is the compact six-point linear tier
 * underlying non-central gPnP initializers such as OpenGV's implementation.
 */
class GP6P {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Solution = PoseSolution;
    using PointMatrix = GeneralizedPointMatrix;
    using Options = GeneralizedSolverOptions;
    using PoseJacobian = GeneralizedPoseJacobian;
    using BearingJacobian = GeneralizedBearingJacobian;

    static std::vector<Solution> solve(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Options& options = Options {})
    {
        if (!generalized_detail::validInput(
                world_points, ray_origins, ray_directions, 6)) {
            return {};
        }
        const PointMatrix directions = generalized_detail::normalizedDirections(ray_directions);
        Eigen::MatrixXd system;
        Eigen::VectorXd rhs;
        buildLinearSystem(world_points, ray_origins, directions, system, rhs);

        const bool central = commonRayOrigin(ray_origins);
        Eigen::Matrix<double, 12, 1> raw;
        if (central) {
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(system, Eigen::ComputeFullV);
            if (svd.info() != Eigen::Success || svd.matrixV().cols() != 12) {
                return {};
            }
            raw = svd.matrixV().col(11);
        } else {
            Eigen::ColPivHouseholderQR<Eigen::MatrixXd> decomposition(system);
            decomposition.setThreshold(1.0e-10);
            if (decomposition.rank() < 12) {
                return {};
            }
            raw = decomposition.solve(rhs);
        }
        if (!raw.allFinite()) {
            return {};
        }

        Eigen::Matrix3d raw_rotation;
        for (int column = 0; column < 3; ++column) {
            for (int row = 0; row < 3; ++row) {
                raw_rotation(row, column) = raw(3 * column + row);
            }
        }
        Eigen::Vector3d raw_translation = raw.tail<3>();
        if (central) {
            Eigen::JacobiSVD<Eigen::Matrix3d> scale_svd(raw_rotation);
            double scale = scale_svd.singularValues().mean();
            if (!std::isfinite(scale) || scale <= generalized_detail::kEpsilon) {
                return {};
            }
            // The homogeneous null vector has arbitrary sign and scale.
            if (raw_rotation.determinant() < 0.0) {
                scale = -scale;
            }
            raw_rotation /= scale;
            raw_translation /= scale;
        }

        Eigen::JacobiSVD<Eigen::Matrix3d> rotation_svd(
            raw_rotation,
            Eigen::ComputeFullU | Eigen::ComputeFullV);
        if (rotation_svd.info() != Eigen::Success) {
            return {};
        }
        Eigen::Matrix3d sign = Eigen::Matrix3d::Identity();
        if ((rotation_svd.matrixU() * rotation_svd.matrixV().transpose()).determinant() < 0.0) {
            sign(2, 2) = -1.0;
        }
        Solution seed;
        seed.R = rotation_svd.matrixU() * sign * rotation_svd.matrixV().transpose();
        seed.t = solveTranslation(
            world_points,
            ray_origins,
            directions,
            seed.R,
            central ? ray_origins.col(0) + raw_translation : raw_translation);
        if (!seed.t.allFinite()) {
            return {};
        }

        const auto refined = generalized_detail::refinePose(
            world_points, ray_origins, directions, seed, options);
        if (!refined
            || !generalized_detail::hasPositiveDepths(
                world_points, ray_origins, directions, *refined)
            || generalized_detail::maximumAngularError(
                   world_points, ray_origins, directions, *refined)
                > std::max(options.maximum_angular_error_rad, 1.0e-6)) {
            return {};
        }
        return {*refined};
    }

    static std::vector<PoseJacobian> jacobianPoseWrtBearings(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Options& options = Options {})
    {
        return generalized_detail::poseJacobiansWrtDirections(
            world_points,
            ray_origins,
            ray_directions,
            solve(world_points, ray_origins, ray_directions, options),
            options.damping);
    }

    static std::vector<BearingJacobian> jacobianBearingsWrtPose(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& ray_directions,
        const Options& options = Options {})
    {
        return generalized_detail::bearingJacobiansWrtPose(
            jacobianPoseWrtBearings(
                world_points, ray_origins, ray_directions, options),
            options.damping);
    }

private:
    static std::pair<Eigen::Vector3d, Eigen::Vector3d> rayNormals(const Eigen::Vector3d& direction)
    {
        Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
        if (std::abs(direction.x()) > 0.8) {
            axis = Eigen::Vector3d::UnitY();
        }
        const Eigen::Vector3d first = direction.cross(axis).normalized();
        const Eigen::Vector3d second = direction.cross(first).normalized();
        return {first, second};
    }

    static void buildLinearSystem(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& directions,
        Eigen::MatrixXd& system,
        Eigen::VectorXd& rhs)
    {
        system = Eigen::MatrixXd::Zero(2 * world_points.cols(), 12);
        rhs = Eigen::VectorXd::Zero(2 * world_points.cols());
        for (Eigen::Index i = 0; i < world_points.cols(); ++i) {
            const auto [first, second] = rayNormals(directions.col(i));
            const std::array<Eigen::Vector3d, 2> normals {first, second};
            for (int equation = 0; equation < 2; ++equation) {
                const Eigen::Index row_index = 2 * i + equation;
                const Eigen::Vector3d& normal = normals[static_cast<std::size_t>(equation)];
                for (int column = 0; column < 3; ++column) {
                    for (int row = 0; row < 3; ++row) {
                        system(row_index, 3 * column + row) = normal(row) * world_points(column, i);
                    }
                }
                system.block<1, 3>(row_index, 9) = normal.transpose();
                rhs(row_index) = normal.dot(ray_origins.col(i));
            }
        }
    }

    static bool commonRayOrigin(const PointMatrix& ray_origins)
    {
        double scale = 1.0;
        double maximum_difference = 0.0;
        for (Eigen::Index i = 1; i < ray_origins.cols(); ++i) {
            scale = std::max(scale, ray_origins.col(i).norm());
            maximum_difference = std::max(
                maximum_difference,
                (ray_origins.col(i) - ray_origins.col(0)).norm());
        }
        return maximum_difference <= 1.0e-10 * scale;
    }

    static Eigen::Vector3d solveTranslation(
        const PointMatrix& world_points,
        const PointMatrix& ray_origins,
        const PointMatrix& directions,
        const Eigen::Matrix3d& rotation,
        const Eigen::Vector3d& fallback)
    {
        Eigen::MatrixXd system(3 * world_points.cols(), 3);
        Eigen::VectorXd rhs(3 * world_points.cols());
        for (Eigen::Index i = 0; i < world_points.cols(); ++i) {
            const Eigen::Matrix3d projection = Eigen::Matrix3d::Identity()
                - directions.col(i) * directions.col(i).transpose();
            system.block<3, 3>(3 * i, 0) = projection;
            rhs.segment<3>(3 * i) = projection * (
                ray_origins.col(i) - rotation * world_points.col(i));
        }
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> decomposition(system);
        decomposition.setThreshold(1.0e-10);
        if (decomposition.rank() < 3) {
            return fallback;
        }
        return decomposition.solve(rhs);
    }
};

} // namespace uvdar_core::pose_estimation::geometric_solver

#endif // GP6P_HPP
