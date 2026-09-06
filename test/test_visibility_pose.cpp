#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include <gtest/gtest.h>

#include "uvdar_core/pose_estimation/geometric_solver/visibility_pose.hpp"
#include "uvdar_core/pose_estimation/uncertainty.hpp"

namespace gs = uvdar_core::pose_estimation::geometric_solver;
namespace unc = uvdar_core::pose_estimation::uncertainty;

namespace {

Eigen::Vector3d unit(const Eigen::Vector3d& value)
{
    return value.normalized();
}

TEST(VisibilityPoseTheory, TwoRayDepthParameterizationIsExact)
{
    constexpr double marker_distance = 0.37;
    const Eigen::Vector3d first_bearing = Eigen::Vector3d::UnitZ();
    for (const double alpha : {0.2, 0.8, 1.4}) {
        const Eigen::Vector3d ray_plane_axis = Eigen::Vector3d::UnitX();
        const Eigen::Vector3d second_bearing =
            std::cos(alpha) * first_bearing
            + std::sin(alpha) * ray_plane_axis;
        for (const double fraction : {0.1, 0.35, 0.7, 0.95}) {
            const double beta = alpha
                + fraction * (std::numbers::pi - alpha);
            const double first_depth = marker_distance
                * std::sin(beta - alpha) / std::sin(alpha);
            const double second_depth = marker_distance
                * std::sin(beta) / std::sin(alpha);
            const Eigen::Vector3d camera_baseline =
                std::cos(beta) * first_bearing
                + std::sin(beta) * ray_plane_axis;

            EXPECT_GT(first_depth, 0.0);
            EXPECT_GT(second_depth, 0.0);
            EXPECT_NEAR(camera_baseline.norm(), 1.0, 1.0e-14);
            EXPECT_LT(
                (second_depth * second_bearing
                    - first_depth * first_bearing
                    - marker_distance * camera_baseline).norm(),
                1.0e-13);
        }
    }
}

TEST(VisibilityPoseTheory, RotatedNormalVisibilityIsAnExactSinusoid)
{
    const Eigen::Vector3d axis = unit({0.3, -0.4, 0.7});
    const Eigen::Vector3d normal = unit({-0.2, 0.8, 0.5});
    const Eigen::Vector3d view = unit({0.6, 0.1, -0.3});
    const double axial_term = axis.dot(normal) * axis.dot(view);
    const double cosine_coefficient = normal.dot(view) - axial_term;
    const double sine_coefficient = axis.cross(normal).dot(view);
    for (const double angle : {
             -std::numbers::pi,
             -1.7,
             -0.2,
             0.0,
             0.9,
             std::numbers::pi}) {
        const Eigen::Vector3d rotated = Eigen::AngleAxisd(angle, axis) * normal;
        const double analytic = axial_term
            + cosine_coefficient * std::cos(angle)
            + sine_coefficient * std::sin(angle);
        EXPECT_NEAR(rotated.dot(view), analytic, 1.0e-14);
    }
}

TEST(VisibilityPoseTheory, ProducesFinitePositiveSemidefiniteMoments)
{
    const auto estimates = gs::VisibilityPoseSolver::solve(
        {-0.1, 0.0, 0.0},
        {0.1, 0.0, 0.0},
        {0.0, 0.0, 1.0},
        {0.0, 0.0, 1.0},
        unit({-0.1, 0.0, 1.0}),
        unit({0.1, 0.0, 1.0}));

    ASSERT_FALSE(estimates.empty());
    double probability_sum = 0.0;
    for (const auto& estimate : estimates) {
        probability_sum += estimate.probability;
        EXPECT_TRUE(estimate.pose.translation.allFinite());
        EXPECT_TRUE(estimate.pose.rotation.allFinite());
        EXPECT_NEAR(estimate.pose.rotation.determinant(), 1.0, 1.0e-10);
        EXPECT_TRUE(estimate.covariance.allFinite());
        EXPECT_LT(
            (estimate.covariance - estimate.covariance.transpose()).norm(),
            1.0e-10);
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
            estimate.covariance);
        ASSERT_EQ(solver.info(), Eigen::Success);
        EXPECT_GE(solver.eigenvalues().minCoeff(), -1.0e-10);
        EXPECT_GT(estimate.covariance.trace(), 0.0);
    }
    EXPECT_NEAR(probability_sum, 1.0, 1.0e-10);
}

TEST(VisibilityPoseTheory, IsInvariantToCorrespondenceOrder)
{
    gs::VisibilityPoseConfig config;
    config.depth_quadrature_order = 96;
    config.spin_quadrature_order = 16;
    const Eigen::Vector3d first_point(-0.16, 0.03, -0.01);
    const Eigen::Vector3d second_point(0.11, -0.02, 0.04);
    const Eigen::Vector3d first_normal = unit({0.2, -0.1, 1.0});
    const Eigen::Vector3d second_normal = unit({-0.1, 0.25, 1.0});
    const Eigen::Vector3d first_bearing = unit({-0.13, 0.08, 1.2});
    const Eigen::Vector3d second_bearing = unit({0.15, 0.02, 1.1});

    const auto forward = gs::VisibilityPoseSolver::solve(
        first_point,
        second_point,
        first_normal,
        second_normal,
        first_bearing,
        second_bearing,
        config);
    const auto reverse = gs::VisibilityPoseSolver::solve(
        second_point,
        first_point,
        second_normal,
        first_normal,
        second_bearing,
        first_bearing,
        config);

    ASSERT_EQ(forward.size(), reverse.size());
    ASSERT_FALSE(forward.empty());
    std::vector<bool> used(reverse.size(), false);
    for (const auto& expected : forward) {
        auto best = reverse.size();
        double best_distance = std::numeric_limits<double>::infinity();
        for (std::size_t index = 0U; index < reverse.size(); ++index) {
            if (used[index]) {
                continue;
            }
            const double distance = unc::poseTangentDistance(
                expected.pose, reverse[index].pose);
            if (distance < best_distance) {
                best = index;
                best_distance = distance;
            }
        }
        ASSERT_LT(best, reverse.size());
        used[best] = true;
        EXPECT_LT(best_distance, 2.0e-2);
        EXPECT_NEAR(expected.probability, reverse[best].probability, 2.0e-3);
        EXPECT_LT((expected.covariance - reverse[best].covariance).norm(), 3.0e-2);
    }
}

TEST(VisibilityPoseTheory, RejectsDegenerateBearings)
{
    const Eigen::Vector3d point1(-0.1, 0.0, 0.0);
    const Eigen::Vector3d point2(0.1, 0.0, 0.0);
    const Eigen::Vector3d normal(0.0, 0.0, 1.0);
    const Eigen::Vector3d bearing = unit({0.1, 0.0, 1.0});
    EXPECT_TRUE(gs::VisibilityPoseSolver::solve(
        point1, point2, normal, normal, bearing, bearing).empty());
    EXPECT_TRUE(gs::VisibilityPoseSolver::solve(
        point1, point1, normal, normal, bearing, unit({-0.1, 0.0, 1.0})).empty());
}

TEST(VisibilityPoseTheory, VisibilityCanExcludeTheWholeDistanceCurve)
{
    gs::VisibilityPoseConfig config;
    config.visibility_half_angle_rad = 20.0 * std::numbers::pi / 180.0;
    const Eigen::Vector3d point1(-0.1, 0.0, 0.0);
    const Eigen::Vector3d point2(0.1, 0.0, 0.0);
    const Eigen::Vector3d common_normal(0.0, 0.0, 1.0);
    const Eigen::Vector3d bearing1(0.0, 0.0, 1.0);
    const Eigen::Vector3d bearing2 = unit({0.9, 0.0, -0.1});
    EXPECT_TRUE(gs::VisibilityPoseSolver::solve(
        point1,
        point2,
        common_normal,
        common_normal,
        bearing1,
        bearing2,
        config).empty());
}

TEST(VisibilityPoseTheory, MixtureCovarianceIncludesBranchAmbiguity)
{
    unc::PoseDistributionComponent first;
    first.pose.translation = {-1.0, 0.0, 0.0};
    first.pose.rotation = Eigen::AngleAxisd(
        -std::numbers::pi / 6.0,
        Eigen::Vector3d::UnitZ()).toRotationMatrix();
    first.covariance = unc::PoseCovariance::Identity() * 0.25;

    unc::PoseDistributionComponent second = first;
    second.pose.translation.x() = 1.0;
    second.pose.rotation = Eigen::AngleAxisd(
        std::numbers::pi / 6.0,
        Eigen::Vector3d::UnitZ()).toRotationMatrix();

    const auto moments = unc::momentMatchPoseDistribution({first, second});
    ASSERT_TRUE(moments.has_value());
    EXPECT_LT(moments->pose.translation.norm(), 1.0e-12);
    EXPECT_LT(Eigen::AngleAxisd(moments->pose.rotation).angle(), 1.0e-12);
    EXPECT_NEAR(moments->covariance(0, 0), 1.25, 1.0e-12);
    EXPECT_NEAR(
        moments->covariance(5, 5),
        0.25 + std::pow(std::numbers::pi / 6.0, 2),
        1.0e-12);
}

} // namespace
